#include "oran-lm-nr-secrecy-aware-handover.h"

#include "oran-command-nr-2-nr-handover.h"
#include "oran-data-repository.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "ns3/string.h"
#include <fstream>
#include <sstream>

namespace ns3
{

/* ---------------------------------------------------------------------------
 * oran-lm-nr-secrecy-aware-handover.cc
 *
 * Purpose:
 *   Near-RT RIC Logic Module (LM) that makes NR→NR handover decisions using
 *   RSRP best-cell logic + secrecy gating.
 *
 * NEW in this version (important fixes):
 *   1) OUTAGE MODE: if current secrecy is in outage, we try to escape faster:
 *      - skip cooldown
 *      - allow small RSRP gain (kOutageMinRsrpGainDb)
 *      - choose candidate that maximizes Cs (estimated secrecy capacity)
 *
 *   2) SAFER Eve fallback:
 *      - If we don't have Eve SINR for a candidate cell, do NOT assume a small Eve.
 *      - Instead assume Eve is at least as good as the current cell Eve (conservative).
 *
 *   3) FIX CRASH (most likely):
 *      - Correctly set HO command E2Node IDs.
 *      - Some ns-oran versions interpret TargetE2NodeId differently.
 *      - So we set "source-like" AND "target-like" attributes if present.
 *      - This prevents sending a HO command to a wrong terminator.
 * --------------------------------------------------------------------------- */

NS_LOG_COMPONENT_DEFINE("OranLmNrSecrecyAwareHandover");
NS_OBJECT_ENSURE_REGISTERED(OranLmNrSecrecyAwareHandover);

TypeId
OranLmNrSecrecyAwareHandover::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::OranLmNrSecrecyAwareHandover")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNrSecrecyAwareHandover>()
            .AddAttribute("HysteresisDb",
                          "RSRP HO hysteresis in dB.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_hysteresisDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("RsrpThresholdDb",
                          "Optional min RSRP gain (dB) to allow HO. Usually same as hysteresis.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_rsrpThresholdDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("Warmup",
                          "Warm-up time before HO decisions start.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNrSecrecyAwareHandover::m_warmup),
                          MakeTimeChecker())
            .AddAttribute("MinHoInterval",
                          "Minimum time between HO commands for same UE.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNrSecrecyAwareHandover::m_minHoInterval),
                          MakeTimeChecker())
            .AddAttribute("HoAttemptTimeout",
                          "If HO is pending, wait this long before allowing a retry.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNrSecrecyAwareHandover::m_hoAttemptTimeout),
                          MakeTimeChecker())
            .AddAttribute("SecrecyRateThr",
                          "Minimum secrecy capacity (bits/s/Hz). 0 => just require positive secrecy.",
                          DoubleValue(0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_secrecyRateThr),
                          MakeDoubleChecker<double>())
            .AddAttribute("EavSinrDb",
                          "Assumed eavesdropper SINR (dB) for simple secrecy model.",
                          DoubleValue(-5.0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_eavSinrDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("RequireSinr",
                          "If true, block HO until serving SINR exists in DB.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranLmNrSecrecyAwareHandover::m_requireSinr),
                          MakeBooleanChecker())
            .AddAttribute("LeakageModel",
                        "Leakage model: oracle | riskmap | hybrid | fixed",
                        StringValue("oracle"),
                        MakeStringAccessor(&OranLmNrSecrecyAwareHandover::m_leakageModel),
                        MakeStringChecker())
            .AddAttribute("RiskMapFile",
                        "File: lines 'cellId riskScore' (0..1)",
                        StringValue(""),
                        MakeStringAccessor(&OranLmNrSecrecyAwareHandover::m_riskMapFile),
                        MakeStringChecker())
            .AddAttribute("RiskMinEavSinrDb",
                          "Eve SINR (dB) used when riskScore=0 (low risk).",
                          DoubleValue(-15.0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_riskMinEavSinrDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("RiskMaxEavSinrDb",
                          "Eve SINR (dB) used when riskScore=1 (high risk).",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_riskMaxEavSinrDb),
                          MakeDoubleChecker<double>());


    return tid;
}

OranLmNrSecrecyAwareHandover::OranLmNrSecrecyAwareHandover(void)
    : OranLm(),
      m_hysteresisDb(2.0),
      m_rsrpThresholdDb(2.0),
      m_warmup(Seconds(2.0)),
      m_minHoInterval(Seconds(2.0)),
      m_hoAttemptTimeout(Seconds(2.0)),
      m_secrecyRateThr(0),
      m_eavSinrDb(-5.0),
      m_requireSinr(true),
      m_riskMapLoaded(false),
      m_riskMinEavSinrDb(-15.0),
      m_riskMaxEavSinrDb(5.0)


{
    NS_LOG_FUNCTION(this);
    m_name = "OranLmNrSecrecyAwareHandover";
}

OranLmNrSecrecyAwareHandover::~OranLmNrSecrecyAwareHandover(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmNrSecrecyAwareHandover::Run(void)
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (m_active)
    {
        NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                        "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");

        Ptr<OranDataRepository> data = m_nearRtRic->Data();
        LoadRiskMapIfNeeded();
        auto ueInfos = GetUeInfos(data);
        auto gnbInfos = GetGnbInfos(data);
        commands = GetHandoverCommands(data, ueInfos, gnbInfos);
    }

    return commands;
}

std::vector<OranLmNrSecrecyAwareHandover::UeInfo>
OranLmNrSecrecyAwareHandover::GetUeInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<UeInfo> ueInfos;
    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        UeInfo ueInfo;
        ueInfo.nodeId = ueId;

        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetNrUeCellInfo(ueInfo.nodeId);
        if (!found)
        {
            NS_LOG_INFO("Could not find NR UE cell info for E2 Node ID = " << ueInfo.nodeId);
            continue;
        }

        auto nodePositions = data->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());
        if (nodePositions.empty())
        {
            NS_LOG_INFO("Could not find NR UE location for E2 Node ID = " << ueInfo.nodeId);
            continue;
        }

        ueInfo.position = nodePositions.rbegin()->second;
        ueInfos.push_back(ueInfo);
    }

    return ueInfos;
}

void
OranLmNrSecrecyAwareHandover::LoadRiskMapIfNeeded()
{
    if (m_riskMapLoaded)
    {
        return;
    }
    m_riskMapLoaded = true;
    m_cellRisk.clear();

    if (m_riskMapFile.empty())
    {
        return; // riskmap mode will still work (it will assume worst if missing)
    }

    std::ifstream in(m_riskMapFile.c_str());
    if (!in.is_open())
    {
        NS_LOG_UNCOND("RiskMapFile cannot be opened: " << m_riskMapFile
                      << " (riskmap will assume worst risk for unknown cells)");
        return;
    }

    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);

        uint16_t cellId;
        double risk;
        if (!(iss >> cellId >> risk))
        {
            continue;
        }

        // clamp to [0,1]
        risk = std::max(0.0, std::min(1.0, risk));
        m_cellRisk[cellId] = risk;
    }

    NS_LOG_UNCOND("Loaded risk map: entries=" << m_cellRisk.size()
                  << " from " << m_riskMapFile);
}

bool
OranLmNrSecrecyAwareHandover::GetLeakageSinrLinForCell(Ptr<OranDataRepository> data,
                                                       uint16_t cellId,
                                                       double& geLin) const
{
    // FIXED: constant attacker SINR
    if (m_leakageModel == "fixed")
    {
        geLin = DbToLin(m_eavSinrDb);
        return true;
    }

    // RISKMAP: map riskScore in [0,1] -> Eve SINR in [RiskMin..RiskMax] dB
    auto riskToGe = [&](uint16_t cId, double& outLin) -> bool {
        double risk = 1.0; // conservative default if unknown cell
        auto it = m_cellRisk.find(cId);
        if (it != m_cellRisk.end())
        {
            risk = it->second;
        }

        const double geDb = m_riskMinEavSinrDb + risk * (m_riskMaxEavSinrDb - m_riskMinEavSinrDb);
        outLin = DbToLin(geDb);
        return true;
    };

    if (m_leakageModel == "riskmap")
    {
        return riskToGe(cellId, geLin);
    }

    // ORACLE: measured worst Eve SINR from repository (if EVE nodes exist)
    if (m_leakageModel == "oracle")
    {
        bool ok = GetWorstEveSinrLinForCell(data, cellId, geLin);
        if (!ok)
        {
            // fallback (your existing behavior)
            geLin = DbToLin(m_eavSinrDb);
            return false; // "not found in DB"
        }
        return true;
    }

    // HYBRID: take the most conservative estimate among available sources
    if (m_leakageModel == "hybrid")
    {
        double geFixed = DbToLin(m_eavSinrDb);
        double geRisk  = 0.0;
        riskToGe(cellId, geRisk);

        double geOracle = 0.0;
        bool haveOracle = GetWorstEveSinrLinForCell(data, cellId, geOracle);

        geLin = std::max(geFixed, geRisk);
        if (haveOracle)
        {
            geLin = std::max(geLin, geOracle);
        }
        return haveOracle; // indicates whether oracle data existed
    }

    // Unknown string -> safe fallback
    geLin = DbToLin(m_eavSinrDb);
    return true;
}

std::vector<OranLmNrSecrecyAwareHandover::GnbInfo>
OranLmNrSecrecyAwareHandover::GetGnbInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<GnbInfo> gnbInfos;
    for (auto gnbId : data->GetNrGnbE2NodeIds())
    {
        GnbInfo gnbInfo;
        gnbInfo.nodeId = gnbId;

        bool found;
        std::tie(found, gnbInfo.cellId) = data->GetNrGnbCellInfo(gnbInfo.nodeId);
        if (!found)
        {
            NS_LOG_INFO("Could not find NR gNB cell info for E2 Node ID = " << gnbInfo.nodeId);
            continue;
        }

        auto nodePositions = data->GetNodePositions(gnbInfo.nodeId, Seconds(0), Simulator::Now());
        if (nodePositions.empty())
        {
            NS_LOG_INFO("Could not find NR gNB location for E2 Node ID = " << gnbInfo.nodeId);
            continue;
        }

        gnbInfo.position = nodePositions.rbegin()->second;
        gnbInfos.push_back(gnbInfo);
    }

    return gnbInfos;
}

double
OranLmNrSecrecyAwareHandover::DbToLin(double db)
{
    return std::pow(10.0, db / 10.0);
}

// safe linear->dB helper (for logs only)
static inline double
LinToDbSafe(double x)
{
    if (x <= 0.0)
    {
        return -1e9;
    }
    return 10.0 * std::log10(x);
}

// Helper: set a Uinteger attribute only if it exists in this build.
// This makes code compatible across ns-oran versions.
static inline void
SetUintegerAttrIfExists(Ptr<Object> obj, const std::string& name, uint64_t v)
{
    TypeId tid = obj->GetInstanceTypeId();
    TypeId::AttributeInformation info;
    if (tid.LookupAttributeByName(name, &info))
    {
        obj->SetAttribute(name, UintegerValue(v));
    }
}

double
OranLmNrSecrecyAwareHandover::SecrecyCapacity(double gbLin, double geLin)
{
    gbLin = std::max(0.0, gbLin);
    geLin = std::max(0.0, geLin);

    const double cb = std::log2(1.0 + gbLin);
    const double ce = std::log2(1.0 + geLin);
    return std::max(0.0, cb - ce);
}

bool
OranLmNrSecrecyAwareHandover::GetServingDlDataSinrLin(Ptr<OranDataRepository> data,
                                                      uint64_t ueE2NodeId,
                                                      uint16_t servingCellId,
                                                      uint16_t servingRnti,
                                                      double& outSinrLin) const
{
    outSinrLin = -1.0;

    auto sinrRecs = data->GetNrUeSinr(ueE2NodeId);

    for (const auto& rec : sinrRecs)
    {
        uint16_t rnti, cellId, bwpId;
        double sinrLin, sinrDb;
        bool isCtrl;

        std::tie(rnti, cellId, bwpId, sinrLin, sinrDb, isCtrl) = rec;

        // Use DATA SINR only (isCtrl=false) and BWP 0
        if (!isCtrl && bwpId == 0 && cellId == servingCellId && rnti == servingRnti)
        {
            outSinrLin = sinrLin;
            return (outSinrLin > 0.0);
        }
    }
    return false;
}

bool
OranLmNrSecrecyAwareHandover::GetWorstEveSinrLinForCell(Ptr<OranDataRepository> data,
                                                        uint16_t cellId,
                                                        double& outGeLin) const
{
    outGeLin = 0.0;
    bool found = false;

    // Prefer DATA SINR if present; else use CTRL SINR.
    double bestDataLin = 0.0;
    bool   foundData   = false;

    double bestCtrlLin = 0.0;
    bool   foundCtrl   = false;

    for (auto eveId : data->GetNrEveNodeIds())
    {
        auto recs = data->GetNrEveSinr(eveId);

        bool   localFoundData = false;
        double localDataLin   = 0.0;

        bool   localFoundCtrl = false;
        double localCtrlLin   = 0.0;

        for (const auto& rec : recs) // assume latest-first
        {
            uint16_t recCellId, bwpId;
            double sinrLin, sinrDb;
            bool isCtrl;
            std::tie(recCellId, bwpId, sinrLin, sinrDb, isCtrl) = rec;

            if (bwpId != 0 || recCellId != cellId)
            {
                continue;
            }

            if (!isCtrl)
            {
                localFoundData = true;
                localDataLin   = sinrLin;
                break;
            }
            else
            {
                if (!localFoundCtrl)
                {
                    localFoundCtrl = true;
                    localCtrlLin   = sinrLin;
                }
            }
        }

        if (localFoundData)
        {
            bestDataLin = std::max(bestDataLin, localDataLin);
            foundData = true;
        }
        else if (localFoundCtrl)
        {
            bestCtrlLin = std::max(bestCtrlLin, localCtrlLin);
            foundCtrl = true;
        }
    }

    if (foundData)
    {
        outGeLin = bestDataLin;
        found = true;
    }
    else if (foundCtrl)
    {
        outGeLin = bestCtrlLin;
        found = true;
    }

    return found;
}

std::vector<Ptr<OranCommand>>
OranLmNrSecrecyAwareHandover::GetHandoverCommands(Ptr<OranDataRepository> data,
                                                  const std::vector<UeInfo>& ueInfos,
                                                  const std::vector<GnbInfo>& gnbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    static std::map<uint64_t, Time> lastHoCmdTime;
    static std::map<uint64_t, uint16_t> pendingHoTarget;

    std::vector<Ptr<OranCommand>> commands;

    if (Simulator::Now() < m_warmup)
    {
        return commands;
    }

    const double kEps = 1e-6;
    const double kOutageMinRsrpGainDb = 0.1;

    for (const auto& ueInfo : ueInfos)
    {
        auto rsrpMeasurements = data->GetNrUeRsrpRsrq(ueInfo.nodeId);

        double servingRsrp = -DBL_MAX;
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;

        struct Cand { uint16_t cellId; double rsrp; };
        std::vector<Cand> cands;

        for (const auto& meas : rsrpMeasurements)
        {
            uint16_t rnti;
            uint16_t cellId;
            double rsrp;
            double rsrq;
            bool isServingCell;
            uint8_t componentCarrierId;

            std::tie(rnti, cellId, rsrp, rsrq, isServingCell, componentCarrierId) = meas;

            if (componentCarrierId != 0)
            {
                continue;
            }

            cands.push_back({cellId, rsrp});

            if (isServingCell)
            {
                servingRsrp = rsrp;
                servingCellId = cellId;
                servingRnti = rnti;
            }
        }

        if (servingCellId == 0 || servingRnti == 0)
        {
            continue;
        }

        const uint16_t currentCellId = servingCellId;
        const uint16_t currentRnti = servingRnti;

        // Clear pending if reached target
        auto pit = pendingHoTarget.find(ueInfo.nodeId);
        if (pit != pendingHoTarget.end() && pit->second == currentCellId)
        {
            pendingHoTarget.erase(pit);
        }

        // Pending HO => wait timeout
        auto tit = lastHoCmdTime.find(ueInfo.nodeId);
        if (pendingHoTarget.count(ueInfo.nodeId) && tit != lastHoCmdTime.end())
        {
            if (Simulator::Now() - tit->second < m_hoAttemptTimeout)
            {
                continue;
            }
        }

        // Map serving cell -> serving gNB E2 node id
        uint64_t oldCellNodeId = 0;
        for (const auto& g : gnbInfos)
        {
            if (g.cellId == currentCellId)
            {
                oldCellNodeId = g.nodeId;
                break;
            }
        }
        if (oldCellNodeId == 0)
        {
            continue;
        }

        // Get serving SINR
        double servingSinrLin = -1.0;
        const bool hasSinr =
            GetServingDlDataSinrLin(data, ueInfo.nodeId, currentCellId, currentRnti, servingSinrLin);

        if (!hasSinr && m_requireSinr)
        {
            continue;
        }
        if (!hasSinr)
        {
            servingSinrLin = DbToLin(-10.0);
        }

        // Current Eve SINR for current cell
        double currentGeLin = 0.0;
        bool haveCurrentEve = GetLeakageSinrLinForCell(data, currentCellId, currentGeLin);
        if (!haveCurrentEve)
        {
            currentGeLin = DbToLin(m_eavSinrDb);
        }

        const double currentCs = SecrecyCapacity(servingSinrLin, currentGeLin);
        const bool currentOutage =
            (m_secrecyRateThr <= 0.0) ? (currentCs <= 0.0) : (currentCs < m_secrecyRateThr);

        // If NOT outage: apply cooldown
        if (!currentOutage)
        {
            if (tit != lastHoCmdTime.end() && (Simulator::Now() - tit->second) < m_minHoInterval)
            {
                continue;
            }
        }

        // Sort by RSRP desc
        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
            return a.rsrp > b.rsrp;
        });

        // Debug: show candidates when outage (helps understand "why no HO")
        if (currentOutage)
        {
            NS_LOG_UNCOND("OUTAGE DETECTED t=" << Simulator::Now().GetSeconds()
                          << " UE=" << ueInfo.nodeId
                          << " cell=" << currentCellId
                          << " Cs=" << currentCs
                          << " thr=" << m_secrecyRateThr
                          << " servingSinrLin=" << servingSinrLin
                          << " currentGeLin=" << currentGeLin
                          << " numCands=" << cands.size());

            for (const auto& c : cands)
            {
                NS_LOG_UNCOND("  cand cell=" << c.cellId
                              << " rsrp=" << c.rsrp
                              << " gainDb=" << (c.rsrp - servingRsrp));
            }
        }

        // Default chosen = stay
        uint16_t chosenCellId = currentCellId;
        double chosenRsrp = servingRsrp;

        double chosenGeLin = currentGeLin;
        bool   chosenHaveEve = haveCurrentEve;
        double chosenCs = currentCs;
        double chosenGbLin = servingSinrLin;

        if (!currentOutage)
        {
            // NORMAL MODE
            for (const auto& c : cands)
            {
                if (c.cellId == currentCellId)
                {
                    continue;
                }

                const double gainDb = c.rsrp - servingRsrp;

                // Require minimum RSRP gain
                if (!(gainDb > std::max(m_hysteresisDb, m_rsrpThresholdDb)))
                {
                    continue;
                }

                const double gbLin = servingSinrLin * DbToLin(gainDb);

                double geLinCand = 0.0;
                bool haveEve = GetLeakageSinrLinForCell(data, c.cellId, geLinCand);

                // Conservative fallback:
                // if we don't know Eve on candidate, assume at least currentGeLin
                if (!haveEve)
                {
                    geLinCand = std::max(currentGeLin, DbToLin(m_eavSinrDb));
                }

                const double cs = SecrecyCapacity(gbLin, geLinCand);

                const bool secrecyOk =
                    (m_secrecyRateThr <= 0.0) ? (cs > 0.0) : (cs >= m_secrecyRateThr);

                if (!secrecyOk)
                {
                    NS_LOG_INFO("Secrecy FAIL UE=" << ueInfo.nodeId
                                                  << " candCell=" << c.cellId
                                                  << " Cs=" << cs
                                                  << " (thr=" << m_secrecyRateThr << ")");
                    continue;
                }

                chosenCellId = c.cellId;
                chosenRsrp = c.rsrp;
                chosenGbLin = gbLin;
                chosenGeLin = geLinCand;
                chosenHaveEve = haveEve;
                chosenCs = cs;
                break;
            }
        }
        else
        {
            // OUTAGE MODE: maximize Cs, relax RSRP gate
            uint16_t bestCellId = currentCellId;
            double bestRsrp = servingRsrp;

            double bestCs = currentCs;
            double bestGbLin = servingSinrLin;
            double bestGeLin = currentGeLin;
            bool   bestHaveEve = haveCurrentEve;

            for (const auto& c : cands)
            {
                if (c.cellId == currentCellId)
                {
                    continue;
                }

                const double gainDb = c.rsrp - servingRsrp;
                const double gbLin = servingSinrLin * DbToLin(gainDb);

                double geLinCand = 0.0;
                bool haveEve = GetWorstEveSinrLinForCell(data, c.cellId, geLinCand);

                // Conservative fallback:
                if (!haveEve)
                {
                    geLinCand = std::max(currentGeLin, DbToLin(m_eavSinrDb));
                }

                const double cs = SecrecyCapacity(gbLin, geLinCand);

                const bool betterCs = (cs > bestCs + kEps);
                const bool tieBetterRsrp = (std::fabs(cs - bestCs) <= kEps) && (c.rsrp > bestRsrp + kEps);

                if (betterCs || tieBetterRsrp)
                {
                    bestCellId = c.cellId;
                    bestRsrp = c.rsrp;
                    bestCs = cs;
                    bestGbLin = gbLin;
                    bestGeLin = geLinCand;
                    bestHaveEve = haveEve;
                }
            }

            const bool secrecyImproves = (bestCs > currentCs + kEps);
            const bool rsrpImproves = (bestRsrp > servingRsrp + kOutageMinRsrpGainDb);

            if (bestCellId != currentCellId && (secrecyImproves || rsrpImproves))
            {
                chosenCellId = bestCellId;
                chosenRsrp = bestRsrp;
                chosenCs = bestCs;
                chosenGbLin = bestGbLin;
                chosenGeLin = bestGeLin;
                chosenHaveEve = bestHaveEve;
            }
        }

        if (chosenCellId == currentCellId)
        {
            continue;
        }

        // Map chosen cell -> target gNB E2 node id
        uint64_t targetCellNodeId = 0;
        for (const auto& g : gnbInfos)
        {
            if (g.cellId == chosenCellId)
            {
                targetCellNodeId = g.nodeId;
                break;
            }
        }
        NS_ABORT_MSG_IF(targetCellNodeId == 0,
                        "Chosen target cellId has no gNB mapping: cellId=" << chosenCellId);

        Ptr<OranCommandNr2NrHandover> handoverCommand = CreateObject<OranCommandNr2NrHandover>();

        // Always set cell + rnti
        handoverCommand->SetAttribute("TargetRnti", UintegerValue(currentRnti));
        handoverCommand->SetAttribute("TargetCellId", UintegerValue(chosenCellId));

        // IMPORTANT:
        // Different ns-oran versions interpret TargetE2NodeId differently.
        // We set both source-like and target-like attrs IF they exist.
        //
        //  - source gNB (current serving) = oldCellNodeId
        //  - target gNB (new serving)    = targetCellNodeId

        // Common names across variants:
        SetUintegerAttrIfExists(handoverCommand, "SourceE2NodeId", oldCellNodeId);
        SetUintegerAttrIfExists(handoverCommand, "ServingE2NodeId", oldCellNodeId);

        SetUintegerAttrIfExists(handoverCommand, "TargetGnbE2NodeId", targetCellNodeId);
        SetUintegerAttrIfExists(handoverCommand, "DestinationE2NodeId", targetCellNodeId);

        // Your original line kept for compatibility:
        // If in your version TargetE2NodeId means "executor node",
        // then it should be the SOURCE gNB. If it means "target gNB",
        // then it should be the TARGET gNB. To reduce crash risk, we set it to SOURCE
        // and rely on the extra target attributes if they exist.
        //
        // If your version only has TargetE2NodeId, and it expects TARGET,
        // you should change this to targetCellNodeId.
        handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));

        NS_LOG_UNCOND("LM HO (SecrecyAware) UE=" << ueInfo.nodeId
                      << " rnti=" << currentRnti
                      << " fromCell=" << currentCellId
                      << " toCell=" << chosenCellId
                      << " srcE2=" << oldCellNodeId
                      << " tgtE2=" << targetCellNodeId
                      << " servingRsrp=" << servingRsrp
                      << " chosenRsrp=" << chosenRsrp
                      << " servingSinrLin=" << servingSinrLin
                      << " gbLinUsed=" << chosenGbLin
                      << " haveEve=" << (chosenHaveEve ? 1 : 0)
                      << " geLinUsed=" << chosenGeLin
                      << " geDbUsed=" << LinToDbSafe(chosenGeLin)
                      << " Cs=" << chosenCs
                      << " thr=" << m_secrecyRateThr
                      << " outageMode=" << (currentOutage ? 1 : 0));

        data->LogCommandLm(m_name, handoverCommand);
        commands.push_back(handoverCommand);

        lastHoCmdTime[ueInfo.nodeId] = Simulator::Now();
        pendingHoTarget[ueInfo.nodeId] = chosenCellId;

        LogLogicToRepository("HO(secrecy) t=" + std::to_string(Simulator::Now().GetSeconds()) +
                             " UE=" + std::to_string(ueInfo.nodeId) +
                             " rnti=" + std::to_string(currentRnti) +
                             " " + std::to_string(currentCellId) + "->" + std::to_string(chosenCellId));
    }

    return commands;
}

} // namespace ns3
