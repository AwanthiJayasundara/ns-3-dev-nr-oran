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

namespace ns3
{

/* ---------------------------------------------------------------------------
 * oran-lm-nr-secrecy-aware-handover.cc  (BRIEF COMMENT)
 *
 * Purpose:
 *   Near-RT RIC Logic Module (LM) that makes NR→NR handover decisions using
 *   normal RSRP best-cell logic, BUT only triggers HO if secrecy is not in outage.
 *
 * Inputs (read from OranDataRepository / SQLite DB):
 *   - UE serving cell + RNTI:        GetNrUeCellInfo()
 *   - UE RSRP/RSRQ measurements:     GetNrUeRsrpRsrq()   (uses CC/BWP 0 only)
 *   - UE SINR samples:              GetNrUeSinr()        (uses bwpId=0, isCtrl=false)
 *   - gNB cellId ↔ gNB E2NodeId map: GetNrGnbCellInfo()
 *   - node positions (optional/log): GetNodePositions()
 *
 * Key idea (Secrecy gate):
 *   - Compute secrecy capacity:
 *        Cs = [ log2(1+gb) - log2(1+ge) ]+
 *   - gb (legitimate link quality) is taken from serving DL-data SINR and
 *     scaled to candidate cell using RSRP gain (heuristic):
 *        gbCand ≈ gbServing * 10^((RSRPcand - RSRPserv)/10)
 *   - ge (eavesdropper link quality) is modeled as constant EavSinrDb.
 *   - Candidate cell is allowed only if:
 *        Cs >= SecrecyRateThr   (or Cs>0 if threshold=0)
 *
 * Handover decision flow (per UE each LM run):
 *   1) Warmup / cooldown / pending-HO timeout checks.
 *   2) Identify serving cell/RNTI from RSRP measurements.
 *   3) Read serving DL-data SINR from DB (optional required by RequireSinr).
 *   4) Sort candidate cells by RSRP descending.
 *   5) Pick first candidate that:
 *        - improves RSRP by > max(HysteresisDb, RsrpThresholdDb)
 *        - passes secrecy gate (not secrecy outage)
 *   6) Issue OranCommandNr2NrHandover (source gNB E2NodeId, UE RNTI, target cellId).
 *
 * Outputs:
 *   - HO commands returned to RIC and logged via LogCommandLm()
 *   - Optional LM action strings via LogLogicToRepository()
 *
 * Notes / assumptions:
 *   - Uses CC/BWP 0 for HO measurements.
 *   - Secrecy model is simplified (constant eavesdropper SINR + SINR-from-RSRP heuristic).
 *   - Requires DataRepository to implement GetNrUeSinr() and store nruesinr rows.
 * 
 * NOTE (candidate selection under secrecy gate):
 *   Candidates are sorted by RSRP (descending). The LM tests them in that order.
 *   If the best-RSRP candidate FAILS the secrecy gate, we DO NOT stop; we "continue"
 *   and evaluate the next best RSRP candidate (and so on) until one passes secrecy.
 *   We only "break" once a secrecy-OK candidate is found. If none pass, no HO is issued.
 *
 *   Important: with the current simplified secrecy model (constant ge and gb derived from
 *   serving SINR scaled by RSRP gain), Cs typically increases with RSRP gain, so if the
 *   strongest candidate fails secrecy, weaker ones will likely fail too. Using per-cell
 *   SINR or per-cell eavesdropper ge can make 2nd-best candidates meaningful.
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
                          DoubleValue(4.0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_hysteresisDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("RsrpThresholdDb",
                          "Optional min RSRP gain (dB) to allow HO. Usually same as hysteresis.",
                          DoubleValue(4.0),
                          MakeDoubleAccessor(&OranLmNrSecrecyAwareHandover::m_rsrpThresholdDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("Warmup",
                          "Warm-up time before HO decisions start.",
                          TimeValue(Seconds(4.0)),
                          MakeTimeAccessor(&OranLmNrSecrecyAwareHandover::m_warmup),
                          MakeTimeChecker())
            .AddAttribute("MinHoInterval",
                          "Minimum time between HO commands for same UE.",
                          TimeValue(Seconds(4.0)),
                          MakeTimeAccessor(&OranLmNrSecrecyAwareHandover::m_minHoInterval),
                          MakeTimeChecker())
            .AddAttribute("HoAttemptTimeout",
                          "If HO is pending, wait this long before allowing a retry.",
                          TimeValue(Seconds(3.0)),
                          MakeTimeAccessor(&OranLmNrSecrecyAwareHandover::m_hoAttemptTimeout),
                          MakeTimeChecker())
            .AddAttribute("SecrecyRateThr",
                          "Minimum secrecy capacity (bits/s/Hz). 0 => just require positive secrecy.",
                          DoubleValue(0.5),
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
                          MakeBooleanChecker());

    return tid;
}

OranLmNrSecrecyAwareHandover::OranLmNrSecrecyAwareHandover(void)
    : OranLm(),
      m_hysteresisDb(4.0),
      m_rsrpThresholdDb(4.0),
      m_warmup(Seconds(4.0)),
      m_minHoInterval(Seconds(4.0)),
      m_hoAttemptTimeout(Seconds(3.0)),
      m_secrecyRateThr(0.5),
      m_eavSinrDb(-5.0),
      m_requireSinr(true)
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

// ---- NEW: safe linear->dB helper for logging ----
static inline double
LinToDbSafe(double x)
{
    if (x <= 0.0)
    {
        return -1e9; // "very small" instead of -inf to keep logs readable
    }
    return 10.0 * std::log10(x);
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

    // Expected tuple: (rnti, cellId, bwpId, sinrLin, sinrDb, isCtrl)
    auto sinrRecs = data->GetNrUeSinr(ueE2NodeId);

    for (const auto& rec : sinrRecs)
    {
        uint16_t rnti, cellId, bwpId;
        double sinrLin, sinrDb;
        bool isCtrl;

        std::tie(rnti, cellId, bwpId, sinrLin, sinrDb, isCtrl) = rec;

        // Use Data SINR only (isCtrl=false) and BWP 0
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

    // --------------------------------------------------------------------
    // OLD (too strict): only accept DATA SINR (isCtrl=false)
    // This makes haveEve=0 if DB rows are mostly is_ctrl=true.
    //
    // for (auto eveId : data->GetNrEveNodeIds())
    // {
    //     auto recs = data->GetNrEveSinr(eveId);
    //     for (const auto& rec : recs) // assume latest-first
    //     {
    //         uint16_t recCellId, bwpId;
    //         double sinrLin, sinrDb;
    //         bool isCtrl;
    //         std::tie(recCellId, bwpId, sinrLin, sinrDb, isCtrl) = rec;
    //
    //         if (!isCtrl && bwpId == 0 && recCellId == cellId)
    //         {
    //             outGeLin = std::max(outGeLin, sinrLin);
    //             found = true;
    //             break;
    //         }
    //     }
    // }
    // return found;
    // --------------------------------------------------------------------

    // --------------------------------------------------------------------
    // NEW: Prefer DATA SINR if present, else fall back to CTRL SINR.
    // This matches your DB situation where nrevesinr has is_ctrl=true.
    // We still keep "worst Eve" idea by taking max SINR across Eve nodes.
    // --------------------------------------------------------------------
    double bestDataLin = 0.0;
    bool   foundData   = false;

    double bestCtrlLin = 0.0;
    bool   foundCtrl   = false;

    for (auto eveId : data->GetNrEveNodeIds())
    {
        auto recs = data->GetNrEveSinr(eveId);

        // For each Eve, pick the latest DATA record for this cell if it exists;
        // otherwise pick the latest CTRL record for this cell.
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

            // keep your BWP0 constraint
            if (bwpId != 0 || recCellId != cellId)
            {
                continue;
            }

            if (!isCtrl)
            {
                // found latest DATA SINR for this Eve+cell
                localFoundData = true;
                localDataLin   = sinrLin;
                break; // DATA is preferred, stop for this Eve
            }
            else
            {
                // remember latest CTRL SINR, but keep searching in case DATA exists later in list
                if (!localFoundCtrl)
                {
                    localFoundCtrl = true;
                    localCtrlLin   = sinrLin;
                }
                // do NOT break; keep scanning for possible DATA
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

    static std::map<uint64_t, Time> lastHoCmdTime;       // UE nodeId -> last HO cmd time
    static std::map<uint64_t, uint16_t> pendingHoTarget; // UE nodeId -> target cell

    std::vector<Ptr<OranCommand>> commands;

    if (Simulator::Now() < m_warmup)
    {
        return commands;
    }

    for (const auto& ueInfo : ueInfos)
    {
        // ---- Pull RSRP measurements (latest set) ----
        auto rsrpMeasurements = data->GetNrUeRsrpRsrq(ueInfo.nodeId);

        double servingRsrp = -DBL_MAX;
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;

        struct Cand
        {
            uint16_t cellId;
            double rsrp;
        };
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

            // Decide HO using CC/BWP 0
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

        // ---- Clear pending if UE reached target ----
        auto pit = pendingHoTarget.find(ueInfo.nodeId);
        if (pit != pendingHoTarget.end() && pit->second == currentCellId)
        {
            pendingHoTarget.erase(pit);
        }

        // ---- Pending HO => wait until timeout before retry ----
        auto tit = lastHoCmdTime.find(ueInfo.nodeId);
        if (pendingHoTarget.count(ueInfo.nodeId) && tit != lastHoCmdTime.end())
        {
            if (Simulator::Now() - tit->second < m_hoAttemptTimeout)
            {
                continue;
            }
        }

        // ---- Cooldown ----
        if (tit != lastHoCmdTime.end() && (Simulator::Now() - tit->second) < m_minHoInterval)
        {
            continue;
        }

        // ---- Map serving cellId -> serving gNB E2 node id ----
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

        // ---- Get serving SINR from DB ----
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

        // ---- Sort candidates by RSRP desc ----
        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
            return a.rsrp > b.rsrp;
        });

        // ---- Choose first secrecy-safe candidate (best RSRP after sorting) ----
        uint16_t chosenCellId = currentCellId;
        double chosenRsrp = servingRsrp;

        // ---- NEW: remember what Eve SINR we actually used for the chosen cell ----
        double chosenGeLin = DbToLin(m_eavSinrDb); // fallback by default
        bool   chosenHaveEve = false;
        double chosenCs = 0.0;
        double chosenGbLin = servingSinrLin;       // optional for log


        //const double geLin = DbToLin(m_eavSinrDb);

        for (const auto& c : cands)
        {
            if (c.cellId == currentCellId)
            {
                continue;
            }

            const double gainDb = c.rsrp - servingRsrp;

            // RSRP improvement gate
            if (!(gainDb > std::max(m_hysteresisDb, m_rsrpThresholdDb)))
            {
                continue;
            }

            // Candidate SINR estimate from serving SINR + RSRP delta
            const double gbLin = servingSinrLin * DbToLin(gainDb);

            // ---- NEW: dynamic eaves SINR for THIS candidate cell ----
            double geLinCand = 0.0;
            bool haveEve = GetWorstEveSinrLinForCell(data, c.cellId, geLinCand);

            // fallback (only if you want)
            if (!haveEve)
            {
                geLinCand = DbToLin(m_eavSinrDb);
            }

            // ---- secrecy using dynamic Eve SINR ----
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

            NS_LOG_INFO("Secrecy PASS UE=" << ueInfo.nodeId
                                          << " candCell=" << chosenCellId
                                          << " gainDb=" << gainDb
                                          << " Cs=" << cs);
            // ---- NEW: store the values that made us accept this candidate ----
            chosenGeLin = geLinCand;
            chosenHaveEve = haveEve;
            chosenCs = cs;
            chosenGbLin = gbLin;

            break;
        }

        if (chosenCellId == currentCellId)
        {
            continue;
        }

        Ptr<OranCommandNr2NrHandover> handoverCommand = CreateObject<OranCommandNr2NrHandover>();
        handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId)); // source gNB
        handoverCommand->SetAttribute("TargetRnti", UintegerValue(currentRnti));
        handoverCommand->SetAttribute("TargetCellId", UintegerValue(chosenCellId));

        // NS_LOG_UNCOND("LM HO (SecrecyAware) UE=" << ueInfo.nodeId
        //                                         << " rnti=" << currentRnti
        //                                         << " fromCell=" << currentCellId
        //                                         << " toCell=" << chosenCellId
        //                                         << " servingRsrp=" << servingRsrp
        //                                         << " chosenRsrp=" << chosenRsrp
        //                                         << " servingSinrLin=" << servingSinrLin
        //                                         << " eavSinrDb=" << m_eavSinrDb
        //                                         << " secrecyThr=" << m_secrecyRateThr);

        NS_LOG_UNCOND("LM HO (SecrecyAware) UE=" << ueInfo.nodeId
        << " rnti=" << currentRnti
        << " fromCell=" << currentCellId
        << " toCell=" << chosenCellId
        << " servingRsrp=" << servingRsrp
        << " chosenRsrp=" << chosenRsrp
        << " servingSinrLin=" << servingSinrLin
        << " gbLinUsed=" << chosenGbLin
        << " haveEve=" << (chosenHaveEve ? 1 : 0)
        << " geLinUsed=" << chosenGeLin
        << " geDbUsed=" << LinToDbSafe(chosenGeLin)
        << " fallbackEavDb=" << m_eavSinrDb
        << " Cs=" << chosenCs
        << " secrecyThr=" << m_secrecyRateThr);


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
