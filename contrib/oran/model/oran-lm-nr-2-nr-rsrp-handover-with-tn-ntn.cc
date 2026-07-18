#include "oran-lm-nr-2-nr-rsrp-handover-with-tn-ntn.h"

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
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrRsrpHandoverWithTnNtn");
NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrRsrpHandoverWithTnNtn);

TypeId
OranLmNr2NrRsrpHandoverWithTnNtn::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrRsrpHandoverWithTnNtn")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrRsrpHandoverWithTnNtn>()
            .AddAttribute("HysteresisDb",
                          "RSRP handover hysteresis margin in dB.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_rsrpThreshold),
                          MakeDoubleChecker<double>())
            .AddAttribute("Warmup",
                          "Warm-up time before HO decisions start.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_warmup),
                          MakeTimeChecker())
            .AddAttribute("MinHoInterval",
                          "Minimum time between HO commands for the same UE.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_minHoInterval),
                          MakeTimeChecker())
            .AddAttribute("HoAttemptTimeout",
                          "If HO is pending, wait this long before allowing a retry.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_hoAttemptTimeout),
                          MakeTimeChecker())
            .AddAttribute("TimeToTrigger",
                          "Time a candidate must continuously remain preferable before HO.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_timeToTrigger),
                          MakeTimeChecker())
            .AddAttribute("MaxUesPerCell",
                          "Fallback global hard cap; 0 disables the global cap.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_maxUesPerCell),
                          MakeUintegerChecker<uint32_t>(0, std::numeric_limits<uint32_t>::max()))
            .AddAttribute("TryNextBest",
                          "If best-RSRP cell is full, try next-best cell; otherwise keep current cell.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_tryNextBest),
                          MakeBooleanChecker())
            .AddAttribute("MinAcceptableRsrpDbm",
                          "Do not issue HO if the chosen target RSRP is below this threshold.",
                          DoubleValue(-120.0),
                          MakeDoubleAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_minAcceptableRsrpDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("LowRsrpRecheck",
                          "Backoff time after a low-RSRP HO failure before re-evaluating that UE.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_lowRsrpRecheck),
                          MakeTimeChecker())
            .AddAttribute("TnMinRsrpDbm",
                          "If serving TN RSRP drops below this, NTN fallback is allowed.",
                          DoubleValue(-110.0),
                          MakeDoubleAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_tnMinRsrpDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("NtnEnterMarginDb",
                          "Extra margin used when entering NTN from TN if TN is still usable.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_ntnEnterMarginDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("TnReturnMarginDb",
                          "Extra margin used when returning from NTN to TN.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&OranLmNr2NrRsrpHandoverWithTnNtn::m_tnReturnMarginDb),
                          MakeDoubleChecker<double>());

    return tid;
}

OranLmNr2NrRsrpHandoverWithTnNtn::OranLmNr2NrRsrpHandoverWithTnNtn(void)
    : OranLm(),
      m_rsrpThreshold(2.0),
      m_warmup(Seconds(2.0)),
      m_minHoInterval(Seconds(2.0)),
      m_hoAttemptTimeout(Seconds(2.0)),
      m_timeToTrigger(Seconds(2.0)),
      m_lowRsrpRecheck(Seconds(2.0)),
      m_maxUesPerCell(0),
      m_tryNextBest(true),
      m_minAcceptableRsrpDbm(-120.0),
      m_tnMinRsrpDbm(-110.0),
      m_ntnEnterMarginDb(2.0),
      m_tnReturnMarginDb(2.0)
{
    NS_LOG_FUNCTION(this);
    m_name = "OranLmNr2NrRsrpHandoverWithTnNtn";
}

OranLmNr2NrRsrpHandoverWithTnNtn::~OranLmNr2NrRsrpHandoverWithTnNtn(void)
{
    NS_LOG_FUNCTION(this);
}

void
OranLmNr2NrRsrpHandoverWithTnNtn::SetCellCapacity(uint16_t cellId, uint32_t maxUes)
{
    NS_LOG_FUNCTION(this << cellId << maxUes);
    m_cellCapacityMap[cellId] = maxUes;
}

void
OranLmNr2NrRsrpHandoverWithTnNtn::SetCellIsNtn(uint16_t cellId, bool isNtn)
{
    NS_LOG_FUNCTION(this << cellId << isNtn);
    m_cellIsNtnMap[cellId] = isNtn;
}

void
OranLmNr2NrRsrpHandoverWithTnNtn::SetCellBackhaulDlSnrDb(uint16_t cellId, double snrDb)
{
    NS_LOG_FUNCTION(this << cellId << snrDb);
    m_cellBackhaulDlSnrDb[cellId] = snrDb;
}

void
OranLmNr2NrRsrpHandoverWithTnNtn::SetCellBackhaulUlSnrDb(uint16_t cellId, double snrDb)
{
    NS_LOG_FUNCTION(this << cellId << snrDb);
    m_cellBackhaulUlSnrDb[cellId] = snrDb;
}

void
OranLmNr2NrRsrpHandoverWithTnNtn::SetDecisionCsvFilename(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);
    m_decisionCsvFilename = filename;
}

void
OranLmNr2NrRsrpHandoverWithTnNtn::EnsureDecisionCsvOpen(void) const
{
    if (m_decisionCsv.is_open() || m_decisionCsvFilename.empty())
    {
        return;
    }

    m_decisionCsv.open(m_decisionCsvFilename, std::ios::out | std::ios::trunc);

    if (m_decisionCsv.is_open())
    {
        m_decisionCsv
            << "time,ueId,rnti,servingCell,servingRsrp,"
            << "candidateCell,candidateRsrp,candidateLoad,candidateEffLoad,candidateCap,"
            << "candidateIsNtn,backhaulDlSnr,backhaulUlSnr,finalChosenCell\n";
    }
}

void
OranLmNr2NrRsrpHandoverWithTnNtn::LogDecisionRow(uint64_t ueId,
                                                 uint16_t rnti,
                                                 uint16_t servingCellId,
                                                 double servingRsrp,
                                                 uint16_t candidateCellId,
                                                 double candidateRsrp,
                                                 uint32_t candidateLoad,
                                                 uint32_t candidateEffLoad,
                                                 uint32_t candidateCap,
                                                 bool candidateIsNtn,
                                                 double backhaulDlSnr,
                                                 double backhaulUlSnr,
                                                 uint16_t finalChosenCell) const
{
    EnsureDecisionCsvOpen();

    if (!m_decisionCsv.is_open())
    {
        return;
    }

    m_decisionCsv
        << Simulator::Now().GetSeconds() << ","
        << ueId << ","
        << rnti << ","
        << servingCellId << ","
        << servingRsrp << ","
        << candidateCellId << ","
        << candidateRsrp << ","
        << candidateLoad << ","
        << candidateEffLoad << ","
        << candidateCap << ","
        << (candidateIsNtn ? 1 : 0) << ","
        << backhaulDlSnr << ","
        << backhaulUlSnr << ","
        << finalChosenCell
        << "\n";
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrRsrpHandoverWithTnNtn::Run(void)
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranCommand>> commands;

    if (m_active)
    {
        NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                        "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");

        Ptr<OranDataRepository> data = m_nearRtRic->Data();
        std::vector<UeInfo> ueInfos = GetUeInfos(data);
        std::vector<GnbInfo> gnbInfos = GetGnbInfos(data);
        commands = GetHandoverCommands(data, ueInfos, gnbInfos);
    }

    return commands;
}

std::vector<OranLmNr2NrRsrpHandoverWithTnNtn::UeInfo>
OranLmNr2NrRsrpHandoverWithTnNtn::GetUeInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<UeInfo> ueInfos;
    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        UeInfo ueInfo;
        ueInfo.nodeId = ueId;

        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetNrUeCellInfo(ueInfo.nodeId);
        if (found)
        {
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                ueInfo.position = nodePositions.rbegin()->second;
                ueInfos.push_back(ueInfo);
            }
            else
            {
                NS_LOG_INFO("Could not find NR UE location for E2 Node ID = " << ueInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find NR UE cell info for E2 Node ID = " << ueInfo.nodeId);
        }
    }
    return ueInfos;
}

std::vector<OranLmNr2NrRsrpHandoverWithTnNtn::GnbInfo>
OranLmNr2NrRsrpHandoverWithTnNtn::GetGnbInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<GnbInfo> gnbInfos;
    for (auto gnbId : data->GetNrGnbE2NodeIds())
    {
        GnbInfo gnbInfo;
        gnbInfo.nodeId = gnbId;

        bool found;
        std::tie(found, gnbInfo.cellId) = data->GetNrGnbCellInfo(gnbInfo.nodeId);
        if (found)
        {
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(gnbInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                gnbInfo.position = nodePositions.rbegin()->second;
                gnbInfos.push_back(gnbInfo);
            }
            else
            {
                NS_LOG_INFO("Could not find NR gNB location for E2 Node ID = " << gnbInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find NR gNB cell info for E2 Node ID = " << gnbInfo.nodeId);
        }
    }
    return gnbInfos;
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrRsrpHandoverWithTnNtn::GetHandoverCommands(Ptr<OranDataRepository> data,
                                                          std::vector<UeInfo> ueInfos,
                                                          std::vector<GnbInfo> gnbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    static std::map<uint64_t, Time> lastHoCmdTime;       // UE nodeId -> last HO cmd time
    static std::map<uint64_t, uint16_t> pendingHoTarget; // UE nodeId -> target cell
    static std::map<uint64_t, Time> lowRsrpBlockUntil;   // UE nodeId -> recheck time
    static std::map<uint64_t, std::pair<uint16_t, Time>> tttState;

auto HasPerCellCap = [&](uint16_t cellId) -> bool {
    return m_cellCapacityMap.find(cellId) != m_cellCapacityMap.end();
};

auto GetCellCap = [&](uint16_t cellId) -> uint32_t {
    auto it = m_cellCapacityMap.find(cellId);
    if (it != m_cellCapacityMap.end())
    {
        return it->second; // explicit per-cell value, including 0 = blocked
    }
    return m_maxUesPerCell; // fallback global cap
};

auto CapacityOk = [&](uint16_t cellId, const std::map<uint16_t, uint32_t>& loadMap) -> bool {
    auto lit = loadMap.find(cellId);
    if (lit == loadMap.end())
    {
        NS_LOG_INFO("LM HO_FAIL_UNKNOWN_CELL_LOAD cell=" << cellId);
        return false;
    }

    auto it = m_cellCapacityMap.find(cellId);

    // Explicit per-cell capacity exists
    if (it != m_cellCapacityMap.end())
    {
        uint32_t cap = it->second;

        // IMPORTANT:
        // explicit per-cell 0 means BLOCKED
        if (cap == 0)
        {
            return false;
        }

        return lit->second < cap;
    }

    // No per-cell entry: use global fallback
    uint32_t cap = m_maxUesPerCell;

    // Global 0 means "no global limit"
    if (cap == 0)
    {
        return true;
    }

    return lit->second < cap;
};

auto IsValidRsrp = [](double rsrp) -> bool {
    return std::isfinite(rsrp) && rsrp > -500.0;
};

auto IsNtnCell = [&](uint16_t cellId) -> bool {
    auto it = m_cellIsNtnMap.find(cellId);
    if (it != m_cellIsNtnMap.end())
    {
        return it->second;
    }
    return false;
};

auto CellKnown = [&](uint16_t cellId) -> bool {
    for (const auto& g : gnbInfos)
    {
        if (g.cellId == cellId)
        {
            return true;
        }
    }
    return false;
};

auto GetServingGnbNodeId = [&](uint16_t servingCellId) -> uint64_t {
    for (const auto& g : gnbInfos)
    {
        if (g.cellId == servingCellId)
        {
            return g.nodeId;
        }
    }
    return 0;
};

std::vector<Ptr<OranCommand>> commands;

    // Real load snapshot
    std::map<uint16_t, uint32_t> realLoad;
    for (const auto& g : gnbInfos)
    {
        realLoad[g.cellId] = 0;
    }

    // Drop stale pending reservations
    for (auto it = pendingHoTarget.begin(); it != pendingHoTarget.end();)
    {
        auto tit = lastHoCmdTime.find(it->first);
        if (tit == lastHoCmdTime.end() ||
            (Simulator::Now() - tit->second) >= m_hoAttemptTimeout)
        {
            it = pendingHoTarget.erase(it);
        }
        else
        {
            ++it;
        }
    }

    // Count currently serving UEs
    for (const auto& ueInfo : ueInfos)
    {
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;

        auto meas = data->GetNrUeRsrpRsrq(ueInfo.nodeId);
        for (const auto& m : meas)
        {
            uint16_t rnti, cellId, ccId;
            double rsrp, rsrq;
            bool isServing;
            std::tie(rnti, cellId, rsrp, rsrq, isServing, ccId) = m;

            if (ccId != 0)
            {
                continue;
            }
            if (!isServing)
            {
                continue;
            }

            servingCellId = cellId;
            servingRnti = rnti;
            break;
        }

        if (servingCellId != 0 && servingRnti != 0)
        {
            realLoad[servingCellId]++;
        }
        else if (ueInfo.cellId != 0)
        {
            realLoad[ueInfo.cellId]++;
        }
    }

    std::map<uint16_t, uint32_t> effectiveLoad = realLoad;
    for (const auto& kv : pendingHoTarget)
    {
        uint16_t tgt = kv.second;
        if (CapacityOk(tgt, effectiveLoad))
        {
            effectiveLoad[tgt]++;
        }
    }

    NS_LOG_INFO("---- LOAD at t=" << Simulator::Now().GetSeconds() << " ----");
    for (const auto& g : gnbInfos)
    {
        uint16_t c = g.cellId;
        uint32_t cap = GetCellCap(c);
        bool hasPerCell = HasPerCellCap(c);
        const char* type = IsNtnCell(c) ? "NTN" : "TN";

        if (hasPerCell && cap == 0)
        {
            NS_LOG_INFO("  cell " << c << " (" << type << ") load=" << realLoad[c]
                        << " (BLOCKED)");
        }
        else if (!hasPerCell && cap == 0)
        {
            NS_LOG_INFO("  cell " << c << " (" << type << ") load=" << realLoad[c]
                        << " (global cap disabled)");
        }
        else
        {
            NS_LOG_INFO("  cell " << c << " (" << type << ") load=" << realLoad[c]
                        << "/" << cap);
        }
    }

    struct Cand
    {
        uint16_t cellId;
        double rsrp;
        bool isNtn;
    };

    for (const auto& ueInfo : ueInfos)
    {
        if (Simulator::Now() < m_warmup)
        {
            continue;
        }

        auto bit = lowRsrpBlockUntil.find(ueInfo.nodeId);
        if (bit != lowRsrpBlockUntil.end())
        {
            if (Simulator::Now() < bit->second)
            {
                continue;
            }
            lowRsrpBlockUntil.erase(bit);
        }

        auto tit = lastHoCmdTime.find(ueInfo.nodeId);
        if (pendingHoTarget.count(ueInfo.nodeId) && tit != lastHoCmdTime.end())
        {
            if (Simulator::Now() - tit->second < m_hoAttemptTimeout)
            {
                continue;
            }
        }

        if (tit != lastHoCmdTime.end() &&
            (Simulator::Now() - tit->second) < m_minHoInterval)
        {
            continue;
        }

        double servingRsrp = -DBL_MAX;
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;

        std::map<uint16_t, double> bestRsrpPerCell;

        auto rsrpMeasurements = data->GetNrUeRsrpRsrq(ueInfo.nodeId);
        for (const auto& m : rsrpMeasurements)
        {
            uint16_t rnti, cellId, ccId;
            double rsrp, rsrq;
            bool isServing;
            std::tie(rnti, cellId, rsrp, rsrq, isServing, ccId) = m;

            if (!IsValidRsrp(rsrp))
            {
                continue;
            }

            // Keep best observed RSRP per cell across all BWPs / CCs
            auto it = bestRsrpPerCell.find(cellId);
            if (it == bestRsrpPerCell.end() || rsrp > it->second)
            {
                bestRsrpPerCell[cellId] = rsrp;
            }

            // Keep the best serving measurement across all BWPs
            if (isServing && rsrp > servingRsrp)
            {
                servingRsrp = rsrp;
                servingCellId = cellId;
                servingRnti = rnti;
            }
        }

        if (servingCellId == 0 || servingRnti == 0 || !IsValidRsrp(servingRsrp))
        {
            continue;
        }

        const bool currentIsNtn = IsNtnCell(servingCellId);

        auto pit = pendingHoTarget.find(ueInfo.nodeId);
        if (pit != pendingHoTarget.end() && pit->second == servingCellId)
        {
            pendingHoTarget.erase(pit);
        }

        uint64_t servingGnbNodeId = GetServingGnbNodeId(servingCellId);
        if (servingGnbNodeId == 0)
        {
            continue;
        }

        std::vector<Cand> cands;
        cands.reserve(bestRsrpPerCell.size());
        for (const auto& kv : bestRsrpPerCell)
        {
            if (!CellKnown(kv.first))
            {
                continue;
            }
            if (!IsValidRsrp(kv.second))
            {
                continue;
            }
            cands.push_back({kv.first, kv.second, IsNtnCell(kv.first)});
        }

        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
            return a.rsrp > b.rsrp;
        });

        uint16_t chosenCell = servingCellId;
        double chosenRsrp = servingRsrp;

                auto LogAllCandidates = [&](uint16_t finalChosenCell) {
            for (const auto& c : cands)
            {
                if (c.cellId == servingCellId)
                {
                    continue;
                }

                double bhDl = -999.0;
                double bhUl = -999.0;

                auto itd = m_cellBackhaulDlSnrDb.find(c.cellId);
                if (itd != m_cellBackhaulDlSnrDb.end())
                {
                    bhDl = itd->second;
                }

                auto itu = m_cellBackhaulUlSnrDb.find(c.cellId);
                if (itu != m_cellBackhaulUlSnrDb.end())
                {
                    bhUl = itu->second;
                }

                uint32_t candLoad = 0;
                auto rlIt = realLoad.find(c.cellId);
                if (rlIt != realLoad.end())
                {
                    candLoad = rlIt->second;
                }

                uint32_t candEffLoad = 0;
                auto elIt = effectiveLoad.find(c.cellId);
                if (elIt != effectiveLoad.end())
                {
                    candEffLoad = elIt->second;
                }

                uint32_t candCap = GetCellCap(c.cellId);

                LogDecisionRow(ueInfo.nodeId,
                               servingRnti,
                               servingCellId,
                               servingRsrp,
                               c.cellId,
                               c.rsrp,
                               candLoad,
                               candEffLoad,
                               candCap,
                               c.isNtn,
                               bhDl,
                               bhUl,
                               finalChosenCell);
            }
        };

        // 1) Prefer a better TN candidate if available.
        auto SelectPreferredTn = [&]() -> bool {
            for (const auto& c : cands)
            {
                if (c.cellId == servingCellId || c.isNtn)
                {
                    continue;
                }
                if (!CapacityOk(c.cellId, effectiveLoad))
                {
                    if (c.isNtn && GetCellCap(c.cellId) == 0)
                    {
                        NS_LOG_INFO("LM HO_FAIL_XHAUL_BLOCKED UE=" << ueInfo.nodeId
                                      << " currCell=" << servingCellId
                                      << " candCell=" << c.cellId
                                      << " candRsrp=" << c.rsrp
                                      << " reason=UAV_AUTONOMY_CAPACITY_0");
                    }
                    if (m_tryNextBest)
                    {
                        continue;
                    }
                    return false;
                }

                if (!currentIsNtn)
                {
                    if (c.rsrp > servingRsrp + m_rsrpThreshold)
                    {
                        chosenCell = c.cellId;
                        chosenRsrp = c.rsrp;
                        return true;
                    }
                }
                else
                {
                    if (c.rsrp >= m_tnMinRsrpDbm &&
                        c.rsrp > servingRsrp + m_tnReturnMarginDb)
                    {
                        chosenCell = c.cellId;
                        chosenRsrp = c.rsrp;
                        return true;
                    }
                }
            }
            return false;
        };

        // 2) Enter NTN only as fallback when TN is weak or absent.
        auto SelectFallbackNtn = [&]() -> bool {
            for (const auto& c : cands)
            {
                if (c.cellId == servingCellId || !c.isNtn)
                {
                    continue;
                }
                if (!CapacityOk(c.cellId, effectiveLoad))
                {
                    if (m_tryNextBest)
                    {
                        continue;
                    }
                    return false;
                }

                if (!currentIsNtn)
                {
                    const bool tnWeak = servingRsrp < m_tnMinRsrpDbm;
                    const bool ntnClearlyBetter = c.rsrp > servingRsrp + m_ntnEnterMarginDb;
                    if (tnWeak || ntnClearlyBetter)
                    {
                        chosenCell = c.cellId;
                        chosenRsrp = c.rsrp;
                        return true;
                    }
                }
                else
                {
                    if (c.rsrp > servingRsrp + m_rsrpThreshold)
                    {
                        chosenCell = c.cellId;
                        chosenRsrp = c.rsrp;
                        return true;
                    }
                }
            }
            return false;
        };

        bool foundTarget = SelectPreferredTn();
        if (!foundTarget)
        {
            foundTarget = SelectFallbackNtn();
        }

        if (!foundTarget || chosenCell == servingCellId)
        {
            LogAllCandidates(0);
            tttState.erase(ueInfo.nodeId);
            continue;
        }

        if (chosenRsrp < m_minAcceptableRsrpDbm)
        {
            NS_LOG_INFO("LM HO_FAIL_LOW_RSRP UE=" << ueInfo.nodeId
                          << " currCell=" << servingCellId
                          << " candCell=" << chosenCell
                          << " candRsrp=" << chosenRsrp
                          << " < min=" << m_minAcceptableRsrpDbm);

            LogAllCandidates(0);
            lowRsrpBlockUntil[ueInfo.nodeId] = Simulator::Now() + m_lowRsrpRecheck;
            tttState.erase(ueInfo.nodeId);
            continue;
        }

        auto tttIt = tttState.find(ueInfo.nodeId);
        if (tttIt == tttState.end() || tttIt->second.first != chosenCell)
        {
            tttState[ueInfo.nodeId] = {chosenCell, Simulator::Now()};
            tttIt = tttState.find(ueInfo.nodeId);
        }

        Time elapsed = Simulator::Now() - tttIt->second.second;
        if (elapsed < m_timeToTrigger)
        {
            NS_LOG_INFO("UE " << ueInfo.nodeId
                          << " TTT_WAIT target=" << chosenCell
                          << " elapsed=" << elapsed.GetSeconds()
                          << "s need=" << m_timeToTrigger.GetSeconds() << "s");

            LogAllCandidates(0);
            continue;
        }

        LogAllCandidates(chosenCell);

        Ptr<OranCommandNr2NrHandover> handoverCommand = CreateObject<OranCommandNr2NrHandover>();
        handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(servingGnbNodeId));
        handoverCommand->SetAttribute("TargetRnti", UintegerValue(servingRnti));
        handoverCommand->SetAttribute("TargetCellId", UintegerValue(chosenCell));

        uint32_t tgtReal = realLoad[chosenCell];
        uint32_t tgtEff = effectiveLoad[chosenCell];
        uint32_t cap = GetCellCap(chosenCell);

        NS_LOG_INFO("LM HO UE=" << ueInfo.nodeId
                      << " rnti=" << servingRnti
                      << " " << servingCellId << "->" << chosenCell
                      << " type=" << (IsNtnCell(chosenCell) ? "NTN" : "TN")
                      << " servingRsrp=" << servingRsrp
                      << " targetRsrp=" << chosenRsrp
                      << " targetLoadReal=" << tgtReal
                      << "/" << cap
                      << " targetLoadEff=" << tgtEff << "/" << cap);

        data->LogCommandLm(m_name, handoverCommand);
        commands.push_back(handoverCommand);

        tttState.erase(ueInfo.nodeId);
        lastHoCmdTime[ueInfo.nodeId] = Simulator::Now();
        pendingHoTarget[ueInfo.nodeId] = chosenCell;

        effectiveLoad[chosenCell]++;

        LogLogicToRepository("HO CMD t=" + std::to_string(Simulator::Now().GetSeconds()) +
                             " UE=" + std::to_string(ueInfo.nodeId) +
                             " rnti=" + std::to_string(servingRnti) +
                             " " + std::to_string(servingCellId) +
                             "->" + std::to_string(chosenCell));
    }

    return commands;
}

} // namespace ns3
