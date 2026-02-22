#include "oran-lm-nr-2-nr-rsrp-handover-with-cell-load.h"

#include "oran-command-nr-2-nr-handover.h"
#include "oran-data-repository.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include <algorithm>
#include <vector>
#include <cfloat>
#include <map>
#include <string>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrRsrpHandoverWithCellLoad");

NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrRsrpHandoverWithCellLoad);

TypeId
OranLmNr2NrRsrpHandoverWithCellLoad::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrRsrpHandoverWithCellLoad")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrRsrpHandoverWithCellLoad>()

            // -----------------------------
            // NO-SECRECY knobs (RSRP-only)
            // -----------------------------
            .AddAttribute("HysteresisDb",
                          "RSRP HO hysteresis margin in dB (RSRP-only LM).",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_rsrpThreshold),
                          MakeDoubleChecker<double>())

            .AddAttribute("Warmup",
                          "Warm-up time before HO decisions start.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_warmup),
                          MakeTimeChecker())

            .AddAttribute("MinHoInterval",
                          "Minimum time between HO commands for the same UE.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_minHoInterval),
                          MakeTimeChecker())

            .AddAttribute("HoAttemptTimeout",
                          "If HO is pending, wait this long before allowing a retry.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_hoAttemptTimeout),
                          MakeTimeChecker())

            .AddAttribute("MaxUesPerCell",
                        "Hard cap: ... 0 disables the cap.",
                        UintegerValue(0), // or keep 10 if you want cap by default
                        MakeUintegerAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_maxUesPerCell),
                        MakeUintegerChecker<uint32_t>(0, std::numeric_limits<uint32_t>::max()))

            .AddAttribute("TryNextBest",
                        "If best-RSRP cell is full, try next-best cell; otherwise keep current cell.",
                        BooleanValue(true),
                        MakeBooleanAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_tryNextBest),
                        MakeBooleanChecker())
            
            .AddAttribute("MinAcceptableRsrpDbm",
                        "If the selected HO target RSRP is below this (dBm), do NOT issue HO; log failure and retry later.",
                        DoubleValue(-120.0),
                        MakeDoubleAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_minAcceptableRsrpDbm),
                        MakeDoubleChecker<double>())

            .AddAttribute("LowRsrpRecheck",
                        "Backoff time after a low-RSRP HO failure before re-evaluating that UE.",
                        TimeValue(Seconds(2.0)),
                        MakeTimeAccessor(&OranLmNr2NrRsrpHandoverWithCellLoad::m_lowRsrpRecheck),
                        MakeTimeChecker());


    return tid;
}

OranLmNr2NrRsrpHandoverWithCellLoad::OranLmNr2NrRsrpHandoverWithCellLoad(void)
    : OranLm(),
      m_rsrpThreshold(2.0),
      m_warmup(Seconds(2.0)),
      m_minHoInterval(Seconds(2.0)),
      m_hoAttemptTimeout(Seconds(2.0)),
      m_maxUesPerCell(0),
      m_tryNextBest(true),
      m_minAcceptableRsrpDbm(-120.0),
      m_lowRsrpRecheck(Seconds(2.0))

{
    NS_LOG_FUNCTION(this);
    m_name = "OranLmNr2NrRsrpHandoverWithCellLoad";
}

OranLmNr2NrRsrpHandoverWithCellLoad::~OranLmNr2NrRsrpHandoverWithCellLoad(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrRsrpHandoverWithCellLoad::Run(void)
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

std::vector<OranLmNr2NrRsrpHandoverWithCellLoad::UeInfo>
OranLmNr2NrRsrpHandoverWithCellLoad::GetUeInfos(Ptr<OranDataRepository> data) const
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

std::vector<OranLmNr2NrRsrpHandoverWithCellLoad::GnbInfo>
OranLmNr2NrRsrpHandoverWithCellLoad::GetGnbInfos(Ptr<OranDataRepository> data) const
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
OranLmNr2NrRsrpHandoverWithCellLoad::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<UeInfo> ueInfos,
    std::vector<GnbInfo> gnbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    static std::map<uint64_t, Time> lastHoCmdTime;       // UE nodeId -> last HO cmd time
    static std::map<uint64_t, uint16_t> pendingHoTarget; // UE nodeId -> target cell
    static std::map<uint64_t, Time> lowRsrpBlockUntil; // UE nodeId -> do not evaluate until this time

    std::vector<Ptr<OranCommand>> commands;

    // -------------------------------------------------------
    // Build: cellId -> current UE count (and reserve pending)
    // Use UE PHY "servingCell" measurements so it's current.
    // -------------------------------------------------------
    std::map<uint16_t, uint32_t> cellUeCount;

    // Make sure ALL gNB cells appear in the map (even if load=0)
    for (const auto& g : gnbInfos)
    {
        cellUeCount[g.cellId] += 0;
    }


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

            if (ccId != 0) continue;      // only BWP/CC 0
            if (!isServing) continue;

            servingCellId = cellId;
            servingRnti = rnti;
            break;
        }

        if (servingCellId != 0 && servingRnti != 0)
        {
            cellUeCount[servingCellId]++;
        }
        else if (ueInfo.cellId != 0) // fallback if no PHY serving found
        {
            cellUeCount[ueInfo.cellId]++;
        }
    }

    // Cleanup stale pending HOs (so we don't reserve capacity forever)
    for (auto it = pendingHoTarget.begin(); it != pendingHoTarget.end(); )
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

    // Reserve capacity for already-pending HOs (conservative but safe)
    for (const auto& kv : pendingHoTarget)
    {
        cellUeCount[kv.second]++; // reserve one slot
    }

    // -------- ADD HERE: per-tick load snapshot --------
    NS_LOG_INFO("---- LM tick t=" << Simulator::Now().GetSeconds() << " ----");
    for (const auto& kv : cellUeCount)
    {
        if (m_maxUesPerCell > 0)
            {
                NS_LOG_INFO("  cell " << kv.first << " load=" << kv.second << "/" << m_maxUesPerCell);
            }
            else
            {
                NS_LOG_INFO("  cell " << kv.first << " load=" << kv.second << " (cap=disabled)");
            }
    }


    // Helper: does cell exist?
    auto CellKnown = [&](uint16_t cellId) -> bool {
        for (const auto& g : gnbInfos) if (g.cellId == cellId) return true;
        return false;
    };

    // Helper: map serving cellId -> serving gNB nodeId (source gNB for HO command)
    auto GetServingGnbNodeId = [&](uint16_t servingCellId) -> uint64_t {
        for (const auto& g : gnbInfos) if (g.cellId == servingCellId) return g.nodeId;
        return 0;
    };

    for (const auto& ueInfo : ueInfos)
    {
        // Warmup gate
        if (Simulator::Now() < m_warmup) continue;

        // Low-RSRP backoff gate: if UE is blocked, skip evaluation until time expires
        auto bit = lowRsrpBlockUntil.find(ueInfo.nodeId);
        if (bit != lowRsrpBlockUntil.end())
        {
            if (Simulator::Now() < bit->second)
            {
                // still in backoff window
                continue;
            }
            // backoff expired -> allow evaluation again
            lowRsrpBlockUntil.erase(bit);
        }

        // If HO pending and not timed out, do not send another
        auto tit = lastHoCmdTime.find(ueInfo.nodeId);
        if (pendingHoTarget.count(ueInfo.nodeId) && tit != lastHoCmdTime.end())
        {
            if (Simulator::Now() - tit->second < m_hoAttemptTimeout) continue;
        }

        // Cooldown
        if (tit != lastHoCmdTime.end() && (Simulator::Now() - tit->second) < m_minHoInterval)
        {
            continue;
        }

        // -------------------------------------------------------
        // Collect measurements: serving + per-cell best RSRP
        // -------------------------------------------------------
        double servingRsrp = -DBL_MAX;
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;

        std::map<uint16_t, double> bestRsrpPerCell; // cellId -> max rsrp seen

        auto rsrpMeasurements = data->GetNrUeRsrpRsrq(ueInfo.nodeId);
        for (const auto& m : rsrpMeasurements)
        {
            uint16_t rnti, cellId, ccId;
            double rsrp, rsrq;
            bool isServing;
            std::tie(rnti, cellId, rsrp, rsrq, isServing, ccId) = m;

            if (ccId != 0) continue;

            // track serving
            if (isServing)
            {
                servingRsrp = rsrp;
                servingCellId = cellId;
                servingRnti = rnti;
            }

            // track best per cell
            auto it = bestRsrpPerCell.find(cellId);
            if (it == bestRsrpPerCell.end() || rsrp > it->second)
            {
                bestRsrpPerCell[cellId] = rsrp;
            }
        }

        if (servingCellId == 0 || servingRnti == 0) continue;

        const uint16_t currentCellId = servingCellId;
        const uint16_t currentRnti   = servingRnti;

        // Clear pending if UE reached target
        auto pit = pendingHoTarget.find(ueInfo.nodeId);
        if (pit != pendingHoTarget.end() && pit->second == currentCellId)
        {
            pendingHoTarget.erase(pit);
        }

        // Map serving cell -> source gNB nodeId
        uint64_t servingGnbNodeId = GetServingGnbNodeId(currentCellId);
        if (servingGnbNodeId == 0) continue;

        // -------------------------------------------------------
        // Build candidate list sorted by RSRP desc
        // -------------------------------------------------------
        struct Cand { uint16_t cellId; double rsrp; };
        std::vector<Cand> cands;
        cands.reserve(bestRsrpPerCell.size());

        for (const auto& kv : bestRsrpPerCell)
        {
            if (!CellKnown(kv.first)) continue; // must exist in gnbInfos
            cands.push_back({kv.first, kv.second});
        }

        std::sort(cands.begin(), cands.end(),
          [](const Cand& a, const Cand& b){ return a.rsrp > b.rsrp; });

        // -------------------------------------------------------
        // Pick target with hysteresis + capacity
        // -------------------------------------------------------
        const double hysteresisDb = m_rsrpThreshold;

        NS_LOG_INFO("UE " << ueInfo.nodeId
                << " servingCell=" << currentCellId
                << " servingRsrp=" << servingRsrp
                << " hystDb=" << hysteresisDb
                << " TryNextBest=" << m_tryNextBest);


        uint16_t chosenCell = currentCellId;
        double   chosenRsrp = servingRsrp;

        // bool consideredBestOnly = !m_tryNextBest;
        bool anyBeatsHyst = false;
        bool anyBeatsHystAndNotFull = false;
        bool lowRsrpFailed = false;

        for (size_t i = 0; i < cands.size(); ++i)
        {
            const auto& c = cands[i];

            if (c.cellId == currentCellId) continue;

            // hysteresis gate
            if (!(c.rsrp > servingRsrp + hysteresisDb))
            {
                NS_LOG_DEBUG("UE " << ueInfo.nodeId << " skip cell " << c.cellId
                                << " (no hyst) cand=" << c.rsrp
                                << " <= " << (servingRsrp + hysteresisDb));
                continue;
            }

            
            anyBeatsHyst = true;

            if (c.rsrp < m_minAcceptableRsrpDbm)
            {
                NS_LOG_UNCOND("LM HO_FAIL_LOW_RSRP UE=" << ueInfo.nodeId
                            << " currCell=" << currentCellId
                            << " candCell=" << c.cellId
                            << " candRsrp=" << c.rsrp
                            << " < min=" << m_minAcceptableRsrpDbm
                            << " recheckAfter=" << m_lowRsrpRecheck.GetSeconds() << "s");

                LogLogicToRepository("HO FAIL LOW_RSRP t=" + std::to_string(Simulator::Now().GetSeconds()) +
                                    " UE=" + std::to_string(ueInfo.nodeId) +
                                    " currCell=" + std::to_string(currentCellId) +
                                    " candCell=" + std::to_string(c.cellId) +
                                    " candRsrp=" + std::to_string(c.rsrp));

                
                lowRsrpBlockUntil[ueInfo.nodeId] = Simulator::Now() + m_lowRsrpRecheck;

                lowRsrpFailed = true;
                break; // stop searching candidates this tick
            }
            

            // capacity gate
            uint32_t load = cellUeCount[c.cellId];
            if (m_maxUesPerCell > 0 && load >= m_maxUesPerCell)
            {
                NS_LOG_INFO("UE " << ueInfo.nodeId << " candidate cell " << c.cellId
                                << " beats hyst but FULL (" << load << "/" << m_maxUesPerCell << ")");

                if (!m_tryNextBest)
                {
                    NS_LOG_INFO("UE " << ueInfo.nodeId
                                    << " TryNextBest=false -> keep current cell " << currentCellId);
                    break; // stop searching
                }
                continue; // try next best
            }

            anyBeatsHystAndNotFull = true;

            chosenCell = c.cellId;
            chosenRsrp = c.rsrp;

            NS_LOG_INFO("UE " << ueInfo.nodeId << " selected cell " << chosenCell
                            << " (rsrp=" << chosenRsrp << ", load=" << load << ")");
            break;
        }

        if (lowRsrpFailed)
        {
            continue; // skip HO command this tick
        }


        // No feasible target => keep serving cell
        if (chosenCell == currentCellId)
        {
            if (!anyBeatsHyst)
            {
                NS_LOG_INFO("UE " << ueInfo.nodeId
                                << " keep current: no neighbor exceeds serving+hysteresis");
            }
            else if (!anyBeatsHystAndNotFull)
            {
                NS_LOG_INFO("UE " << ueInfo.nodeId
                                << " keep current: all candidates that beat hysteresis are FULL");
            }
            continue;
        }


        // Emit HO command
        Ptr<OranCommandNr2NrHandover> handoverCommand = CreateObject<OranCommandNr2NrHandover>();
        handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(servingGnbNodeId)); // source gNB
        handoverCommand->SetAttribute("TargetRnti", UintegerValue(currentRnti));
        handoverCommand->SetAttribute("TargetCellId", UintegerValue(chosenCell));         // target cell

        if (m_maxUesPerCell > 0)
        {
            NS_LOG_UNCOND("LM HO UE=" << ueInfo.nodeId
                                    << " rnti=" << currentRnti
                                    << " " << currentCellId << "->" << chosenCell
                                    << " servingRsrp=" << servingRsrp
                                    << " targetRsrp=" << chosenRsrp
                                    << " hystDb=" << hysteresisDb
                                    << " targetLoad=" << cellUeCount[chosenCell] << "/" << m_maxUesPerCell);
        }
        else
        {
            NS_LOG_UNCOND("LM HO UE=" << ueInfo.nodeId
                                    << " rnti=" << currentRnti
                                    << " " << currentCellId << "->" << chosenCell
                                    << " servingRsrp=" << servingRsrp
                                    << " targetRsrp=" << chosenRsrp
                                    << " hystDb=" << hysteresisDb
                                    << " targetLoad=" << cellUeCount[chosenCell] << " (cap=disabled)");
        }

        data->LogCommandLm(m_name, handoverCommand);
        commands.push_back(handoverCommand);

        lastHoCmdTime[ueInfo.nodeId] = Simulator::Now();
        pendingHoTarget[ueInfo.nodeId] = chosenCell;

        // Reserve capacity immediately so multiple HOs in SAME tick don't exceed cap
        cellUeCount[chosenCell]++;

        LogLogicToRepository("HO CMD t=" + std::to_string(Simulator::Now().GetSeconds()) +
                             " UE=" + std::to_string(ueInfo.nodeId) +
                             " rnti=" + std::to_string(currentRnti) +
                             " " + std::to_string(currentCellId) + "->" + std::to_string(chosenCell));
    }

    return commands;
}

} // namespace ns3
