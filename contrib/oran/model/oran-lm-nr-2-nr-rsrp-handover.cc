#include "oran-lm-nr-2-nr-rsrp-handover.h"

#include "oran-command-nr-2-nr-handover.h"
#include "oran-data-repository.h"

#include "ns3/abort.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cfloat>
#include <map>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNr2NrRsrpHandover");

NS_OBJECT_ENSURE_REGISTERED(OranLmNr2NrRsrpHandover);

TypeId
OranLmNr2NrRsrpHandover::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::OranLmNr2NrRsrpHandover")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNr2NrRsrpHandover>()

            // -----------------------------
            // NO-SECRECY knobs (RSRP-only)
            // -----------------------------
            .AddAttribute("HysteresisDb",
                          "RSRP HO hysteresis margin in dB (RSRP-only LM).",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranLmNr2NrRsrpHandover::m_rsrpThreshold),
                          MakeDoubleChecker<double>())

            .AddAttribute("Warmup",
                          "Warm-up time before HO decisions start.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandover::m_warmup),
                          MakeTimeChecker())

            .AddAttribute("MinHoInterval",
                          "Minimum time between HO commands for the same UE.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandover::m_minHoInterval),
                          MakeTimeChecker())

            .AddAttribute("HoAttemptTimeout",
                          "If HO is pending, wait this long before allowing a retry.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNr2NrRsrpHandover::m_hoAttemptTimeout),
                          MakeTimeChecker());

    return tid;
}

OranLmNr2NrRsrpHandover::OranLmNr2NrRsrpHandover(void)
    : OranLm(),
      m_rsrpThreshold(2.0),
      m_warmup(Seconds(2.0)),
      m_minHoInterval(Seconds(2.0)),
      m_hoAttemptTimeout(Seconds(2.0))
{
    NS_LOG_FUNCTION(this);
    m_name = "OranLmNr2NrRsrpHandover";
}

OranLmNr2NrRsrpHandover::~OranLmNr2NrRsrpHandover(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
OranLmNr2NrRsrpHandover::Run(void)
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

std::vector<OranLmNr2NrRsrpHandover::UeInfo>
OranLmNr2NrRsrpHandover::GetUeInfos(Ptr<OranDataRepository> data) const
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

std::vector<OranLmNr2NrRsrpHandover::GnbInfo>
OranLmNr2NrRsrpHandover::GetGnbInfos(Ptr<OranDataRepository> data) const
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
OranLmNr2NrRsrpHandover::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<OranLmNr2NrRsrpHandover::UeInfo> ueInfos,
    std::vector<OranLmNr2NrRsrpHandover::GnbInfo> gnbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    static std::map<uint64_t, Time> lastHoCmdTime;       // UE nodeId -> last HO cmd time
    static std::map<uint64_t, uint16_t> pendingHoTarget; // UE nodeId -> target cell

    // ----------------------------
    // OLD fixed constants (kept)
    // ----------------------------
    // static const Time warmup = Seconds(4.0);
    // static const Time minHoInterval = Seconds(4.0);
    // static const Time hoAttemptTimeout = Seconds(3.0);

    std::vector<Ptr<OranCommand>> commands;

    for (auto ueInfo : ueInfos)
    {
        // ---- Warmup gate ----
        // if (Simulator::Now() < warmup) { continue; }   // OLD
        if (Simulator::Now() < m_warmup)
        {
            continue;
        }

        // ---- Get best cell + serving cell from measurements ----
        double bestRsrp = -DBL_MAX;
        uint16_t bestCellId = ueInfo.cellId;

        double servingRsrp = -DBL_MAX;
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;

        auto rsrpMeasurements = data->GetNrUeRsrpRsrq(ueInfo.nodeId);
        for (auto rsrpMeasurement : rsrpMeasurements)
        {
            uint16_t rnti;
            uint16_t cellId;
            double rsrp;
            double rsrq;
            bool isServingCell;
            uint16_t componentCarrierId;
            std::tie(rnti, cellId, rsrp, rsrq, isServingCell, componentCarrierId) = rsrpMeasurement;

            // Decide HO using CC/BWP 0 only (same as your code)
            if (componentCarrierId != 0)
            {
                continue;
            }

            if (isServingCell)
            {
                servingRsrp = rsrp;
                servingCellId = cellId; // use serving cellId from PHY meas
                servingRnti = rnti;     // use serving RNTI from PHY meas
            }

            if (rsrp > bestRsrp)
            {
                bestRsrp = rsrp;
                bestCellId = cellId;
            }
        }

        // If we don't know who is serving right now, don't risk a stale command
        if (servingCellId == 0 || servingRnti == 0)
        {
            continue;
        }

        const uint16_t currentCellId = servingCellId;
        const uint16_t currentRnti   = servingRnti;

        // ---- Clear pending if UE reached target ----
        auto pit = pendingHoTarget.find(ueInfo.nodeId);
        if (pit != pendingHoTarget.end() && pit->second == currentCellId)
        {
            pendingHoTarget.erase(pit);
        }

        // ---- If HO pending and not timed out, do not send another ----
        auto tit = lastHoCmdTime.find(ueInfo.nodeId);
        if (pendingHoTarget.count(ueInfo.nodeId) && tit != lastHoCmdTime.end())
        {
            // if (Simulator::Now() - tit->second < hoAttemptTimeout) { continue; } // OLD
            if (Simulator::Now() - tit->second < m_hoAttemptTimeout)
            {
                continue;
            }
        }

        // ---- Cooldown ----
        // if (tit != lastHoCmdTime.end() && (Simulator::Now() - tit->second) < minHoInterval) // OLD
        if (tit != lastHoCmdTime.end() && (Simulator::Now() - tit->second) < m_minHoInterval)
        {
            continue;
        }

        // ---- Map serving cellId to serving gNB E2 node id ----
        uint64_t oldCellNodeId = 0;
        for (const auto& gnbInfo : gnbInfos)
        {
            if (currentCellId == gnbInfo.cellId)
            {
                oldCellNodeId = gnbInfo.nodeId;
                break;
            }
        }
        if (oldCellNodeId == 0)
        {
            continue;
        }

        // ---- (Optional but recommended) Ensure target cell exists in gnbInfos ----
        bool targetKnown = false;
        for (const auto& g : gnbInfos)
        {
            if (g.cellId == bestCellId)
            {
                targetKnown = true;
                break;
            }
        }
        if (!targetKnown)
        {
            continue;
        }

        // ---- Decide HO (compare against CURRENT serving cell) ----
        if (bestCellId != currentCellId)
        {
            // ----------------------------
            // OLD constant hysteresis (kept)
            // ----------------------------
            // const double hysteresisDb = 4.0;

            // NEW: use attribute HysteresisDb (stored in m_rsrpThreshold)
            const double hysteresisDb = m_rsrpThreshold;

            // Only HO if best RSRP beats serving RSRP by hysteresis margin
            if (servingRsrp > -DBL_MAX && !(bestRsrp > servingRsrp + hysteresisDb))
            {
                continue;
            }

            Ptr<OranCommandNr2NrHandover> handoverCommand = CreateObject<OranCommandNr2NrHandover>();
            handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId)); // source gNB
            handoverCommand->SetAttribute("TargetRnti", UintegerValue(currentRnti));
            handoverCommand->SetAttribute("TargetCellId", UintegerValue(bestCellId));      // target cell

            NS_LOG_UNCOND("LM HO decision UE nodeId=" << ueInfo.nodeId
                                                     << " rnti=" << currentRnti
                                                     << " fromCell=" << currentCellId
                                                     << " toCell=" << bestCellId
                                                     << " bestRsrp=" << bestRsrp
                                                     << " servingRsrp=" << servingRsrp
                                                     << " hystDb=" << hysteresisDb);

            data->LogCommandLm(m_name, handoverCommand);
            commands.push_back(handoverCommand);

            lastHoCmdTime[ueInfo.nodeId] = Simulator::Now();
            pendingHoTarget[ueInfo.nodeId] = bestCellId;

            LogLogicToRepository("HO CMD t=" + std::to_string(Simulator::Now().GetSeconds()) +
                                 " UE=" + std::to_string(ueInfo.nodeId) +
                                 " rnti=" + std::to_string(currentRnti) +
                                 " " + std::to_string(currentCellId) + "->" + std::to_string(bestCellId));
        }
    }

    return commands;
}

} // namespace ns3
