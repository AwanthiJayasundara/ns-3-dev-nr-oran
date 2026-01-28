

#include "oran-lm-nr-2-nr-rsrp-handover.h"

#include "oran-command-nr-2-nr-handover.h"
#include "oran-data-repository.h"

#include "ns3/abort.h"
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
    static TypeId tid = TypeId("ns3::OranLmNr2NrRsrpHandover")
                            .SetParent<OranLm>()
                            .AddConstructor<OranLmNr2NrRsrpHandover>();

    return tid;
}

OranLmNr2NrRsrpHandover::OranLmNr2NrRsrpHandover(void)
    : OranLm()
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

    // Return the commands.
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
        // Get the current cell ID and RNTI of the UE and record it.
        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetNrUeCellInfo(ueInfo.nodeId);
        if (found)
        {
            // Get the latest location of the UE.
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location informtaion for this UE
                // so record it for a later analysis.
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
        // Get the cell ID of this gNB and record it.
        bool found;
        std::tie(found, gnbInfo.cellId) = data->GetNrGnbCellInfo(gnbInfo.nodeId);
        if (found)
        {
            // Get all known locations of the gNB.
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(gnbInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location information for this
                // gNB so record it for a later analysis.
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

// std::vector<Ptr<OranCommand>>
// OranLmNr2NrRsrpHandover::GetHandoverCommands(
//     Ptr<OranDataRepository> data,
//     std::vector<OranLmNr2NrRsrpHandover::UeInfo> ueInfos,
//     std::vector<OranLmNr2NrRsrpHandover::GnbInfo> gnbInfos) const
// {
//     NS_LOG_FUNCTION(this << data);

//     std::vector<Ptr<OranCommand>> commands;

//     // Compare the rsrp of each active gNB with the rsrp of each UEs
//     // and see if that UE is currently being served by the max rsrp cell. If
//     // there is a max rsrp gNB to the UE then the currently serving cell then
//     // issue a handover command.
//     for (auto ueInfo : ueInfos)
//     {
//         double max = -DBL_MAX;              // The maximum RSRP recorded.
//         uint64_t oldCellNodeId;             // The ID of the cell currently serving the UE.
//         uint16_t newCellId = ueInfo.cellId; // The ID of the closest cell.
//         auto rsrpMeasurements = data->GetNrUeRsrpRsrq(ueInfo.nodeId);
//         for (auto rsrpMeasurement : rsrpMeasurements)
//         {
//             uint16_t rnti;
//             uint16_t cellId;
//             double rsrp;
//             double rsrq;
//             bool isServingCell;
//             uint16_t componentCarrierId;
//             std::tie(rnti, cellId, rsrp, rsrq, isServingCell, componentCarrierId) = rsrpMeasurement;
//             LogLogicToRepository("RSRP from UE with RNTI " + std::to_string(rnti) + " in CellID " +
//                                  std::to_string(ueInfo.cellId) + " to gNB with CellID " +
//                                  std::to_string(cellId) + " is " + std::to_string(rsrp));

//             // Check if the RSRP is greater than the current maximum
//             if (rsrp > max)
//             {
//                 // Record the new maximum
//                 max = rsrp;
//                 // Record the ID of the cell that produced the new maximum.
//                 newCellId = cellId;

//                 LogLogicToRepository("RSRP to gNB with CellID " + std::to_string(cellId) +
//                                      " is largest so far");
//             }
//         }

//         for (const auto& gnbInfo : gnbInfos)
//         {
//             // Check if this cell is the currently serving this UE.
//             if (ueInfo.cellId == gnbInfo.cellId)
//             {
//                 // It is, so indicate record the ID of the cell that is
//                 // currently serving the UE.
//                 oldCellNodeId = gnbInfo.nodeId;
//             }
//         }

//         // Check if the ID of the closest cell is different from ID of the cell
//         // that is currently serving the UE
//         if (newCellId != ueInfo.cellId)
//         {
//             // It is, so issue a handover command.
//             Ptr<OranCommandNr2NrHandover> handoverCommand =
//                 CreateObject<OranCommandNr2NrHandover>();
//             // Send the command to the cell currently serving the UE.
//             handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));
//             // Use the RNTI that the current cell is using to identify the UE.
//             handoverCommand->SetAttribute("TargetRnti", UintegerValue(ueInfo.rnti));
//             // Give the current cell the ID of the new cell to handover to.
//             handoverCommand->SetAttribute("TargetCellId", UintegerValue(newCellId));
//             // Log the command to the storage
//             data->LogCommandLm(m_name, handoverCommand);
//             // Add the command to send.
//             commands.push_back(handoverCommand);

//             LogLogicToRepository("gNB (CellID " + std::to_string(newCellId) + ")" +
//                                  " is different than the currently attached gNB" + " (CellID " +
//                                  std::to_string(ueInfo.cellId) + ")." +
//                                  " Issuing handover command.");
//         }
//     }
//     return commands;
// }

std::vector<Ptr<OranCommand>>
OranLmNr2NrRsrpHandover::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<OranLmNr2NrRsrpHandover::UeInfo> ueInfos,
    std::vector<OranLmNr2NrRsrpHandover::GnbInfo> gnbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    static std::map<uint64_t, Time> lastHoCmdTime;       // UE nodeId -> last HO cmd time
    static std::map<uint64_t, uint16_t> pendingHoTarget; // UE nodeId -> target cell

    static const Time warmup = Seconds(4.0);
    static const Time minHoInterval = Seconds(4.0);
    static const Time hoAttemptTimeout = Seconds(3.0);

    std::vector<Ptr<OranCommand>> commands;

    for (auto ueInfo : ueInfos)
    {
        if (Simulator::Now() < warmup)
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

            // only decide HO using DL measurements (BWP/CC 0)
            if (componentCarrierId != 0)
            {
                continue;
            }

            if (isServingCell)
            {
                servingRsrp = rsrp;
                servingCellId = cellId;  // <<< FIX 2: use serving cellId from PHY meas
                servingRnti = rnti;      // <<< FIX 2: use serving RNTI from PHY meas
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

        const uint16_t currentCellId = servingCellId; // <<< use serving cell, NOT ueInfo.cellId
        const uint16_t currentRnti   = servingRnti;   // <<< use serving RNTI, NOT ueInfo.rnti

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
            if (Simulator::Now() - tit->second < hoAttemptTimeout)
            {
                continue;
            }
        }

        // ---- Cooldown ----
        if (tit != lastHoCmdTime.end() && (Simulator::Now() - tit->second) < minHoInterval)
        {
            continue;
        }

        // ---- Map serving cellId to serving gNB E2 node id ----
        uint64_t oldCellNodeId = 0;
        for (const auto& gnbInfo : gnbInfos)
        {
            if (currentCellId == gnbInfo.cellId)   // <<< FIX 2: use currentCellId
            {
                oldCellNodeId = gnbInfo.nodeId;
                break;
            }
        }
        if (oldCellNodeId == 0)
        {
            continue;
        }

        // ---- Decide HO (compare against CURRENT serving cell) ----
        if (bestCellId != currentCellId)
        {
            const double hysteresisDb = 4.0;
            if (servingRsrp > -DBL_MAX && !(bestRsrp > servingRsrp + hysteresisDb))
            {
                continue;
            }

            Ptr<OranCommandNr2NrHandover> handoverCommand = CreateObject<OranCommandNr2NrHandover>();
            handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId)); // source gNB
            handoverCommand->SetAttribute("TargetRnti", UintegerValue(currentRnti));       // <<< FIX 2
            handoverCommand->SetAttribute("TargetCellId", UintegerValue(bestCellId));      // target cell
            
            NS_LOG_UNCOND("LM HO decision UE nodeId=" << ueInfo.nodeId
                << " rnti=" << currentRnti
                << " fromCell=" << currentCellId
                << " toCell=" << bestCellId
                << " bestRsrp=" << bestRsrp
                << " servingRsrp=" << servingRsrp);


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
