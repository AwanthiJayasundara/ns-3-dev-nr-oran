

#include "oran-lm-nr-2-nr-rsrp-handover.h"

#include "oran-command-nr-2-nr-handover.h"
#include "oran-data-repository.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cfloat>

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

    // Static map to track last attempted handover target for each UE (by UE Node ID).
    static std::map<uint64_t, uint16_t> lastHandoverTarget;

    std::vector<Ptr<OranCommand>> commands;

    // **Reset last attempted target if handover succeeded**:
    // If the UE's current cell matches the last attempted target, clear the record (handover was successful).
    for (auto ueInfo : ueInfos)
    {
        auto it = lastHandoverTarget.find(ueInfo.nodeId);
        if (it != lastHandoverTarget.end() && it->second == ueInfo.cellId)
        {
            // Handover to that target succeeded, so remove tracking.
            lastHandoverTarget.erase(it);
        }
    }

    // Compare the RSRP of each active gNB with the RSRP of each UE and decide handovers.
    for (auto ueInfo : ueInfos)
    {
        double max = -DBL_MAX;               // Highest RSRP found
        double secondMax = -DBL_MAX;         // Second-highest RSRP found
        uint64_t oldCellNodeId = 0;          // Node ID of the currently serving cell (gNB)
        uint16_t newCellId = ueInfo.cellId;  // Cell ID of the best (highest RSRP) candidate (initialized to current cell)
        uint16_t secondBestCellId = ueInfo.cellId; // Cell ID of the second-best candidate (initialized to current)

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

            LogLogicToRepository("RSRP from UE with RNTI " + std::to_string(rnti) +
                                 " in CellID " + std::to_string(ueInfo.cellId) +
                                 " to gNB with CellID " + std::to_string(cellId) +
                                 " is " + std::to_string(rsrp));

            // Check if this RSRP is greater than the current maximum.
            if (rsrp > max)
            {
                // Update second-best to previous best before updating max.
                secondMax = max;
                secondBestCellId = newCellId;
                // Update best RSRP and cell.
                max = rsrp;
                newCellId = cellId;
                LogLogicToRepository("RSRP to gNB with CellID " + std::to_string(cellId) +
                                     " is largest so far");
            }
            // Else if this RSRP is the second highest so far (and not equal to the max).
            else if (rsrp > secondMax && cellId != newCellId)
            {
                secondMax = rsrp;
                secondBestCellId = cellId;
                LogLogicToRepository("RSRP to gNB with CellID " + std::to_string(cellId) +
                                     " is second largest so far");
            }
        }

        // Find the Node ID of the currently serving gNB (old cell).
        for (const auto& gnbInfo : gnbInfos)
        {
            if (ueInfo.cellId == gnbInfo.cellId)
            {
                oldCellNodeId = gnbInfo.nodeId;
                break;
            }
        }

        // Determine if a handover should be issued:
        // Check if the best candidate cell is different from the UE's current cell.
        if (newCellId != ueInfo.cellId)
        {
            bool alreadyTriedBest = false;
            auto it = lastHandoverTarget.find(ueInfo.nodeId);
            if (it != lastHandoverTarget.end() && it->second == newCellId)
            {
                // We have attempted a handover to newCellId before (and UE is still on old cell, meaning it likely failed).
                alreadyTriedBest = true;
            }

            uint16_t targetCellForHandover;
            if (!alreadyTriedBest)
            {
                // **Primary Handover Attempt**: Use the best (highest RSRP) cell.
                targetCellForHandover = newCellId;
            }
            else
            {
                // **Fallback Handover Attempt**: The best was already tried and failed, use second-best cell.
                targetCellForHandover = secondBestCellId;
                LogLogicToRepository("Best candidate (CellID " + std::to_string(newCellId) +
                                     ") was already tried and failed. Using second-best (CellID " +
                                     std::to_string(secondBestCellId) + ") for handover.");
            }

            // Only issue the command if the target is different from current and a valid cell.
            if (targetCellForHandover != ueInfo.cellId)
            {
                Ptr<OranCommandNr2NrHandover> handoverCommand = CreateObject<OranCommandNr2NrHandover>();
                handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));    // source (current) gNB
                handoverCommand->SetAttribute("TargetRnti", UintegerValue(ueInfo.rnti));          // UE's RNTI on source
                handoverCommand->SetAttribute("TargetCellId", UintegerValue(targetCellForHandover)); // target cell ID

                // Log and record the handover command.
                data->LogCommandLm(m_name, handoverCommand);
                commands.push_back(handoverCommand);
                lastHandoverTarget[ueInfo.nodeId] = targetCellForHandover;  // record this attempt

                LogLogicToRepository(std::string("Issuing handover command: UE ") +
                                     std::to_string(ueInfo.nodeId) + " from CellID " +
                                     std::to_string(ueInfo.cellId) + " to CellID " +
                                     std::to_string(targetCellForHandover) +
                                     (alreadyTriedBest ? " (second-best candidate)." : " (best candidate)."));
            }
        }
        // (Optional) Else: newCellId == ueInfo.cellId, no better cell found or already on best cell, no handover.
    }

    return commands;
}


} // namespace ns3
