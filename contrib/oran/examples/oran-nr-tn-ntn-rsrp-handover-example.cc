#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

#include "ns3/nr-radio-environment-map-helper.h"

#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"

#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/system-path.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <list>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <limits>
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNrTnNtnRsrpHandoverExample");

/**
 * Usage example of the ORAN NR models for TN/NTN RSRP handover.
 *
 * The scenario creates a terrestrial service area and a shifted non-terrestrial
 * service area. UEs move in each area, attach to NR gNBs, receive downlink UDP
 * traffic, and periodically report location, serving cell, RSRP, and QoS
 * measurements to the Near-RT RIC.
 *
 * The RIC runs an RSRP-based NR-to-NR Logic Module that can trigger handovers as
 * UEs move between available cells. FlowMonitor and trace callbacks are used to
 * record throughput, delay, jitter, packet delivery, UE positions, and handover
 * events for later analysis.
 *
 * To see all configurable options, run:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-tn-ntn-rsrp-handover-example --PrintHelp"
 * \endcode
 *
 * A basic run command is:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-tn-ntn-rsrp-handover-example"
 * \endcode
 */

// =========================
//  CONFIG CONSTANTS
// =========================
static constexpr double AREA_B_OFFSET_X = 6000.0;   // NTN area is shifted +6000m on X
static constexpr double TN_HALF = 1000.0;           // TN bounds: [-1000..1000]
static constexpr double NTN_HALF = 2000.0;          // NTN bounds: [-2000..2000] around offset
static constexpr double GNB_HEIGHT_TN = 25.0;

// Output dirs
static std::string ns3_dir = "results/nr/tn-ntn/";
static std::string s_trafficTraceFile = ns3_dir + "nr-traffic-trace.tr";
static std::string s_positionTraceFile = ns3_dir + "nr-position-trace.tr";
static std::string s_handoverTraceFile = ns3_dir + "nr-handover-trace.tr";
static std::string s_rsrpUeTraceFile   = ns3_dir + "nr-rsrp-ue.tr";
//static std::string s_rsrpEveTraceFile  = ns3_dir + "nr-rsrp-eve.tr";
//static std::string s_secrecyTraceFile  = ns3_dir + "secrecy-vs-time.txt";

// =========================
//  GLOBALS (QoS like your code)
// =========================
Time management_interval = Seconds(4);

std::vector<Ipv4Address> user_ip;
std::vector<double> user_delay;
std::vector<double> user_jitter;
std::vector<double> user_throughput;
std::vector<double> user_pdr;
std::vector<double> user_plr;

// =========================
//  GLOBALS (Security overlay)
// =========================
// Store each UE's latest serving cell and RSRP (dBm)
static std::vector<uint16_t> g_ueServingCell;
static std::vector<double>   g_ueServingRsrpDbm;

/*
// OLD (BUGGY): keeps MAX RSRP forever for each cell (never decreases)
// For each cellId, store the maximum RSRP (dBm) observed by ANY eavesdropper (worst eavesdropper)
static std::unordered_map<uint16_t, double> g_eveMaxRsrpDbmByCell;
*/

//  store "latest" sample per eaves per cell, then compute worst eaves “now”
struct EveSample
{
    double rsrpDbm = -300.0;
    Time   t = Seconds(0);
};

// g_eveLast[eveIdx][cellId] = {last rsrp, time}
//static std::vector<std::unordered_map<uint16_t, EveSample>> g_eveLast;

// Helper: string for UniformRandomVariable
static std::string
UniformRv(double min, double max)
{
    std::ostringstream oss;
    oss << "ns3::UniformRandomVariable[Min=" << min << "|Max=" << max << "]";
    return oss.str();
}

// =========================
//  YOUR TRAFFIC TRACE (unchanged)
// =========================
void RxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);
    std::ofstream rxOutFile(s_trafficTraceFile, std::ios_base::app);
    rxOutFile << Simulator::Now().GetSeconds() << " " << ueId << " RX " << p->GetSize() << std::endl;
}

void TxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);
    std::ofstream txOutFile(s_trafficTraceFile, std::ios_base::app);
    txOutFile << Simulator::Now().GetSeconds() << " " << ueId << " TX " << p->GetSize() << std::endl;
}

int get_user_id_from_ipv4(Ipv4Address ip)
{
    for (uint32_t i = 0; i < user_ip.size(); i++)
    {
        if (user_ip[i] == ip)
        {
            return (int)i;
        }
    }
    return -1;
}

// =========================
//  FLOWMONITOR (mostly your code, small cleanups)
// =========================
void ThroughputMonitor(FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
    uint32_t lostPacketSum = 0;
    double PDR = 0.0, PLR = 0.0, Delay = 0.0, Jitter = 0.0, Throughput = 0.0;

    auto flowStats = flowMon->GetFlowStats();

    auto ue_network = Ipv4Address("7.0.0.0");
    auto ue_network_mask = Ipv4Mask("255.0.0.0");

    Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());

    for (auto stats : flowStats)
    {
        if (Simulator::Now() - stats.second.timeLastTxPacket > management_interval &&
            Simulator::Now() - stats.second.timeLastRxPacket > management_interval)
        {
            continue;
        }

        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);

        if (!ue_network_mask.IsMatch(ue_network, fiveTuple.destinationAddress))
            continue;

        int rx_packets = stats.second.rxPackets;
        int tx_packets = stats.second.txPackets;

        PDR = (double)(100 * rx_packets) / (tx_packets > 0 ? tx_packets : 1);

        // OLD (can underflow if rx>tx):
        // lostPacketSum = (uint32_t)(tx_packets - rx_packets);

        lostPacketSum = (tx_packets >= rx_packets) ? (uint32_t)(tx_packets - rx_packets) : 0;

        PLR = (double)(lostPacketSum * 100) / (tx_packets > 0 ? tx_packets : 1);

        Delay = (rx_packets > 0) ? (stats.second.delaySum.GetSeconds()) / (rx_packets) : 0.0;
        Jitter = (rx_packets > 0) ? (stats.second.jitterSum.GetSeconds() / rx_packets) : 0.0;

        Throughput = (stats.second.timeLastRxPacket > stats.second.timeFirstTxPacket)
                         ? (stats.second.rxBytes * 8.0 /
                            (stats.second.timeLastRxPacket.GetSeconds() -
                             stats.second.timeFirstTxPacket.GetSeconds()) /
                            1024 / 1024)
                         : 0.0;

        int receiver_id = get_user_id_from_ipv4(fiveTuple.destinationAddress);
        if (receiver_id != -1)
        {
            user_delay[receiver_id] = Delay;
            user_jitter[receiver_id] = Jitter;
            user_throughput[receiver_id] = Throughput;
            user_pdr[receiver_id] = PDR;
            user_plr[receiver_id] = PLR;
        }
    }

    std::ofstream qos_vs_time;
    qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::app);
    for (uint32_t ue = 0; ue < user_delay.size(); ++ue)
    {
        qos_vs_time << Simulator::Now().GetSeconds() << "," << ue << "," << user_delay[ue] << ","
                    << user_jitter[ue] << "," << user_throughput[ue] << "," << user_pdr[ue] << "," << user_plr[ue]
                    << std::endl;
    }

    flowMon->ResetAllStats();
    Simulator::Schedule(management_interval, ThroughputMonitor, fmhelper, flowMon);
}

// =========================
//  POSITION TRACE (same as you had)
// =========================
void TracePositions(NodeContainer nodes)
{
    std::ofstream posOutFile(s_positionTraceFile, std::ios_base::app);

    posOutFile << Simulator::Now().GetSeconds();
    for (uint32_t i = 0; i < nodes.GetN(); i++)
    {
        Vector pos = nodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        posOutFile << " " << pos.x << " " << pos.y;
    }
    posOutFile << std::endl;

    Simulator::Schedule(Seconds(1), &TracePositions, nodes);
}

void NotifyHandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::app);
    hoOutFile << Simulator::Now().GetSeconds() << " " << imsi << " " << cellid << " " << rnti << std::endl;
}

// =========================
//  SECURITY: UE + EVE MEASUREMENT TRACES
// =========================
static void
UeMeasTraceCb(uint32_t ueIdx,
              Ptr<OutputStreamWrapper> stream,
              uint16_t rnti,
              uint16_t cellId,
              double rsrp,
              double rsrq,
              bool servingCell,
              uint8_t componentCarrierId)
{
    *stream->GetStream() << Simulator::Now().GetSeconds() << " UE " << ueIdx
                         << " rnti " << rnti
                         << " cell " << cellId
                         << " rsrp " << rsrp
                         << " rsrq " << rsrq
                         << " serving " << servingCell
                         << " cc " << (uint32_t)componentCarrierId
                         << std::endl;

    if (servingCell && ueIdx < g_ueServingCell.size())
    {
        g_ueServingCell[ueIdx] = cellId;
        g_ueServingRsrpDbm[ueIdx] = rsrp;
    }
}

// static void
// EveMeasTraceCb(uint32_t eveIdx,
//                Ptr<OutputStreamWrapper> stream,
//                uint16_t rnti,
//                uint16_t cellId,
//                double rsrp,
//                double rsrq,
//                bool servingCell,
//                uint8_t componentCarrierId)
// {
//     *stream->GetStream() << Simulator::Now().GetSeconds() << " EVE " << eveIdx
//                          << " rnti " << rnti
//                          << " cell " << cellId
//                          << " rsrp " << rsrp
//                          << " rsrq " << rsrq
//                          << " serving " << servingCell
//                          << " cc " << (uint32_t)componentCarrierId
//                          << std::endl;

//     /*
//     // OLD (BUGGY): max forever
//     auto it = g_eveMaxRsrpDbmByCell.find(cellId);
//     if (it == g_eveMaxRsrpDbmByCell.end())
//     {
//         g_eveMaxRsrpDbmByCell[cellId] = rsrp;
//     }
//     else
//     {
//         if (rsrp > it->second)
//         {
//             it->second = rsrp;
//         }
//     }
//     */

//     // store latest sample for THIS eaves + THIS cell
//     if (eveIdx < g_eveLast.size())
//     {
//         g_eveLast[eveIdx][cellId] = { rsrp, Simulator::Now() };
//     }
// }

// =========================
//  SECURITY: periodic secrecy computation
// =========================
// static double DbToLinear(double x_db) { return std::pow(10.0, x_db / 10.0); }
// static double Log2(double x) { return std::log(x) / std::log(2.0); }

// void SecrecyMonitor(double bandwidthHz, double noiseFigureDb, double secrecyTargetBitsPerHz)
// {
//     const double noiseFloorDbm = -174.0 + 10.0 * std::log10(bandwidthHz) + noiseFigureDb;

//     std::ofstream out(s_secrecyTraceFile, std::ios_base::app);

//     for (uint32_t u = 0; u < g_ueServingCell.size(); ++u)
//     {
//         uint16_t cellId = g_ueServingCell[u];
//         double ueRsrpDbm = g_ueServingRsrpDbm[u];

//         if (cellId == 0 || ueRsrpDbm <= -200.0)
//             continue;

//         double snrUeDb = ueRsrpDbm - noiseFloorDbm;
//         double seUe = Log2(1.0 + DbToLinear(snrUeDb));

//         /*
//         // OLD: read "max forever" eaves
//         double eveRsrpDbm = -1e9;
//         auto it = g_eveMaxRsrpDbmByCell.find(cellId);
//         if (it != g_eveMaxRsrpDbmByCell.end())
//         {
//             eveRsrpDbm = it->second;
//         }
//         */

//         // compute worst eaves "now" using recent samples
//         double eveRsrpDbm = -1e9;
//         Time maxAge = Seconds(1.0); // ignore too-old samples

//         for (uint32_t e = 0; e < g_eveLast.size(); ++e)
//         {
//             auto it2 = g_eveLast[e].find(cellId);
//             if (it2 == g_eveLast[e].end())
//                 continue;

//             if (Simulator::Now() - it2->second.t > maxAge)
//                 continue;

//             eveRsrpDbm = std::max(eveRsrpDbm, it2->second.rsrpDbm);
//         }

//         double seEve = 0.0;
//         if (eveRsrpDbm > -200.0)
//         {
//             double snrEveDb = eveRsrpDbm - noiseFloorDbm;
//             seEve = Log2(1.0 + DbToLinear(snrEveDb));
//         }

//         double secrecy = std::max(seUe - seEve, 0.0);

//         bool covered = (snrUeDb > -5.0);
//         bool outage = (secrecy < secrecyTargetBitsPerHz);

//         out << Simulator::Now().GetSeconds()
//             << ",UE," << u
//             << ",cell," << cellId
//             << ",rsrpUeDbm," << ueRsrpDbm
//             << ",rsrpEveDbm," << eveRsrpDbm
//             << ",seUe," << seUe
//             << ",seEve," << seEve
//             << ",secrecy," << secrecy
//             << ",covered," << covered
//             << ",outage," << outage
//             << std::endl;
//     }

//     out.close();

//     Simulator::Schedule(management_interval,
//                         &SecrecyMonitor,
//                         bandwidthHz,
//                         noiseFigureDb,
//                         secrecyTargetBitsPerHz);
// }

// =========================
//  MOBILITY INSTALL (unchanged from your version)
// =========================
void InstallMobilityTnNtn(NodeContainer staticNodes,
                          NodeContainer tnGnbs,
                          NodeContainer ntnGnbs,
                          NodeContainer ueA,
                          NodeContainer ueB,
                          //NodeContainer eveA,
                          //NodeContainer eveB,
                          bool enableNtn)
{
    {
        Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
        allocator->Add(Vector(0, 0, 0));
        MobilityHelper h;
        h.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        h.SetPositionAllocator(allocator);
        h.Install(staticNodes);
    }

    {
        Ptr<ListPositionAllocator> gnbPosition = CreateObject<ListPositionAllocator>();
        gnbPosition->Add(Vector(0, 0, GNB_HEIGHT_TN));
        gnbPosition->Add(Vector(500, 500, GNB_HEIGHT_TN));
        gnbPosition->Add(Vector(-500, 500, GNB_HEIGHT_TN));
        gnbPosition->Add(Vector(500, -500, GNB_HEIGHT_TN));
        gnbPosition->Add(Vector(-500, -500, GNB_HEIGHT_TN));

        MobilityHelper gnbHelper;
        gnbHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        gnbHelper.SetPositionAllocator(gnbPosition);
        gnbHelper.Install(tnGnbs);
    }

    {
        double xMin = AREA_B_OFFSET_X - NTN_HALF;
        double xMax = AREA_B_OFFSET_X + NTN_HALF;

        Ptr<RandomBoxPositionAllocator> gnbBox = CreateObject<RandomBoxPositionAllocator>();
        gnbBox->SetAttribute("X", StringValue(UniformRv(xMin + 10.0, xMax - 10.0)));
        gnbBox->SetAttribute("Y", StringValue(UniformRv(-NTN_HALF + 10.0, NTN_HALF - 10.0)));
        gnbBox->SetAttribute("Z", StringValue(UniformRv(100.0, 200.0)));

        MobilityHelper gnbHelper;
        gnbHelper.SetPositionAllocator(gnbBox);
        gnbHelper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                   "Bounds", RectangleValue(Rectangle(xMin, xMax, -NTN_HALF, NTN_HALF)),
                                   "Speed",  StringValue(UniformRv(20.0, 30.0)),
                                   "Pause",  StringValue(UniformRv(0.5, 2.0)));
        gnbHelper.Install(ntnGnbs);
    }

    {
        Ptr<RandomBoxPositionAllocator> box = CreateObject<RandomBoxPositionAllocator>();
        box->SetAttribute("X", StringValue(UniformRv(-990.0, 990.0)));
        box->SetAttribute("Y", StringValue(UniformRv(-990.0, 990.0)));
        box->SetAttribute("Z", StringValue(UniformRv(1.5, 2.0)));

        MobilityHelper ueHelper;
        ueHelper.SetPositionAllocator(box);
        ueHelper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                  "Bounds", RectangleValue(Rectangle(-TN_HALF, TN_HALF, -TN_HALF, TN_HALF)),
                                  "Speed",  StringValue(UniformRv(2.0, 10.0)),
                                  "Pause",  StringValue(UniformRv(1.0, 6.0)));
        ueHelper.Install(ueA);
    }

    {
        double xMin = AREA_B_OFFSET_X - NTN_HALF;
        double xMax = AREA_B_OFFSET_X + NTN_HALF;

        Ptr<RandomBoxPositionAllocator> box = CreateObject<RandomBoxPositionAllocator>();
        box->SetAttribute("X", StringValue(UniformRv(xMin + 10.0, xMax - 10.0)));
        box->SetAttribute("Y", StringValue(UniformRv(-NTN_HALF + 10.0, NTN_HALF - 10.0)));
        box->SetAttribute("Z", StringValue(UniformRv(1.5, 2.0)));

        MobilityHelper ueHelper;
        ueHelper.SetPositionAllocator(box);
        ueHelper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                  "Bounds", RectangleValue(Rectangle(xMin, xMax, -NTN_HALF, NTN_HALF)),
                                  "Speed",  StringValue(UniformRv(2.0, 10.0)),
                                  "Pause",  StringValue(UniformRv(1.0, 6.0)));
        ueHelper.Install(ueB);
    }

    // {
    //     Ptr<ListPositionAllocator> pa = CreateObject<ListPositionAllocator>();
    //     pa->Add(Vector(900.0, 0.0, 1.5));
    //     pa->Add(Vector(-900.0, 0.0, 1.5));
    //     pa->Add(Vector(0.0, 900.0, 1.5));

    //     MobilityHelper h;
    //     h.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    //     h.SetPositionAllocator(pa);
    //     h.Install(eveA);
    // }

    // {
    //     Ptr<ListPositionAllocator> pb = CreateObject<ListPositionAllocator>();
    //     pb->Add(Vector(AREA_B_OFFSET_X + 1900.0, 0.0, 1.5));
    //     pb->Add(Vector(AREA_B_OFFSET_X - 1900.0, 0.0, 1.5));
    //     pb->Add(Vector(AREA_B_OFFSET_X, 1900.0, 1.5));

    //     MobilityHelper h;
    //     h.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    //     h.SetPositionAllocator(pb);
    //     h.Install(eveB);
    // }
}

int main(int argc, char* argv[])
{
    bool verbose = false;
    bool useOran = true;
    bool useOnnx = false;
    bool useTorch = false;
    bool useRsrp = true;

    bool enableNtn = true;
    //bool enableEves = true;
    uint32_t numUesA = 50;
    uint32_t numUesB = 50;
    uint32_t numTnGnbs = 5;
    uint32_t numNtnGnbs = 5;

    // uint32_t nEveA = 3;
    // uint32_t nEveB = 3;

    double lmQueryInterval = 4;
    double maxWaitTime = 0.010;
    double txDelay = 0.1;
    bool remMode = false;
    int32_t remRbId = -1;
    std::string handoverAlgorithm = "ns3::NrNoOpHandoverAlgorithm";
    Time simTime = Seconds(25);
    std::string dbFileName = "tn-ntn-oran-repository.db";
    std::string lateCommandPolicy = "DROP";

    bool ofdma = false;
    std::string schedKind = "RR";

    double secrecyTargetBitsPerHz = 0.10;

    CommandLine cmd;
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.AddValue("use-oran", "Enable O-RAN", useOran);
    cmd.AddValue("use-onnx-lm", "Use ONNX LM", useOnnx);
    cmd.AddValue("use-torch-lm", "Use Torch LM", useTorch);
    cmd.AddValue("use-rsrp-lm", "Use RSRP LM", useRsrp);
    cmd.AddValue("enable-ntn", "Enable NTN gNBs (TN-only vs TN+NTN)", enableNtn);
    //cmd.AddValue("enable-eves", "Enable eavesdroppers + secrecy logging", enableEves);

    cmd.AddValue("num-ues-a", "UEs in Area A (TN)", numUesA);
    cmd.AddValue("num-ues-b", "UEs in Area B (NTN offset)", numUesB);

    // cmd.AddValue("n-eve-a", "Eavesdroppers in Area A", nEveA);
    // cmd.AddValue("n-eve-b", "Eavesdroppers in Area B", nEveB);

    cmd.AddValue("sim-time", "Traffic duration", simTime);
    cmd.AddValue("lm-query-interval", "LM query interval", lmQueryInterval);
    cmd.AddValue("tx-delay", "E2 transmission delay", txDelay);
    cmd.AddValue("handover-algorithm", "Handover algorithm", handoverAlgorithm);
    cmd.AddValue("db-file", "DB filename", dbFileName);

    cmd.AddValue("rem-mode", "Generate radio environment map", remMode);
    cmd.AddValue("rem-rb-id", "RB id (unused by your helper)", remRbId);
    cmd.AddValue("ofdma", "Use OFDMA (1) or TDMA (0)", ofdma);
    cmd.AddValue("sched", "Scheduler kind: RR, PF, MR, Qos", schedKind);

    cmd.AddValue("secrecy-target", "Secrecy target (bits/s/Hz)", secrecyTargetBitsPerHz);

    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
                    "Cannot use LM without enabling O-RAN.");
    NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1, "Cannot use more than one LM simultaneously.");
    NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NrNoOpHandoverAlgorithm" && (useOnnx || useTorch || useRsrp),
                    "Avoid conflicts: use NoOp HO when LM controls HO.");

    std::filesystem::create_directories(ns3_dir);

    { std::ofstream f(s_trafficTraceFile, std::ios_base::trunc); }
    { std::ofstream f(s_positionTraceFile, std::ios_base::trunc); }
    { std::ofstream f(s_handoverTraceFile, std::ios_base::trunc); }
    { std::ofstream f(s_rsrpUeTraceFile, std::ios_base::trunc); }
    //{ std::ofstream f(s_rsrpEveTraceFile, std::ios_base::trunc); }
    // {
    //     std::ofstream f(s_secrecyTraceFile, std::ios_base::trunc);
    //     f << "time,type,id,cell,rsrpUeDbm,rsrpEveDbm,seUe,seEve,secrecy,covered,outage\n";
    // }
    {
        std::ofstream f(ns3_dir + "qos-vs-time.txt", std::ios_base::trunc);
        f << "Time,UE,Delay,Jitter,Throughput,PDR,PLR\n";
    }

    // =========================
    // Create nodes
    // =========================
    NodeContainer tnGnbNodes, ntnGnbNodes;
    NodeContainer ueAreaA, ueAreaB;
    NodeContainer ueNodes;
    NodeContainer allGnbsActive;
    NodeContainer remoteHostContainer;

    tnGnbNodes.Create(numTnGnbs);
    ntnGnbNodes.Create(numNtnGnbs);

    ueAreaA.Create(numUesA);
    ueAreaB.Create(numUesB);

    ueNodes.Add(ueAreaA);
    ueNodes.Add(ueAreaB);

    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // NodeContainer eveA, eveB, eveNodes;
    // if (enableEves)
    // {
    //     eveA.Create(nEveA);
    //     eveB.Create(nEveB);
    //     eveNodes.Add(eveA);
    //     eveNodes.Add(eveB);

    //     // init per-eaves storage (must be AFTER eaves exist)
    //     g_eveLast.clear();
    //     g_eveLast.resize(eveNodes.GetN());
    // }

    allGnbsActive.Add(tnGnbNodes);
    if (enableNtn)
    {
        allGnbsActive.Add(ntnGnbNodes);
    }

    // =========================
    // Mobility
    // =========================
    InstallMobilityTnNtn(remoteHostContainer,
                         tnGnbNodes,
                         ntnGnbNodes,
                         ueAreaA,
                         ueAreaB,
                        //  eveA,
                        //  eveB,
                         enableNtn);

    // =========================
    // NR + Channel setup
    // =========================
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    std::string propScenario = "UMa";
    bool enableShadowing = false;

    channelHelper->ConfigureFactories(propScenario, "Default");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(10)));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetHandoverAlgorithmType(handoverAlgorithm);

    auto setSchedulerIfAvailable = [&](const std::string& name) -> bool {
        TypeId tid;
        if (TypeId::LookupByNameFailSafe(name, &tid))
        {
            nrHelper->SetSchedulerTypeId(tid);
            NS_LOG_UNCOND(std::string("NR: using ") + name);
            return true;
        }
        return false;
    };

    std::vector<std::string> candidates;
    {
        std::stringstream ss;
        ss << "ns3::NrMacScheduler" << (ofdma ? "Ofdma" : "Tdma") << schedKind;
        candidates.push_back(ss.str());
    }
    if (ofdma)
    {
        candidates.push_back("ns3::NrMacSchedulerOfdmaRR");
        candidates.push_back("ns3::NrMacSchedulerOfdmaPF");
        candidates.push_back("ns3::NrMacSchedulerOfdmaMR");
        candidates.push_back("ns3::NrMacSchedulerOfdmaQos");
    }
    else
    {
        candidates.push_back("ns3::NrMacSchedulerTdmaRR");
        candidates.push_back("ns3::NrMacSchedulerTdmaPF");
        candidates.push_back("ns3::NrMacSchedulerTdmaMR");
        candidates.push_back("ns3::NrMacSchedulerTdmaQos");
    }

    bool schedSet = false;
    for (const auto& n : candidates)
    {
        if (setSchedulerIfAvailable(n))
        {
            schedSet = true;
            break;
        }
    }
    NS_ABORT_MSG_IF(!schedSet, "No concrete MAC scheduler found.");

    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    double txPower = 38;
    double ueTxPower = 23;
    bool enableHarqRetx = false;
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(enableHarqRetx));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPower));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPower));

    double gnbNoiseFigure = 7.0;
    double ueNoiseFigure = 13.0;
    nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(gnbNoiseFigure));
    nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(ueNoiseFigure));

    double dlCentralFrequency = 4e9;
    double ulCentralFrequency = 3.9e9;
    double bandBw = 20e6;

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf dlConf(dlCentralFrequency, bandBw, 1);
    OperationBandInfo dlBand = ccBwpCreator.CreateOperationBandContiguousCc(dlConf);

    CcBwpCreator::SimpleOperationBandConf ulConf(ulCentralFrequency, bandBw, 1);
    OperationBandInfo ulBand = ccBwpCreator.CreateOperationBandContiguousCc(ulConf);

    std::vector<std::reference_wrapper<OperationBandInfo>> bands;
    bands.emplace_back(std::ref(dlBand));
    bands.emplace_back(std::ref(ulBand));

    bool enableFading = true;
    uint8_t bandMask = NrChannelHelper::INIT_PROPAGATION |
                       (enableFading ? NrChannelHelper::INIT_FADING : 0);
    channelHelper->AssignChannelsToBands(bands, bandMask);

    BandwidthPartInfoPtrVector Bwps = CcBwpCreator::GetAllBwps(bands);

    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
        TypeIdValue(QuasiOmniDirectPathBeamforming::GetTypeId()));
    if (enableFading)
    {
        nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    }

    uint32_t bwpIdForLowLat = 0;
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForLowLat));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("NGBR_LOW_LAT_EMBB", UintegerValue(bwpIdForLowLat));

    nrHelper->Initialize();

    // =========================
    // EPC + Internet (remote host)
    // =========================
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    Ptr<Node> pgw = epcHelper->GetPgwNode();

    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Gb/s")));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(65000));
    p2ph.SetChannelAttribute("Delay", TimeValue(MilliSeconds(0)));

    NetDeviceContainer internetDevices = p2ph.Install(pgw, remoteHost);
    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign(internetDevices);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ptr<Ipv4StaticRouting> remoteHostStaticRouting =
        ipv4RoutingHelper.GetStaticRouting(remoteHost->GetObject<Ipv4>());
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"),
                                               Ipv4Mask("255.0.0.0"),
                                               1);

    // =========================
    // Install NR devices
    // =========================
    NetDeviceContainer gnbNrDevs;
    NetDeviceContainer ueNrDevs;
    //NetDeviceContainer eveNrDevs;

    gnbNrDevs = nrHelper->InstallGnbDevice(allGnbsActive, Bwps);
    ueNrDevs  = nrHelper->InstallUeDevice(ueNodes, Bwps);

    // if (enableEves && eveNodes.GetN() > 0)
    // {
    //     eveNrDevs = nrHelper->InstallUeDevice(eveNodes, Bwps);
    // }

    // FDD patterns
    static const std::string dlPattern = "DL|DL|DL|DL|DL|DL|DL|DL|DL|UL|";
    static const std::string ulPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|DL|";

    for (uint32_t gnbIndex = 0; gnbIndex < gnbNrDevs.GetN(); ++gnbIndex)
    {
        Ptr<NetDevice> gnbDev = gnbNrDevs.Get(gnbIndex);

        Ptr<NrGnbPhy> gnbPhyDl = nrHelper->GetGnbPhy(gnbDev, 0);
        gnbPhyDl->SetAttribute("Pattern", StringValue(dlPattern));

        Ptr<NrGnbPhy> gnbPhyUl = nrHelper->GetGnbPhy(gnbDev, 1);
        gnbPhyUl->SetAttribute("Pattern", StringValue(ulPattern));
    }

    // Update configs (kept as you had)
    for (auto it = gnbNrDevs.Begin(); it != gnbNrDevs.End(); ++it)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(*it);
        if (gnb) gnb->UpdateConfig();
    }
    for (auto it = ueNrDevs.Begin(); it != ueNrDevs.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue) ue->UpdateConfig();
    }
    // for (auto it = eveNrDevs.Begin(); it != eveNrDevs.End(); ++it)
    // {
    //     Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
    //     if (ue) ue->UpdateConfig();
    // }

    // X2 among ACTIVE gNB nodes (unchanged)
    nrHelper->AddX2Interface(allGnbsActive);

    // =========================================================
    // ✅ FIX for your crash:
    // Install IPv4 stack on UEs *BEFORE* AttachToMaxRsrpGnb()
    // because Attach activates QoS flows and asserts ueIpv4/ueIpv6 exists.
    // =========================================================

    // OLD (buggy order): internet.Install(ueNodes) was later
    // internet.Install(ueNodes);

    internet.Install(ueNodes);

    // if (enableEves && eveNodes.GetN() > 0)
    // {
    //     // Eaves also call AttachToMaxRsrpGnb(), so they also need IPv4 installed
    //     internet.Install(eveNodes);
    // }

    // Assign UE IPs NOW (before attach is OK, and safer)
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNrDevs));

    // Ipv4InterfaceContainer eveIpIface;
    // if (enableEves && eveNrDevs.GetN() > 0)
    // {
    //     eveIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(eveNrDevs));
    // }

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // if (enableEves && eveNodes.GetN() > 0)
    // {
    //     for (uint32_t e = 0; e < eveNodes.GetN(); ++e)
    //     {
    //         Ptr<Node> eveNode = eveNodes.Get(e);
    //         Ptr<Ipv4StaticRouting> eveStaticRouting =
    //             ipv4RoutingHelper.GetStaticRouting(eveNode->GetObject<Ipv4>());
    //         eveStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    //     }
    // }

    // OLD (crashes if IPv4 not installed):
    // nrHelper->AttachToMaxRsrpGnb(ueNrDevs, gnbNrDevs);

    nrHelper->AttachToMaxRsrpGnb(ueNrDevs, gnbNrDevs);
    // if (enableEves && eveNrDevs.GetN() > 0)
    // {
    //     nrHelper->AttachToMaxRsrpGnb(eveNrDevs, gnbNrDevs);
    // }

    // =========================
    // Apps (your code, unchanged except it now uses ueIpIface already created)
    // =========================
    uint16_t basePort = 1000;
    ApplicationContainer remoteApps;
    ApplicationContainer ueApps;

    Ptr<RandomVariableStream> onTimeRv = CreateObject<UniformRandomVariable>();
    onTimeRv->SetAttribute("Min", DoubleValue(0.1));
    onTimeRv->SetAttribute("Max", DoubleValue(0.5));
    Ptr<RandomVariableStream> offTimeRv = CreateObject<UniformRandomVariable>();
    offTimeRv->SetAttribute("Min", DoubleValue(0.1));
    offTimeRv->SetAttribute("Max", DoubleValue(0.5));

    for (uint16_t i = 0; i < ueNodes.GetN(); i++)
    {
        uint16_t port = basePort * (i + 1);

        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), port));
        ueApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(i)));
        ueApps.Get(i)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));

        Ptr<OnOffApplication> streamingServer = CreateObject<OnOffApplication>();
        remoteApps.Add(streamingServer);
        streamingServer->SetAttribute("Remote",
            AddressValue(InetSocketAddress(ueIpIface.GetAddress(i), port)));
        streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("3000000bps")));
        streamingServer->SetAttribute("PacketSize", UintegerValue(1500));
        streamingServer->SetAttribute("OnTime", PointerValue(onTimeRv));
        streamingServer->SetAttribute("OffTime", PointerValue(offTimeRv));

        remoteHost->AddApplication(streamingServer);
        streamingServer->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
    }

    remoteApps.Start(Seconds(2));
    remoteApps.Stop(simTime + Seconds(10));
    ueApps.Start(Seconds(1));
    ueApps.Stop(simTime + Seconds(15));

    // =========================
    if (useOran == true)
    {
        if (!dbFileName.empty())
        {
            ::remove(dbFileName.c_str());
        }

        TypeId defaultLmTid = TypeId::LookupByName("ns3::OranLmNoop");
        Ptr<OranLm> defaultLm = nullptr;
        Ptr<OranDataRepository> dataRepository = CreateObject<OranDataRepositorySqlite>();
        Ptr<OranCmm> cmm = CreateObject<OranCmmHandover>();
        Ptr<OranNearRtRic> nearRtRic = CreateObject<OranNearRtRic>();
        Ptr<OranNearRtRicE2Terminator> nearRtRicE2Terminator = CreateObject<OranNearRtRicE2Terminator>();

        if (useOnnx == true)
        {
            NS_ABORT_MSG_IF(!TypeId::LookupByNameFailSafe("ns3::OranLmNr2NrOnnxHandover", &defaultLmTid),
                            "ONNX LM not found.");
        }
        else if (useTorch == true)
        {
            NS_ABORT_MSG_IF(!TypeId::LookupByNameFailSafe("ns3::OranLmNr2NrTorchHandover", &defaultLmTid),
                            "Torch LM not found.");
        }
        else if (useRsrp == true)
        {
            defaultLmTid = TypeId::LookupByName("ns3::OranLmNr2NrRsrpHandover");
        }

        ObjectFactory defaultLmFactory;
        defaultLmFactory.SetTypeId(defaultLmTid);
        defaultLm = defaultLmFactory.Create<OranLm>();

        dataRepository->SetAttribute("DatabaseFile", StringValue(dbFileName));
        defaultLm->SetAttribute("Verbose", BooleanValue(verbose));
        defaultLm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        cmm->SetAttribute("NearRtRic", PointerValue(nearRtRic));

        nearRtRicE2Terminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        nearRtRicE2Terminator->SetAttribute("DataRepository", PointerValue(dataRepository));
        nearRtRicE2Terminator->SetAttribute(
            "TransmissionDelayRv",
            StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(txDelay) + "]"));

        nearRtRic->SetAttribute("DefaultLogicModule", PointerValue(defaultLm));
        nearRtRic->SetAttribute("E2Terminator", PointerValue(nearRtRicE2Terminator));
        nearRtRic->SetAttribute("DataRepository", PointerValue(dataRepository));
        nearRtRic->SetAttribute("LmQueryInterval", TimeValue(Seconds(lmQueryInterval)));
        nearRtRic->SetAttribute("ConflictMitigationModule", PointerValue(cmm));
        nearRtRic->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
        nearRtRic->SetAttribute("E2NodeInactivityIntervalRv",
                                StringValue("ns3::ConstantRandomVariable[Constant=2]"));
        nearRtRic->SetAttribute("LmQueryMaxWaitTime", TimeValue(Seconds(maxWaitTime)));
        nearRtRic->SetAttribute("LmQueryLateCommandPolicy", StringValue(lateCommandPolicy));

        Simulator::Schedule(Seconds(1), &OranNearRtRic::Start, nearRtRic);

        // UE terminators on legitimate UEs only
        for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrUeCellInfo> nrUeCellInfoReporter = CreateObject<OranReporterNrUeCellInfo>();
            Ptr<OranReporterAppLoss> appLossReporter = CreateObject<OranReporterAppLoss>();
            Ptr<OranReporterNrUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterNrUeRsrpRsrq>();
            Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));

            appLossReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            remoteApps.Get(idx)->TraceConnectWithoutContext("Tx",
                MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
            ueApps.Get(idx)->TraceConnectWithoutContext("Rx",
                MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));

            for (uint32_t netDevIdx = 0; netDevIdx < ueNodes.Get(idx)->GetNDevices(); netDevIdx++)
            {
                Ptr<NrUeNetDevice> nrUeDevice =
                    ueNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();
                if (nrUeDevice)
                {
                    Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(0);
                    uePhy->TraceConnectWithoutContext(
                        "ReportUeMeasurements",
                        MakeCallback(&ns3::OranReporterNrUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
                }
            }

            nrUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrUeTerminator->SetAttribute("RegistrationIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrUeTerminator->SetAttribute("SendIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=2]"));

            nrUeTerminator->AddReporter(locationReporter);
            nrUeTerminator->AddReporter(nrUeCellInfoReporter);
            nrUeTerminator->AddReporter(rsrpRsrqReporter);
            nrUeTerminator->AddReporter(appLossReporter);
            nrUeTerminator->SetAttribute("TransmissionDelayRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                     std::to_string(txDelay) + "]"));

            nrUeTerminator->Attach(ueNodes.Get(idx));
            Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
        }

        // gNB terminators on ACTIVE gNBs only
        for (uint32_t idx = 0; idx < allGnbsActive.GetN(); idx++)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrCellLoad> nrCellLoadReporter = CreateObject<OranReporterNrCellLoad>();
            Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));

            auto dev = gnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();
            auto mac = dev->GetMac(0);
            mac->TraceConnectWithoutContext(
                "DlScheduling",
                MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));

            nrGnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrGnbTerminator->SetAttribute("RegistrationIntervalRv",
                                          StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrGnbTerminator->SetAttribute("SendIntervalRv",
                                          StringValue("ns3::ConstantRandomVariable[Constant=1]"));

            nrGnbTerminator->AddReporter(locationReporter);
            nrGnbTerminator->AddReporter(nrCellLoadReporter);
            nrGnbTerminator->Attach(allGnbsActive.Get(idx));
            nrGnbTerminator->SetAttribute("TransmissionDelayRv",
                                          StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                      std::to_string(txDelay) + "]"));

            Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorNrGnb::Activate, nrGnbTerminator);
        }
    }

    // =========================
    // Connect handover trace
    // =========================
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkGnb));

    // =========================
    // SECURITY TRACES + Secrecy monitor scheduling
    // =========================
    g_ueServingCell.assign(ueNodes.GetN(), 0);
    g_ueServingRsrpDbm.assign(ueNodes.GetN(), -300.0);

    Ptr<OutputStreamWrapper> ueMeasStream  = Create<OutputStreamWrapper>(s_rsrpUeTraceFile, std::ios::out);
    //Ptr<OutputStreamWrapper> eveMeasStream = Create<OutputStreamWrapper>(s_rsrpEveTraceFile, std::ios::out);

    for (uint32_t i = 0; i < ueNrDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> d = ueNrDevs.Get(i)->GetObject<NrUeNetDevice>();
        if (!d) continue;
        Ptr<NrUePhy> phy = d->GetPhy(0);
        if (!phy) continue;

        phy->TraceConnectWithoutContext("ReportUeMeasurements",
            MakeBoundCallback(&UeMeasTraceCb, i, ueMeasStream));
    }

    // if (enableEves && eveNrDevs.GetN() > 0)
    // {
    //     for (uint32_t i = 0; i < eveNrDevs.GetN(); ++i)
    //     {
    //         Ptr<NrUeNetDevice> d = eveNrDevs.Get(i)->GetObject<NrUeNetDevice>();
    //         if (!d) continue;
    //         Ptr<NrUePhy> phy = d->GetPhy(0);
    //         if (!phy) continue;

    //         phy->TraceConnectWithoutContext("ReportUeMeasurements",
    //             MakeBoundCallback(&EveMeasTraceCb, i, eveMeasStream));
    //     }

    //     Simulator::Schedule(management_interval,
    //                         &SecrecyMonitor,
    //                         bandBw,
    //                         ueNoiseFigure,
    //                         secrecyTargetBitsPerHz);
    // }

    // =========================
    // FlowMonitor + QoS vectors
    // =========================
    user_ip.resize(ueNodes.GetN());
    user_delay.assign(ueNodes.GetN(), 0);
    user_jitter.assign(ueNodes.GetN(), 0);
    user_throughput.assign(ueNodes.GetN(), 0);
    user_pdr.assign(ueNodes.GetN(), 100);
    user_plr.assign(ueNodes.GetN(), 0);

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;
    flowHelper.Install(remoteHost);
    flowMonitor = flowHelper.Install(ueNodes);

    Simulator::Schedule(management_interval, ThroughputMonitor, &flowHelper, flowMonitor);

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        Ptr<Ipv4> ipv4 = ueNodes.Get(i)->GetObject<Ipv4>();
        Ipv4Address ipAddr = ipv4->GetAddress(1, 0).GetLocal();
        user_ip[i] = ipAddr;
    }

    Simulator::Schedule(Seconds(1), &TracePositions, ueNodes);

    Ptr<NrRadioEnvironmentMapHelper> remHelper = CreateObject<NrRadioEnvironmentMapHelper>();
    if (remMode)
    {
        remHelper->SetAttribute("SimTag", StringValue("tn-ntn"));
        remHelper->SetAttribute("XMin", DoubleValue(-1200.0));
        remHelper->SetAttribute("XMax", DoubleValue(1200.0));
        remHelper->SetAttribute("XRes", UintegerValue(500));
        remHelper->SetAttribute("YMin", DoubleValue(-1200.0));
        remHelper->SetAttribute("YMax", DoubleValue(1200.0));
        remHelper->SetAttribute("YRes", UintegerValue(500));
        remHelper->SetAttribute("Z", DoubleValue(1.0));

        Ptr<NetDevice> rrdDevice = ueNrDevs.Get(0);
        uint8_t bwpId = 0;

        Simulator::Schedule(Seconds(10.0),
                            &NrRadioEnvironmentMapHelper::CreateRem,
                            remHelper,
                            gnbNrDevs,
                            rrdDevice,
                            bwpId);
    }

    Simulator::Stop(simTime + Seconds(15));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
