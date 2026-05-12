#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

// Explicit include for the NR REM helper you provided
#include "ns3/nr-radio-environment-map-helper.h"

#include "ns3/nr-gnb-net-device.h" 
#include "ns3/nr-ue-net-device.h"   

#include "ns3/packet-sink-helper.h"
#include "ns3/traffic-generator-helper.h"
#include "ns3/traffic-generator-3gpp-generic-video.h"
#include "ns3/traffic-generator-ngmn-voip.h"
#include "ns3/xr-traffic-mixer-helper.h"

// NS-3 headers
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/system-path.h"

// STL
#include <cmath>
#include <cstdio>  
#include <filesystem>
#include <fstream>
#include <list>
#include <sstream>
#include <vector>
#include <unordered_map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNr2NrRsrpUavUeHandoverCellLoadTnNtn");

/**
 * Usage example of the ORAN NR models for TN/NTN load-aware handover.
 *
 * Minimum required versions for reproducibility:
 *   - ns-3 version: 3.39 or later
 *   - 5G-LENA version: 2.6 or later
 *
 * The scenario consists of X NR UES1 UEs (moving randomly) and Y NR ground UEs (static) inside a large 2D area
 * and served by Z fixed gNB macro cells. Each UES1 receives downlink UDP traffic
 * from a remote host through an NR EPC (NrPointToPointEpcHelper). 
 *
 * The NR radio access uses a 3GPP UMa propagation scenario with optional fading
 * enabled, and an Ideal Beamforming helper (Quasi-Omni direct path beamforming).

 * A concrete NR scheduler is selected at runtime (OFDMA/TDMA with RR/PF/MR/QoS),
 * and the UEs initially attach to the gNB offering the maximum RSRP. X2 interfaces
 * are enabled to support inter-gNB handovers.
 *
 * In ORAN mode, UES1 UEs report periodic measurements (location, serving cell info,
 * and application loss metrics) to a Near-RT RIC using E2 node terminators.
 * The Near-RT RIC runs an RSRP-based Logic Module (OranLmNr2NrRsrpHandover) that
 * can decide and trigger NR-to-NR handovers. gNB-side cell load is also reported
 * via NR scheduling callbacks to support conflict mitigation.
 *
 * FlowMonitor is used to compute per-UES1 QoS metrics (delay, jitter, throughput,
 * and packet delivery ratio) periodically and write them to trace files over time.
 * Additionally, node mobility positions and successful handover events are logged.
 *
 * Ground UEs are added with constant position (No HO> we can change if want) and connect
 * with RIC too so load-aware handover decisions can be made based on the total number of 
 * UEs (UES1 + ground) connected to each gNB. 
 * 
 * Cell load capacity is set so no intial attachement or handover will be triggered for a 
 * gNB that has already reached the maximum number of UEs. 
 * 
 * NR can split the spectrum into Bandwidth Parts (BWPs) and configure each one differently.
 * FDM (Frequency Division Multiplexing) Split the total spectrum into different frequency 
 * pieces Each piece can serve a different purpose
 * keep everything in one BWP because video can dominate resources voice packets may get delayed
 * uplink-heavy traffic may suffer if DL traffic takes most capacity
 * scheduler becomes less efficient
 * ////////////////////////////
 * This scenario combines:
 *   (1) application-level traffic models (video / voice-like generators), and
 *   (2) radio-level traffic steering (different QoS flows mapped to different BWPs).
 * 
 * UES1 downlink (remoteHost -> UES1):
 *   Uses TrafficGenerator3gppGenericVideo, so packets are generated according
 *   to a video-like traffic model instead of a generic OnOff source.
 *   This is still simulated UDP traffic, but its timing/rate behavior is meant
 *   to resemble video service traffic more closely.
 *
 * UES1 uplink (UES1 -> remoteHost):
 *   Also uses TrafficGenerator3gppGenericVideo, but with a lower data rate/FPS
 *   than downlink. This represents lighter video-like uplink traffic from the UES1.
 * 
 * ofdm = true and schedKind="RR" are set by default to use OFDMA with Round Robin scheduling,
 * for fh control in 7.2x split : https://cttc-lena.gitlab.io/nr/manual/nr-module.html#fronthaul-control
 *
 *   RequiredFhDlThroughput reports the required DL fronthaul throughput per BWP.
 *   UsedAirRbs reports how many DL air-interface RBs were actually used per BWP.
 *   These traces help compare fronthaul demand versus actual radio resource use.
 * 
 * In this scenario, when 5G-LENA Fronthaul Control is enabled, the fronthaul model 
 * assumes functional split 7.2x.
 * after this PDR is much less
 *
 * To see all configurable options, run:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-tn-ntn-load-aware-handover-example --PrintHelp"
 * \endcode
 *
 * A basic run command is:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-tn-ntn-load-aware-handover-example"
 * \endcode
 */

const static float TN_GNB_HEIGHT = 25;


// Variables
uint32_t numGroundUesS1 = 18; // UES1 UEs (moving, with handover and QoS monitoring)
uint32_t numGroundUesS2 = 18;   // ground UEs in addition to UES1 for tn

uint32_t numTnGnbs = 3;
uint32_t numNtnGnbs = 5; // e.g., uav gnbs for ntn area

// Metrics collection interval
Time management_interval = Seconds(2);

// UES1 ips vector
std::vector<Ipv4Address> user_ip;

// Vectors with the most recent metrics for each UES1
// DL (existing)
std::vector<double> user_delay_dl;
std::vector<double> user_jitter_dl;
std::vector<double> user_throughput_dl;
std::vector<double> user_pdr_dl;

// UL (NEW)
std::vector<double> user_delay_ul;
std::vector<double> user_jitter_ul;
std::vector<double> user_throughput_ul;
std::vector<double> user_pdr_ul;

// // static std::string s_trafficTraceFile;
static std::string s_positionTraceFile;
static std::string s_handoverTraceFile;
static std::string s_flowStatTraceFile;
static std::string ns3_dir;
//fh control trace files
static std::ofstream g_fhTraceFile;
static std::ofstream g_airTraceFile;

// --- ns-3 NS_LOG output redirection (LogComponentEnable -> file via std::clog) ---
static std::ofstream g_nsLogFile;
static std::streambuf* g_oldClogBuf = nullptr;

static std::unordered_map<uint32_t, uint64_t> g_ipToImsi; // IPv4.Get() -> IMSI
static std::unordered_map<uint64_t, std::string> g_imsiRole; // IMSI -> "UES1"/"GND"

static inline uint64_t
LookupImsiFromIp(const Ipv4Address& ip)
{
    auto it = g_ipToImsi.find(ip.Get());
    return (it == g_ipToImsi.end()) ? 0 : it->second;
}

static inline const char*
LookupRoleFromImsi(uint64_t imsi)
{
    auto it = g_imsiRole.find(imsi);
    return (it == g_imsiRole.end()) ? "UNK" : it->second.c_str();
}

// static std::ofstream g_uncondFile;
// static std::streambuf* g_oldCoutBuf = nullptr;

// Tracing rsrp, rsrq, and sinr rsrq is set to zero for now
void
TraceRsrpRsrqSinr(Ptr<OutputStreamWrapper> stream,
                  uint16_t rnti,
                  uint16_t cellId,
                  double rsrp,
                  double rsrq, //zero for nr check nr-ue-phy
                  bool servingCell,
                  uint8_t componentCarrierId)

{
    *stream->GetStream() << Simulator::Now().GetSeconds() << " " << rnti << " " << cellId << " "
                         << rsrp << " " << rsrq << " " << servingCell << " "
                         << static_cast<uint32_t>(componentCarrierId) << std::endl;
}

void
ReportFhTrace(const SfnSf& sfn, uint16_t physCellId, uint16_t bwpId, uint64_t reqFh)
{
    if (!g_fhTraceFile.is_open())
    {
        g_fhTraceFile.open(ns3_dir + "fh-trace.txt", std::ios::out | std::ios::trunc);
        g_fhTraceFile << "Time,CellId,BwpId,RequiredFhDlThroughput\n";
    }

    g_fhTraceFile << Simulator::Now().GetSeconds() << ","
                  << physCellId << ","
                  << bwpId << ","
                  << reqFh << "\n";
}

void
ReportAiTrace(const SfnSf& sfn, uint16_t physCellId, uint16_t bwpId, uint32_t airRbs)
{
    if (!g_airTraceFile.is_open())
    {
        g_airTraceFile.open(ns3_dir + "air-rbs-trace.txt", std::ios::out | std::ios::trunc);
        g_airTraceFile << "Time,CellId,BwpId,UsedAirRbs\n";
    }

    g_airTraceFile << Simulator::Now().GetSeconds() << ","
                   << physCellId << ","
                   << bwpId << ","
                   << airRbs << "\n";
}

// Helper function that returns the UES1 id associated with a specific IP
int
get_user_id_from_ipv4(Ipv4Address ip)
{
    for (uint32_t i = 0; i < numGroundUesS1; i++)
    {
        if (user_ip[i] == ip)
        {
            return i;
        }
    }
    return -1;
}

// Function that calculates QoS metrics periodically using FlowMonitor
void
ThroughputMonitor(FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
    std::ofstream flowLog(s_flowStatTraceFile, std::ios_base::app);

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

        // Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);
        // if (!ue_network_mask.IsMatch(ue_network, fiveTuple.destinationAddress))
        //     continue;

        // Ipv4Address ueIp = fiveTuple.destinationAddress; // for your DL traffic
        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);

        bool isDl = ue_network_mask.IsMatch(ue_network, fiveTuple.destinationAddress); // remoteHost -> UE
        bool isUl = ue_network_mask.IsMatch(ue_network, fiveTuple.sourceAddress);      // UE -> remoteHost
        if (!(isDl || isUl)) continue;

        // UE IP = the UE side of the flow
        Ipv4Address ueIp = isDl ? fiveTuple.destinationAddress : fiveTuple.sourceAddress;
        uint64_t imsi = LookupImsiFromIp(ueIp);
        const char* role = LookupRoleFromImsi(imsi);

        int rx_packets = stats.second.rxPackets;
        int tx_packets = stats.second.txPackets;

        double PDR = 100.0 * rx_packets / (tx_packets > 0 ? tx_packets : 1);
        //double lost = tx_packets - rx_packets;
        //double PLR = 100.0 * lost / (tx_packets > 0 ? tx_packets : 1);
        double Delay = (rx_packets > 0) ? stats.second.delaySum.GetSeconds() / rx_packets : 0.0;
        double Jitter = (rx_packets > 0) ? stats.second.jitterSum.GetSeconds() / rx_packets : 0.0;

        double Throughput = (stats.second.timeLastRxPacket > stats.second.timeFirstTxPacket)
            ? (stats.second.rxBytes * 8.0 /
               (stats.second.timeLastRxPacket.GetSeconds() -
                stats.second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024)
            : 0.0;

        //double duration = stats.second.timeLastRxPacket.GetSeconds() -
        //                  stats.second.timeFirstTxPacket.GetSeconds();

        flowLog 
                << Simulator::Now().GetSeconds() << ","
                // << stats.first << ","
                // << fiveTuple.sourceAddress << ","
                // << fiveTuple.destinationAddress << ","
                << role << ","
                << imsi << ","
                // << tx_packets << ","
                // << rx_packets << ","
                // << lost << ","
                // << PDR << ","
                // << PLR << ","
                // << Delay << ","
                // << Jitter << ","
                // << duration << ","
                // << stats.second.timeLastRxPacket.GetSeconds() << ","
                // << Throughput
                << "\n";

        int receiver_id = get_user_id_from_ipv4(ueIp);
        if (receiver_id != -1)
        {
            if (isDl)
            {
                user_delay_dl[receiver_id] = Delay;
                user_jitter_dl[receiver_id] = Jitter;
                user_throughput_dl[receiver_id] = Throughput;
                user_pdr_dl[receiver_id] = PDR;
            }
            else if (isUl)
            {
                user_delay_ul[receiver_id] = Delay;
                user_jitter_ul[receiver_id] = Jitter;
                user_throughput_ul[receiver_id] = Throughput;
                user_pdr_ul[receiver_id] = PDR;
            }
        }
    }

    std::ofstream qos_vs_time;
    qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::app);
    double t = Simulator::Now().GetSeconds();
    for (uint32_t ue = 0; ue < numGroundUesS1; ++ue)
    {
        // DL line
        qos_vs_time << t << "," << ue << ",DL,"
                    << user_delay_dl[ue] << "," << user_jitter_dl[ue] << ","
                    << user_throughput_dl[ue] << "," << user_pdr_dl[ue] << "\n";

        // UL line
        qos_vs_time << t << "," << ue << ",UL,"
                    << user_delay_ul[ue] << "," << user_jitter_ul[ue] << ","
                    << user_throughput_ul[ue] << "," << user_pdr_ul[ue] << "\n";
    }

    //flowMon->ResetAllStats();

    Simulator::Schedule(management_interval, ThroughputMonitor, fmhelper, flowMon);
}

//Trace each node's location
void
TracePositions(NodeContainer nodes)
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

void
NotifyHandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::app);
    hoOutFile << Simulator::Now().GetSeconds() << " " << imsi << " " << cellid << " " << rnti
              << std::endl;
}

void install_mobility(NodeContainer staticNodes,
                      NodeContainer tnGnbNodes,
                      NodeContainer ntnGnbNodes,
                      NodeContainer groundUeNodesS1,
                      NodeContainer groundUeNodesS2)
{
    // --------------------------------------------------
    // 0) Remote host / static node
    // --------------------------------------------------
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    allocator->Add(Vector(0, 0, 0));

    MobilityHelper staticNodesHelper;
    staticNodesHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    staticNodesHelper.SetPositionAllocator(allocator);
    staticNodesHelper.Install(staticNodes);

    // ==================================================
    // 1) TN gNBs: fixed terrestrial gNBs in TN area
    //    TN area: x = [0,1000], y = [-1000,1000]
    // ==================================================
    Ptr<ListPositionAllocator> tnGnbPosition = CreateObject<ListPositionAllocator>();

    const double TN_X_MIN = 0.0;
    const double TN_X_MAX = 2000.0;
    const double TN_Y_MIN = -1000.0;
    const double TN_Y_MAX = 1000.0;
    const double TN_Z     = TN_GNB_HEIGHT;

    const double minDist = 200.0;
    const uint32_t maxAttempts = 20000;

    Ptr<UniformRandomVariable> tnUx = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> tnUy = CreateObject<UniformRandomVariable>();
    tnUx->SetAttribute("Min", DoubleValue(TN_X_MIN));
    tnUx->SetAttribute("Max", DoubleValue(TN_X_MAX));
    tnUy->SetAttribute("Min", DoubleValue(TN_Y_MIN));
    tnUy->SetAttribute("Max", DoubleValue(TN_Y_MAX));

    std::vector<Vector> tnPlaced;
    tnPlaced.reserve(tnGnbNodes.GetN());

    auto TnFarEnough = [&](const Vector& cand) -> bool
    {
        for (const auto& p : tnPlaced)
        {
            const double dx = cand.x - p.x;
            const double dy = cand.y - p.y;
            const double d  = std::sqrt(dx * dx + dy * dy);
            if (d < minDist)
            {
                return false;
            }
        }
        return true;
    };

    for (uint32_t i = 0; i < tnGnbNodes.GetN(); ++i)
    {
        bool ok = false;
        for (uint32_t a = 0; a < maxAttempts; ++a)
        {
            Vector cand(tnUx->GetValue(), tnUy->GetValue(), TN_Z);
            if (TnFarEnough(cand))
            {
                tnGnbPosition->Add(cand);
                tnPlaced.push_back(cand);
                ok = true;
                break;
            }
        }

        NS_ABORT_MSG_IF(!ok,
                        "Could not place TN gNB " << i
                        << " in TN area with minDist=" << minDist);
    }

    MobilityHelper tnGnbHelper;
    tnGnbHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    tnGnbHelper.SetPositionAllocator(tnGnbPosition);
    tnGnbHelper.Install(tnGnbNodes);

    // ==================================================
    // 2) NTN/UAV gNBs: moving aerial gNBs in separate area
    //    NTN area: x = [2000,6000], y = [-1000,1000]
    //    initial altitude z = [100,200]
    // ==================================================
    Ptr<RandomBoxPositionAllocator> ntnGnbPosition = CreateObject<RandomBoxPositionAllocator>();
    ntnGnbPosition->SetAttribute("X",
        StringValue("ns3::UniformRandomVariable[Min=2100.0|Max=5900.0]"));
    ntnGnbPosition->SetAttribute("Y",
        StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
    ntnGnbPosition->SetAttribute("Z",
        StringValue("ns3::UniformRandomVariable[Min=100.0|Max=200.0]")); // UAV altitude between 100 and 200 meters

    MobilityHelper ntnGnbHelper;
    ntnGnbHelper.SetPositionAllocator(ntnGnbPosition);
    ntnGnbHelper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                  "Bounds",
                                  StringValue("2000|6000|-1000|1000"),
                                  "Speed",
                                  StringValue("ns3::UniformRandomVariable[Min=8.0|Max=16.0]"),
                                  "Pause",
                                  StringValue("ns3::UniformRandomVariable[Min=0.5|Max=2.0]"));
    ntnGnbHelper.Install(ntnGnbNodes);

    // ==================================================
    // 3) UE Set 1: moving across whole TN+NTN area
    //    whole area: x = [0,6000], y = [-1000,1000]
    // ==================================================
    Ptr<RandomBoxPositionAllocator> boxS1 = CreateObject<RandomBoxPositionAllocator>();
    boxS1->SetAttribute("X",
        StringValue("ns3::UniformRandomVariable[Min=100.0|Max=5900.0]"));
    boxS1->SetAttribute("Y",
        StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
    boxS1->SetAttribute("Z",
        StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

    MobilityHelper ueS1Helper;
    ueS1Helper.SetPositionAllocator(boxS1);
    ueS1Helper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                "Bounds",
                                StringValue("0|6000|-1000|1000"),
                                "Speed",
                                StringValue("ns3::UniformRandomVariable[Min=2.0|Max=10.0]"),
                                "Pause",
                                StringValue("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
    ueS1Helper.Install(groundUeNodesS1);

    // ==================================================
    // 4) UE Set 2: static across whole TN+NTN area
    // ==================================================
    Ptr<RandomBoxPositionAllocator> boxS2 = CreateObject<RandomBoxPositionAllocator>();
    boxS2->SetAttribute("X",
        StringValue("ns3::UniformRandomVariable[Min=100.0|Max=5900.0]"));
    boxS2->SetAttribute("Y",
        StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
    boxS2->SetAttribute("Z",
        StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

    // MobilityHelper ueS2Helper;
    // ueS2Helper.SetPositionAllocator(boxS2);
    // ueS2Helper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // ueS2Helper.Install(groundUeNodesS2);
    MobilityHelper ueS2Helper;
    ueS2Helper.SetPositionAllocator(boxS2);
    ueS2Helper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                "Bounds",
                                StringValue("0|6000|-1000|1000"),
                                "Speed",
                                StringValue("ns3::UniformRandomVariable[Min=2.0|Max=10.0]"),
                                "Pause",
                                StringValue("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
    ueS2Helper.Install(groundUeNodesS2);
}

bool
IsTopLevelSourceDir(std::string path)
{
    bool haveVersion = false;
    bool haveLicense = false;

    std::list<std::string> files = SystemPath::ReadFiles(path);
    for (std::list<std::string>::const_iterator i = files.begin(); i != files.end(); ++i)
    {
        if (*i == "VERSION")
        {
            haveVersion = true;
        }
        else if (*i == "LICENSE")
        {
            haveLicense = true;
        }
    }
    return haveVersion && haveLicense;
}

std::string
GetTopLevelSourceDir()
{
    std::string self = SystemPath::FindSelfDirectory();
    std::list<std::string> elements = SystemPath::Split(self);
    while (!elements.empty())
    {
        std::string path = SystemPath::Join(elements.begin(), elements.end());
        if (IsTopLevelSourceDir(path))
        {
            return path + "/";
        }
        elements.pop_back();
    }
    NS_FATAL_ERROR("Could not find source directory from self=" << self);
    return "";
}

static void
WriteFlowReportToFile(Ptr<FlowMonitor> monitor,
                      FlowMonitorHelper* helper,
                      const std::string& filename)
{
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(helper->GetClassifier());

    std::ofstream out(filename, std::ios::out | std::ios::trunc);
    auto stats = monitor->GetFlowStats();

    for (const auto& kv : stats)
    {
        uint32_t flowId = kv.first;
        const auto& st  = kv.second;
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

        std::string proto = (t.protocol == 6) ? "TCP" : (t.protocol == 17 ? "UDP" : "OTHER");

        out << "Flow " << flowId << " (" << t.sourceAddress << ":" << t.sourcePort
            << " -> " << t.destinationAddress << ":" << t.destinationPort
            << ") proto " << proto << "\n";

        out << "  Tx Packets: " << st.txPackets << "\n";
        out << "  Tx Bytes:   " << st.txBytes << "\n";

        // Offered rate (Tx)
        double txDuration = (st.timeLastTxPacket > st.timeFirstTxPacket)
            ? (st.timeLastTxPacket.GetSeconds() - st.timeFirstTxPacket.GetSeconds())
            : 0.0;
        double txOfferedMbps = (txDuration > 0)
            ? (st.txBytes * 8.0 / txDuration / 1e6)
            : 0.0;

        out << "  TxOffered:  " << txOfferedMbps << " Mbps\n";
        out << "  Rx Bytes:   " << st.rxBytes << "\n";

        // Throughput (Rx)
        double rxDuration = (st.timeLastRxPacket > st.timeFirstRxPacket)
            ? (st.timeLastRxPacket.GetSeconds() - st.timeFirstRxPacket.GetSeconds())
            : 0.0;
        double thrMbps = (rxDuration > 0)
            ? (st.rxBytes * 8.0 / rxDuration / 1e6)
            : 0.0;

        out << "  Throughput: " << thrMbps << " Mbps\n";

        double meanDelayMs  = (st.rxPackets > 0) ? (1000.0 * st.delaySum.GetSeconds()  / st.rxPackets) : 0.0;
        double meanJitterMs = (st.rxPackets > 0) ? (1000.0 * st.jitterSum.GetSeconds() / st.rxPackets) : 0.0;

        out << "  Mean delay:  " << meanDelayMs  << " ms\n";
        out << "  Mean jitter:  " << meanJitterMs << " ms\n";
        out << "  Rx Packets: " << st.rxPackets << "\n";
    }

    out.close();
}

int
main(int argc, char* argv[])
{
    bool verbose = false;
    bool useOran = true;
    bool useOnnx = false;
    bool useTorch = false;
    bool useRsrp = true; // use the RSRP-driven ORAN LM
    double lmQueryInterval = 2; // Mitigate the ping pong handovers
    double maxWaitTime = 0.010;
    double txDelay = 0.1;
    bool remMode = false; // [0]: REM disabled; [1]: generate REM
    int32_t remRbId = -1; // kept for compatibility (not used by this REM helper)
    std::string handoverAlgorithm = "ns3::NrNoOpHandoverAlgorithm";
    Time simTime = Seconds(25);
    std::string dbFileName = "oran-repository-tn-ntn.db";
    std::string lateCommandPolicy = "DROP";

    //uint32_t maxUesPerCell = 20; // ORAN LM parameter: maximum number of UEs per cell (for load-aware handover decisions)
    uint32_t maxUesPerCellTn  = 5;
    uint32_t maxUesPerCellNtn = 3;
    
    ///£
    double groundAttachDelay = 6.0; // seconds
    ///£
    // Scheduler CLI knobs (safe defaults to a concrete scheduler)
    bool ofdma = true;            // true=OFDMA, false=TDMA
    //In this scenario, BWPs already separate the main service types (voice, UES1 DL, UES1 UL).
    // Therefore, QoS scheduling is less critical than in a mixed-traffic single-BWP setup.
    // QoS scheduler becomes more useful when multiple traffic classes compete within the same BWP.
    std::string schedKind = "PF"; // RR | PF | MR | Qos
    // UES1 UL is configured lighter than UES1 DL: 1 Mbps / 30 fps vs 5 Mbps / 60 fps
    double ueS1DlVideoRateMbps = 2.0;
    uint16_t ueS1DlVideoFps = 30;
    double ueS1UlVideoRateMbps = 0.5;
    uint16_t ueS1UlVideoFps = 15;

    CommandLine cmd;
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.AddValue("use-oran", "Indicates whether ORAN should be used or not", useOran);
    cmd.AddValue("use-onnx-lm", "Use the ONNX LM", useOnnx);
    cmd.AddValue("use-torch-lm", "Use the PyTorch LM", useTorch);
    cmd.AddValue("use-rsrp-lm", "Use the RSRP-based LM", useRsrp);
    cmd.AddValue("sim-time", "The duration for which traffic should flow", simTime);
    cmd.AddValue("lm-query-interval", "The LM query interval", lmQueryInterval);
    cmd.AddValue("tx-delay", "The E2 terminator's transmission delay", txDelay);
    cmd.AddValue("handover-algorithm", "Specify which handover algorithm to use", handoverAlgorithm);
    cmd.AddValue("db-file", "Specify the DB file to create", dbFileName);
    cmd.AddValue("num-uess1", "Number of UES1", numGroundUesS1);
    cmd.AddValue("num-tn-gnbs", "Number of TN gNBs", numTnGnbs);
    cmd.AddValue("num-ntn-gnbs", "Number of NTN gNBs", numNtnGnbs);
    cmd.AddValue("rem-mode", "Generate radio environment map", remMode);
    cmd.AddValue("rem-rb-id", "RB id", remRbId);
    cmd.AddValue("ofdma", "Use OFDMA (1) or TDMA (0)", ofdma);
    cmd.AddValue("sched", "Scheduler kind: RR, PF, MR, Qos", schedKind);
    cmd.AddValue("num-ground-ues", "Number of ground UEs", numGroundUesS2);
    cmd.AddValue("ground-attach-delay", "Delay before attaching ground UEs (s)", groundAttachDelay);
    cmd.AddValue("max-ues-tn",  "Max UEs per TN cell",  maxUesPerCellTn);
    cmd.AddValue("max-ues-ntn", "Max UEs per NTN cell", maxUesPerCellNtn);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
                    "Cannot use ML LM or RSRP LM without enabling O-RAN.");
    NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1, "Cannot use more than one LM simultaneously.");
    NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NrNoOpHandoverAlgorithm" && (useOnnx || useTorch || useRsrp),
                    "Cannot use non-noop handover algorithm with ML/RSRP LM (avoid conflicts).");

    std::ostringstream runTag;
    runTag << "ueS1_" << numGroundUesS1 << "_ueS2_" << numGroundUesS2 << "_tnGnb_" << numTnGnbs << "_ntnGnb_" << numNtnGnbs << "_tnCap_" << maxUesPerCellTn << "_ntnCap_" << maxUesPerCellNtn;

    // Base output folder for this run
    ns3_dir = "results/nr/tn-ntn/" + runTag.str() + "/";

    // Update file paths to be inside ns3_dir
    s_positionTraceFile = ns3_dir + "position-trace.tr";
    s_handoverTraceFile = ns3_dir + "handover-trace.tr";
    s_flowStatTraceFile = ns3_dir + "flow-stats.log";

    // Ensure results/nr/ directory exists
    std::filesystem::create_directories(ns3_dir);
    // ---- Redirect NS_LOG (std::clog) to a file ----
    g_nsLogFile.open(ns3_dir + "ns3-oran-lm.log", std::ios::out | std::ios::trunc);
    g_oldClogBuf = std::clog.rdbuf(g_nsLogFile.rdbuf());

    // // ---- Redirect NS_LOG_UNCOND (std::cout) to a separate file ----
    // g_uncondFile.open(ns3_dir + "init-attach.log", std::ios::out | std::ios::trunc);
    // g_oldCoutBuf = std::cout.rdbuf(g_uncondFile.rdbuf());

    LogComponentEnable("OranLmNr2NrRsrpHandoverWithCellLoad", LOG_LEVEL_INFO);
    LogComponentEnable("NrHelper", LOG_LEVEL_INFO);

    // Increase the buffer size to accomodate the application demand
    bool enablePdcpDiscarding = false;
    uint32_t discardTimerMs = 0;
    uint32_t reorderingTimerMs = 100;
    Config::SetDefault("ns3::NrRlcUm::EnablePdcpDiscarding", BooleanValue(enablePdcpDiscarding));
    Config::SetDefault("ns3::NrRlcUm::DiscardTimerMs", UintegerValue(discardTimerMs));
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(reorderingTimerMs)));
    bool useUdp = true;
    Config::SetDefault("ns3::NrGnbRrc::QosFlowToRlcMapping",
                       EnumValue(useUdp ? NrGnbRrc::RLC_UM_ALWAYS : NrGnbRrc::RLC_AM_ALWAYS));

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));//100 * 1024
    //Config::SetDefault("ns3::NrGnbRrc::MaxUesPerCell", UintegerValue(maxUesPerCell));

    int channelUpdatePeriod = 100;
    int channelConditionUpdatePeriod = 200;
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(channelUpdatePeriod)));

    //Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(43));

    // Create gNB and UES1
    NodeContainer groundUeNodesS1;
    NodeContainer groundUeNodesS2;
    NodeContainer tnGnbNodes;
    NodeContainer ntnGnbNodes;
    NodeContainer allGnbNodes;

    tnGnbNodes.Create(numTnGnbs);
    ntnGnbNodes.Create(numNtnGnbs);
    groundUeNodesS1.Create(numGroundUesS1);
    groundUeNodesS2.Create(numGroundUesS2);
    allGnbNodes.Add(tnGnbNodes);
    allGnbNodes.Add(ntnGnbNodes);

    // Create ChannelHelper API
    // if (isLos)
    // {
    //     propChannelCondition = "LOS";
    // }
    /*
    - Rural Macro (RMa)
        - Urban Macro (UMa)
        - Indoor Hotspot in an open plan office scenario (InH-OfficeOpen)
        - Indoor Hotspot in a mixed plan office scenario (InH-OfficeMixed)
        - Vehicle-to-vehicle in a highway scenario (V2V-Highway)
        - Vehicle-to-vehicle in an urban scenario (V2V-Urban)
        - Urban Micro (UMi)
        - Indoor Hotspot (InH)
        - Indoor Factory (InF)
        - Non-Terrestrial Network in a dense urban scenario (NTN-DenseUrban)
        - Non-Terrestrial Network in an urban scenario (NTN-Urban)
        - Non-Terrestrial Network in a suburban scenario (NTN-Suburban)
        - Non-Terrestrial Network in a rural scenario (NTN-Rural)
    */

    //Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    //NodeDistributionScenarioInterface* scenario{nullptr};
    //std::string propScenario = "UMa"; //Urban Macro
    bool enableShadowing = false;

    //std::string propChannelCondition = "LOS";
    // NS_ABORT_MSG_UNLESS(
    //     propScenario == "UMa",
    //     "Unsupported scenario " << scenario << ". Supported valuess1: UMa, RMa");
    // // Configure the factories for the channel creation
    // channelHelper->ConfigureFactories(propScenario, "Default");
    // channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));
    // if (!isLos)
    // {
    
    // --- TN channel helper ---
    Ptr<NrChannelHelper> tnChannelHelper = CreateObject<NrChannelHelper>();
    tnChannelHelper->ConfigureFactories("UMa", "Default");
    tnChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));

    // --- NTN channel helper ---
    // The 3GPP NTN channel models (NTN-Urban, NTN-Suburban etc.) were designed for satellites
    // at 600km–35,000km altitude where geocentric coordinates matter because the Earth's curvature
    // is significant. At 100–200m altitude your UAVs are essentially elevated terrestrial nodes.
    // 3GPP TR 36.777 which treats UAVs below 300m as "low-altitude platforms" and recommends
    // terrestrial channel models (UMa/UMi) for them, not the NTN satellite models.
    Ptr<NrChannelHelper> ntnChannelHelper = CreateObject<NrChannelHelper>();
    ntnChannelHelper->ConfigureFactories("UMa", "Default");
    ntnChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));

    tnChannelHelper->SetChannelConditionModelAttribute(
        "UpdatePeriod", TimeValue(MilliSeconds(channelConditionUpdatePeriod)));
    ntnChannelHelper->SetChannelConditionModelAttribute(
        "UpdatePeriod", TimeValue(MilliSeconds(channelConditionUpdatePeriod)));
    // }
    //ObjectFactory distanceBasedChannelFactory;
    
    // Create the NR helper
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    
    nrHelper->SetHandoverAlgorithmType(handoverAlgorithm);

    auto setSchedulerIfAvailable = [&](const std::string& name) -> bool {
        TypeId tid;
        if (TypeId::LookupByNameFailSafe(name, &tid))
        {
            NS_LOG_UNCOND(std::string("NR: trying ") + name);
            nrHelper->SetSchedulerTypeId(tid); 
            NS_LOG_UNCOND(std::string("NR: using ") + name);
            return true;
        }
        return false;
    };

    std::vector<std::string> candidates;
    {
        std::stringstream ss;
        ss << "ns3::NrMacScheduler" << (ofdma ? "Ofdma" : "Tdma") << schedKind; // e.g., NrMacSchedulerOfdmaRR
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
    NS_ABORT_MSG_IF(!schedSet,
                    "NR: No concrete MAC scheduler found. "
                    "Build should include one of: NrMacSchedulerOfdmaRR/PF/MR/Qos "
                    "or NrMacSchedulerTdmaRR/PF/MR/Qos.");

    std::string errorModel = "ns3::NrEesmIrT2";

    nrHelper->SetDlErrorModel(errorModel);
    nrHelper->SetUlErrorModel(errorModel);

    // Both DL and UL AMC will have the same model behind.
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    double txTnPower = 43;
    double txNtnPower = 38;
    double ueTxPower = 23;
    bool enableHarqRetx = false;

    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(enableHarqRetx));
    //nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPower));
    //nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPower));

    uint8_t fixedMcs = 0;
    bool useFixedMcs = false; //Realistic behavior, where MCS is adapted based on channel conditions and CQI
    

    nrHelper->SetSchedulerAttribute("FixedMcsDl", BooleanValue(useFixedMcs));
    nrHelper->SetSchedulerAttribute("FixedMcsUl", BooleanValue(useFixedMcs));
    if (useFixedMcs)
    {
        nrHelper->SetSchedulerAttribute("StartingMcsDl", UintegerValue(fixedMcs));
        nrHelper->SetSchedulerAttribute("StartingMcsUl", UintegerValue(fixedMcs));
    }

    double gnbNoiseFigure = 7.0;
    double ueNoiseFigure = 13.0;

    // Noise figure for the gNB
    nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(gnbNoiseFigure));
    // Noise figure for the UE
    nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(ueNoiseFigure));

    nrHelper->EnableFhControl();
    nrHelper->SetFhControlAttribute("FhControlMethod", StringValue("OptimizeRBs"));
    nrHelper->SetFhControlAttribute("FhCapacity", UintegerValue(2000));   // Mbps, example
    nrHelper->SetFhControlAttribute("OverheadDyn", UintegerValue(32));    // or 100 if you want heavier overhead


    // ---- TDD single-carrier setup (ONE band, ONE BWP) ----
    // bool enableFading = true;
    // uint8_t bandMask = NrChannelHelper::INIT_PROPAGATION |
    //                 (enableFading ? NrChannelHelper::INIT_FADING : 0);

    // double centralFrequency = 4e9;
    // double bandBw = 20e6;

    // CcBwpCreator ccBwpCreator;
    // CcBwpCreator::SimpleOperationBandConf conf(centralFrequency, bandBw, 1);
    // OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(conf);

    // std::vector<std::reference_wrapper<OperationBandInfo>> bands;
    // bands.emplace_back(std::ref(band));

    // channelHelper->AssignChannelsToBands(bands, bandMask);

    // // BWP 0
    // BandwidthPartInfoPtrVector Bwps = CcBwpCreator::GetAllBwps(bands);
    ////////////////////////////////
    BandwidthPartInfoPtrVector allBwps;

    // ---- TDD + FDD setup (BWP0=TDD, BWP1=FDD-DL, BWP2=FDD-UL) ----
    bool enableFading = true;
    uint8_t bandMask = NrChannelHelper::INIT_PROPAGATION |
                    (enableFading ? NrChannelHelper::INIT_FADING : 0);

    CcBwpCreator ccBwpCreator;

    // Frequencies
    double centralFrequencyBand1 = 4.0e9;
    double bandwidthBand1        = 20e6;

    double centralFrequencyBand2 = 4.2e9;
    double bandwidthBand2        = 20e6;

    // TDD band: 1 BWP
    CcBwpCreator::SimpleOperationBandConf bandConfTdd(centralFrequencyBand1, bandwidthBand1, 1);

    // FDD band: 2 BWPs (DL + UL)
    CcBwpCreator::SimpleOperationBandConf bandConfFdd(centralFrequencyBand2, bandwidthBand2, 1);
    bandConfFdd.m_numBwp = 2;

    OperationBandInfo bandTdd = ccBwpCreator.CreateOperationBandContiguousCc(bandConfTdd);
    OperationBandInfo bandFdd = ccBwpCreator.CreateOperationBandContiguousCc(bandConfFdd);

    // std::vector<std::reference_wrapper<OperationBandInfo>> bands;
    // bands.emplace_back(std::ref(bandTdd));
    // bands.emplace_back(std::ref(bandFdd));

    //channelHelper->AssignChannelsToBands(bands, bandMask);

    std::vector<std::reference_wrapper<OperationBandInfo>> tnBands;
    tnBands.emplace_back(std::ref(bandTdd));

    std::vector<std::reference_wrapper<OperationBandInfo>> ntnBands;
    ntnBands.emplace_back(std::ref(bandFdd));

    // Assign propagation models
    tnChannelHelper->AssignChannelsToBands(tnBands, bandMask);
    ntnChannelHelper->AssignChannelsToBands(ntnBands, bandMask);

    std::vector<std::reference_wrapper<OperationBandInfo>> bands;
    bands.emplace_back(std::ref(bandTdd));
    bands.emplace_back(std::ref(bandFdd));

    // BWP0=TDD, BWP1=FDD-DL, BWP2=FDD-UL
    allBwps = CcBwpCreator::GetAllBwps(bands);
    ////////////////////////////////


    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
        TypeIdValue(QuasiOmniDirectPathBeamforming::GetTypeId()));
    if (enableFading)
    {
        nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    }
    
    //The network interface installed on the node (e.g., 5G modem)
    NetDeviceContainer tnGnbNrDevs;
    NetDeviceContainer ntnGnbNrDevs;
    NetDeviceContainer allGnbNrDevs;
    NetDeviceContainer groundNrDevsS1; 
    NetDeviceContainer groundNrDevsS2;

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    // Initialize nrHelper
    nrHelper->Initialize(); 

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Create a single remote host
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    // IP configuration
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
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    install_mobility(remoteHostContainer, tnGnbNodes, ntnGnbNodes, groundUeNodesS1, groundUeNodesS2);

    ///////////////////////////////////////////////////////////

    // BWP indices (match the design above)
    uint32_t bwpTdd     = 0; // Ground
    uint32_t bwpFddDl   = 1; // UES1 downlink
    uint32_t bwpFddUl   = 2; // UES1 uplink

    // Route QoS flows to BWPs (same idea as CTTC example)
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpTdd));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(bwpFddDl));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(bwpFddUl));//GBR_GAMING

    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpTdd));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(bwpFddDl));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(bwpFddUl));

    // Install devices with ALL BWPs (IMPORTANT)
    tnGnbNrDevs  = nrHelper->InstallGnbDevice(tnGnbNodes, allBwps);
    ntnGnbNrDevs = nrHelper->InstallGnbDevice(ntnGnbNodes, allBwps);

    // Set TN gNB RRC capacity individually
    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(tnGnbNrDevs.Get(i));
        NS_ABORT_MSG_IF(!gnb, "TN device is not NrGnbNetDevice");
        gnb->GetRrc()->SetAttribute("MaxUesPerCell", UintegerValue(maxUesPerCellTn));
    }

    // Set NTN gNB RRC capacity individually
    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
        NS_ABORT_MSG_IF(!gnb, "NTN device is not NrGnbNetDevice");
        gnb->GetRrc()->SetAttribute("MaxUesPerCell", UintegerValue(maxUesPerCellNtn));
    }

    allGnbNrDevs.Add(tnGnbNrDevs);
    allGnbNrDevs.Add(ntnGnbNrDevs);     
    
    groundNrDevsS1    = nrHelper->InstallUeDevice(groundUeNodesS1, allBwps);
    groundNrDevsS2    = nrHelper->InstallUeDevice(groundUeNodesS2, allBwps);
    const uint32_t numBwps = 3;

    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        for (uint32_t b = 0; b < numBwps; ++b)
        {
            nrHelper->GetGnbPhy(tnGnbNrDevs.Get(i), b)
                ->SetAttribute("TxPower", DoubleValue(txTnPower));
        }
    }

    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        for (uint32_t b = 0; b < numBwps; ++b)
        {
            nrHelper->GetGnbPhy(ntnGnbNrDevs.Get(i), b)
                ->SetAttribute("TxPower", DoubleValue(txNtnPower));
        }
    }
    ////////////////////////////////////////////////////////////

    nrHelper->ConfigureFhControl(allGnbNrDevs);

    for (auto it = allGnbNrDevs.Begin(); it != allGnbNrDevs.End(); ++it)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(*it);
        NS_ABORT_MSG_IF(!gnb, "Device is not NrGnbNetDevice");

        gnb->GetNrFhControl()->TraceConnectWithoutContext(
            "RequiredFhDlThroughput",
            MakeCallback(&ReportFhTrace));

        gnb->GetNrFhControl()->TraceConnectWithoutContext(
            "UsedAirRbs",
            MakeCallback(&ReportAiTrace));
    }


    // allGnbNrDevs = nrHelper->InstallGnbDevice(tnGnbNodes, Bwps);
    // groundNrDevsS1 = nrHelper->InstallUeDevice(groundUeNodesS1, Bwps);
    // groundNrDevsS2= nrHelper->InstallUeDevice(groundUeNodesS2, Bwps); 

    // -------- Role map: IMSI -> UES1/UES2 --------
    g_imsiRole.clear();

    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(groundNrDevsS1.Get(i));
        NS_ABORT_MSG_IF(!ue, "groundNrDevsS1[" << i << "] is not NrUeNetDevice");
        g_imsiRole[ue->GetImsi()] = "UES1";
    }

    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(groundNrDevsS2.Get(i));
        NS_ABORT_MSG_IF(!ue, "groundNrDevsS2[" << i << "] is not NrUeNetDevice");
        g_imsiRole[ue->GetImsi()] = "UES2";
    }

    // TDD pattern (7 DL, 3 UL)

    // for (uint32_t gnbIndex = 0; gnbIndex < allGnbNrDevs.GetN(); ++gnbIndex)
    // {
    //     Ptr<NetDevice> gnbDev = allGnbNrDevs.Get(gnbIndex);
    //     Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbDev, 0); // only BWP0 exists now
    //     gnbPhy->SetAttribute("Pattern", StringValue(tddPattern));
    // }
    ////////////////////////////
    static const std::string tddPattern   = "DL|DL|DL|DL|DL|DL|DL|UL|UL|UL|";
    static const std::string fddDlPattern = "DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|";
    static const std::string fddUlPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";

    for (uint32_t gnbIndex = 0; gnbIndex < allGnbNrDevs.GetN(); ++gnbIndex)
    {
        Ptr<NetDevice> gnbDev = allGnbNrDevs.Get(gnbIndex);

        // BWP0: TDD (ground)
        nrHelper->GetGnbPhy(gnbDev, 0)->SetAttribute("Pattern", StringValue(tddPattern));

        // BWP1: FDD DL (UES1 DL)
        nrHelper->GetGnbPhy(gnbDev, 1)->SetAttribute("Pattern", StringValue(fddDlPattern));

        // BWP2: FDD UL (UES1 UL)
        nrHelper->GetGnbPhy(gnbDev, 2)->SetAttribute("Pattern", StringValue(fddUlPattern));

        // OPTIONAL (CTTC does this): ensure gNB does not transmit on UL-only BWP
        nrHelper->GetGnbPhy(gnbDev, 2)->SetAttribute("TxPower", DoubleValue(0.0));
    }
    ///////////////////////////////


    // Apply final configuration after set patterns + output links
    for (auto it = allGnbNrDevs.Begin(); it != allGnbNrDevs.End(); ++it)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(*it);
        if (gnb) gnb->UpdateConfig();
    }

    for (auto it = groundNrDevsS1.Begin(); it != groundNrDevsS1.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue) ue->UpdateConfig();
    }

    for (auto it = groundNrDevsS2.Begin(); it != groundNrDevsS2.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue) ue->UpdateConfig();
    }

    //////////////////////////////////////////
    // Link the FDD BWPs (same idea as CTTC example)
    for (uint32_t i = 0; i < allGnbNrDevs.GetN(); ++i)
    {
        NrHelper::GetBwpManagerGnb(allGnbNrDevs.Get(i))->SetOutputLink(2, 1); // UL->DL mapping
    }

    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        NrHelper::GetBwpManagerUe(groundNrDevsS1.Get(i))->SetOutputLink(1, 2); // DL->UL mapping
    }
    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        NrHelper::GetBwpManagerUe(groundNrDevsS2.Get(i))->SetOutputLink(1, 2);
    }
    /////////////////////////////////////////

    //nrHelper->ConfigureFhControl(allGnbNrDevs);
    // nrHelper->SetAttribute("InitMaxUesPerCell", UintegerValue(maxUesPerCellTn));
    // nrHelper->SetAttribute("InitMinRsrpDbm",   DoubleValue(-120.0));
    // nrHelper->SetAttribute("InitRetryInterval", TimeValue(Seconds(2.0)));
    // //initial attach helper
    // nrHelper->AttachToMaxRsrpGnb(groundNrDevsS1, allGnbNrDevs);
    // //nrHelper->AttachToMaxRsrpGnb(groundNrDevs, allGnbNrDevs);
    // Time tLateAttach = Seconds(groundAttachDelay);

    // // Simulator::Schedule(tLateAttach, [nrHelper, groundNrDevsS2, allGnbNrDevs]() {
    // //     nrHelper->AttachToMaxRsrpGnb(groundNrDevsS2, allGnbNrDevs); // public container overload
    // // });
    // Simulator::Schedule(tLateAttach, [nrHelper, groundNrDevsS2, allGnbNrDevs, maxUesPerCellNtn]() {
    //     nrHelper->SetAttribute("InitMaxUesPerCell", UintegerValue(maxUesPerCellNtn));
    //     nrHelper->SetAttribute("InitMinRsrpDbm",    DoubleValue(-120.0));
    //     nrHelper->SetAttribute("InitRetryInterval", TimeValue(Seconds(2.0)));
    //     nrHelper->AttachToMaxRsrpGnb(groundNrDevsS2, allGnbNrDevs);
    //     // if we attach S2 to allGnbNrDevs, some UEs might end up attached to 
    //     //NTN gNBs which have lower capacity and cause congestion that prevents S1 UEs from attaching. 
    //     //By attaching S2 only to TN gNBs, we ensure they don't interfere with S1's initial attach and can still handover to NTN later if needed.
    //     // tnGnbNrDevs  is a subset of allGnbNrDevs, so S2 UEs can still handover to NTN gNBs after initial attach if the algorithm decides to do so based on RSRP and load.
    // });
    // Register per-cell caps BEFORE attach
    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(tnGnbNrDevs.Get(i));
        nrHelper->SetCellCapacity(gnb->GetCellId(), maxUesPerCellTn);
    }
    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
        nrHelper->SetCellCapacity(gnb->GetCellId(), maxUesPerCellNtn);
    }

    // Set global fallback to 0 (per-cell map handles everything)
    nrHelper->SetAttribute("InitMaxUesPerCell", UintegerValue(0));
    nrHelper->SetAttribute("InitMinRsrpDbm",    DoubleValue(-120.0));
    nrHelper->SetAttribute("InitRetryInterval", TimeValue(Seconds(2.0)));

    // S1 attach - per-cell caps enforced correctly UE → measure RSRP to every gNB in allGnbNrDevs
    nrHelper->AttachToMaxRsrpGnb(groundNrDevsS1, allGnbNrDevs);

    // S2 late attach - same per-cell caps still registered, no re-registration needed
    Time tLateAttach = Seconds(groundAttachDelay);
    Simulator::Schedule(tLateAttach, [nrHelper, groundNrDevsS2, allGnbNrDevs]() {
        nrHelper->AttachToMaxRsrpGnb(groundNrDevsS2, allGnbNrDevs);
    });

    nrHelper->AddX2Interface(allGnbNodes);

    internet.Install(groundUeNodesS1);
    internet.Install(groundUeNodesS2);
    Ipv4InterfaceContainer ueIpIfaceS1;
    Ipv4InterfaceContainer ueIpIfaceS2;
    ueIpIfaceS1 = epcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNrDevsS1));
    ueIpIfaceS2 = epcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNrDevsS2));

    // -------- IP -> IMSI map to identify UES1 and Ground UEs --------
    g_ipToImsi.clear();

    // UES1 IPs
    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(groundNrDevsS1.Get(i));
        NS_ABORT_MSG_IF(!ue, "groundNrDevsS1[" << i << "] not NrUeNetDevice");
        Ipv4Address ip = ueIpIfaceS1.GetAddress(i);
        g_ipToImsi[ip.Get()] = ue->GetImsi();
    }

    // Ground IPs
    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(groundNrDevsS2.Get(i));
        NS_ABORT_MSG_IF(!ue, "groundNrDevsS2[" << i << "] not NrUeNetDevice");
        Ipv4Address ip = ueIpIfaceS2.GetAddress(i);
        g_ipToImsi[ip.Get()] = ue->GetImsi();
    }

    for (uint32_t u = 0; u < groundUeNodesS1.GetN(); ++u)
    {
        Ptr<Node> ueNode = groundUeNodesS1.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    NS_LOG_UNCOND(std::string("Test"));
    
    for (uint32_t u = 0; u < groundUeNodesS2.GetN(); ++u)
    {
        Ptr<Node> ueNode = groundUeNodesS2.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Install and start applications on UES1 and remote host
    ApplicationContainer remoteApps;   // DL senders on remote host (to UES1)
    ApplicationContainer ueAppsS1;      // DL sinks on UES1

    // (E1) ADD THESE TWO for UES1 uplink:
    ApplicationContainer remoteUlSinks; // UL sinks on remote host (from UES1)
    ApplicationContainer ueUlAppsS1;     // UL senders on UES1

    // remoteHost IP address on the PGW-remoteHost point-to-point link
    Ipv4Address remoteHostIp = internetIpIfaces.GetAddress(1);

    for (uint32_t i = 0; i < groundUeNodesS1.GetN(); ++i)
    {
        // -----------------------
        // DL (remoteHost -> UES1)
        // UES1 UL uses the same Generic Video traffic model as DL, but with lower rate and frame rate
        // (1 Mbps, 30 fps) to represent a lighter uplink stream than the downlink video (5 Mbps, 60 fps).
        // -----------------------
        uint16_t dlPort = 10000 + i;

        // Receiver at UES1 stays the same
        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        ueAppsS1.Add(dlSink.Install(groundUeNodesS1.Get(i)));

        // Sender at remote host: Generic Video traffic generator
        TrafficGeneratorHelper videoHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(ueIpIfaceS1.GetAddress(i), dlPort),
                                        TrafficGenerator3gppGenericVideo::GetTypeId());

        ApplicationContainer dlVideoApps = videoHelper.Install(remoteHost);

        Ptr<TrafficGenerator3gppGenericVideo> dlVideoApp =
            DynamicCast<TrafficGenerator3gppGenericVideo>(dlVideoApps.Get(0));

        NS_ABORT_MSG_IF(!dlVideoApp, "Could not cast to TrafficGenerator3gppGenericVideo");

        dlVideoApp->SetAttribute("DataRate", DoubleValue(ueS1DlVideoRateMbps)); // Mbps
        dlVideoApp->SetAttribute("Fps", UintegerValue(ueS1DlVideoFps));

        remoteApps.Add(dlVideoApps);

        // -----------------------
        // UL (UES1 -> remoteHost)
        // Use Generic Video instead of OnOff
        // -----------------------
        uint16_t ulPort = 12000 + i;

        // UL sink on remote host stays the same
        PacketSinkHelper ulSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        remoteUlSinks.Add(ulSink.Install(remoteHost));

        // UL sender on UES1: Generic Video traffic generator
        TrafficGeneratorHelper ulVideoHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(remoteHostIp, ulPort),
                                            TrafficGenerator3gppGenericVideo::GetTypeId());

        ApplicationContainer ulVideoApps = ulVideoHelper.Install(groundUeNodesS1.Get(i));

        Ptr<TrafficGenerator3gppGenericVideo> ulVideoApp =
            DynamicCast<TrafficGenerator3gppGenericVideo>(ulVideoApps.Get(0));

        NS_ABORT_MSG_IF(!ulVideoApp, "Could not cast UL app to TrafficGenerator3gppGenericVideo");

        // Pick lighter UL traffic than DL
        ulVideoApp->SetAttribute("DataRate", DoubleValue(ueS1UlVideoRateMbps));   // Mbps
        ulVideoApp->SetAttribute("Fps", UintegerValue(ueS1UlVideoFps));

        ueUlAppsS1.Add(ulVideoApps);

        // ----------------------------------------------------
        // (E2) QoS FLOW ACTIVATION (this is what maps to BWPs)
        //   - DL uses GBR_CONV_VIDEO -> BWP1 (FDD-DL)
        //   - UL uses GBR_LIVE_UL_71 -> BWP2 (FDD-UL)
        // ----------------------------------------------------
        Ptr<NetDevice> ueDev = groundNrDevsS1.Get(i);

        // Simulator::Schedule(Seconds(2.5), [nrHelper, ueDev, dlPort, ulPort]() {
        //     // DL QoS rule: match the UE local port (dlPort)
        //     NrQosFlow videoFlow(NrQosFlow::GBR_CONV_VIDEO);
        //     Ptr<NrQosRule> videoRule = Create<NrQosRule>();
        //     NrQosRule::PacketFilter dlpf;
        //     dlpf.localPortStart = dlPort;
        //     dlpf.localPortEnd   = dlPort;
        //     videoRule->Add(dlpf);

        //     // UL QoS rule: match the remote port on remoteHost (ulPort), uplink direction
        //     NrQosFlow gamingFlow(NrQosFlow::GBR_GAMING);
        //     Ptr<NrQosRule> gamingRule = Create<NrQosRule>();
        //     NrQosRule::PacketFilter ulpf;
        //     ulpf.remotePortStart = ulPort;
        //     ulpf.remotePortEnd   = ulPort;
        //     ulpf.direction       = NrQosRule::UPLINK;
        //     gamingRule->Add(ulpf);

        //     nrHelper->ActivateDedicatedQosFlow(ueDev, videoFlow,  videoRule);
        //     nrHelper->ActivateDedicatedQosFlow(ueDev, gamingFlow, gamingRule);
        // });
        NrQosFlow videoFlow(NrQosFlow::GBR_CONV_VIDEO);
        Ptr<NrQosRule> videoRule = Create<NrQosRule>();
        NrQosRule::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd   = dlPort;
        videoRule->Add(dlpf);

        NrQosFlow gamingFlow(NrQosFlow::GBR_LIVE_UL_71);
        Ptr<NrQosRule> gamingRule = Create<NrQosRule>();
        NrQosRule::PacketFilter ulpf;
        ulpf.remotePortStart = ulPort;
        ulpf.remotePortEnd   = ulPort;
        ulpf.direction       = NrQosRule::UPLINK;
        gamingRule->Add(ulpf);

        // Activate QoS flows for UES1 traffic classification and BWP routing
        nrHelper->ActivateDedicatedQosFlow(groundNrDevsS1.Get(i), videoFlow,  videoRule);
        nrHelper->ActivateDedicatedQosFlow(groundNrDevsS1.Get(i), gamingFlow, gamingRule);
    }

    // Start/stop (DL)
    ueAppsS1.Start(Seconds(1.0));           // sinks can start early
    ueAppsS1.Stop(simTime + Seconds(15));

    remoteApps.Start(Seconds(2.0));        // DL starts
    remoteApps.Stop(simTime + Seconds(10));

    // Start/stop (UL)
    remoteUlSinks.Start(Seconds(1.0));     // sinks can start early
    remoteUlSinks.Stop(simTime + Seconds(15));

    ueUlAppsS1.Start(Seconds(3.0));         // UL starts after QoS activation
    ueUlAppsS1.Stop(simTime + Seconds(15));
    ////////

    //ground UEs traffic (same remote host, different ports)

    uint16_t groundBasePort = 20000;              // NEW port base (avoid collision)
    ApplicationContainer ueAppsS2;              // sinks on ground UEs
    ApplicationContainer groundRemoteAppsS2;        // OnOff on remote host

    for (uint16_t i = 0; i < groundUeNodesS2.GetN(); i++)
    {
        uint16_t port = groundBasePort + i;

        ////////////////
        Ptr<NetDevice> gUeDev = groundNrDevsS2.Get(i);
        NrQosFlow voiceFlow(NrQosFlow::GBR_CONV_VOICE);
        Ptr<NrQosRule> voiceRule = Create<NrQosRule>();
        NrQosRule::PacketFilter gdl;
        gdl.localPortStart = port;
        gdl.localPortEnd   = port;
        voiceRule->Add(gdl);

        Simulator::Schedule(tLateAttach,
            [nrHelper, gUeDev, voiceFlow, voiceRule]() {
                nrHelper->ActivateDedicatedQosFlow(gUeDev, voiceFlow, voiceRule);
            });
        //////////////

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
        ueAppsS2.Add(dlSink.Install(groundUeNodesS2.Get(i)));

        // Sender at remote host: NGMN VoIP traffic generator
        TrafficGeneratorHelper voiceHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(ueIpIfaceS2.GetAddress(i), port),
                                        TrafficGeneratorNgmnVoip::GetTypeId());

        ApplicationContainer voiceApps = voiceHelper.Install(remoteHost);
        groundRemoteAppsS2.Add(voiceApps);
    }

    // groundRemoteAppsS2.Start(Seconds(2));
    // ueAppsS2.Start(Seconds(1));
    ueAppsS2.Start(Seconds(1)); // sinks can start early, harmless
    groundRemoteAppsS2.Start(tLateAttach + Seconds(0.5)); // send after attach

    groundRemoteAppsS2.Stop(simTime + Seconds(10));
    ueAppsS2.Stop(simTime + Seconds(15));

    // ORAN BEGIN
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
                            "ONNX LM not found. Were ONNX headers and libraries found during");
        }
        else if (useTorch == true)
        {
            NS_ABORT_MSG_IF(!TypeId::LookupByNameFailSafe("ns3::OranLmNr2NrTorchHandover", &defaultLmTid),
                            "Torch LM not found. Were Torch headers and libraries found during");
        }
        else if (useRsrp == true)
        {
            defaultLmTid = TypeId::LookupByName("ns3::OranLmNr2NrRsrpHandoverWithCellLoad");
        }

        if (useTorch)
        {
            auto path = StringValue(ns3_dir + "saved_trained_classification_pytorch.pt");
            Config::SetDefault("ns3::OranLmNr2NrTorchHandover::TorchModelPath", StringValue(path));
        }
        ObjectFactory defaultLmFactory;
        defaultLmFactory.SetTypeId(defaultLmTid);
        defaultLm = defaultLmFactory.Create<OranLm>();

        // defaultLm->SetAttribute("MaxUesPerCell", UintegerValue(0)); 

        // defaultLm->SetAttribute("TryNextBest", BooleanValue(true)); // try next best otherwise keep current
        defaultLm->SetAttribute("MaxUesPerCell", UintegerValue(0)); // per-cell map takes over

        if (useRsrp)
        {
            Ptr<OranLmNr2NrRsrpHandoverWithCellLoad> rsrpLm =
                DynamicCast<OranLmNr2NrRsrpHandoverWithCellLoad>(defaultLm);
            if (rsrpLm)
            {
                for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
                {
                    Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(tnGnbNrDevs.Get(i));
                    rsrpLm->SetCellCapacity(gnb->GetCellId(), maxUesPerCellTn);
                }
                for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
                {
                    Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
                    rsrpLm->SetCellCapacity(gnb->GetCellId(), maxUesPerCellNtn);
                }
            }
        }

        defaultLm->SetAttribute("TryNextBest", BooleanValue(true)); // try next best otherwise keep current

        defaultLm->SetAttribute("MinAcceptableRsrpDbm", DoubleValue(-120.0)); // default is -120 dbm

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
        nearRtRic->SetAttribute("LmQueryMaxWaitTime",
                                TimeValue(Seconds(maxWaitTime))); // 0 means wait for all LMs to finish
        nearRtRic->SetAttribute("LmQueryLateCommandPolicy", StringValue(lateCommandPolicy));


        Simulator::Schedule(Seconds(2.5), &OranNearRtRic::Start, nearRtRic);

        for (uint32_t idx = 0; idx < groundUeNodesS1.GetN(); idx++)
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
            ueAppsS1.Get(idx)->TraceConnectWithoutContext("Rx",
                                                        MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));
          
            //The UES1’s physical layer (NrUePhy) periodically measures:RSRP (signal strength),
                                                                    //RSRQ (signal quality),
                                                                    //SINR (interference/noise level).
            for (uint32_t netDevIdx = 0; netDevIdx < groundUeNodesS1.Get(idx)->GetNDevices(); netDevIdx++)
            {
                Ptr<NrUeNetDevice> nrUeDevice =
                    groundUeNodesS1.Get(idx)->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();
                if (nrUeDevice)
                {
                    Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(0);
                    uePhy->TraceConnectWithoutContext(
                        "ReportUeMeasurements",
                        MakeCallback(&ns3::OranReporterNrUeRsrpRsrq::ReportRsrpRsrq,
                                    rsrpRsrqReporter));
                }
            }

            nrUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrUeTerminator->SetAttribute("RegistrationIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrUeTerminator->SetAttribute("SendIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=1]"));// increase to mitigate ping pong handovers

            nrUeTerminator->AddReporter(locationReporter);
            nrUeTerminator->AddReporter(nrUeCellInfoReporter);
            nrUeTerminator->AddReporter(rsrpRsrqReporter);
            nrUeTerminator->AddReporter(appLossReporter);
            nrUeTerminator->SetAttribute("TransmissionDelayRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                     std::to_string(txDelay) + "]"));

            nrUeTerminator->Attach(groundUeNodesS1.Get(idx));
            Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
        }

        // Ground UE -> RIC (reporters + terminator)
        for (uint32_t idx = 0; idx < groundUeNodesS2.GetN(); idx++)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrUeCellInfo> nrUeCellInfoReporter = CreateObject<OranReporterNrUeCellInfo>();
            Ptr<OranReporterAppLoss> appLossReporter = CreateObject<OranReporterAppLoss>();
            Ptr<OranReporterNrUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterNrUeRsrpRsrq>();
            Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));

            // AppLoss: use GROUND traffic apps
            appLossReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            groundRemoteAppsS2.Get(idx)->TraceConnectWithoutContext(
                "Tx", MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
            ueAppsS2.Get(idx)->TraceConnectWithoutContext(
                "Rx", MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));

            // RSRP/RSRQ measurements from the ground UE PHY
            for (uint32_t netDevIdx = 0; netDevIdx < groundUeNodesS2.Get(idx)->GetNDevices(); netDevIdx++)
            {
                Ptr<NrUeNetDevice> nrUeDevice =
                    groundUeNodesS2.Get(idx)->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();
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
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));

            nrUeTerminator->AddReporter(locationReporter);
            nrUeTerminator->AddReporter(nrUeCellInfoReporter);
            nrUeTerminator->AddReporter(rsrpRsrqReporter);
            nrUeTerminator->AddReporter(appLossReporter);

            nrUeTerminator->SetAttribute("TransmissionDelayRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(txDelay) + "]"));

            // nrUeTerminator->Attach(groundUeNodesS2.Get(idx));
            // Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
            nrUeTerminator->Attach(groundUeNodesS2.Get(idx));

            // Activate E2 only after the UE is attached (small guard offset)
            Simulator::Schedule(tLateAttach + Seconds(1.0),
                                &OranE2NodeTerminatorNrUe::Activate,
                                nrUeTerminator);
        }

    for (uint32_t idx = 0; idx < allGnbNodes.GetN(); idx++)
    {
        Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
        Ptr<OranReporterNrCellLoad> nrCellLoadReporter = CreateObject<OranReporterNrCellLoad>();
        Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

        locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
        nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));

        auto dev = allGnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();

        dev->GetMac(0)->TraceConnectWithoutContext(
            "DlScheduling",
            MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));

        dev->GetMac(1)->TraceConnectWithoutContext(
            "DlScheduling",
            MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));

        nrGnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        nrGnbTerminator->SetAttribute("RegistrationIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        nrGnbTerminator->SetAttribute("SendIntervalRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=1]"));

        nrGnbTerminator->AddReporter(locationReporter);
        nrGnbTerminator->AddReporter(nrCellLoadReporter);
        nrGnbTerminator->Attach(allGnbNodes.Get(idx));
        nrGnbTerminator->SetAttribute("TransmissionDelayRv",
                                    StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                std::to_string(txDelay) + "]"));
        Simulator::Schedule(Seconds(1.5),
                            &OranE2NodeTerminatorNrGnb::Activate,
                            nrGnbTerminator);
    }

    }
    // ORAN END

    // Erase the trace files if they exist
    std::ofstream posOutFile(s_positionTraceFile, std::ios_base::trunc);
    posOutFile.close();
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::trunc);
    hoOutFile.close();

    // Start tracing node locations
    Simulator::Schedule(Seconds(1), &TracePositions, groundUeNodesS1);

    // Connect to handover trace so we know when a handover is successfully performed
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkGnb));

    Ptr<OutputStreamWrapper> rsrpRsrqSinrTraceStream =
        Create<OutputStreamWrapper>(ns3_dir + "rsrp-trace.tr", std::ios::out);
    for (NetDeviceContainer::Iterator it = groundNrDevsS1.Begin(); it != groundNrDevsS1.End(); ++it)
    {
        Ptr<NetDevice> device = *it;
        Ptr<NrUeNetDevice> nrUeDevice = device->GetObject<NrUeNetDevice>();
        if (nrUeDevice)
        {
            Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(0);
            uePhy->TraceConnectWithoutContext(
                "ReportUeMeasurements",
                MakeBoundCallback(&TraceRsrpRsrqSinr, rsrpRsrqSinrTraceStream));
        }
    }

    // FlowMonitor setup
    user_ip.resize(numGroundUesS1);
    user_delay_dl.assign(numGroundUesS1, 0);
    user_jitter_dl.assign(numGroundUesS1, 0);
    user_throughput_dl.assign(numGroundUesS1, 0);
    user_pdr_dl.assign(numGroundUesS1, 0);

    user_delay_ul.assign(numGroundUesS1, 0);
    user_jitter_ul.assign(numGroundUesS1, 0);
    user_throughput_ul.assign(numGroundUesS1, 0);
    user_pdr_ul.assign(numGroundUesS1, 0);

    // Ptr<FlowMonitor> flowMonitor;
    // FlowMonitorHelper flowHelper;

    // flowHelper.Install(remoteHost);
    // NodeContainer allUes;
    // allUes.Add(groundUeNodesS1);
    // allUes.Add(groundUeNodesS2);
    // flowMonitor = flowHelper.Install(allUes);
    FlowMonitorHelper flowHelper;

    NodeContainer nodesToMonitor;
    nodesToMonitor.Add(remoteHostContainer);   // include remote host
    nodesToMonitor.Add(groundUeNodesS1);
    nodesToMonitor.Add(groundUeNodesS2);

    Ptr<FlowMonitor> flowMonitor = flowHelper.Install(nodesToMonitor);

    std::ofstream qos_vs_time;
    qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::trunc);
    qos_vs_time << "Time,UE,Dir,Delay,Jitter,Throughput,PDR" << std::endl;
    Simulator::Schedule(management_interval, ThroughputMonitor, &flowHelper, flowMonitor);


    // populate user ip map
    for (uint32_t i = 0; i < groundUeNodesS1.GetN(); i++)
    {
        // Ptr<Ipv4> remoteIpv4 = groundUeNodesS1.Get(i)->GetObject<Ipv4>();
        // Ipv4Address remoteIpAddr = remoteIpv4->GetAddress(1, 0).GetLocal();
        user_ip[i] = ueIpIfaceS1.GetAddress(i);
    }

    // ---- NR Radio Environment Map ----
    Ptr<NrRadioEnvironmentMapHelper> remHelper = CreateObject<NrRadioEnvironmentMapHelper>();
    if (remMode)
    {
        remHelper->SetAttribute("SimTag", StringValue("tn-ntn-start"));

        // Whole scenario area:
        // TN area  : x = [0,1000]
        // NTN area : x = [1000,2000]
        // Full map : x = [0,2000], y = [-1000,1000]
        remHelper->SetAttribute("XMin", DoubleValue(0.0));
        remHelper->SetAttribute("XMax", DoubleValue(6000.0));
        remHelper->SetAttribute("XRes", UintegerValue(500));

        remHelper->SetAttribute("YMin", DoubleValue(-1000.0));
        remHelper->SetAttribute("YMax", DoubleValue(1000.0));
        remHelper->SetAttribute("YRes", UintegerValue(500));

        // Evaluate REM at ground-user height
        remHelper->SetAttribute("Z", DoubleValue(1.5));

        // Use one UE from set 1 as the receiver reference
        Ptr<NetDevice> rrdDevice = groundNrDevsS1.Get(0);

        // BWP1 = DL BWP in your setup
        uint8_t bwpId = 1;

        // Create REM at t = 10 s
        Simulator::Schedule(Seconds(10.0),
                            &NrRadioEnvironmentMapHelper::CreateRem,
                            remHelper,
                            allGnbNrDevs, // transmitters: TN + NTN/UAV gNBs
                            rrdDevice,    // receiver
                            bwpId);
    }

    std::ofstream flowOutFile(s_flowStatTraceFile, std::ios_base::trunc);
    flowOutFile << "Time,Role,IMSI\n";
    flowOutFile.close();

    // Tell the simulator how long to run
    Simulator::Stop(simTime + Seconds(15));
    Simulator::Run();
    WriteFlowReportToFile(flowMonitor, &flowHelper, ns3_dir + "final-flow-report.txt");

    if (g_oldClogBuf) { std::clog.rdbuf(g_oldClogBuf); }
    if (g_nsLogFile.is_open()) { g_nsLogFile.close(); }

    // if (g_oldCoutBuf) { std::cout.rdbuf(g_oldCoutBuf); }
    // if (g_uncondFile.is_open()) { g_uncondFile.close(); }
    if (g_fhTraceFile.is_open()) { g_fhTraceFile.close(); }
    if (g_airTraceFile.is_open()) { g_airTraceFile.close(); }   

    Simulator::Destroy();
    return 0;
}
