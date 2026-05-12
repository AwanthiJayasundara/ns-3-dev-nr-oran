#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-radio-environment-map-helper.h"
#include "ns3/oran-lm-nr-2-nr-rsrp-handover-with-tn-ntn.h"

#include "ns3/nr-gnb-net-device.h"
#include "ns3/nr-ue-net-device.h"

#include "ns3/packet-sink-helper.h"
#include "ns3/traffic-generator-helper.h"
#include "ns3/traffic-generator-ngmn-voip.h"
#include "ns3/xr-traffic-mixer-helper.h"

#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/system-path.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <list>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNrTnNtnHybridXr");

/**
 * O-RAN-driven hybrid TN/NTN NR access with QoS monitoring.
 *
 * IMPORTANT IMPLEMENTATION NOTE:
 * --------------------------------
 * In 5G-LENA, BWPs are static and each BWP is its own spectrum channel.
 * Also, attached UEs must have the same BWP configuration as the gNB.
 * Therefore, this example keeps a COMMON BWP configuration across all UEs
 * and all gNBs, but separates TN and NTN by:
 *   - assigning different channel models to different BWPs/bands,
 *   - activating only the TN BWPs on TN cells,
 *   - activating only the NTN BWPs on NTN cells.
 *
 * So the separation is done by CHANNEL/BWP ACTIVATION, not by installing
 * different BWP-vector sizes on different gNB classes.
 */

static constexpr double TN_GNB_HEIGHT = 25.0;
static constexpr double NTN_OVERLAY_HEIGHT = 600.0;

// Scenario size
static constexpr double AREA_X_MIN = 0.0;
static constexpr double AREA_X_MAX = 4000.0;
static constexpr double AREA_Y_MIN = -1000.0;
static constexpr double AREA_Y_MAX = 1000.0;

// Global knobs
uint32_t numGroundUesS1 = 100;
uint32_t numGroundUesS2 = 100;
uint32_t numTnGnbs = 5;
uint32_t numNtnGnbs = 4;
Time management_interval = Seconds(2);

// Per-UES1 IP/metrics
std::vector<Ipv4Address> user_ip;
std::vector<double> user_delay_dl;
std::vector<double> user_jitter_dl;
std::vector<double> user_throughput_dl;
std::vector<double> user_pdr_dl;
std::vector<double> user_delay_ul;
std::vector<double> user_jitter_ul;
std::vector<double> user_throughput_ul;
std::vector<double> user_pdr_ul;

// Output paths
static std::string s_ueS1PositionTraceFile;
static std::string s_ueS2PositionTraceFile;
static std::string s_ntnPositionTraceFile;
static std::string s_handoverTraceFile;
static std::string s_flowStatTraceFile;
static std::string ns3_dir;

// FH traces
static std::ofstream g_fhTraceFile;
static std::ofstream g_airTraceFile;

// TN/NTN classification and measurements
static std::set<uint16_t> g_ntnCellIds;
static std::map<std::pair<uint16_t, uint16_t>, double> g_latestRsrp;

// Logging redirection
static std::ofstream g_nsLogFile;
static std::streambuf* g_oldClogBuf = nullptr;

// IP/IMSI maps
static std::unordered_map<uint32_t, uint64_t> g_ipToImsi;
static std::unordered_map<uint64_t, std::string> g_imsiRole;

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

void
TraceRsrpRsrqSinr(Ptr<OutputStreamWrapper> stream,
                  uint16_t rnti,
                  uint16_t cellId,
                  double rsrp,
                  double rsrq,
                  bool servingCell,
                  uint8_t componentCarrierId)
{
    g_latestRsrp[{rnti, cellId}] = rsrp;

    const char* type = g_ntnCellIds.count(cellId) ? "NTN" : "TN";

    *stream->GetStream() << Simulator::Now().GetSeconds() << " "
                         << rnti << " "
                         << cellId << " "
                         << type << " "
                         << rsrp << " "
                         << rsrq << " "
                         << servingCell << " "
                         << static_cast<uint32_t>(componentCarrierId) << "\n";
}

void
ReportFhTrace(const SfnSf&, uint16_t physCellId, uint16_t bwpId, uint64_t reqFh)
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
ReportAiTrace(const SfnSf&, uint16_t physCellId, uint16_t bwpId, uint32_t airRbs)
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

int
get_user_id_from_ipv4(Ipv4Address ip)
{
    for (uint32_t i = 0; i < numGroundUesS1; ++i)
    {
        if (user_ip[i] == ip)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void
ThroughputMonitor(FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon)
{
    std::ofstream flowLog(s_flowStatTraceFile, std::ios_base::app);

    const auto flowStats = flowMon->GetFlowStats();
    const auto ue_network = Ipv4Address("7.0.0.0");
    const auto ue_network_mask = Ipv4Mask("255.0.0.0");
    Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());

    for (const auto& stats : flowStats)
    {
        if (Simulator::Now() - stats.second.timeLastTxPacket > management_interval &&
            Simulator::Now() - stats.second.timeLastRxPacket > management_interval)
        {
            continue;
        }

        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);

        const bool isDl = ue_network_mask.IsMatch(ue_network, fiveTuple.destinationAddress);
        const bool isUl = ue_network_mask.IsMatch(ue_network, fiveTuple.sourceAddress);
        if (!(isDl || isUl))
        {
            continue;
        }

        Ipv4Address ueIp = isDl ? fiveTuple.destinationAddress : fiveTuple.sourceAddress;
        uint64_t imsi = LookupImsiFromIp(ueIp);
        const char* role = LookupRoleFromImsi(imsi);

        int rx_packets = static_cast<int>(stats.second.rxPackets);
        int tx_packets = static_cast<int>(stats.second.txPackets);

        double pdr = 100.0 * rx_packets / (tx_packets > 0 ? tx_packets : 1);
        double delay = (rx_packets > 0) ? stats.second.delaySum.GetSeconds() / rx_packets : 0.0;
        double jitter = (rx_packets > 0) ? stats.second.jitterSum.GetSeconds() / rx_packets : 0.0;

        double throughput =
            (stats.second.timeLastRxPacket > stats.second.timeFirstTxPacket)
                ? (stats.second.rxBytes * 8.0 /
                   (stats.second.timeLastRxPacket.GetSeconds() -
                    stats.second.timeFirstTxPacket.GetSeconds()) /
                   1024.0 / 1024.0)
                : 0.0;

        flowLog << Simulator::Now().GetSeconds() << "," << role << "," << imsi << "\n";

        int receiver_id = get_user_id_from_ipv4(ueIp);
        if (receiver_id != -1)
        {
            if (isDl)
            {
                user_delay_dl[receiver_id] = delay;
                user_jitter_dl[receiver_id] = jitter;
                user_throughput_dl[receiver_id] = throughput;
                user_pdr_dl[receiver_id] = pdr;
            }
            else
            {
                user_delay_ul[receiver_id] = delay;
                user_jitter_ul[receiver_id] = jitter;
                user_throughput_ul[receiver_id] = throughput;
                user_pdr_ul[receiver_id] = pdr;
            }
        }
    }

    std::ofstream qos_vs_time(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::app);
    const double t = Simulator::Now().GetSeconds();
    for (uint32_t ue = 0; ue < numGroundUesS1; ++ue)
    {
        qos_vs_time << t << "," << ue << ",DL,"
                    << user_delay_dl[ue] << "," << user_jitter_dl[ue] << ","
                    << user_throughput_dl[ue] << "," << user_pdr_dl[ue] << "\n";

        qos_vs_time << t << "," << ue << ",UL,"
                    << user_delay_ul[ue] << "," << user_jitter_ul[ue] << ","
                    << user_throughput_ul[ue] << "," << user_pdr_ul[ue] << "\n";
    }

    Simulator::Schedule(management_interval, ThroughputMonitor, fmhelper, flowMon);
}

void
TraceUeS1Positions(NodeContainer ues)
{
    std::ofstream f(s_ueS1PositionTraceFile, std::ios_base::app);
    const double t = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        Vector p = ues.Get(i)->GetObject<MobilityModel>()->GetPosition();
        f << t << " UE" << i << " " << p.x << " " << p.y << " " << p.z << "\n";
    }

    Simulator::Schedule(Seconds(1), &TraceUeS1Positions, ues);
}

void
TraceUeS2Positions(NodeContainer ues)
{
    std::ofstream f(s_ueS2PositionTraceFile, std::ios_base::app);
    const double t = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        Vector p = ues.Get(i)->GetObject<MobilityModel>()->GetPosition();
        f << t << " UE_S2_" << i << " " << p.x << " " << p.y << " " << p.z << "\n";
    }

    Simulator::Schedule(Seconds(1), &TraceUeS2Positions, ues);
}

void
TraceNtnPositions(NodeContainer ntnNodes)
{
    std::ofstream f(s_ntnPositionTraceFile, std::ios_base::app);
    const double t = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < ntnNodes.GetN(); ++i)
    {
        Vector p = ntnNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        f << t << " NTN" << i << " " << p.x << " " << p.y << " " << p.z << "\n";
    }

    Simulator::Schedule(Seconds(1), &TraceNtnPositions, ntnNodes);
}

void
NotifyHandoverEndOkGnb(std::string, uint64_t imsi, uint16_t targetCellId, uint16_t rnti)
{
    double targetRsrp = 0.0;
    double servingRsrp = 0.0;

    auto it = g_latestRsrp.find({rnti, targetCellId});
    if (it != g_latestRsrp.end())
    {
        targetRsrp = it->second;
    }

    for (const auto& kv : g_latestRsrp)
    {
        if (kv.first.first == rnti && kv.first.second != targetCellId)
        {
            servingRsrp = std::max(servingRsrp, kv.second);
        }
    }

    const char* type = g_ntnCellIds.count(targetCellId) ? "NTN" : "TN";

    std::ofstream f(s_handoverTraceFile, std::ios_base::app);
    f << Simulator::Now().GetSeconds()
      << " IMSI=" << imsi
      << " TargetCell=" << targetCellId
      << " Type=" << type
      << " TargetRSRP=" << targetRsrp
      << " ServingRSRP=" << servingRsrp
      << "\n";
}

void
install_mobility(NodeContainer staticNodes,
                 NodeContainer tnGnbNodes,
                 NodeContainer ntnGnbNodes,
                 NodeContainer groundUeNodesS1,
                 NodeContainer groundUeNodesS2)
{
    Ptr<ListPositionAllocator> staticAlloc = CreateObject<ListPositionAllocator>();
    staticAlloc->Add(Vector(0, 0, 0));

    MobilityHelper staticNodesHelper;
    staticNodesHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    staticNodesHelper.SetPositionAllocator(staticAlloc);
    staticNodesHelper.Install(staticNodes);

    // TN macro layer
    Ptr<ListPositionAllocator> tnGnbPosition = CreateObject<ListPositionAllocator>();

    const double minDist = 300.0;
    const uint32_t maxAttempts = 20000;

    Ptr<UniformRandomVariable> tnUx = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> tnUy = CreateObject<UniformRandomVariable>();
    tnUx->SetAttribute("Min", DoubleValue(AREA_X_MIN));
    tnUx->SetAttribute("Max", DoubleValue(AREA_X_MAX));
    tnUy->SetAttribute("Min", DoubleValue(AREA_Y_MIN));
    tnUy->SetAttribute("Max", DoubleValue(AREA_Y_MAX));

    std::vector<Vector> tnPlaced;
    tnPlaced.reserve(tnGnbNodes.GetN());

    auto TnFarEnough = [&](const Vector& cand) -> bool {
        for (const auto& p : tnPlaced)
        {
            const double dx = cand.x - p.x;
            const double dy = cand.y - p.y;
            const double d = std::sqrt(dx * dx + dy * dy);
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
            Vector cand(tnUx->GetValue(), tnUy->GetValue(), TN_GNB_HEIGHT);
            if (TnFarEnough(cand))
            {
                tnGnbPosition->Add(cand);
                tnPlaced.push_back(cand);
                ok = true;
                break;
            }
        }
        NS_ABORT_MSG_IF(!ok, "Could not place TN gNB " << i);
    }

    MobilityHelper tnGnbHelper;
    tnGnbHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    tnGnbHelper.SetPositionAllocator(tnGnbPosition);
    tnGnbHelper.Install(tnGnbNodes);

    // NTN overlay layer
    Ptr<ListPositionAllocator> ntnGnbPosition = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < ntnGnbNodes.GetN(); ++i)
    {
        Ptr<GeocentricConstantPositionMobilityModel> mob =
            CreateObject<GeocentricConstantPositionMobilityModel>();

        // Example only: latitude [deg], longitude [deg], altitude [m]
        double lat = 53.34;                  // around Dublin, for example
        double lon = -6.26 + 0.02 * i;
        double alt = 600000.0;               // 600 km if you want LEO-like altitude

        mob->SetGeographicPosition(Vector(lat, lon, alt));
        ntnGnbNodes.Get(i)->AggregateObject(mob);
    }

    // UEs
    Ptr<RandomBoxPositionAllocator> boxS1 = CreateObject<RandomBoxPositionAllocator>();
    boxS1->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=100.0|Max=3900.0]"));
    boxS1->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
    boxS1->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

    MobilityHelper ueS1Helper;
    ueS1Helper.SetPositionAllocator(boxS1);
    ueS1Helper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                "Bounds", StringValue("0|4000|-1000|1000"),
                                "Speed", StringValue("ns3::UniformRandomVariable[Min=2.0|Max=10.0]"),
                                "Pause", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
    ueS1Helper.Install(groundUeNodesS1);

    Ptr<RandomBoxPositionAllocator> boxS2 = CreateObject<RandomBoxPositionAllocator>();
    boxS2->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=100.0|Max=3900.0]"));
    boxS2->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
    boxS2->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

    MobilityHelper ueS2Helper;
    ueS2Helper.SetPositionAllocator(boxS2);
    ueS2Helper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                "Bounds", StringValue("0|4000|-1000|1000"),
                                "Speed", StringValue("ns3::UniformRandomVariable[Min=2.0|Max=10.0]"),
                                "Pause", StringValue("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
    ueS2Helper.Install(groundUeNodesS2);
}

static void
WriteFlowReportToFile(Ptr<FlowMonitor> monitor, FlowMonitorHelper* helper, const std::string& filename)
{
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(helper->GetClassifier());

    std::ofstream out(filename, std::ios::out | std::ios::trunc);
    auto stats = monitor->GetFlowStats();

    for (const auto& kv : stats)
    {
        uint32_t flowId = kv.first;
        const auto& st = kv.second;
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);

        std::string proto = (t.protocol == 6) ? "TCP" : (t.protocol == 17 ? "UDP" : "OTHER");

        out << "Flow " << flowId << " (" << t.sourceAddress << ":" << t.sourcePort
            << " -> " << t.destinationAddress << ":" << t.destinationPort
            << ") proto " << proto << "\n";

        out << "  Tx Packets: " << st.txPackets << "\n";
        out << "  Tx Bytes:   " << st.txBytes << "\n";

        double txDuration = (st.timeLastTxPacket > st.timeFirstTxPacket)
                                ? (st.timeLastTxPacket.GetSeconds() - st.timeFirstTxPacket.GetSeconds())
                                : 0.0;
        double txOfferedMbps = (txDuration > 0) ? (st.txBytes * 8.0 / txDuration / 1e6) : 0.0;
        out << "  TxOffered:  " << txOfferedMbps << " Mbps\n";
        out << "  Rx Bytes:   " << st.rxBytes << "\n";

        double rxDuration = (st.timeLastRxPacket > st.timeFirstRxPacket)
                                ? (st.timeLastRxPacket.GetSeconds() - st.timeFirstRxPacket.GetSeconds())
                                : 0.0;
        double thrMbps = (rxDuration > 0) ? (st.rxBytes * 8.0 / rxDuration / 1e6) : 0.0;
        out << "  Throughput: " << thrMbps << " Mbps\n";

        double meanDelayMs = (st.rxPackets > 0) ? (1000.0 * st.delaySum.GetSeconds() / st.rxPackets) : 0.0;
        double meanJitterMs = (st.rxPackets > 0) ? (1000.0 * st.jitterSum.GetSeconds() / st.rxPackets) : 0.0;

        out << "  Mean delay:  " << meanDelayMs << " ms\n";
        out << "  Mean jitter: " << meanJitterMs << " ms\n";
        out << "  Rx Packets: " << st.rxPackets << "\n";
    }
}

int
main(int argc, char* argv[])
{
    bool verbose = false;
    bool useOran = true;
    bool useOnnx = false;
    bool useTorch = false;
    bool useRsrp = true;
    double lmQueryInterval = 2.0;
    double maxWaitTime = 0.010;
    double txDelay = 0.1;
    bool remMode = false;
    std::string handoverAlgorithm = "ns3::NrNoOpHandoverAlgorithm";
    Time simTime = Seconds(25);
    std::string dbFileName = "oran-repository-tn-ntn.db";
    std::string lateCommandPolicy = "DROP";

    uint32_t maxUesPerCellTn = 20;
    uint32_t maxUesPerCellNtn = 100;
    double groundAttachDelay = 6.0;
    bool ofdma = true;
    std::string schedKind = "PF";
    double hysteresisDb = 2.0;
    std::string xrAppType = "VR";

    CommandLine cmd;
    cmd.AddValue("verbose", "Enable verbose LM logging", verbose);
    cmd.AddValue("use-oran", "Enable O-RAN", useOran);
    cmd.AddValue("use-onnx-lm", "Use ONNX LM", useOnnx);
    cmd.AddValue("use-torch-lm", "Use Torch LM", useTorch);
    cmd.AddValue("use-rsrp-lm", "Use RSRP LM", useRsrp);
    cmd.AddValue("sim-time", "Simulation time", simTime);
    cmd.AddValue("lm-query-interval", "LM query interval", lmQueryInterval);
    cmd.AddValue("tx-delay", "E2 delay", txDelay);
    cmd.AddValue("handover-algorithm", "Base handover algorithm", handoverAlgorithm);
    cmd.AddValue("db-file", "Repository DB file", dbFileName);
    cmd.AddValue("num-uess1", "Number of UES1", numGroundUesS1);
    cmd.AddValue("num-ground-ues", "Number of UES2", numGroundUesS2);
    cmd.AddValue("num-tn-gnbs", "Number of TN gNBs", numTnGnbs);
    cmd.AddValue("num-ntn-gnbs", "Number of NTN gNBs", numNtnGnbs);
    cmd.AddValue("ground-attach-delay", "Late attach delay", groundAttachDelay);
    cmd.AddValue("max-ues-tn", "TN cell capacity", maxUesPerCellTn);
    cmd.AddValue("max-ues-ntn", "NTN cell capacity", maxUesPerCellNtn);
    cmd.AddValue("sched", "Scheduler kind", schedKind);
    cmd.AddValue("ofdma", "Use OFDMA", ofdma);
    cmd.AddValue("rem-mode", "Enable REM", remMode);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
                    "Cannot use LM without enabling O-RAN.");
    NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1,
                    "Cannot use more than one LM simultaneously.");
    NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NrNoOpHandoverAlgorithm" &&
                        (useOnnx || useTorch || useRsrp),
                    "Cannot combine custom LM with non-noop baseline HO algorithm.");

    std::ostringstream runTag;
    runTag << "ueS1_" << numGroundUesS1
           << "_ueS2_" << numGroundUesS2
           << "_tnGnb_" << numTnGnbs
           << "_ntnGnb_" << numNtnGnbs
           << "_tnCap_" << maxUesPerCellTn
           << "_ntnCap_" << maxUesPerCellNtn
           << "_hyst_" << hysteresisDb;

    ns3_dir = "results/nr/tn-ntn/" + runTag.str() + "/";
    std::filesystem::create_directories(ns3_dir);

    s_ueS1PositionTraceFile = ns3_dir + "ues1-position-trace.tr";
    s_ueS2PositionTraceFile = ns3_dir + "ues2-position-trace.tr";
    s_ntnPositionTraceFile = ns3_dir + "ntn-position-trace.tr";
    s_handoverTraceFile = ns3_dir + "handover-trace.tr";
    s_flowStatTraceFile = ns3_dir + "flow-stats.log";

    Ptr<OutputStreamWrapper> rsrpTrace =
        Create<OutputStreamWrapper>(ns3_dir + "rsrp-trace.tr", std::ios::out);
    *rsrpTrace->GetStream() << "Time RNTI CellId CellType RSRP RSRQ Serving CCID\n";

    g_nsLogFile.open(ns3_dir + "ns3-oran-lm.log", std::ios::out | std::ios::trunc);
    g_oldClogBuf = std::clog.rdbuf(g_nsLogFile.rdbuf());

    LogComponentEnable("OranLmNr2NrRsrpHandoverWithTnNtn", LOG_LEVEL_INFO);
    LogComponentEnable("NrHelper", LOG_LEVEL_INFO);

    Config::SetDefault("ns3::NrRlcUm::EnablePdcpDiscarding", BooleanValue(false));
    Config::SetDefault("ns3::NrRlcUm::DiscardTimerMs", UintegerValue(0));
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(100)));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    Config::SetDefault("ns3::NrGnbRrc::QosFlowToRlcMapping",
                       EnumValue(NrGnbRrc::RLC_UM_ALWAYS));
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(100)));

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

    Ptr<NrChannelHelper> tnChannelHelper = CreateObject<NrChannelHelper>();
    tnChannelHelper->ConfigureFactories("UMa", "Default");
    tnChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    tnChannelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                       TimeValue(MilliSeconds(200)));

    Ptr<NrChannelHelper> ntnChannelHelper = CreateObject<NrChannelHelper>();
    ntnChannelHelper->ConfigureFactories("NTN-Urban", "Default");
    ntnChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    ntnChannelHelper->SetChannelConditionModelAttribute("UpdatePeriod",
                                                        TimeValue(MilliSeconds(200)));

    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetHandoverAlgorithmType(handoverAlgorithm);

    auto setSchedulerIfAvailable = [&](const std::string& name) -> bool {
        TypeId tid;
        if (TypeId::LookupByNameFailSafe(name, &tid))
        {
            NS_LOG_UNCOND(std::string("NR: using ") + name);
            nrHelper->SetSchedulerTypeId(tid);
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
    NS_ABORT_MSG_IF(!schedSet, "No supported concrete NR scheduler found.");

    nrHelper->SetDlErrorModel("ns3::NrEesmIrT2");
    nrHelper->SetUlErrorModel("ns3::NrEesmIrT2");
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(false));
    nrHelper->SetGnbPhyAttribute("NoiseFigure", DoubleValue(7.0));
    nrHelper->SetUePhyAttribute("NoiseFigure", DoubleValue(13.0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));

    nrHelper->EnableFhControl();
    nrHelper->SetFhControlAttribute("FhControlMethod", StringValue("OptimizeRBs"));
    nrHelper->SetFhControlAttribute("FhCapacity", UintegerValue(10000));
    nrHelper->SetFhControlAttribute("OverheadDyn", UintegerValue(32));

    // ---------- COMMON BWP CONFIGURATION FOR ALL DEVICES ----------
    // BWP 0: TN TDD
    // BWP 1: NTN DL
    // BWP 2: NTN UL
    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConfTn(4.0e9, 20e6, 1);
    CcBwpCreator::SimpleOperationBandConf bandConfNtn(4.2e9, 20e6, 1);
    bandConfNtn.m_numBwp = 2;

    OperationBandInfo bandTn = ccBwpCreator.CreateOperationBandContiguousCc(bandConfTn);
    OperationBandInfo bandNtn = ccBwpCreator.CreateOperationBandContiguousCc(bandConfNtn);

    std::vector<std::reference_wrapper<OperationBandInfo>> tnBands{std::ref(bandTn)};
    std::vector<std::reference_wrapper<OperationBandInfo>> ntnBands{std::ref(bandNtn)};

    const uint8_t bandMask = NrChannelHelper::INIT_PROPAGATION | NrChannelHelper::INIT_FADING;
    tnChannelHelper->AssignChannelsToBands(tnBands, bandMask);
    ntnChannelHelper->AssignChannelsToBands(ntnBands, bandMask);

    std::vector<std::reference_wrapper<OperationBandInfo>> allBands{std::ref(bandTn), std::ref(bandNtn)};
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps(allBands);

    const uint32_t BWP_TN = 0;
    const uint32_t BWP_NTN_DL = 1;
    const uint32_t BWP_NTN_UL = 2;
    // const uint32_t NUM_BWPS = 3;

    // IMPORTANT:
    // Use TN BWP as the common service BWP for static QoS mapping.
    // The NTN fallback decision is still done at CELL LEVEL by the LM,
    // but the RF/channel separation stays explicit by BWP/channel instances.
    // If later you want per-service NTN routing, redesign the QoS/BWP mapping too.
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(BWP_TN));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(BWP_TN));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(BWP_TN));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(BWP_TN));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(BWP_TN));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(BWP_TN));

    Ptr<IdealBeamformingHelper> bf = CreateObject<IdealBeamformingHelper>();
    bf->SetAttribute("BeamformingMethod", TypeIdValue(QuasiOmniDirectPathBeamforming::GetTypeId()));
    nrHelper->SetBeamformingHelper(bf);

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    nrHelper->SetEpcHelper(epcHelper);
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));
    nrHelper->Initialize();

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

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
    remoteHostStaticRouting->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);

    install_mobility(remoteHostContainer, tnGnbNodes, ntnGnbNodes, groundUeNodesS1, groundUeNodesS2);

    // Install all devices with the SAME BWP config
    NetDeviceContainer tnGnbNrDevs = nrHelper->InstallGnbDevice(tnGnbNodes, allBwps);
    NetDeviceContainer ntnGnbNrDevs = nrHelper->InstallGnbDevice(ntnGnbNodes, allBwps);
    NetDeviceContainer allGnbNrDevs;
    allGnbNrDevs.Add(tnGnbNrDevs);
    allGnbNrDevs.Add(ntnGnbNrDevs);

    NetDeviceContainer groundNrDevsS1 = nrHelper->InstallUeDevice(groundUeNodesS1, allBwps);
    NetDeviceContainer groundNrDevsS2 = nrHelper->InstallUeDevice(groundUeNodesS2, allBwps);

    // Per-layer cell capacity and type map
    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(tnGnbNrDevs.Get(i));
        gnb->GetRrc()->SetAttribute("MaxUesPerCell", UintegerValue(maxUesPerCellTn));
        nrHelper->SetCellCapacity(gnb->GetCellId(), maxUesPerCellTn);
    }
    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
        gnb->GetRrc()->SetAttribute("MaxUesPerCell", UintegerValue(maxUesPerCellNtn));
        nrHelper->SetCellCapacity(gnb->GetCellId(), maxUesPerCellNtn);
        g_ntnCellIds.insert(gnb->GetCellId());
    }

    // Explicitly activate only TN BWPs on TN cells and only NTN BWPs on NTN cells
    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NetDevice> dev = tnGnbNrDevs.Get(i);
        nrHelper->GetGnbPhy(dev, BWP_TN)->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|UL|UL|UL|"));
        nrHelper->GetGnbPhy(dev, BWP_TN)->SetAttribute("TxPower", DoubleValue(43.0));

        nrHelper->GetGnbPhy(dev, BWP_NTN_DL)->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
        nrHelper->GetGnbPhy(dev, BWP_NTN_DL)->SetAttribute("TxPower", DoubleValue(0.0));

        nrHelper->GetGnbPhy(dev, BWP_NTN_UL)->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
        nrHelper->GetGnbPhy(dev, BWP_NTN_UL)->SetAttribute("TxPower", DoubleValue(0.0));
    }

    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NetDevice> dev = ntnGnbNrDevs.Get(i);
        nrHelper->GetGnbPhy(dev, BWP_TN)->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|UL|UL|UL|"));
        nrHelper->GetGnbPhy(dev, BWP_TN)->SetAttribute("TxPower", DoubleValue(0.0));

        nrHelper->GetGnbPhy(dev, BWP_NTN_DL)->SetAttribute("Pattern", StringValue("DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|"));
        nrHelper->GetGnbPhy(dev, BWP_NTN_DL)->SetAttribute("TxPower", DoubleValue(38.0));

        nrHelper->GetGnbPhy(dev, BWP_NTN_UL)->SetAttribute("Pattern", StringValue("UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|"));
        nrHelper->GetGnbPhy(dev, BWP_NTN_UL)->SetAttribute("TxPower", DoubleValue(0.0));
    }

    // UE config and output links (keep full config common)
    for (uint32_t i = 0; i < allGnbNrDevs.GetN(); ++i)
    {
        NrHelper::GetBwpManagerGnb(allGnbNrDevs.Get(i))->SetOutputLink(BWP_NTN_UL, BWP_NTN_DL);
    }
    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        NrHelper::GetBwpManagerUe(groundNrDevsS1.Get(i))->SetOutputLink(BWP_NTN_DL, BWP_NTN_UL);
    }
    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        NrHelper::GetBwpManagerUe(groundNrDevsS2.Get(i))->SetOutputLink(BWP_NTN_DL, BWP_NTN_UL);
    }

    for (uint32_t i = 0; i < allGnbNrDevs.GetN(); ++i)
    {
        DynamicCast<NrGnbNetDevice>(allGnbNrDevs.Get(i))->UpdateConfig();
    }
    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        DynamicCast<NrUeNetDevice>(groundNrDevsS1.Get(i))->UpdateConfig();
    }
    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        DynamicCast<NrUeNetDevice>(groundNrDevsS2.Get(i))->UpdateConfig();
    }

    nrHelper->ConfigureFhControl(allGnbNrDevs);
    for (auto it = allGnbNrDevs.Begin(); it != allGnbNrDevs.End(); ++it)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(*it);
        gnb->GetNrFhControl()->TraceConnectWithoutContext("RequiredFhDlThroughput", MakeCallback(&ReportFhTrace));
        gnb->GetNrFhControl()->TraceConnectWithoutContext("UsedAirRbs", MakeCallback(&ReportAiTrace));
    }

    nrHelper->SetAttribute("InitMaxUesPerCell", UintegerValue(0));
    nrHelper->SetAttribute("InitMinRsrpDbm", DoubleValue(-120.0));
    nrHelper->SetAttribute("InitRetryInterval", TimeValue(Seconds(2.0)));

    // TN-first initial attach
    nrHelper->AttachToMaxRsrpGnb(groundNrDevsS1, tnGnbNrDevs);
    Time tLateAttach = Seconds(groundAttachDelay);
    Simulator::Schedule(tLateAttach, [nrHelper, groundNrDevsS2, tnGnbNrDevs]() {
        nrHelper->AttachToMaxRsrpGnb(groundNrDevsS2, tnGnbNrDevs);
    });

    nrHelper->AddX2Interface(allGnbNodes);

    internet.Install(groundUeNodesS1);
    internet.Install(groundUeNodesS2);

    Ipv4InterfaceContainer ueIpIfaceS1 = epcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNrDevsS1));
    Ipv4InterfaceContainer ueIpIfaceS2 = epcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNrDevsS2));

    g_imsiRole.clear();
    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        g_imsiRole[DynamicCast<NrUeNetDevice>(groundNrDevsS1.Get(i))->GetImsi()] = "UES1";
    }
    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        g_imsiRole[DynamicCast<NrUeNetDevice>(groundNrDevsS2.Get(i))->GetImsi()] = "UES2";
    }

    g_ipToImsi.clear();
    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        auto ue = DynamicCast<NrUeNetDevice>(groundNrDevsS1.Get(i));
        g_ipToImsi[ueIpIfaceS1.GetAddress(i).Get()] = ue->GetImsi();
    }
    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        auto ue = DynamicCast<NrUeNetDevice>(groundNrDevsS2.Get(i));
        g_ipToImsi[ueIpIfaceS2.GetAddress(i).Get()] = ue->GetImsi();
    }

    for (uint32_t u = 0; u < groundUeNodesS1.GetN(); ++u)
    {
        auto rt = ipv4RoutingHelper.GetStaticRouting(groundUeNodesS1.Get(u)->GetObject<Ipv4>());
        rt->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    for (uint32_t u = 0; u < groundUeNodesS2.GetN(); ++u)
    {
        auto rt = ipv4RoutingHelper.GetStaticRouting(groundUeNodesS2.Get(u)->GetObject<Ipv4>());
        rt->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    Ipv4Address remoteHostIp = internetIpIfaces.GetAddress(1);

    // ---------- Traffic ----------
    ApplicationContainer xrDlSinks;
    ApplicationContainer xrUlSinks;
    ApplicationContainer xrDlSenders;
    ApplicationContainer xrUlSenders;

    NrXrConfig dlConfig = VR_DL1;
    NrXrConfig ulConfig = VR_UL;
    if (xrAppType == "AR")
    {
        dlConfig = AR_M3;
        ulConfig = VR_UL;
    }
    else if (xrAppType == "CG")
    {
        dlConfig = CG_DL1;
        ulConfig = CG_UL;
    }

    for (uint32_t i = 0; i < groundUeNodesS1.GetN(); ++i)
    {
        uint16_t dlPort = 10000 + i;
        uint16_t ulPort = 12000 + i;

        PacketSinkHelper dlSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        xrDlSinks.Add(dlSink.Install(groundUeNodesS1.Get(i)));

        XrTrafficMixerHelper xrDlHelper;
        xrDlHelper.ConfigureXr(dlConfig);
        std::vector<Address> dlAddresses{InetSocketAddress(ueIpIfaceS1.GetAddress(i), dlPort)};
        xrDlSenders.Add(xrDlHelper.Install("ns3::UdpSocketFactory", dlAddresses, remoteHost));

        PacketSinkHelper ulSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        xrUlSinks.Add(ulSink.Install(remoteHost));

        XrTrafficMixerHelper xrUlHelper;
        xrUlHelper.ConfigureXr(ulConfig);
        std::vector<Address> ulAddresses{InetSocketAddress(remoteHostIp, ulPort)};
        xrUlSenders.Add(xrUlHelper.Install("ns3::UdpSocketFactory", ulAddresses, groundUeNodesS1.Get(i)));

        NrQosFlow xrDlFlow(NrQosFlow::GBR_CONV_VIDEO);
        Ptr<NrQosRule> xrDlRule = Create<NrQosRule>();
        NrQosRule::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd = dlPort;
        dlpf.direction = NrQosRule::DOWNLINK;
        xrDlRule->Add(dlpf);

        NrQosFlow xrUlFlow(NrQosFlow::GBR_LIVE_UL_71);
        Ptr<NrQosRule> xrUlRule = Create<NrQosRule>();
        NrQosRule::PacketFilter ulpf;
        ulpf.remotePortStart = ulPort;
        ulpf.remotePortEnd = ulPort;
        ulpf.direction = NrQosRule::UPLINK;
        xrUlRule->Add(ulpf);

        nrHelper->ActivateDedicatedQosFlow(groundNrDevsS1.Get(i), xrDlFlow, xrDlRule);
        nrHelper->ActivateDedicatedQosFlow(groundNrDevsS1.Get(i), xrUlFlow, xrUlRule);
    }

    xrDlSinks.Start(Seconds(1.0));
    xrDlSinks.Stop(simTime + Seconds(15));
    xrDlSenders.Start(Seconds(2.0));
    xrDlSenders.Stop(simTime + Seconds(10));
    xrUlSinks.Start(Seconds(1.0));
    xrUlSinks.Stop(simTime + Seconds(15));
    xrUlSenders.Start(Seconds(3.0));
    xrUlSenders.Stop(simTime + Seconds(15));

    uint16_t groundBasePort = 20000;
    ApplicationContainer ueAppsS2;
    ApplicationContainer groundRemoteAppsS2;

    for (uint16_t i = 0; i < groundUeNodesS2.GetN(); ++i)
    {
        uint16_t port = groundBasePort + i;
        Ptr<NetDevice> gUeDev = groundNrDevsS2.Get(i);

        NrQosFlow voiceFlow(NrQosFlow::GBR_CONV_VOICE);
        Ptr<NrQosRule> voiceRule = Create<NrQosRule>();
        NrQosRule::PacketFilter gdl;
        gdl.localPortStart = port;
        gdl.localPortEnd = port;
        voiceRule->Add(gdl);

        Simulator::Schedule(tLateAttach, [nrHelper, gUeDev, voiceFlow, voiceRule]() {
            nrHelper->ActivateDedicatedQosFlow(gUeDev, voiceFlow, voiceRule);
        });

        PacketSinkHelper dlSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
        ueAppsS2.Add(dlSink.Install(groundUeNodesS2.Get(i)));

        TrafficGeneratorHelper voiceHelper("ns3::UdpSocketFactory",
                                           InetSocketAddress(ueIpIfaceS2.GetAddress(i), port),
                                           TrafficGeneratorNgmnVoip::GetTypeId());
        groundRemoteAppsS2.Add(voiceHelper.Install(remoteHost));
    }

    ueAppsS2.Start(Seconds(1));
    groundRemoteAppsS2.Start(tLateAttach + Seconds(0.5));
    groundRemoteAppsS2.Stop(simTime + Seconds(10));
    ueAppsS2.Stop(simTime + Seconds(15));

    // ---------- O-RAN ----------
    if (useOran)
    {
        if (!dbFileName.empty())
        {
            ::remove(dbFileName.c_str());
        }

        TypeId defaultLmTid = TypeId::LookupByName("ns3::OranLmNoop");
        if (useOnnx)
        {
            NS_ABORT_MSG_IF(!TypeId::LookupByNameFailSafe("ns3::OranLmNr2NrOnnxHandover", &defaultLmTid),
                            "ONNX LM not found.");
        }
        else if (useTorch)
        {
            NS_ABORT_MSG_IF(!TypeId::LookupByNameFailSafe("ns3::OranLmNr2NrTorchHandover", &defaultLmTid),
                            "Torch LM not found.");
        }
        else if (useRsrp)
        {
            defaultLmTid = TypeId::LookupByName("ns3::OranLmNr2NrRsrpHandoverWithTnNtn");
        }

        Ptr<OranDataRepository> dataRepository = CreateObject<OranDataRepositorySqlite>();
        Ptr<OranCmm> cmm = CreateObject<OranCmmHandover>();
        Ptr<OranNearRtRic> nearRtRic = CreateObject<OranNearRtRic>();
        Ptr<OranNearRtRicE2Terminator> nearRtRicE2Terminator = CreateObject<OranNearRtRicE2Terminator>();

        ObjectFactory defaultLmFactory;
        defaultLmFactory.SetTypeId(defaultLmTid);
        Ptr<OranLm> defaultLm = defaultLmFactory.Create<OranLm>();
        defaultLm->SetAttribute("MaxUesPerCell", UintegerValue(0));
        defaultLm->SetAttribute("TryNextBest", BooleanValue(true));
        defaultLm->SetAttribute("MinAcceptableRsrpDbm", DoubleValue(-120.0));
        defaultLm->SetAttribute("HysteresisDb", DoubleValue(hysteresisDb));
        defaultLm->SetAttribute("Verbose", BooleanValue(verbose));
        defaultLm->SetAttribute("NearRtRic", PointerValue(nearRtRic));

        if (useRsrp)
        {
            Ptr<OranLmNr2NrRsrpHandoverWithTnNtn> rsrpLm = DynamicCast<OranLmNr2NrRsrpHandoverWithTnNtn>(defaultLm);
            if (rsrpLm)
            {
                for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
                {
                    auto gnb = DynamicCast<NrGnbNetDevice>(tnGnbNrDevs.Get(i));
                    rsrpLm->SetCellCapacity(gnb->GetCellId(), maxUesPerCellTn);
                    rsrpLm->SetCellIsNtn(gnb->GetCellId(), false);
                }
                for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
                {
                    auto gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
                    rsrpLm->SetCellCapacity(gnb->GetCellId(), maxUesPerCellNtn);
                    rsrpLm->SetCellIsNtn(gnb->GetCellId(), true);
                }
                rsrpLm->SetAttribute("TnMinRsrpDbm", DoubleValue(-110.0));
                rsrpLm->SetAttribute("NtnEnterMarginDb", DoubleValue(3.0));
                rsrpLm->SetAttribute("TnReturnMarginDb", DoubleValue(5.0));
            }
        }

        if (useTorch)
        {
            Config::SetDefault("ns3::OranLmNr2NrTorchHandover::TorchModelPath",
                               StringValue(ns3_dir + "saved_trained_classification_pytorch.pt"));
        }

        dataRepository->SetAttribute("DatabaseFile", StringValue(dbFileName));
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

        Simulator::Schedule(Seconds(2.5), &OranNearRtRic::Start, nearRtRic);

        auto installUeTerminator = [&](Ptr<Node> ueNode,
                                       Ptr<Application> txApp,
                                       Ptr<Application> rxApp,
                                       Time activateAt) {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrUeCellInfo> nrUeCellInfoReporter = CreateObject<OranReporterNrUeCellInfo>();
            Ptr<OranReporterAppLoss> appLossReporter = CreateObject<OranReporterAppLoss>();
            Ptr<OranReporterNrUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterNrUeRsrpRsrq>();
            Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            appLossReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));

            txApp->TraceConnectWithoutContext("Tx", MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
            rxApp->TraceConnectWithoutContext("Rx", MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));

            for (uint32_t netDevIdx = 0; netDevIdx < ueNode->GetNDevices(); ++netDevIdx)
            {
                Ptr<NrUeNetDevice> nrUeDevice = ueNode->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();
                if (nrUeDevice)
                {
                    nrUeDevice->GetPhy(0)->TraceConnectWithoutContext(
                        "ReportUeMeasurements",
                        MakeCallback(&ns3::OranReporterNrUeRsrpRsrq::ReportRsrpRsrq, rsrpRsrqReporter));
                }
            }

            nrUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrUeTerminator->SetAttribute("RegistrationIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrUeTerminator->SetAttribute("SendIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrUeTerminator->SetAttribute(
                "TransmissionDelayRv",
                StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(txDelay) + "]"));

            nrUeTerminator->AddReporter(locationReporter);
            nrUeTerminator->AddReporter(nrUeCellInfoReporter);
            nrUeTerminator->AddReporter(rsrpRsrqReporter);
            nrUeTerminator->AddReporter(appLossReporter);
            nrUeTerminator->Attach(ueNode);
            Simulator::Schedule(activateAt, &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
        };

        for (uint32_t idx = 0; idx < groundUeNodesS1.GetN(); ++idx)
        {
            installUeTerminator(groundUeNodesS1.Get(idx), xrDlSenders.Get(idx), xrDlSinks.Get(idx), Seconds(2.0));
        }
        for (uint32_t idx = 0; idx < groundUeNodesS2.GetN(); ++idx)
        {
            installUeTerminator(groundUeNodesS2.Get(idx), groundRemoteAppsS2.Get(idx), ueAppsS2.Get(idx),
                                tLateAttach + Seconds(1.0));
        }

        for (uint32_t idx = 0; idx < allGnbNodes.GetN(); ++idx)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrCellLoad> nrCellLoadReporter = CreateObject<OranReporterNrCellLoad>();
            Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));

            auto dev = allGnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();
            dev->GetMac(BWP_TN)->TraceConnectWithoutContext(
                "DlScheduling", MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));
            dev->GetMac(BWP_NTN_DL)->TraceConnectWithoutContext(
                "DlScheduling", MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));

            nrGnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrGnbTerminator->SetAttribute("RegistrationIntervalRv",
                                          StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrGnbTerminator->SetAttribute("SendIntervalRv",
                                          StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrGnbTerminator->SetAttribute(
                "TransmissionDelayRv",
                StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(txDelay) + "]"));
            nrGnbTerminator->AddReporter(locationReporter);
            nrGnbTerminator->AddReporter(nrCellLoadReporter);
            nrGnbTerminator->Attach(allGnbNodes.Get(idx));
            Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorNrGnb::Activate, nrGnbTerminator);
        }
    }

    std::ofstream(s_ueS1PositionTraceFile, std::ios_base::trunc).close();
    std::ofstream(s_ueS2PositionTraceFile, std::ios_base::trunc).close();
    std::ofstream(s_ntnPositionTraceFile, std::ios_base::trunc).close();
    std::ofstream(s_handoverTraceFile, std::ios_base::trunc).close();
    std::ofstream(s_flowStatTraceFile, std::ios_base::trunc) << "Time,Role,IMSI\n";

    Simulator::Schedule(Seconds(1), &TraceUeS1Positions, groundUeNodesS1);
    Simulator::Schedule(Seconds(1), &TraceNtnPositions, ntnGnbNodes);
    Simulator::Schedule(tLateAttach, &TraceUeS2Positions, groundUeNodesS2);

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk", MakeCallback(&NotifyHandoverEndOkGnb));

    for (auto it = groundNrDevsS1.Begin(); it != groundNrDevsS1.End(); ++it)
    {
        auto ue = (*it)->GetObject<NrUeNetDevice>();
        ue->GetPhy(0)->TraceConnectWithoutContext(
            "ReportUeMeasurements", MakeBoundCallback(&TraceRsrpRsrqSinr, rsrpTrace));
    }
    for (auto it = groundNrDevsS2.Begin(); it != groundNrDevsS2.End(); ++it)
    {
        auto ue = (*it)->GetObject<NrUeNetDevice>();
        ue->GetPhy(0)->TraceConnectWithoutContext(
            "ReportUeMeasurements", MakeBoundCallback(&TraceRsrpRsrqSinr, rsrpTrace));
    }

    user_ip.resize(numGroundUesS1);
    user_delay_dl.assign(numGroundUesS1, 0.0);
    user_jitter_dl.assign(numGroundUesS1, 0.0);
    user_throughput_dl.assign(numGroundUesS1, 0.0);
    user_pdr_dl.assign(numGroundUesS1, 0.0);
    user_delay_ul.assign(numGroundUesS1, 0.0);
    user_jitter_ul.assign(numGroundUesS1, 0.0);
    user_throughput_ul.assign(numGroundUesS1, 0.0);
    user_pdr_ul.assign(numGroundUesS1, 0.0);

    FlowMonitorHelper flowHelper;
    NodeContainer nodesToMonitor;
    nodesToMonitor.Add(remoteHostContainer);
    nodesToMonitor.Add(groundUeNodesS1);
    nodesToMonitor.Add(groundUeNodesS2);
    Ptr<FlowMonitor> flowMonitor = flowHelper.Install(nodesToMonitor);

    std::ofstream qosVsTime(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::trunc);
    qosVsTime << "Time,UE,Dir,Delay,Jitter,Throughput,PDR\n";
    Simulator::Schedule(management_interval, ThroughputMonitor, &flowHelper, flowMonitor);

    for (uint32_t i = 0; i < groundUeNodesS1.GetN(); ++i)
    {
        user_ip[i] = ueIpIfaceS1.GetAddress(i);
    }

    if (remMode)
    {
        Ptr<NrRadioEnvironmentMapHelper> remHelper = CreateObject<NrRadioEnvironmentMapHelper>();
        remHelper->SetAttribute("SimTag", StringValue("tn-ntn-hybrid"));
        remHelper->SetAttribute("XMin", DoubleValue(AREA_X_MIN));
        remHelper->SetAttribute("XMax", DoubleValue(AREA_X_MAX));
        remHelper->SetAttribute("XRes", UintegerValue(400));
        remHelper->SetAttribute("YMin", DoubleValue(AREA_Y_MIN));
        remHelper->SetAttribute("YMax", DoubleValue(AREA_Y_MAX));
        remHelper->SetAttribute("YRes", UintegerValue(250));
        remHelper->SetAttribute("Z", DoubleValue(1.5));

        Ptr<NetDevice> rrdDevice = groundNrDevsS1.Get(0);
        Simulator::Schedule(Seconds(10.0),
                            &NrRadioEnvironmentMapHelper::CreateRem,
                            remHelper,
                            allGnbNrDevs,
                            rrdDevice,
                            static_cast<uint8_t>(BWP_TN));
    }

    Simulator::Stop(simTime + Seconds(15));
    Simulator::Run();

    WriteFlowReportToFile(flowMonitor, &flowHelper, ns3_dir + "final-flow-report.txt");

    if (g_oldClogBuf)
    {
        std::clog.rdbuf(g_oldClogBuf);
    }
    if (g_nsLogFile.is_open())
    {
        g_nsLogFile.close();
    }
    if (g_fhTraceFile.is_open())
    {
        g_fhTraceFile.close();
    }
    if (g_airTraceFile.is_open())
    {
        g_airTraceFile.close();
    }

    Simulator::Destroy();
    return 0;
}
