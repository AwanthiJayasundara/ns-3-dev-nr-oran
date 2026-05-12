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

NS_LOG_COMPONENT_DEFINE("OranNrUavGroundTrafficHandoverExample");

/**
 * Usage example of the ORAN NR models for UAV and ground UE traffic handover.
 *
 * Minimum required versions for reproducibility:
 *   - ns-3 version: 3.39 or later
 *   - 5G-LENA version: 2.6 or later
 *
 * The scenario consists of X NR UAV UEs (moving randomly) and Y NR ground UEs (static) inside a large 2D area
 * and served by Z fixed gNB macro cells. Each UAV receives downlink UDP traffic
 * from a remote host through an NR EPC (NrPointToPointEpcHelper). 
 *
 * The NR radio access uses a 3GPP UMa propagation scenario with optional fading
 * enabled, and an Ideal Beamforming helper (Quasi-Omni direct path beamforming).

 * A concrete NR scheduler is selected at runtime (OFDMA/TDMA with RR/PF/MR/QoS),
 * and the UEs initially attach to the gNB offering the maximum RSRP. X2 interfaces
 * are enabled to support inter-gNB handovers.
 *
 * In ORAN mode, UAV UEs report periodic measurements (location, serving cell info,
 * and application loss metrics) to a Near-RT RIC using E2 node terminators.
 * The Near-RT RIC runs an RSRP-based Logic Module (OranLmNr2NrRsrpHandover) that
 * can decide and trigger NR-to-NR handovers. gNB-side cell load is also reported
 * via NR scheduling callbacks to support conflict mitigation.
 *
 * FlowMonitor is used to compute per-UAV QoS metrics (delay, jitter, throughput,
 * and packet delivery ratio) periodically and write them to trace files over time.
 * Additionally, node mobility positions and successful handover events are logged.
 *
 * Ground UEs are added with constant position (No HO> we can change if want) and connect
 * with RIC too so load-aware handover decisions can be made based on the total number of 
 * UEs (UAV + ground) connected to each gNB. 
 * 
 * Cell load capacity is set so no intial attachement or handover will be triggered for a 
 * gNB that has already reached the maximum number of UEs. 
 *
 * To see all configurable options, run:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-uav-ground-traffic-handover-example --PrintHelp"
 * \endcode
 *
 * A basic run command is:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-uav-ground-traffic-handover-example"
 * \endcode
 */

const static float GNB_HEIGHT = 25;

// Variables
uint32_t numUAVs = 25;
uint32_t numGnbs = 5;
uint32_t numGroundUes = 20;   // ground UEs in addition to UAVs

// Metrics collection interval
Time management_interval = Seconds(4);

// UAVs ips vector
std::vector<Ipv4Address> user_ip;

// Vectors with the most recent metrics for each UAV
std::vector<double> user_delay;
std::vector<double> user_jitter;
std::vector<double> user_throughput;
std::vector<double> user_pdr;

// // static std::string s_trafficTraceFile;
static std::string s_positionTraceFile;
static std::string s_handoverTraceFile;
static std::string s_flowStatTraceFile;
static std::string ns3_dir;

// --- ns-3 NS_LOG output redirection (LogComponentEnable -> file via std::clog) ---
static std::ofstream g_nsLogFile;
static std::streambuf* g_oldClogBuf = nullptr;

static std::unordered_map<uint32_t, uint64_t> g_ipToImsi; // IPv4.Get() -> IMSI
static std::unordered_map<uint64_t, std::string> g_imsiRole; // IMSI -> "UAV"/"GND"

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

// Helper function that returns the UAV id associated with a specific IP
int
get_user_id_from_ipv4(Ipv4Address ip)
{
    for (uint32_t i = 0; i < numUAVs; i++)
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

        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);
        if (!ue_network_mask.IsMatch(ue_network, fiveTuple.destinationAddress))
            continue;

        Ipv4Address ueIp = fiveTuple.destinationAddress; // for your DL traffic
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

        int receiver_id = get_user_id_from_ipv4(fiveTuple.destinationAddress);
        if (receiver_id != -1)
        {
            user_delay[receiver_id] = Delay;
            user_jitter[receiver_id] = Jitter;
            user_throughput[receiver_id] = Throughput;
            user_pdr[receiver_id] = PDR;
        }
    }

    std::ofstream qos_vs_time;
    qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::app);
    for (uint32_t ue = 0; ue < numUAVs; ++ue)
    {
        qos_vs_time << Simulator::Now().GetSeconds() << "," << ue << "," << user_delay[ue] << ","
                    << user_jitter[ue] << "," << user_throughput[ue] << "," << user_pdr[ue]
                    << std::endl;
    }

    flowMon->ResetAllStats();

    Simulator::Schedule(management_interval, ThroughputMonitor, fmhelper, flowMon);
}

// Trace each node's location
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
                      NodeContainer gnbNodes,
                      NodeContainer uavNodes,
                      NodeContainer groundUeNodes)
{
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    allocator->Add(Vector(0, 0, 0));
    MobilityHelper staticNodesHelper;
    staticNodesHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    staticNodesHelper.SetPositionAllocator(allocator);
    staticNodesHelper.Install(staticNodes);

    // Install Mobility Model for gNBs
    // Ptr<ListPositionAllocator> gnbPosition = CreateObject<ListPositionAllocator>();
    
    // gnbPosition->Add(Vector(0, 0, GNB_HEIGHT));
    // gnbPosition->Add(Vector(500, 500, GNB_HEIGHT));
    // gnbPosition->Add(Vector(-500, 500, GNB_HEIGHT));
    // gnbPosition->Add(Vector(500, -500, GNB_HEIGHT));
    // gnbPosition->Add(Vector(-500, -500, GNB_HEIGHT));

    // MobilityHelper gnbHelper;
    // gnbHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // gnbHelper.SetPositionAllocator(gnbPosition);
    // gnbHelper.Install(gnbNodes);

    // Install Mobility Model for gNBs (RANDOM positions with minimum distance)
    Ptr<ListPositionAllocator> gnbPosition = CreateObject<ListPositionAllocator>();

    // --- area bounds (match your UAV bounds) ---
    const double HALF = 1000.0;   // [-HALF, +HALF] in X and Y
    const double z    = GNB_HEIGHT;

    // --- minimum separation between any two gNBs ---
    const double minDist = 200.0; // meters (tune this)

    // --- random generators for X/Y ---
    Ptr<UniformRandomVariable> ux = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> uy = CreateObject<UniformRandomVariable>();
    ux->SetAttribute("Min", DoubleValue(-HALF));
    ux->SetAttribute("Max", DoubleValue( HALF));
    uy->SetAttribute("Min", DoubleValue(-HALF));
    uy->SetAttribute("Max", DoubleValue( HALF));

    // Track placed positions so we can enforce min distance
    std::vector<Vector> placed;
    placed.reserve(gnbNodes.GetN());

    // Distance check helper
    auto FarEnough = [&](const Vector& cand) -> bool {
        for (const auto& p : placed)
        {
            const double dx = cand.x - p.x;
            const double dy = cand.y - p.y;
            const double d  = std::sqrt(dx*dx + dy*dy);
            if (d < minDist)
            {
                return false;
            }
        }
        return true;
    };

    const uint32_t maxAttempts = 20000;

    for (uint32_t i = 0; i < gnbNodes.GetN(); ++i)
    {
        bool ok = false;

        for (uint32_t a = 0; a < maxAttempts; ++a)
        {
            Vector cand(ux->GetValue(), uy->GetValue(), z);

            if (FarEnough(cand))
            {
                gnbPosition->Add(cand);
                placed.push_back(cand);
                ok = true;
                break;
            }
        }

        NS_ABORT_MSG_IF(!ok,
            "Could not place gNB " << i
            << " with minDist=" << minDist << "m in area [-" << HALF << "," << HALF
            << "]. Reduce minDist or increase area.");
    }

    MobilityHelper gnbHelper;
    gnbHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbHelper.SetPositionAllocator(gnbPosition);
    gnbHelper.Install(gnbNodes);


    // Install Mobility Model for UAVs
    //different uavs has different x, y, z initial value inside the boundry
    //initial placement so make lttle change to bound valuavs so avoids immediate boundary hits at t=0
    Ptr<RandomBoxPositionAllocator> box = CreateObject<RandomBoxPositionAllocator>();
    box->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=-990|Max=990]"));
    box->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=-990|Max=990]"));
    box->SetAttribute("Z", StringValue("ns3::UniformRandomVariable[Min=100.0|Max=200.0]"));

    //uav moves the z axis where it initially selected between 100 and 200 m
    MobilityHelper uavHelper;
    uavHelper.SetPositionAllocator(box);
    uavHelper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                              "Bounds",
                              StringValue("-1000|1000|-1000|1000"),
                              "Speed",
                              StringValue("ns3::UniformRandomVariable[Min=20.0|Max=30.0]"),
                              //// No pausing (set a Constant>0 if want them to stop sometimes)-hovering
                              //if not use "Pause",  StringValue("ns3::ConstantRandomVariable[Constant=0.0]")
                              "Pause",
                              StringValue("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
    uavHelper.Install(uavNodes);

    // Ground UE mobility (near-ground)
    // -------------------------------
    Ptr<RandomBoxPositionAllocator> gbox = CreateObject<RandomBoxPositionAllocator>();
    gbox->SetAttribute("X", StringValue("ns3::UniformRandomVariable[Min=-990|Max=990]"));
    gbox->SetAttribute("Y", StringValue("ns3::UniformRandomVariable[Min=-990|Max=990]"));
    gbox->SetAttribute("Z", StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

    MobilityHelper gueHelper;
    gueHelper.SetPositionAllocator(gbox);
    gueHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    // gueHelper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
    //                            "Bounds", StringValue("-1000|1000|-1000|1000"),
    //                            "Speed",  StringValue("ns3::UniformRandomVariable[Min=0.5|Max=1.5]"),
    //                            "Pause",  StringValue("ns3::UniformRandomVariable[Min=0.5|Max=3.0]"));
    gueHelper.Install(groundUeNodes);
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
    std::string dbFileName = "oran-repository-uav.db";
    std::string lateCommandPolicy = "DROP";

    uint32_t maxUesPerCell = 10; // ORAN LM parameter: maximum number of UEs per cell (for load-aware handover decisions)
    ///£
    double groundAttachDelay = 6.0; // seconds
    ///£
    // Scheduler CLI knobs (safe defaults to a concrete scheduler)
    bool ofdma = false;            // true=OFDMA, false=TDMA
    std::string schedKind = "RR"; // RR | PF | MR | Qos

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
    cmd.AddValue("num-uavs", "Number of UAVs", numUAVs);
    cmd.AddValue("num-gnbs", "Number of gNBs", numGnbs);
    cmd.AddValue("rem-mode", "Generate radio environment map", remMode);
    cmd.AddValue("rem-rb-id", "RB id", remRbId);
    cmd.AddValue("ofdma", "Use OFDMA (1) or TDMA (0)", ofdma);
    cmd.AddValue("sched", "Scheduler kind: RR, PF, MR, Qos", schedKind);
    cmd.AddValue("num-ground-ues", "Number of ground UEs", numGroundUes);
    cmd.AddValue("ground-attach-delay", "Delay before attaching ground UEs (s)", groundAttachDelay);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
                    "Cannot use ML LM or RSRP LM without enabling O-RAN.");
    NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1, "Cannot use more than one LM simultaneously.");
    NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NrNoOpHandoverAlgorithm" && (useOnnx || useTorch || useRsrp),
                    "Cannot use non-noop handover algorithm with ML/RSRP LM (avoid conflicts).");

    std::ostringstream runTag;
    runTag << "uav" << numUAVs << "_gnd" << numGroundUes << "_gnb" << numGnbs << "_cellLoad" << maxUesPerCell;

    // Base output folder for this run
    ns3_dir = "results/nr/uav/" + runTag.str() + "/";

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
    Config::SetDefault("ns3::NrGnbRrc::MaxUesPerCell", UintegerValue(maxUesPerCell));

    int channelUpdatePeriod = 100;
    int channelConditionUpdatePeriod = 200;
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(channelUpdatePeriod)));

    //Config::SetDefault("ns3::NrGnbPhy::TxPower", DoubleValue(43));

    // Create gNB and UAV
    NodeContainer uavNodes;
    NodeContainer groundUeNodes;
    NodeContainer gnbNodes;
    gnbNodes.Create(numGnbs);
    uavNodes.Create(numUAVs);
    groundUeNodes.Create(numGroundUes);
    
    // Create ChannelHelper API
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    // if (isLos)
    // {
    //     propChannelCondition = "LOS";
    // }

    NodeDistributionScenarioInterface* scenario{nullptr};
    std::string propScenario = "UMa"; //Urban Macro
    bool enableShadowing = false;
    //std::string propChannelCondition = "LOS";
    NS_ABORT_MSG_UNLESS(
        propScenario == "UMa",
        "Unsupported scenario " << scenario << ". Supported valuavs: UMa, RMa");
    // Configure the factories for the channel creation
    channelHelper->ConfigureFactories(propScenario, "Default");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));
    // if (!isLos)
    // {
    channelHelper->SetChannelConditionModelAttribute(
            "UpdatePeriod",
            TimeValue(MilliSeconds(channelConditionUpdatePeriod)));
    // }
    ObjectFactory distanceBasedChannelFactory;
    
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

    // Both DL and UL AMC will have the same model behind.
    nrHelper->SetGnbDlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));
    nrHelper->SetGnbUlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    double txPower = 38;
    double ueTxPower = 23;
    bool enableHarqRetx = false;

    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(enableHarqRetx));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPower));
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


    // ---- TDD single-carrier setup (ONE band, ONE BWP) ----
    bool enableFading = true;
    uint8_t bandMask = NrChannelHelper::INIT_PROPAGATION |
                    (enableFading ? NrChannelHelper::INIT_FADING : 0);

    double centralFrequency = 4e9;
    double bandBw = 20e6;

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf conf(centralFrequency, bandBw, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(conf);

    std::vector<std::reference_wrapper<OperationBandInfo>> bands;
    bands.emplace_back(std::ref(band));

    channelHelper->AssignChannelsToBands(bands, bandMask);

    // BWP 0
    BandwidthPartInfoPtrVector Bwps = CcBwpCreator::GetAllBwps(bands);


    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
        TypeIdValue(QuasiOmniDirectPathBeamforming::GetTypeId()));
    if (enableFading)
    {
        nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    }
    
    //The network interface installed on the node (e.g., 5G modem)
    NetDeviceContainer gnbNrDevs;
    NetDeviceContainer uavNrDevs; 
    NetDeviceContainer groundNrDevs;

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

    install_mobility(remoteHostContainer, gnbNodes, uavNodes, groundUeNodes);

    gnbNrDevs = nrHelper->InstallGnbDevice(gnbNodes, Bwps);
    uavNrDevs = nrHelper->InstallUeDevice(uavNodes, Bwps);
    groundNrDevs = nrHelper->InstallUeDevice(groundUeNodes, Bwps); 

    // -------- Role map: IMSI -> UAV/GND --------
    g_imsiRole.clear();

    for (uint32_t i = 0; i < uavNrDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(uavNrDevs.Get(i));
        NS_ABORT_MSG_IF(!ue, "uavNrDevs[" << i << "] is not NrUeNetDevice");
        g_imsiRole[ue->GetImsi()] = "UAV";
    }

    for (uint32_t i = 0; i < groundNrDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(groundNrDevs.Get(i));
        NS_ABORT_MSG_IF(!ue, "groundNrDevs[" << i << "] is not NrUeNetDevice");
        g_imsiRole[ue->GetImsi()] = "GND";
    }

    // TDD pattern (7 DL, 3 UL)
    static const std::string tddPattern = "DL|DL|DL|DL|DL|DL|DL|UL|UL|UL|";

    for (uint32_t gnbIndex = 0; gnbIndex < gnbNrDevs.GetN(); ++gnbIndex)
    {
        Ptr<NetDevice> gnbDev = gnbNrDevs.Get(gnbIndex);
        Ptr<NrGnbPhy> gnbPhy = nrHelper->GetGnbPhy(gnbDev, 0); // only BWP0 exists now
        gnbPhy->SetAttribute("Pattern", StringValue(tddPattern));
    }


    // Apply final configuration after set patterns + output links
    for (auto it = gnbNrDevs.Begin(); it != gnbNrDevs.End(); ++it)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(*it);
        if (gnb) gnb->UpdateConfig();
    }

    for (auto it = uavNrDevs.Begin(); it != uavNrDevs.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue) ue->UpdateConfig();
    }

    for (auto it = groundNrDevs.Begin(); it != groundNrDevs.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue) ue->UpdateConfig();
    }

    //nrHelper->ConfigureFhControl(gnbNrDevs);
    nrHelper->SetAttribute("InitMaxUesPerCell", UintegerValue(maxUesPerCell));
    nrHelper->SetAttribute("InitMinRsrpDbm",   DoubleValue(-120.0));
    nrHelper->SetAttribute("InitRetryInterval", TimeValue(Seconds(2.0)));
    //initial attach helper
    nrHelper->AttachToMaxRsrpGnb(uavNrDevs, gnbNrDevs);
    //nrHelper->AttachToMaxRsrpGnb(groundNrDevs, gnbNrDevs);
    Time tGroundAttach = Seconds(groundAttachDelay);

    Simulator::Schedule(tGroundAttach, [nrHelper, groundNrDevs, gnbNrDevs]() {
        nrHelper->AttachToMaxRsrpGnb(groundNrDevs, gnbNrDevs); // public container overload
    });

    nrHelper->AddX2Interface(gnbNodes);

    internet.Install(uavNodes);
    internet.Install(groundUeNodes);
    Ipv4InterfaceContainer ueIpIface;
    Ipv4InterfaceContainer groundIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(uavNrDevs));
    groundIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNrDevs));

    // -------- IP -> IMSI map to identify UAVs and Ground UEs --------
    g_ipToImsi.clear();

    // UAV IPs
    for (uint32_t i = 0; i < uavNrDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(uavNrDevs.Get(i));
        NS_ABORT_MSG_IF(!ue, "uavNrDevs[" << i << "] not NrUeNetDevice");
        Ipv4Address ip = ueIpIface.GetAddress(i);
        g_ipToImsi[ip.Get()] = ue->GetImsi();
    }

    // Ground IPs
    for (uint32_t i = 0; i < groundNrDevs.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(groundNrDevs.Get(i));
        NS_ABORT_MSG_IF(!ue, "groundNrDevs[" << i << "] not NrUeNetDevice");
        Ipv4Address ip = groundIpIface.GetAddress(i);
        g_ipToImsi[ip.Get()] = ue->GetImsi();
    }

    for (uint32_t u = 0; u < uavNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = uavNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }
    NS_LOG_UNCOND(std::string("Test"));
    
    for (uint32_t u = 0; u < groundUeNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = groundUeNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // Install and start applications on UAVs and remote host
    uint16_t basePort = 10000;
    ApplicationContainer remoteApps;
    ApplicationContainer uavApps;

    Ptr<RandomVariableStream> onTimeRv = CreateObject<UniformRandomVariable>();
    onTimeRv->SetAttribute("Min", DoubleValue(0.1));
    onTimeRv->SetAttribute("Max", DoubleValue(0.5));
    Ptr<RandomVariableStream> offTimeRv = CreateObject<UniformRandomVariable>();
    offTimeRv->SetAttribute("Min", DoubleValue(0.1));
    offTimeRv->SetAttribute("Max", DoubleValue(0.5));

    for (uint16_t i = 0; i < uavNodes.GetN(); i++)
    {
        uint16_t port = basePort * (i + 1);

        PacketSinkHelper dlPacketSinkHelper("ns3::UdpSocketFactory",
                                            InetSocketAddress(Ipv4Address::GetAny(), port));
        uavApps.Add(dlPacketSinkHelper.Install(uavNodes.Get(i)));
        // uavApps.Get(i)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));

        Ptr<OnOffApplication> streamingServer = CreateObject<OnOffApplication>();
        remoteApps.Add(streamingServer);
        streamingServer->SetAttribute("Remote",
                                      AddressValue(InetSocketAddress(ueIpIface.GetAddress(i), port)));
        // streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("3000000bps")));
        streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("500kbps"))); // try 0.5–1 Mbps
        streamingServer->SetAttribute("PacketSize", UintegerValue(1500));
        streamingServer->SetAttribute("OnTime", PointerValue(onTimeRv));
        streamingServer->SetAttribute("OffTime", PointerValue(offTimeRv));

        remoteHost->AddApplication(streamingServer);
        // streamingServer->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
    }

    remoteApps.Start(Seconds(2));
    remoteApps.Stop(simTime + Seconds(10));
    uavApps.Start(Seconds(1));
    uavApps.Stop(simTime + Seconds(15));

    //ground UEs traffic (same remote host, different ports)

    uint16_t groundBasePort = 20000;              // NEW port base (avoid collision)
    ApplicationContainer groundApps;              // sinks on ground UEs
    ApplicationContainer groundRemoteApps;        // OnOff on remote host

    for (uint16_t i = 0; i < groundUeNodes.GetN(); i++)
    {
        uint16_t port = groundBasePort + i;

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
        groundApps.Add(dlSink.Install(groundUeNodes.Get(i)));

        Ptr<OnOffApplication> src = CreateObject<OnOffApplication>();
        groundRemoteApps.Add(src);

        src->SetAttribute("Remote",
            AddressValue(InetSocketAddress(groundIpIface.GetAddress(i), port)));
        src->SetAttribute("DataRate", DataRateValue(DataRate("300kbps"))); // light load
        src->SetAttribute("PacketSize", UintegerValue(1000));
        src->SetAttribute("OnTime", PointerValue(onTimeRv));
        src->SetAttribute("OffTime", PointerValue(offTimeRv));

        remoteHost->AddApplication(src);
    }

    // groundRemoteApps.Start(Seconds(2));
    // groundApps.Start(Seconds(1));
    groundApps.Start(Seconds(1)); // sinks can start early, harmless
    groundRemoteApps.Start(tGroundAttach + Seconds(0.5)); // send after attach

    groundRemoteApps.Stop(simTime + Seconds(10));
    groundApps.Stop(simTime + Seconds(15));

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

        defaultLm->SetAttribute("MaxUesPerCell", UintegerValue(maxUesPerCell)); //default 10
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

        for (uint32_t idx = 0; idx < uavNodes.GetN(); idx++)
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
            uavApps.Get(idx)->TraceConnectWithoutContext("Rx",
                                                        MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));
          
            //The UAV’s physical layer (NrUePhy) periodically measures:RSRP (signal strength),
                                                                    //RSRQ (signal quality),
                                                                    //SINR (interference/noise level).
            for (uint32_t netDevIdx = 0; netDevIdx < uavNodes.Get(idx)->GetNDevices(); netDevIdx++)
            {
                Ptr<NrUeNetDevice> nrUeDevice =
                    uavNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();
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

            nrUeTerminator->Attach(uavNodes.Get(idx));
            Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
        }

        // Ground UE -> RIC (reporters + terminator)
        for (uint32_t idx = 0; idx < groundUeNodes.GetN(); idx++)
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
            groundRemoteApps.Get(idx)->TraceConnectWithoutContext(
                "Tx", MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
            groundApps.Get(idx)->TraceConnectWithoutContext(
                "Rx", MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));

            // RSRP/RSRQ measurements from the ground UE PHY
            for (uint32_t netDevIdx = 0; netDevIdx < groundUeNodes.Get(idx)->GetNDevices(); netDevIdx++)
            {
                Ptr<NrUeNetDevice> nrUeDevice =
                    groundUeNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();
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

            // nrUeTerminator->Attach(groundUeNodes.Get(idx));
            // Simulator::Schedule(Seconds(2), &OranE2NodeTerminatorNrUe::Activate, nrUeTerminator);
            nrUeTerminator->Attach(groundUeNodes.Get(idx));

            // Activate E2 only after the UE is attached (small guard offset)
            Simulator::Schedule(tGroundAttach + Seconds(1.0),
                                &OranE2NodeTerminatorNrUe::Activate,
                                nrUeTerminator);
        }

        for (uint32_t idx = 0; idx < gnbNodes.GetN(); idx++)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrCellLoad> nrCellLoadReporter = CreateObject<OranReporterNrCellLoad>();
            Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));

            auto dev = gnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();
            auto mac = dev->GetMac(0); // Use BWP 0 (DL BWP)
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
            nrGnbTerminator->Attach(gnbNodes.Get(idx));
            nrGnbTerminator->SetAttribute("TransmissionDelayRv",
                                          StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                      std::to_string(txDelay) + "]"));
            Simulator::Schedule(Seconds(1.5), &OranE2NodeTerminatorNrGnb::Activate, nrGnbTerminator);
        }

    }
    // ORAN END

    // Erase the trace files if they exist
    std::ofstream posOutFile(s_positionTraceFile, std::ios_base::trunc);
    posOutFile.close();
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::trunc);
    hoOutFile.close();

    // Start tracing node locations
    Simulator::Schedule(Seconds(1), &TracePositions, uavNodes);

    // Connect to handover trace so we know when a handover is successfully performed
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkGnb));

    Ptr<OutputStreamWrapper> rsrpRsrqSinrTraceStream =
        Create<OutputStreamWrapper>(ns3_dir + "rsrp-trace.tr", std::ios::out);
    for (NetDeviceContainer::Iterator it = uavNrDevs.Begin(); it != uavNrDevs.End(); ++it)
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
    user_ip.resize(numUAVs);
    user_delay.assign(numUAVs, 0);
    user_jitter.assign(numUAVs, 0);
    user_throughput.assign(numUAVs, 0);
    user_pdr.assign(numUAVs, 0);

    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowHelper;

    flowHelper.Install(remoteHost);
    NodeContainer allUes;
    allUes.Add(uavNodes);
    allUes.Add(groundUeNodes);
    flowMonitor = flowHelper.Install(allUes);

    std::ofstream qos_vs_time;
    qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::trunc);
    qos_vs_time << "Time,UE,Delay,Jitter,Throughput,PDR" << std::endl;
    Simulator::Schedule(management_interval, ThroughputMonitor, &flowHelper, flowMonitor);


    // populate user ip map
    for (uint32_t i = 0; i < uavNodes.GetN(); i++)
    {
        Ptr<Ipv4> remoteIpv4 = uavNodes.Get(i)->GetObject<Ipv4>();
        Ipv4Address remoteIpAddr = remoteIpv4->GetAddress(1, 0).GetLocal();
        user_ip[i] = remoteIpAddr;
    }

    // ---- NR Radio Environment Map ----
    Ptr<NrRadioEnvironmentMapHelper> remHelper = CreateObject<NrRadioEnvironmentMapHelper>();
    if (remMode)
    {
        remHelper->SetAttribute("SimTag", StringValue("start"));
        remHelper->SetAttribute("XMin", DoubleValue(-600.0));
        remHelper->SetAttribute("XMax", DoubleValue(600.0));
        remHelper->SetAttribute("XRes", UintegerValue(500));
        remHelper->SetAttribute("YMin", DoubleValue(-200.0));
        remHelper->SetAttribute("YMax", DoubleValue(1500.0));
        remHelper->SetAttribute("YRes", UintegerValue(500));
        remHelper->SetAttribute("Z", DoubleValue(1.0));

        Ptr<NetDevice> rrdDevice = uavNrDevs.Get(0); // UAV0 as receiver
        uint8_t bwpId = 0; // DL BWP

        // Create REM at t=10s (no static Install() API in your helper)
        Simulator::Schedule(Seconds(10.0),
                            &NrRadioEnvironmentMapHelper::CreateRem,
                            remHelper,
                            gnbNrDevs,  // transmitters
                            rrdDevice,  // receiver
                            bwpId);
    }

    std::ofstream flowOutFile(s_flowStatTraceFile, std::ios_base::trunc);
    flowOutFile << "Time,Role,IMSI\n";
    flowOutFile.close();

    // Tell the simulator how long to run
    Simulator::Stop(simTime + Seconds(15));
    Simulator::Run();

    if (g_oldClogBuf) { std::clog.rdbuf(g_oldClogBuf); }
    if (g_nsLogFile.is_open()) { g_nsLogFile.close(); }

    // if (g_oldCoutBuf) { std::cout.rdbuf(g_oldCoutBuf); }
    // if (g_uncondFile.is_open()) { g_uncondFile.close(); }   

    Simulator::Destroy();
    return 0;
}
