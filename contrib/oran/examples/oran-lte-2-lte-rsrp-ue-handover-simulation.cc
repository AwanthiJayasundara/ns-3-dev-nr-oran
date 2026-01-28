
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/oran-module.h"
#include "ns3/point-to-point-module.h"

//Library
#include <fstream>
#include <vector>


#include <math.h>
#include <iomanip>
#include <cmath>
//NS3
#include "ns3/ipv4-address.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/log.h"

using namespace ns3;

/**
 * Example of ORAN-driven LTE multi-cell handover with QoS monitoring.
 *
 * The scenario consists of 200 LTE UEs moving randomly across 7 macro eNBs.
 * Each UE receives downlink UDP traffic from a remote host, while the network
 * operates with a 20 MHz DL / 10 MHz UL LTE configuration and round-robin
 * scheduling at the eNBs.
 *
 * The UEs report their location, serving Cell ID, and RSRP/RSRQ measurements
 * to a Near-RT RIC via O-RAN E2 terminators. In the RIC, an RSRP-based Logic
 * Module (LM) periodically evaluates these measurements and, when appropriate,
 * issues LTE-to-LTE handover commands. eNB-side cell load is also reported
 * through ORAN, and FlowMonitor is used to track per-UE QoS (delay, jitter,
 * throughput, and PDR) over time.
 *
 * This example demonstrates how to integrate ORAN reporting, RSRP-based
 * handover control, and QoS monitoring in a large-scale LTE multi-cell
 * scenario excute with max 150  mobile UEs.
 */


const static float ENB_HEIGHT = 30;

//Varibles
uint32_t numUEs = 100;
uint32_t numMacroCells = 7;

// Metrics collection interval
Time management_interval = Seconds(10);

// UEs ips vector
std::vector<Ipv4Address> user_ip;

// Vectors with the most recent metrics for each EU
std::vector<double> user_delay;
std::vector<double> user_jitter;
std::vector<double> user_throughput;
std::vector<double> user_pdr;


static std::string s_trafficTraceFile = "results/lte/traffic-rsrp-trace.tr";
static std::string s_positionTraceFile = "results/lte/position-rsrp-trace.tr";
static std::string s_handoverTraceFile = "results/lte/handover-rsrp-trace.tr";
static std::string ns3_dir = "results/lte/";

// Function that will save the traces of RX'd packets
void
RxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);

    std::ofstream rxOutFile(s_trafficTraceFile, std::ios_base::app);
    rxOutFile << Simulator::Now().GetSeconds() << " " << ueId << " RX " << p->GetSize()
              << std::endl;
}

// Function that will save the traces of TX'd packets
void
TxTrace(Ptr<const Packet> p, const Address& from, const Address& to)
{
    uint16_t ueId = (InetSocketAddress::ConvertFrom(to).GetPort() / 1000);

    std::ofstream rxOutFile(s_trafficTraceFile, std::ios_base::app);
    rxOutFile << Simulator::Now().GetSeconds() << " " << ueId << " TX " << p->GetSize()
              << std::endl;
}

// Helper function that returns the UE id==
// associated with a specific IP
int get_user_id_from_ipv4(Ipv4Address ip)
{

  for (uint32_t i = 0; i < numUEs; i++)
  {
    if (user_ip[i] == ip)
    {
      return i;
    }
  }
  return -1;
}

// Function that calculates qos metrics periodically
// using FlowMonitor
void ThroughputMonitor(FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon)
{
  uint32_t LostPacketsum = 0;
  float PDR, PLR, Delay, Jitter, Throughput;
  auto flowStats = flowMon->GetFlowStats();

  // Swap for UEs subnet
  auto ue_network = Ipv4Address("7.0.0.0");
  auto ue_network_mask = Ipv4Mask("255.0.0.0");

  Ptr<Ipv4FlowClassifier> classing =
      DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());

	//cell_throughput.assign(numCells, 0);
  for (auto stats : flowStats)
  {
    // disconsider old flows by checking if last packet was sent or received
    // before this round management interval
    if (Simulator::Now() - stats.second.timeLastTxPacket > management_interval &&
		Simulator::Now() - stats.second.timeLastRxPacket > management_interval)
    {
      continue;
    }

    // find flow characteristics
    Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(stats.first);

	if(!ue_network_mask.IsMatch(ue_network, fiveTuple.destinationAddress))
		continue;

	int rx_packets = stats.second.rxPackets;
	int tx_packets = stats.second.txPackets;
	tx_packets = tx_packets>=rx_packets ? tx_packets:rx_packets;
    PDR = (double)(100 * rx_packets) / (tx_packets);
    LostPacketsum = (double)(tx_packets) - (rx_packets);
    PLR = (double)(LostPacketsum * 100) / tx_packets;
    Delay = (stats.second.delaySum.GetSeconds()) / (rx_packets);
	Jitter = (stats.second.jitterSum.GetSeconds()) / (rx_packets - 1);
    Throughput = stats.second.rxBytes * 8.0 /
                 (stats.second.timeLastRxPacket.GetSeconds() -
                  stats.second.timeFirstTxPacket.GetSeconds()) /
                 1024 / 1024;

    std::cout << "Flow ID     : " << stats.first << " ; "
              << fiveTuple.sourceAddress << " -----> "
              << fiveTuple.destinationAddress << std::endl;
    std::cout << "Tx Packets = " << tx_packets << std::endl;
    std::cout << "Rx Packets = " << rx_packets << std::endl;
    std::cout << "Lost Packets = "
              << (tx_packets) - (rx_packets)
              << std::endl;
    std::cout << "Packets Delivery Ratio (PDR) = " << PDR << "%" << std::endl;
    std::cout << "Packets Lost Ratio (PLR) = " << PLR << "%" << std::endl;
    std::cout << "Delay = " << Delay << " Seconds" << std::endl;
    std::cout << "Total Duration    : "
              << stats.second.timeLastRxPacket.GetSeconds() -
                     stats.second.timeFirstTxPacket.GetSeconds()
              << " Seconds" << std::endl;
    std::cout << "Last Received Packet  : "
              << stats.second.timeLastRxPacket.GetSeconds() << " Seconds"
              << std::endl;
    std::cout << "Throughput: " << Throughput << " mbps" << std::endl;
    std::cout << "Throughput in bytes: " << Throughput * 1024 * 1024 / 8 << " Bps" << std::endl;
    // receiving node, used to catch user downlink traffic
    std::cout << "target node = " << get_user_id_from_ipv4(
            fiveTuple.destinationAddress) << std::endl;
    std::cout << "-------------------------------------------------------------"
                 "--------------"
              << std::endl;

    // received id will be -1 in case it is not a mobile user
    int receiver_id = get_user_id_from_ipv4(fiveTuple.destinationAddress);
    if (receiver_id != -1)
    {
      user_delay[receiver_id] = Delay;
      user_jitter[receiver_id] = Jitter;
      user_throughput[receiver_id] = Throughput;
      user_pdr[receiver_id] = PDR;
      //cell_throughput[getCellId(receiver_id)] += Throughput;
    }
  }

	std::ofstream qos_vs_time;
	qos_vs_time.open("results/lte/rsrp-qos-vs-time.txt", std::ofstream::out | std::ofstream::app);
  for(uint32_t ue=0; ue < numUEs; ++ue)
  {
	qos_vs_time << Simulator::Now().GetSeconds() << ","
		<< ue << ","
		//<< user_requests[ue].get_name() << ","
		<< user_delay[ue] << ","
		<< user_jitter[ue] << ","
		<< user_throughput[ue] << ","
		<< user_pdr[ue] << std::endl;
  }

  flowMon->ResetAllStats();

  // schedule itself in 1sec
  Simulator::Schedule(management_interval, ThroughputMonitor, fmhelper,
                      flowMon);
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
NotifyHandoverEndOkEnb(std::string context, uint64_t imsi, uint16_t cellid, uint16_t rnti)
{
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::app);
    hoOutFile << Simulator::Now().GetSeconds() << " " << imsi << " " << cellid << " " << rnti
              << std::endl;
}

NS_LOG_COMPONENT_DEFINE("OranLte2LteRsrpUeHandoverSimulation");

void install_mobility(NodeContainer staticNodes, NodeContainer mcNodes, NodeContainer ueNodes)
{
	Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
	allocator->Add (Vector(0, 0, 0));
	MobilityHelper staticNodesHelper;
	staticNodesHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	staticNodesHelper.SetPositionAllocator (allocator);
	staticNodesHelper.Install(staticNodes);

    // Install Mobility Model for eNBs
	Ptr<ListPositionAllocator> mcPosition = CreateObject<ListPositionAllocator> ();
	//(0,0)(433,250)(433,750)(0,1000)(-433,750)(-433,250)(0,500)
	mcPosition->Add (Vector(0, 0, ENB_HEIGHT));
	mcPosition->Add (Vector(433, 250, ENB_HEIGHT));
	mcPosition->Add (Vector(433, 750, ENB_HEIGHT));
	mcPosition->Add (Vector(0, 1000, ENB_HEIGHT));
	mcPosition->Add (Vector(-433, 750, ENB_HEIGHT));
	mcPosition->Add (Vector(-433, 250, ENB_HEIGHT));
	mcPosition->Add (Vector(0, 500, ENB_HEIGHT));
	MobilityHelper mcHelper;
	mcHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	mcHelper.SetPositionAllocator(mcPosition);
	mcHelper.Install(mcNodes);

    // Install Mobility Model for UEs
	MobilityHelper ueHelper;
	ueHelper.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
								"Bounds", StringValue ("-500|500|-100|1100"),
								"Speed", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.5]"),
								"Pause", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
	ueHelper.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
								 "X", StringValue ("ns3::UniformRandomVariable[Min=-450.0|Max=450.0]"),
								 "Y", StringValue ("ns3::UniformRandomVariable[Min=-50.0|Max=1050.0]"));
	ueHelper.Install (ueNodes);
}

bool IsTopLevelSourceDir (std::string path)
{
	bool haveVersion = false;
	bool haveLicense = false;

	//
	// If there's a file named VERSION and a file named LICENSE in this
	// directory, we assume it's our top level source directory.
	//

	std::list<std::string> files = SystemPath::ReadFiles (path);
	for (std::list<std::string>::const_iterator i = files.begin (); i != files.end (); ++i)
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

std::string GetTopLevelSourceDir ()
{
	std::string self = SystemPath::FindSelfDirectory ();
	std::list<std::string> elements = SystemPath::Split (self);
	while (!elements.empty ())
	{
		std::string path = SystemPath::Join (elements.begin (), elements.end ());
		if (IsTopLevelSourceDir (path))
		{
			return path + "/";
		}
		elements.pop_back ();
	}
	NS_FATAL_ERROR ("Could not find source directory from self=" << self);
	return "";
}

int
main(int argc, char* argv[])
{
    bool verbose = false;
    bool useOran = false;
    bool useOnnx = false;
    bool useTorch = false;
    bool useRsrp = true;
    double lmQueryInterval = 1;
    double txDelay = 0.1;
	bool remMode = false; // [0]: REM disabled; [1]: generate REM
	int32_t remRbId = -1;
    std::string handoverAlgorithm = "ns3::NoOpHandoverAlgorithm";
    Time simTime = Seconds(100);
    std::string dbFileName = "oran-repository-lte.db";

    CommandLine cmd;
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.AddValue("use-oran", "Indicates whether ORAN should be used or not", useOran);
    cmd.AddValue("use-onnx-lm", "Indicates whether the ONNX LM should be used or not", useOnnx);
    cmd.AddValue("use-torch-lm",
                 "Indicates whether the PyTorch LM should be used or not",
                 useTorch);
    cmd.AddValue("use-rsrp-lm",
                 "Indicates whether the rsrp LM should be used or not",
                 useRsrp);
    cmd.AddValue("sim-time", "The duration for which traffic should flow", simTime);
    cmd.AddValue("lm-query-interval", "The LM query interval", lmQueryInterval);
    cmd.AddValue("tx-delay", "The E2 termiantor's transmission delay", txDelay);
    cmd.AddValue("handover-algorithm",
                 "Specify which handover algorithm to use",
                 handoverAlgorithm);
    cmd.AddValue("db-file", "Specify the DB file to create", dbFileName);
    cmd.AddValue("traffic-trace-file",
                 "Specify the traffic trace file to create",
                 s_trafficTraceFile);
    cmd.AddValue("position-trace-file",
                 "Specify the position trace file to create",
                 s_positionTraceFile);
    cmd.AddValue("handover-trace-file",
                 "Specify the handover trace file to create",
                 s_handoverTraceFile);
	cmd.AddValue("num-ues", "Number of UEs", numUEs);
	cmd.AddValue("rem-mode", "Generate radio environment map", remMode);
	cmd.AddValue("rem-rb-id", "RB id", remRbId);
    cmd.Parse(argc, argv);

    // NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
    //                 "Cannot use ML LM or rsrp LM without enabling O-RAN.");
    // NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1,
    //                 "Cannot use more than one LM simultaneously.");
    // NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NoOpHandoverAlgorithm" &&
    //                     (useOnnx || useTorch || useRsrp),
    //                 "Cannot use non-noop handover algorithm with ML LM or rsrp LM.");

	ns3_dir = GetTopLevelSourceDir();

    // Increase the buffer size to accomodate the application demand
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(100 * 1024));
    // Disabled to prevent the automatic cell reselection when signal quality is bad.
    Config::SetDefault("ns3::LteUePhy::EnableRlfDetection", BooleanValue(false));

    // Configure the LTE parameters (pathloss, bandwidth, scheduler)
	Config::SetDefault("ns3::LteEnbPhy::TxPower", DoubleValue(43));
    Ptr<LteHelper> lteHelper = CreateObject<LteHelper>();
    lteHelper->SetAttribute("PathlossModel", StringValue("ns3::Cost231PropagationLossModel"));
    lteHelper->SetEnbDeviceAttribute("DlBandwidth", UintegerValue(100));
    lteHelper->SetEnbDeviceAttribute("UlBandwidth", UintegerValue(50));
    lteHelper->SetSchedulerType("ns3::RrFfMacScheduler");
	//lteHelper->SetSchedulerAttribute("UlCqiFilter", EnumValue(FfMacScheduler::PUSCH_UL_CQI));
    lteHelper->SetSchedulerAttribute("HarqEnabled", BooleanValue(true));
    
    
    if (useOran)
    {
        // O-RAN mode: disable the built-in handover
        lteHelper->SetHandoverAlgorithmType(handoverAlgorithm);
    }
    else
    {
        // Classic mode: use RSRP-based handover
        // (you can also choose A3, A2/A4, etc., depending on your targets)
        lteHelper->SetHandoverAlgorithmType("ns3::A3RsrpHandoverAlgorithm");
        lteHelper->SetHandoverAlgorithmAttribute ("Hysteresis", DoubleValue (1.0));
        lteHelper->SetHandoverAlgorithmAttribute ("TimeToTrigger", TimeValue (MilliSeconds (64)));
    }

    //lteHelper->SetHandoverAlgorithmType(handoverAlgorithm);

    // Deploy the EPC
    Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper>();
    lteHelper->SetEpcHelper(epcHelper);

	//lteHelper->SetFfrAlgorithmType("ns3::LteFrHardAlgorithm");

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

    // Create eNB and UE
    NodeContainer ueNodes;
    NodeContainer enbNodes;
    enbNodes.Create(numMacroCells);
    ueNodes.Create(numUEs);

	install_mobility(remoteHostContainer, enbNodes, ueNodes);

    NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice(enbNodes);
    NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice(ueNodes);

    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueLteDevs));
    // Assign IP address to UEs, and install applications
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        // Set the default gateway for the UE
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

	lteHelper->AttachToClosestEnb(ueLteDevs, enbLteDevs);

    lteHelper->AddX2Interface(enbNodes);

    // Install and start applications on UEs and remote host
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
        // Enable the tracing of RX packets
        ueApps.Get(i)->TraceConnectWithoutContext("RxWithAddresses", MakeCallback(&RxTrace));

        Ptr<OnOffApplication> streamingServer = CreateObject<OnOffApplication>();
        remoteApps.Add(streamingServer);
        // Attributes
        streamingServer->SetAttribute(
            "Remote",
            AddressValue(InetSocketAddress(ueIpIface.GetAddress(i), port)));
        streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("3000000bps")));
        streamingServer->SetAttribute("PacketSize", UintegerValue(1500));
        streamingServer->SetAttribute("OnTime", PointerValue(onTimeRv));
        streamingServer->SetAttribute("OffTime", PointerValue(offTimeRv));

        remoteHost->AddApplication(streamingServer);
        streamingServer->TraceConnectWithoutContext("TxWithAddresses", MakeCallback(&TxTrace));
    }

    // Inidcate when to start streaming
    remoteApps.Start(Seconds(2));
    // Indicate when to stop streaming
    remoteApps.Stop(simTime + Seconds(10));

    // UE applications start listening
    ueApps.Start(Seconds(1));
    // UE applications stop listening
    ueApps.Stop(simTime + Seconds(15));

    // ORAN BEGIN
    if (useOran == true)
    {
        // ---- added: safer RIC / E2 activation times to avoid racing with initial RRC reconfiguration
        const double ricStart   = 2.0;   // start the Near-RT RIC after initial attach settles
        const double e2StartUe  = 3.0;   // activate UE E2 terminators after the RIC is up
        const double e2StartEnb = 2.5;   // activate eNB E2 terminators after the RIC is up

        // ---- keep: DB reset
        if (!dbFileName.empty())
        {
            std::remove(dbFileName.c_str());
        }

        TypeId defaultLmTid = TypeId::LookupByName("ns3::OranLmNoop");

        Ptr<OranLm> defaultLm = nullptr;
        Ptr<OranDataRepository> dataRepository = CreateObject<OranDataRepositorySqlite>();
        Ptr<OranCmm> cmm = CreateObject<OranCmmHandover>();
        Ptr<OranNearRtRic> nearRtRic = CreateObject<OranNearRtRic>();
        Ptr<OranNearRtRicE2Terminator> nearRtRicE2Terminator =
            CreateObject<OranNearRtRicE2Terminator>();

        if (useOnnx == true)
        {
            NS_ABORT_MSG_IF(
                !TypeId::LookupByNameFailSafe("ns3::OranLmLte2LteOnnxHandover", &defaultLmTid),
                "ONNX LM not found. Were the ONNX headers and libraries found during the config "
                "operation?");
        }
        else if (useTorch == true)
        {
            NS_ABORT_MSG_IF(
                !TypeId::LookupByNameFailSafe("ns3::OranLmLte2LteTorchHandover", &defaultLmTid),
                "Torch LM not found. Were the Torch headers and libraries found during the config "
                "operation?");
        }
        else if (useRsrp == true)
        {
            defaultLmTid = TypeId::LookupByName("ns3::OranLmLte2LteRsrpHandover");
        }

        if (useTorch)
        {
            auto path = StringValue(ns3_dir + "saved_trained_classification_pytorch.pt");
            Config::SetDefault("ns3::OranLmLte2LteTorchHandover::TorchModelPath", StringValue(path));
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

        // ---- added: be gentler with early HO decisions (if user passed a tiny value, bump it)
        double effectiveLmInterval = lmQueryInterval;
        if (effectiveLmInterval < 2.0)
        {
            NS_LOG_WARN("LmQueryInterval too small for large scenarios; bumping to 5s to avoid "
                        "early HO during initial RRC config.");
            effectiveLmInterval = 5.0;
        }
        nearRtRic->SetAttribute("LmQueryInterval", TimeValue(Seconds(effectiveLmInterval)));

        nearRtRic->SetAttribute("ConflictMitigationModule", PointerValue(cmm));

        // ---- changed: start the RIC after attach settles
        Simulator::Schedule(Seconds(ricStart), &OranNearRtRic::Start, nearRtRic);

        // ---- UE E2 nodes
        for (uint32_t idx = 0; idx < ueNodes.GetN(); idx++)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterLteUeCellInfo> lteUeCellInfoReporter =
                CreateObject<OranReporterLteUeCellInfo>();
            Ptr<OranReporterAppLoss> appLossReporter = CreateObject<OranReporterAppLoss>();
            Ptr<OranReporterLteUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterLteUeRsrpRsrq>();
            Ptr<OranE2NodeTerminatorLteUe> lteUeTerminator =
                CreateObject<OranE2NodeTerminatorLteUe>();

            locationReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
            lteUeCellInfoReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
            rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));

            for (uint32_t netDevIdx = 0; netDevIdx < ueNodes.Get(idx)->GetNDevices(); netDevIdx++)
            {
                Ptr<LteUeNetDevice> lteUeDevice =
                    ueNodes.Get(idx)->GetDevice(netDevIdx)->GetObject<LteUeNetDevice>();
                if (lteUeDevice)
                {
                    Ptr<LteUePhy> uePhy = lteUeDevice->GetPhy();
                    uePhy->TraceConnectWithoutContext(
                        "ReportUeMeasurements",
                        MakeCallback(&ns3::OranReporterLteUeRsrpRsrq::ReportRsrpRsrq,
                                    rsrpRsrqReporter));
                }
            }

            appLossReporter->SetAttribute("Terminator", PointerValue(lteUeTerminator));
            remoteApps.Get(idx)->TraceConnectWithoutContext(
                "Tx",
                MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
            ueApps.Get(idx)->TraceConnectWithoutContext(
                "Rx",
                MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));

            lteUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            lteUeTerminator->SetAttribute("RegistrationIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            lteUeTerminator->SetAttribute("SendIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));

            lteUeTerminator->AddReporter(locationReporter);
            lteUeTerminator->AddReporter(lteUeCellInfoReporter);
            lteUeTerminator->AddReporter(rsrpRsrqReporter);
            lteUeTerminator->AddReporter(appLossReporter);
            lteUeTerminator->SetAttribute("TransmissionDelayRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(txDelay) + "]"));

            lteUeTerminator->Attach(ueNodes.Get(idx));

            // ---- changed: activate after RIC start
            Simulator::Schedule(Seconds(e2StartUe),
                                &OranE2NodeTerminatorLteUe::Activate, lteUeTerminator);
        }

        // ---- eNB E2 nodes
        for (uint32_t idx = 0; idx < enbNodes.GetN(); idx++)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterLteCellLoad> lteCellLoadReporter =
                CreateObject<OranReporterLteCellLoad>();
            Ptr<OranE2NodeTerminatorLteEnb> lteEnbTerminator =
                CreateObject<OranE2NodeTerminatorLteEnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));
            lteCellLoadReporter->SetAttribute("Terminator", PointerValue(lteEnbTerminator));

            auto dev = enbLteDevs.Get(idx)->GetObject<LteEnbNetDevice>();
            auto mac = dev->GetMac();
            mac->TraceConnectWithoutContext(
                "DlScheduling",
                MakeCallback(&ns3::OranReporterLteCellLoad::DlScheduled, lteCellLoadReporter));

            lteEnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            lteEnbTerminator->SetAttribute("RegistrationIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            lteEnbTerminator->SetAttribute("SendIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));

            lteEnbTerminator->AddReporter(locationReporter);
            lteEnbTerminator->AddReporter(lteCellLoadReporter);
            lteEnbTerminator->Attach(enbNodes.Get(idx));
            lteEnbTerminator->SetAttribute("TransmissionDelayRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(txDelay) + "]"));

            // ---- changed: activate after RIC start
            Simulator::Schedule(Seconds(e2StartEnb),
                                &OranE2NodeTerminatorLteEnb::Activate, lteEnbTerminator);
        }
    }
    // ORAN END

    // Erase the trace files if they exist
    std::ofstream trafficOutFile(s_trafficTraceFile, std::ios_base::trunc);
    trafficOutFile.close();
    std::ofstream posOutFile(s_positionTraceFile, std::ios_base::trunc);
    posOutFile.close();
    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::trunc);
    hoOutFile.close();

    // Start tracing node locations
    Simulator::Schedule(Seconds(1), &TracePositions, ueNodes);

    // Connect to handover trace so we know when a handover is successfully performed
    Config::Connect("/NodeList/*/DeviceList/*/LteEnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkEnb));

    // Simulator::Run()
	user_ip.resize(numUEs);
	user_delay.assign(numUEs, 0);
	user_jitter.assign(numUEs, 0);
	user_throughput.assign(numUEs, 0);
	user_pdr.assign(numUEs, 100);

	Ptr<FlowMonitor> flowMonitor;
	FlowMonitorHelper flowHelper;

	// mudar remoteHost pro nó servidor
	flowHelper.Install(remoteHost);

	// mudar ueNodes pro NodeContainer que contém os UEs
	flowMonitor = flowHelper.Install(ueNodes);

	std::ofstream qos_vs_time;
	qos_vs_time.open("results/lte/rsrp-qos-vs-time.txt", std::ofstream::out | std::ofstream::trunc);
	qos_vs_time << "Time,UE,Delay,Jitter,Throughput,PDR" << std::endl;
	Simulator::Schedule(management_interval, ThroughputMonitor, &flowHelper, flowMonitor);

	// populate user ip map
	for (uint32_t i = 0; i < ueNodes.GetN(); i++)
	{
		Ptr<Ipv4> remoteIpv4 = ueNodes.Get(i)->GetObject<Ipv4>();
		Ipv4Address remoteIpAddr = remoteIpv4->GetAddress(1, 0).GetLocal();
		user_ip[i] = remoteIpAddr;
	}

		Ptr<RadioEnvironmentMapHelper> remHelper = CreateObject<RadioEnvironmentMapHelper> ();
	if (remMode){
		Ptr<SpectrumChannel> dlChannel = lteHelper->GetDownlinkSpectrumChannel ();
		uint32_t dlChannelId = dlChannel->GetId ();
		NS_LOG_INFO ("DL ChannelId: " << dlChannelId);
		remHelper->SetAttribute ("Channel", PointerValue (dlChannel));
		remHelper->SetAttribute ("XMin", DoubleValue (-600.0));
		remHelper->SetAttribute ("XMax", DoubleValue (600.0));
		remHelper->SetAttribute ("XRes", UintegerValue (500));
		remHelper->SetAttribute ("YMin", DoubleValue (-200.0));
		remHelper->SetAttribute ("YMax", DoubleValue (1500.0));
		remHelper->SetAttribute ("YRes", UintegerValue (500));
		remHelper->SetAttribute ("Z", DoubleValue (1.0));
		remHelper->SetAttribute ("Bandwidth", UintegerValue (100));
		remHelper->SetAttribute ("Earfcn", UintegerValue (1300));

		if (remRbId >= 0)
        {
          remHelper->SetAttribute ("UseDataChannel", BooleanValue (true));
          remHelper->SetAttribute ("RbId", IntegerValue (remRbId));
        }

		remHelper->SetAttribute ("OutputFile", StringValue ("rem-start.out"));
		Simulator::Schedule (Seconds (10.0),&RadioEnvironmentMapHelper::Install,remHelper);
	}
    
    /* Enabling Tracing for the simulation scenario */
    // lteHelper->EnablePhyTraces();
    // lteHelper->EnableMacTraces();
    // lteHelper->EnableRlcTraces();
    // lteHelper->EnablePdcpTraces();

    // Tell the simulator how long to run
    Simulator::Stop(simTime + Seconds(20));
    // Run the simulation
    Simulator::Run();
    // Clean up used resources
    Simulator::Destroy();

    return 0;
}
