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
#include "ns3/antenna-model.h"
#include "ns3/channel-condition-model.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/net-device.h"
#include "ns3/random-variable-stream.h"
#include "ns3/spectrum-model.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/spectrum-value.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/uniform-planar-array.h"
#include "ns3/nr-hybrid-sat-epc-helper.h"
#include "ns3/oran-lm-nr-2-nr-rsrp-handover-with-tn-ntn.h"

// NS-3 headers
#include "ns3/flow-monitor-module.h"
#include "ns3/ipv4-address.h"
#include "ns3/log.h"
#include "ns3/system-path.h"

#include "ns3/geocentric-constant-position-mobility-model.h"
#include <memory>

// STL
#include <cmath>
#include <cstdio>  
#include <filesystem>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <set>
#include <map>
#include <algorithm>
#include <complex>
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNrUavXhaulAutonomyExample");

/**
 * Usage example of the ORAN NR models for xHaul-aware UAV autonomy studies.
 *
 * The scenario consists of terrestrial gNBs, UAV gNBs, and two ground UE groups
 * placed around a Dublin reference point. UES1 UEs generate monitored traffic
 * and can be handed over, while UES2 UEs add background load. UAV gNBs are
 * dynamic cell nodes; depending on the deployment mode, they may operate only
 * with terrestrial xHaul or with both terrestrial xHaul and satellite support.
 *
 * Handover mechanism used in this scenario:
 *   1. Initial attachment is performed by 5G-LENA's AttachToMaxRsrpGnb().
 *      Each UE initially attaches to the cell with the strongest measured RSRP,
 *      subject to per-cell capacity and minimum-RSRP constraints.
 *   2. Runtime handover is controlled by the O-RAN Near-RT RIC, not by the
 *      built-in NR A3/A2-A4 handover algorithm. The NR helper is kept on
 *      NrNoOpHandoverAlgorithm so that only the RIC/xApp-like logic issues
 *      handover commands.
 *   3. The default RIC logic module is OranLmNr2NrRsrpHandoverWithTnNtn.
 *      It uses UE-reported RSRP, hysteresis, minimum acceptable RSRP, cell
 *      capacity, TN/NTN cell type, and optional backhaul health information.
 *   4. The CMM is OranCmmHandover, which executes the selected NR-to-NR
 *      handover command after the logic module chooses a target cell.
 *
 * The xHaul/autonomy part is deliberately separated from the basic handover
 * path. The xhaul-autonomy-trace.csv estimates the UAV-to-ground-donor xHaul
 * RSRP, maps it to UAV autonomy modes, and records the active UAV backhaul mode.
 * In TN+UAV+satellite mode, degraded or unreachable xHaul can switch the UAV
 * S1-U route from direct TN backhaul to satellite fallback while leaving the
 * O-RAN/xApp handover decision process unchanged.
 *
 * To see all configurable options, run:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-uav-xhaul-autonomy-example --PrintHelp"
 * ./ns3 run "oran-nr-uav-xhaul-autonomy-example --deployment-mode=tn-uav --sim-time=40 --enable-flow-monitor=1 --enable-position-trace=1 --enable-handover-trace=1 --enable-decision-csv=1"
 * \endcode
 *
 * To run the three commented comparison scenarios, use:
 *
 * \code{.unparsed}
 * bash contrib/oran/examples/run-uav-xhaul-autonomy-scenarios.sh
 * \endcode
 *
 * A basic run command is:
 *
 * \code{.unparsed}
 * ./ns3 run "oran-nr-uav-xhaul-autonomy-example"
 * \endcode
 */

const static float TN_GNB_HEIGHT = 25;


// Variables
uint32_t numGroundUesS1 = 115; // UES1 
uint32_t numGroundUesS2 = 115; // UES2

uint32_t numTnGnbs = 8;
uint32_t numNtnGnbs = 6; // e.g., uav gnbs for ntn area

static const double g_refLat = 53.3498;   // Dublin city centre
static const double g_refLon = -6.2603;

static double TN_GNB_HALF_W_M  = 2500.0;
static double TN_GNB_HALF_H_M  = 1200.0;

static double UAV_AREA_HALF_W_M = 3000.0;
static double UAV_AREA_HALF_H_M = 1500.0;

static double UE_AREA_HALF_W_M  = 3000.0;
static double UE_AREA_HALF_H_M  = 1500.0;
static double g_uavSpeedMps = 10.0;
static double g_uavMissionTargetScale = 1.0;

//uint32_t maxUesPerCell = 20; // ORAN LM parameter: maximum number of UEs per cell (for load-aware handover decisions)
uint32_t maxUesPerCellTn  = 20;
uint32_t maxUesPerCellNtn = 10;

// Metrics collection interval
Time management_interval = Seconds(2);
static double g_mobilityUpdateMs = 200.0;
static double g_positionTraceIntervalSec = 1.0;

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

struct FlowIntervalSnapshot
{
    uint64_t txPackets = 0;
    uint64_t rxPackets = 0;
    uint64_t rxBytes = 0;
    Time delaySum = Seconds(0);
    Time jitterSum = Seconds(0);
};

static std::map<uint32_t, FlowIntervalSnapshot> g_prevFlowStats;
static Time g_lastQosSampleTime = Seconds(0);

// // static std::string s_trafficTraceFile;
static std::string s_ueS1PositionTraceFile;
static std::string s_ueS2PositionTraceFile;
static std::string s_uavPositionTraceFile;
static std::string s_handoverTraceFile;
static std::string s_handoverFailureTraceFile;
static std::string s_flowStatTraceFile;
static std::string s_tnInfrastructureTraceFile;
static std::string ns3_dir;
//fh control trace files
// static std::ofstream g_fhTraceFile;
// static std::ofstream g_airTraceFile;

static std::string s_satBackhaulTraceFile;
static std::string s_xhaulAutonomyTraceFile;

static std::map<uint16_t, double> g_backhaulDlSnrDb;
static std::map<uint16_t, double> g_backhaulUlSnrDb;

static std::set<uint16_t> g_ntnCellIds;
static std::map<std::pair<uint16_t,uint16_t>, double> g_latestRsrp;

static Ptr<OranLmNr2NrRsrpHandoverWithTnNtn> g_rsrpLm = nullptr;
static Ptr<NormalRandomVariable> g_xhaulShadowingRv = CreateObject<NormalRandomVariable>();
static Ptr<NormalRandomVariable> g_xhaulFadingRv = CreateObject<NormalRandomVariable>();

static std::string g_deploymentMode = "tn-uav-satellite";
static double g_satBackhaulMinSnrDb = 0.0;
static bool g_enableOnboardUavRic = true;
static bool g_enableXhaulChannelVariation = false;
static double g_xhaulShadowingStddevDb = 0.0;
static double g_xhaulFadingStddevDb = 0.0;

struct UavAutonomyXappContext
{
    NodeContainer tnGnbs;
    NodeContainer uavs;
    NetDeviceContainer tnGnbNrDevs;
    NetDeviceContainer uavGnbNrDevs;
    Ptr<HybridSatEpcHelper> epcHelper;
    double xhaulTxPowerDbm = 43.0;
    double xhaulFrequencyHz = 4.0e9;
    double xhaulMaxDonorDistanceM = 10000.0;
    double healthyThresholdDbm = -95.0;
    double degradedThresholdDbm = -115.0;
    double degradationStartSec = -1.0;
    double degradationStopSec = -1.0;
    double degradationPenaltyDb = 0.0;
    double e2TxDelaySec = 0.0;
    double e2SendIntervalSec = 0.0;
    double lmQueryIntervalSec = 0.0;
    bool enableXhaulChannelVariation = false;
    double xhaulShadowingStddevDb = 0.0;
    double xhaulFadingStddevDb = 0.0;
    bool enableOnboardUavRic = true;
};

static std::unique_ptr<UavAutonomyXappContext> g_uavAutonomyXappCtx;

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

static void
PrintSimulationProgress(Time interval, Time stopTime)
{
    const double now = Simulator::Now().GetSeconds();
    const double stop = stopTime.GetSeconds();
    const double pct = (stop > 0.0) ? (100.0 * now / stop) : 100.0;

    std::cout << "\r[progress] sim t=" << std::fixed << std::setprecision(2)
              << now << " / " << stop << " s (" << std::setprecision(1)
              << std::min(100.0, pct) << "%)" << std::flush;

    if (Simulator::Now() + interval < stopTime)
    {
        Simulator::Schedule(interval, &PrintSimulationProgress, interval, stopTime);
    }
    else
    {
        Simulator::Schedule(stopTime - Simulator::Now(), []() {
            std::cout << "\r[progress] sim t=" << std::fixed << std::setprecision(2)
                      << Simulator::Now().GetSeconds() << " s (done)" << std::endl;
        });
    }
}

// static std::ofstream g_uncondFile;
// static std::streambuf* g_oldCoutBuf = nullptr;

// Tracing rsrp, rsrq, and sinr rsrq is set to zero for now
void
TraceRsrpRsrqSinr(Ptr<OutputStreamWrapper> stream,
                  uint16_t rnti,
                  uint16_t cellId,
                  double rsrp,
                  double rsrq,
                  bool servingCell,
                  uint8_t componentCarrierId)
{
    g_latestRsrp[{rnti,cellId}] = rsrp;
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
    flowMon->CheckForLostPackets();
    auto flowStats = flowMon->GetFlowStats();
    auto ue_network = Ipv4Address("7.0.0.0");
    auto ue_network_mask = Ipv4Mask("255.0.0.0");
    Ptr<Ipv4FlowClassifier> classing =
        DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());
    Time now = Simulator::Now();
    double t = now.GetSeconds();
    double monitorIntervalSec = g_lastQosSampleTime.IsZero()
                                    ? t
                                    : (now - g_lastQosSampleTime).GetSeconds();

    std::vector<uint64_t> dlTxPackets(numGroundUesS1, 0);
    std::vector<uint64_t> dlRxPackets(numGroundUesS1, 0);
    std::vector<uint64_t> dlRxBytes(numGroundUesS1, 0);
    std::vector<Time> dlDelaySum(numGroundUesS1, Seconds(0));
    std::vector<Time> dlJitterSum(numGroundUesS1, Seconds(0));

    std::vector<uint64_t> ulTxPackets(numGroundUesS1, 0);
    std::vector<uint64_t> ulRxPackets(numGroundUesS1, 0);
    std::vector<uint64_t> ulRxBytes(numGroundUesS1, 0);
    std::vector<Time> ulDelaySum(numGroundUesS1, Seconds(0));
    std::vector<Time> ulJitterSum(numGroundUesS1, Seconds(0));

    for (const auto& stats : flowStats)
    {
        uint32_t flowId = stats.first;
        const auto& st = stats.second;

        Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow(flowId);

        bool isDl = ue_network_mask.IsMatch(ue_network, fiveTuple.destinationAddress);
        bool isUl = ue_network_mask.IsMatch(ue_network, fiveTuple.sourceAddress);
        if (!(isDl || isUl))
        {
            continue;
        }

        Ipv4Address ueIp = isDl ? fiveTuple.destinationAddress : fiveTuple.sourceAddress;
        // uint64_t imsi = LookupImsiFromIp(ueIp);
        // const char* role = LookupRoleFromImsi(imsi);

        // -------- interval deltas --------
        FlowIntervalSnapshot& prev = g_prevFlowStats[flowId];

        uint64_t dTxPackets = st.txPackets - prev.txPackets;
        uint64_t dRxPackets = st.rxPackets - prev.rxPackets;
        uint64_t dRxBytes   = st.rxBytes   - prev.rxBytes;

        Time dDelaySum  = st.delaySum  - prev.delaySum;
        Time dJitterSum = st.jitterSum - prev.jitterSum;

        // Save current snapshot for next interval
        prev.txPackets = st.txPackets;
        prev.rxPackets = st.rxPackets;
        prev.rxBytes   = st.rxBytes;
        prev.delaySum  = st.delaySum;
        prev.jitterSum = st.jitterSum;

        int receiver_id = get_user_id_from_ipv4(ueIp);
        if (receiver_id != -1)
        {
            if (isDl)
            {
                dlTxPackets[receiver_id] += dTxPackets;
                dlRxPackets[receiver_id] += dRxPackets;
                dlRxBytes[receiver_id] += dRxBytes;
                dlDelaySum[receiver_id] += dDelaySum;
                dlJitterSum[receiver_id] += dJitterSum;
            }
            else if (isUl)
            {
                ulTxPackets[receiver_id] += dTxPackets;
                ulRxPackets[receiver_id] += dRxPackets;
                ulRxBytes[receiver_id] += dRxBytes;
                ulDelaySum[receiver_id] += dDelaySum;
                ulJitterSum[receiver_id] += dJitterSum;
            }
        }
    }

    std::ofstream qos_vs_time;
    qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::app);

    for (uint32_t ue = 0; ue < numGroundUesS1; ++ue)
    {
        user_delay_dl[ue] = (dlRxPackets[ue] > 0)
                                ? (dlDelaySum[ue].GetSeconds() / static_cast<double>(dlRxPackets[ue]))
                                : 0.0;
        user_jitter_dl[ue] = (dlRxPackets[ue] > 0)
                                 ? (dlJitterSum[ue].GetSeconds() / static_cast<double>(dlRxPackets[ue]))
                                 : 0.0;
        user_throughput_dl[ue] = (monitorIntervalSec > 0.0)
                                     ? (static_cast<double>(dlRxBytes[ue]) * 8.0 /
                                        monitorIntervalSec / 1024.0 / 1024.0)
                                     : 0.0;
        user_pdr_dl[ue] = (dlTxPackets[ue] > 0)
                              ? (100.0 * static_cast<double>(dlRxPackets[ue]) /
                                 static_cast<double>(dlTxPackets[ue]))
                              : 0.0;

        user_delay_ul[ue] = (ulRxPackets[ue] > 0)
                                ? (ulDelaySum[ue].GetSeconds() / static_cast<double>(ulRxPackets[ue]))
                                : 0.0;
        user_jitter_ul[ue] = (ulRxPackets[ue] > 0)
                                 ? (ulJitterSum[ue].GetSeconds() / static_cast<double>(ulRxPackets[ue]))
                                 : 0.0;
        user_throughput_ul[ue] = (monitorIntervalSec > 0.0)
                                     ? (static_cast<double>(ulRxBytes[ue]) * 8.0 /
                                        monitorIntervalSec / 1024.0 / 1024.0)
                                     : 0.0;
        user_pdr_ul[ue] = (ulTxPackets[ue] > 0)
                              ? (100.0 * static_cast<double>(ulRxPackets[ue]) /
                                 static_cast<double>(ulTxPackets[ue]))
                              : 0.0;

        qos_vs_time << t << "," << ue << ",DL,"
                    << user_delay_dl[ue] << ","
                    << user_jitter_dl[ue] << ","
                    << user_throughput_dl[ue] << ","
                    << user_pdr_dl[ue] << "\n";

        qos_vs_time << t << "," << ue << ",UL,"
                    << user_delay_ul[ue] << ","
                    << user_jitter_ul[ue] << ","
                    << user_throughput_ul[ue] << ","
                    << user_pdr_ul[ue] << "\n";
    }

    g_lastQosSampleTime = now;
    Simulator::Schedule(management_interval, ThroughputMonitor, fmhelper, flowMon);
}


//Trace each node's location

void TraceUeS1Positions(NodeContainer ues)
{
    std::ofstream f(s_ueS1PositionTraceFile, std::ios_base::app);
    double t = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        auto mob = DynamicCast<GeocentricConstantPositionMobilityModel>(
            ues.Get(i)->GetObject<MobilityModel>());

        Vector geo = mob->GetGeographicPosition();

        f << t << " UE" << i
          << " " << geo.x   // lat
          << " " << geo.y   // lon
          << " " << geo.z   // alt
          << "\n";
    }

    Simulator::Schedule(Seconds(g_positionTraceIntervalSec), &TraceUeS1Positions, ues);
}

void TraceUeS2Positions(NodeContainer ues)
{
    std::ofstream f(s_ueS2PositionTraceFile, std::ios_base::app);
    double t = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < ues.GetN(); ++i)
    {
        auto mob = DynamicCast<GeocentricConstantPositionMobilityModel>(
            ues.Get(i)->GetObject<MobilityModel>());

        Vector geo = mob->GetGeographicPosition();

        f << t << " UE_S2_" << i
          << " " << geo.x
          << " " << geo.y
          << " " << geo.z
          << "\n";
    }

    Simulator::Schedule(Seconds(g_positionTraceIntervalSec), &TraceUeS2Positions, ues);
}

void TraceUavPositions(NodeContainer uavs)
{
    std::ofstream f(s_uavPositionTraceFile, std::ios_base::app);
    double t = Simulator::Now().GetSeconds();

    for (uint32_t i = 0; i < uavs.GetN(); ++i)
    {
        auto mob = DynamicCast<GeocentricConstantPositionMobilityModel>(
            uavs.Get(i)->GetObject<MobilityModel>());

        Vector geo = mob->GetGeographicPosition();

        f << t << " UAV" << i
          << " " << geo.x
          << " " << geo.y
          << " " << geo.z
          << "\n";
    }

    Simulator::Schedule(Seconds(g_positionTraceIntervalSec), &TraceUavPositions, uavs);
}

static double
EstimateFreeSpaceRsrpDbm(double txPowerDbm, double frequencyHz, double distanceMeters)
{
    // This is a lightweight xHaul health proxy:
    //   best TN donor gNB transmit power - free-space path loss.
    // It is intentionally simple so that the first comparison isolates the
    // effect of xHaul-aware UAV autonomy. It should be described as an
    // estimated xHaul RSRP, not as a full measured NR donor-link RSRP.
    const double d = std::max(distanceMeters, 1.0);
    const double fGhz = frequencyHz / 1e9;
    const double fsplDb = 32.4 + 20.0 * std::log10(fGhz) + 20.0 * std::log10(d);
    return txPowerDbm - fsplDb;
}

static std::string
ClassifyXhaulState(double rsrpDbm, double healthyThresholdDbm, double degradedThresholdDbm)
{
    // Three-state xHaul health model used by the UAV autonomy policy.
    // Thresholds are configurable so the paper can test optimistic and harsh
    // donor-link assumptions without recompiling the scenario.
    if (rsrpDbm >= healthyThresholdDbm)
    {
        return "HEALTHY";
    }
    if (rsrpDbm >= degradedThresholdDbm)
    {
        return "DEGRADED";
    }
    return "UNREACHABLE";
}

static std::string
SelectUavAutonomyMode(const std::string& deploymentMode,
                      const std::string& xhaulState,
                      bool satBackhaulHealthy)
{
    // UAV Autonomy xApp-like state machine:
    //   HEALTHY xHaul:
    //      terrestrial O-RAN/xApp control remains available and the UAV acts as
    //      a normal coverage-extension gNB.
    //   DEGRADED/UNREACHABLE xHaul + healthy satellite:
    //      onboard UAV RIC/xApp control is activated and satellite fallback
    //      keeps the UAV serviceable, so normal UE handover can remain enabled.
    //   DEGRADED/UNREACHABLE xHaul without satellite:
    //      the UAV may still offer access coverage, but normal UE handover to
    //      it is blocked by setting its effective O-RAN cell capacity to 0.
    if (deploymentMode == "tn-only")
    {
        return "NO_UAV";
    }
    if (xhaulState == "HEALTHY")
    {
        return "TN_CONTROLLED_COVERAGE_EXTENSION";
    }
    if (xhaulState == "DEGRADED" &&
        deploymentMode == "tn-uav-satellite" &&
        satBackhaulHealthy)
    {
        return "ONBOARD_AUTONOMY_WITH_SATELLITE_BACKHAUL";
    }
    if (xhaulState == "UNREACHABLE" &&
        deploymentMode == "tn-uav-satellite" &&
        satBackhaulHealthy)
    {
        return "ONBOARD_EMERGENCY_CONTROL_WITH_SATELLITE_BACKHAUL";
    }
    if (xhaulState == "DEGRADED")
    {
        return "ONBOARD_LOCAL_AUTONOMY_SERVICE_LIMITED";
    }
    return "AUTONOMOUS_LOCAL_ISLAND";
}

static void
RunUavAutonomyXappPolicy(const UavAutonomyXappContext& ctx)
{
    // Periodic cross-layer trace for the proposed article idea.
    //
    // Inputs recorded here:
    //   - estimated UAV-to-TN donor xHaul RSRP,
    //   - xHaul state: HEALTHY / DEGRADED / UNREACHABLE,
    //   - optional satellite backhaul SNR state,
    //   - E2 timing knobs used by the RIC control loop.
    //
    // Output:
    //   - selected UAV autonomy mode.
    //
    // This trace is meant to be joined with qos-vs-time.txt, handover-trace.tr,
    // handover-failure-trace.tr, and sat-backhaul-trace.txt during analysis.
    std::ofstream out(s_xhaulAutonomyTraceFile, std::ios_base::app);
    const double now = Simulator::Now().GetSeconds();
    const bool degradationActive =
        ctx.degradationStartSec >= 0.0 && now >= ctx.degradationStartSec &&
        now <= ctx.degradationStopSec;

    for (uint32_t uavIdx = 0; uavIdx < ctx.uavs.GetN(); ++uavIdx)
    {
        Ptr<MobilityModel> uavMob = ctx.uavs.Get(uavIdx)->GetObject<MobilityModel>();
        if (!uavMob)
        {
            continue;
        }

        double bestRsrpDbm = -1e9;
        double bestDonorDistanceM = -1.0;
        double bestChannelVariationDb = 0.0;
        uint16_t bestDonorCellId = 0;
        for (uint32_t tnIdx = 0; tnIdx < ctx.tnGnbs.GetN(); ++tnIdx)
        {
            Ptr<MobilityModel> tnMob = ctx.tnGnbs.Get(tnIdx)->GetObject<MobilityModel>();
            Ptr<NrGnbNetDevice> tnDev = ctx.tnGnbNrDevs.Get(tnIdx)->GetObject<NrGnbNetDevice>();
            if (!tnMob || !tnDev)
            {
                continue;
            }

            const double distanceMeters =
                CalculateDistance(uavMob->GetPosition(), tnMob->GetPosition());
            if (distanceMeters > ctx.xhaulMaxDonorDistanceM)
            {
                continue;
            }

            double rsrpDbm =
                EstimateFreeSpaceRsrpDbm(ctx.xhaulTxPowerDbm, ctx.xhaulFrequencyHz, distanceMeters);
            double channelVariationDb = 0.0;
            if (ctx.enableXhaulChannelVariation)
            {
                // Lightweight urban xHaul proxy. Shadowing is zero-mean in dB;
                // fading is modeled as an additional non-negative fast loss.
                // This avoids a hand-picked time-window penalty while still
                // allowing donor RSRP to fluctuate due to channel conditions.
                const double shadowingDb =
                    g_xhaulShadowingRv->GetValue(0.0, ctx.xhaulShadowingStddevDb);
                const double fadingLossDb =
                    std::abs(g_xhaulFadingRv->GetValue(0.0, ctx.xhaulFadingStddevDb));
                channelVariationDb = shadowingDb - fadingLossDb;
                rsrpDbm += channelVariationDb;
            }
            if (degradationActive)
            {
                rsrpDbm -= ctx.degradationPenaltyDb;
            }

            if (rsrpDbm > bestRsrpDbm)
            {
                bestRsrpDbm = rsrpDbm;
                bestDonorDistanceM = distanceMeters;
                bestChannelVariationDb = channelVariationDb;
                bestDonorCellId = tnDev->GetCellId();
            }
        }

        Ptr<NrGnbNetDevice> uavDev = ctx.uavGnbNrDevs.Get(uavIdx)->GetObject<NrGnbNetDevice>();
        const uint16_t uavCellId = uavDev ? uavDev->GetCellId() : 0;

        const bool xhaulConnected = bestDonorCellId != 0;
        const std::string xhaulState =
            xhaulConnected
                ? ClassifyXhaulState(bestRsrpDbm, ctx.healthyThresholdDbm, ctx.degradedThresholdDbm)
                : "UNREACHABLE";

        double satDl = -999.0;
        double satUl = -999.0;
        auto itDl = g_backhaulDlSnrDb.find(uavCellId);
        auto itUl = g_backhaulUlSnrDb.find(uavCellId);
        if (itDl != g_backhaulDlSnrDb.end())
        {
            satDl = itDl->second;
        }
        if (itUl != g_backhaulUlSnrDb.end())
        {
            satUl = itUl->second;
        }
        const bool satHealthy =
            itDl != g_backhaulDlSnrDb.end() && itUl != g_backhaulUlSnrDb.end() &&
            std::min(satDl, satUl) >= g_satBackhaulMinSnrDb;
        const bool onboardUavRicAvailable =
            ctx.enableOnboardUavRic && g_deploymentMode == "tn-uav-satellite";
        const bool onboardUavRicActive =
            onboardUavRicAvailable && xhaulState != "HEALTHY";

        const bool useSatelliteBackhaul =
            onboardUavRicActive && satHealthy;
        const std::string uavMode =
            SelectUavAutonomyMode(g_deploymentMode, xhaulState, useSatelliteBackhaul);
        const bool allowNormalUeHandover =
            xhaulState == "HEALTHY" || useSatelliteBackhaul;
        const std::string backhaulMode = useSatelliteBackhaul
                                             ? "SATELLITE_FALLBACK"
                                             : (xhaulConnected ? "TN_DIRECT" : "TN_UNREACHABLE");
        const std::string controlPath =
            xhaulState == "HEALTHY"
                ? "TN_E2"
                : (onboardUavRicActive
                       ? (satHealthy ? "ONBOARD_LOCAL_CONTROL_WITH_SAT_FEEDBACK"
                                     : "ONBOARD_LOCAL_CONTROL")
                       : "LOCAL_AUTONOMY");
        const std::string activeUavRic =
            xhaulState == "HEALTHY"
                ? "TN_NEAR_RT_RIC"
                : (onboardUavRicActive ? "ONBOARD_UAV_RIC" : "LOCAL_UAV_AUTONOMY");
        const std::string onboardUavRicState =
            onboardUavRicAvailable ? (onboardUavRicActive ? "ACTIVE" : "STANDBY") : "DISABLED";
        if (ctx.epcHelper)
        {
            ctx.epcHelper->SetNtnBackhaulMode(uavCellId,
                                              useSatelliteBackhaul ? "satellite" : "tn");
        }
        if (g_rsrpLm)
        {
            g_rsrpLm->SetCellCapacity(uavCellId,
                                      allowNormalUeHandover ? maxUesPerCellNtn : 0);
            g_rsrpLm->SetCellBackhaulDlSnrDb(uavCellId, satDl);
            g_rsrpLm->SetCellBackhaulUlSnrDb(uavCellId, satUl);
        }
        if (g_nsLogFile.is_open())
        {
            std::clog << "UAV_AUTONOMY_XAPP"
                      << " Time=" << now
                      << " DeploymentMode=" << g_deploymentMode
                      << " UavIndex=" << uavIdx
                      << " UavCellId=" << uavCellId
                      << " BestDonorCellId=" << bestDonorCellId
                      << " BestDonorDistanceM=" << bestDonorDistanceM
                      << " XhaulConnected=" << (xhaulConnected ? 1 : 0)
                      << " XhaulChannelVariationDb=" << bestChannelVariationDb
                      << " XhaulState=" << xhaulState
                      << " SatBackhaulHealthy=" << (satHealthy ? 1 : 0)
                      << " OnboardUavRicAvailable=" << (onboardUavRicAvailable ? 1 : 0)
                      << " OnboardUavRicState=" << onboardUavRicState
                      << " BackhaulMode=" << backhaulMode
                      << " ControlPath=" << controlPath
                      << " ActiveUavRic=" << activeUavRic
                      << " NormalUeHandoverAllowed=" << (allowNormalUeHandover ? 1 : 0)
                      << " UavMode=" << uavMode
                      << "\n";
        }

        out << now << ","
            << g_deploymentMode << ","
            << uavIdx << ","
            << uavCellId << ","
            << bestDonorCellId << ","
            << bestDonorDistanceM << ","
            << (xhaulConnected ? 1 : 0) << ","
            << bestChannelVariationDb << ","
            << bestRsrpDbm << ","
            << xhaulState << ","
            << (degradationActive ? 1 : 0) << ","
            << satDl << ","
            << satUl << ","
            << (satHealthy ? 1 : 0) << ","
            << (onboardUavRicAvailable ? 1 : 0) << ","
            << onboardUavRicState << ","
            << backhaulMode << ","
            << controlPath << ","
            << activeUavRic << ","
            << (allowNormalUeHandover ? 1 : 0) << ","
            << ctx.e2TxDelaySec << ","
            << ctx.e2SendIntervalSec << ","
            << ctx.lmQueryIntervalSec << ","
            << uavMode << "\n";
    }
}

void
TraceXhaulAutonomy(NodeContainer tnGnbs,
                   NodeContainer uavs,
                   NetDeviceContainer tnGnbNrDevs,
                   NetDeviceContainer uavGnbNrDevs,
                   Ptr<HybridSatEpcHelper> epcHelper,
                   double xhaulTxPowerDbm,
                   double xhaulFrequencyHz,
                   double xhaulMaxDonorDistanceM,
                   double healthyThresholdDbm,
                   double degradedThresholdDbm,
                   double degradationStartSec,
                   double degradationStopSec,
                   double degradationPenaltyDb,
                   double e2TxDelaySec,
                   double e2SendIntervalSec,
                   double lmQueryIntervalSec,
                   double sampleIntervalSec)
{
    UavAutonomyXappContext ctx;
    ctx.tnGnbs = tnGnbs;
    ctx.uavs = uavs;
    ctx.tnGnbNrDevs = tnGnbNrDevs;
    ctx.uavGnbNrDevs = uavGnbNrDevs;
    ctx.epcHelper = epcHelper;
    ctx.xhaulTxPowerDbm = xhaulTxPowerDbm;
    ctx.xhaulFrequencyHz = xhaulFrequencyHz;
    ctx.xhaulMaxDonorDistanceM = xhaulMaxDonorDistanceM;
    ctx.healthyThresholdDbm = healthyThresholdDbm;
    ctx.degradedThresholdDbm = degradedThresholdDbm;
    ctx.degradationStartSec = degradationStartSec;
    ctx.degradationStopSec = degradationStopSec;
    ctx.degradationPenaltyDb = degradationPenaltyDb;
    ctx.e2TxDelaySec = e2TxDelaySec;
    ctx.e2SendIntervalSec = e2SendIntervalSec;
    ctx.lmQueryIntervalSec = lmQueryIntervalSec;
    ctx.enableXhaulChannelVariation = g_enableXhaulChannelVariation;
    ctx.xhaulShadowingStddevDb = g_xhaulShadowingStddevDb;
    ctx.xhaulFadingStddevDb = g_xhaulFadingStddevDb;
    ctx.enableOnboardUavRic = g_enableOnboardUavRic;

    RunUavAutonomyXappPolicy(ctx);

    Simulator::Schedule(Seconds(sampleIntervalSec),
                        &TraceXhaulAutonomy,
                        tnGnbs,
                        uavs,
                        tnGnbNrDevs,
                        uavGnbNrDevs,
                        epcHelper,
                        xhaulTxPowerDbm,
                        xhaulFrequencyHz,
                        xhaulMaxDonorDistanceM,
                        healthyThresholdDbm,
                        degradedThresholdDbm,
                        degradationStartSec,
                        degradationStopSec,
                        degradationPenaltyDb,
                        e2TxDelaySec,
                        e2SendIntervalSec,
                        lmQueryIntervalSec,
                        sampleIntervalSec);
}

class OranLmUavAutonomyControl : public OranLm
{
  public:
    static TypeId GetTypeId();
    OranLmUavAutonomyControl();

  private:
    std::vector<Ptr<OranCommand>> Run() override;
};

NS_OBJECT_ENSURE_REGISTERED(OranLmUavAutonomyControl);

TypeId
OranLmUavAutonomyControl::GetTypeId()
{
    static TypeId tid = TypeId("OranLmUavAutonomyControl")
                            .SetParent<OranLm>()
                            .AddConstructor<OranLmUavAutonomyControl>();
    return tid;
}

OranLmUavAutonomyControl::OranLmUavAutonomyControl()
    : OranLm()
{
    m_name = "UAV_AUTONOMY_XAPP";
}

std::vector<Ptr<OranCommand>>
OranLmUavAutonomyControl::Run()
{
    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");

    if (g_uavAutonomyXappCtx)
    {
        RunUavAutonomyXappPolicy(*g_uavAutonomyXappCtx);
        LogLogicToRepository("UAV Autonomy xApp updated UAV backhaul/control path state");
    }
    else
    {
        LogLogicToRepository("UAV Autonomy xApp skipped: no UAV context");
    }

    return {};
}

void
NotifyHandoverEndOkGnb(std::string context, uint64_t imsi, uint16_t targetCellId, uint16_t rnti)
{
    double targetRsrp = -1e9;
    double servingRsrp = -1e9;

    auto it = g_latestRsrp.find({rnti,targetCellId});
    if (it != g_latestRsrp.end())
        targetRsrp = it->second;

    for (auto &kv : g_latestRsrp)
    {
        if (kv.first.first == rnti &&
            kv.first.second != targetCellId)
        {
            servingRsrp = std::max(servingRsrp, kv.second);
        }
    }

    double bhDl = -999.0;
    double bhUl = -999.0;

    auto itd = g_backhaulDlSnrDb.find(targetCellId);
    if (itd != g_backhaulDlSnrDb.end())
    {
        bhDl = itd->second;
    }

    auto itu = g_backhaulUlSnrDb.find(targetCellId);
    if (itu != g_backhaulUlSnrDb.end())
    {
        bhUl = itu->second;
    }

    const char* type =
        g_ntnCellIds.count(targetCellId) ? "NTN" : "TN";

    std::ofstream f(s_handoverTraceFile,std::ios_base::app);

    f << Simulator::Now().GetSeconds()
      << " IMSI=" << imsi
      << " TargetCell=" << targetCellId
      << " Type=" << type
      << " TargetRSRP=" << targetRsrp
      << " ServingRSRP=" << servingRsrp
      << " BackhaulDlSnr=" << bhDl
      << " BackhaulUlSnr=" << bhUl
      << "\n";

    if (g_nsLogFile.is_open())
    {
        std::clog << "TRACE HO_SUCCESS"
                  << " Time=" << Simulator::Now().GetSeconds()
                  << " IMSI=" << imsi
                  << " TargetCell=" << targetCellId
                  << " Type=" << type
                  << " TargetRSRP=" << targetRsrp
                  << " ServingRSRP=" << servingRsrp
                  << " BackhaulDlSnr=" << bhDl
                  << " BackhaulUlSnr=" << bhUl
                  << "\n";
    }
}

void
NotifyHandoverFailureGnb(std::string reason,
                         std::string context,
                         uint64_t imsi,
                         uint16_t rnti,
                         uint16_t cellId)
{
    std::ofstream f(s_handoverFailureTraceFile, std::ios_base::app);
    f << Simulator::Now().GetSeconds()
      << " Side=GNB"
      << " Reason=" << reason
      << " IMSI=" << imsi
      << " Cell=" << cellId
      << " RNTI=" << rnti
      << " Context=" << context
      << "\n";

    if (g_nsLogFile.is_open())
    {
        std::clog << "TRACE HO_FAILURE"
                  << " Time=" << Simulator::Now().GetSeconds()
                  << " Side=GNB"
                  << " Reason=" << reason
                  << " IMSI=" << imsi
                  << " Cell=" << cellId
                  << " RNTI=" << rnti
                  << " Context=" << context
                  << "\n";
    }
}

void
NotifyHandoverEndErrorUe(std::string context, uint64_t imsi, uint16_t cellId, uint16_t rnti)
{
    std::ofstream f(s_handoverFailureTraceFile, std::ios_base::app);
    f << Simulator::Now().GetSeconds()
      << " Side=UE"
      << " Reason=HandoverEndError"
      << " IMSI=" << imsi
      << " Cell=" << cellId
      << " RNTI=" << rnti
      << " Context=" << context
      << "\n";

    if (g_nsLogFile.is_open())
    {
        std::clog << "TRACE HO_FAILURE"
                  << " Time=" << Simulator::Now().GetSeconds()
                  << " Side=UE"
                  << " Reason=HandoverEndError"
                  << " IMSI=" << imsi
                  << " Cell=" << cellId
                  << " RNTI=" << rnti
                  << " Context=" << context
                  << "\n";
    }
}

static void
ApplyTnInfrastructureDegradation(Ptr<NrHelper> nrHelper,
                                 NetDeviceContainer tnGnbNrDevs,
                                 double nominalTxPowerDbm,
                                 double degradationPenaltyDb,
                                 bool degradationActive)
{
    // Controlled terrestrial-infrastructure stress model.
    //
    // This reduces the TN gNB transmit power during a configured time window so
    // that the TN-only baseline also experiences a visible degradation period.
    // The UAV xHaul degradation remains a separate metric; this one affects the
    // UE-facing terrestrial access layer and therefore helps demonstrate why
    // TN-only coverage/service may become insufficient during an incident.
    const double activeTxPowerDbm =
        degradationActive ? nominalTxPowerDbm - degradationPenaltyDb : nominalTxPowerDbm;

    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(tnGnbNrDevs.Get(i), 0)->SetAttribute("TxPower",
                                                                 DoubleValue(activeTxPowerDbm));
        nrHelper->GetGnbPhy(tnGnbNrDevs.Get(i), 1)->SetAttribute("TxPower",
                                                                 DoubleValue(activeTxPowerDbm));
        nrHelper->GetGnbPhy(tnGnbNrDevs.Get(i), 2)->SetAttribute("TxPower",
                                                                 DoubleValue(0.0));
    }

    std::ofstream out(s_tnInfrastructureTraceFile, std::ios_base::app);
    out << Simulator::Now().GetSeconds() << ","
        << (degradationActive ? 1 : 0) << ","
        << activeTxPowerDbm << ","
        << degradationPenaltyDb << "\n";
}


struct GeoWaypointState
{
    Ptr<GeocentricConstantPositionMobilityModel> mob;
    double centerLatDeg;
    double centerLonDeg;
    double fixedAltM;
    double halfWidthM;
    double halfHeightM;
    double speedMps;
    double targetEastM = 0.0;
    double targetNorthM = 0.0;
    bool hasTarget = false;
    double reachThresholdM = 1.0;
};

static void
SelectNewGeoWaypoint(GeoWaypointState* st,
                     Ptr<UniformRandomVariable> eastRv,
                     Ptr<UniformRandomVariable> northRv)
{
    st->targetEastM = eastRv->GetValue(-st->halfWidthM, st->halfWidthM);
    st->targetNorthM = northRv->GetValue(-st->halfHeightM, st->halfHeightM);
    st->hasTarget = true;
}

static void
MoveGeoNodeRandomWaypoint(GeoWaypointState* st,
                          Ptr<UniformRandomVariable> eastRv,
                          Ptr<UniformRandomVariable> northRv,
                          double stepMs)
{
    double dt = stepMs / 1000.0;

    Vector geo = st->mob->GetGeographicPosition();
    double latDeg = geo.x;
    double lonDeg = geo.y;
    double altM   = st->fixedAltM;

    double centerLatRad = st->centerLatDeg * M_PI / 180.0;
    double metersPerDegLat = 111320.0;
    double metersPerDegLon = 111320.0 * std::cos(centerLatRad);

    double eastOffsetM  = (lonDeg - st->centerLonDeg) * metersPerDegLon;
    double northOffsetM = (latDeg - st->centerLatDeg) * metersPerDegLat;

    if (!st->hasTarget)
    {
        SelectNewGeoWaypoint(st, eastRv, northRv);
    }

    double dEast = st->targetEastM - eastOffsetM;
    double dNorth = st->targetNorthM - northOffsetM;
    double distanceToTarget = std::sqrt(dEast * dEast + dNorth * dNorth);

    if (distanceToTarget <= st->reachThresholdM)
    {
        SelectNewGeoWaypoint(st, eastRv, northRv);
        dEast = st->targetEastM - eastOffsetM;
        dNorth = st->targetNorthM - northOffsetM;
        distanceToTarget = std::sqrt(dEast * dEast + dNorth * dNorth);
    }

    double moveDistance = st->speedMps * dt;
    double nextEastM = eastOffsetM;
    double nextNorthM = northOffsetM;

    if (distanceToTarget > 1e-9)
    {
        double stepDistance = std::min(moveDistance, distanceToTarget);
        nextEastM += stepDistance * (dEast / distanceToTarget);
        nextNorthM += stepDistance * (dNorth / distanceToTarget);
    }

    nextEastM = std::max(-st->halfWidthM, std::min(st->halfWidthM, nextEastM));
    nextNorthM = std::max(-st->halfHeightM, std::min(st->halfHeightM, nextNorthM));

    lonDeg = st->centerLonDeg + nextEastM / metersPerDegLon;
    latDeg = st->centerLatDeg + nextNorthM / metersPerDegLat;

    st->mob->SetGeographicPosition(Vector(latDeg, lonDeg, altM));

    Simulator::Schedule(MilliSeconds(stepMs),
                        &MoveGeoNodeRandomWaypoint,
                        st,
                        eastRv,
                        northRv,
                        stepMs);
}

static std::vector<std::unique_ptr<GeoWaypointState>> g_geoStates;

static Vector
RandomGeoFromCenter(double centerLatDeg,
                    double centerLonDeg,
                    double altM,
                    double halfWidthM,
                    double halfHeightM,
                    Ptr<UniformRandomVariable> eastRv,
                    Ptr<UniformRandomVariable> northRv)
{
    double eastM = eastRv->GetValue(-halfWidthM, halfWidthM);
    double northM = northRv->GetValue(-halfHeightM, halfHeightM);

    double metersPerDegLat = 111320.0;
    double metersPerDegLon = 111320.0 * std::cos(centerLatDeg * M_PI / 180.0);

    double latDeg = centerLatDeg + northM / metersPerDegLat;
    double lonDeg = centerLonDeg + eastM / metersPerDegLon;

    return Vector(latDeg, lonDeg, altM); // geographic: lat, lon, alt
}

///
struct LocalPoint2d
{
    double eastM = 0.0;
    double northM = 0.0;
};

static std::vector<GeoWaypointState*> g_uavStates;

// Geographic <-> local helpers
static LocalPoint2d
GeoToLocal(const Vector& geo, double g_refLatDeg, double g_refLonDeg)
{
    double metersPerDegLat = 111320.0;
    double metersPerDegLon = 111320.0 * std::cos(g_refLatDeg * M_PI / 180.0);

    LocalPoint2d p;
    p.eastM  = (geo.y - g_refLonDeg) * metersPerDegLon;
    p.northM = (geo.x - g_refLatDeg) * metersPerDegLat;
    return p;
}

static Vector
LocalToGeo(double eastM, double northM, double altM, double g_refLatDeg, double g_refLonDeg)
{
    double metersPerDegLat = 111320.0;
    double metersPerDegLon = 111320.0 * std::cos(g_refLatDeg * M_PI / 180.0);

    double latDeg = g_refLatDeg + northM / metersPerDegLat;
    double lonDeg = g_refLonDeg + eastM / metersPerDegLon;

    return Vector(latDeg, lonDeg, altM);
}

// Controlled mover: move to assigned target, then hover there.
// No random re-targeting here.
static void
MoveGeoNodeToAssignedTarget(GeoWaypointState* st, double stepMs)
{
    double dt = stepMs / 1000.0;

    Vector geo = st->mob->GetGeographicPosition();
    LocalPoint2d cur = GeoToLocal(geo, st->centerLatDeg, st->centerLonDeg);

    // No target yet -> just wait
    if (!st->hasTarget)
    {
        Simulator::Schedule(MilliSeconds(stepMs),
                            &MoveGeoNodeToAssignedTarget,
                            st,
                            stepMs);
        return;
    }

    double dEast = st->targetEastM - cur.eastM;
    double dNorth = st->targetNorthM - cur.northM;
    double distance = std::sqrt(dEast * dEast + dNorth * dNorth);

    // Already at target -> hover
    if (distance <= st->reachThresholdM)
    {
        Simulator::Schedule(MilliSeconds(stepMs),
                            &MoveGeoNodeToAssignedTarget,
                            st,
                            stepMs);
        return;
    }

    double moveDistance = st->speedMps * dt;
    double stepDistance = std::min(moveDistance, distance);

    double nextEast  = cur.eastM  + stepDistance * (dEast  / distance);
    double nextNorth = cur.northM + stepDistance * (dNorth / distance);

    nextEast  = std::max(-st->halfWidthM,  std::min(st->halfWidthM,  nextEast));
    nextNorth = std::max(-st->halfHeightM, std::min(st->halfHeightM, nextNorth));

    Vector nextGeo = LocalToGeo(nextEast,
                                nextNorth,
                                st->fixedAltM,
                                st->centerLatDeg,
                                st->centerLonDeg);

    st->mob->SetGeographicPosition(nextGeo);

    Simulator::Schedule(MilliSeconds(stepMs),
                        &MoveGeoNodeToAssignedTarget,
                        st,
                        stepMs);
}

static bool
IsUeUnderserved(Ptr<Node> ueNode, double rsrpThreshDbm)
{
    Ptr<NrUeNetDevice> ueDev = nullptr;

    for (uint32_t i = 0; i < ueNode->GetNDevices(); ++i)
    {
        ueDev = DynamicCast<NrUeNetDevice>(ueNode->GetDevice(i));
        if (ueDev)
        {
            break;
        }
    }

    if (!ueDev)
    {
        return false;
    }

    Ptr<NrUeRrc> rrc = ueDev->GetRrc();
    if (!rrc)
    {
        return true;
    }

    if (rrc->GetCellId() == 0 || rrc->GetState() != NrUeRrc::CONNECTED_NORMALLY)
    {
        return true;
    }

    uint16_t servingCell = rrc->GetCellId();
    uint16_t rnti = rrc->GetRnti();

    auto it = g_latestRsrp.find({rnti, servingCell});
    if (it == g_latestRsrp.end())
    {
        return true;
    }

    return (it->second < rsrpThreshDbm);
}

static std::vector<LocalPoint2d>
RunKMeans(const std::vector<LocalPoint2d>& points, uint32_t k, uint32_t iterations = 10)
{
    std::vector<LocalPoint2d> centroids;
    if (points.empty() || k == 0)
    {
        return centroids;
    }

    k = std::min<uint32_t>(k, points.size());
    centroids.assign(points.begin(), points.begin() + k);

    std::vector<uint32_t> labels(points.size(), 0);

    for (uint32_t iter = 0; iter < iterations; ++iter)
    {
        // Assign
        for (uint32_t i = 0; i < points.size(); ++i)
        {
            double bestDist = std::numeric_limits<double>::max();
            uint32_t bestId = 0;

            for (uint32_t c = 0; c < centroids.size(); ++c)
            {
                double dx = points[i].eastM - centroids[c].eastM;
                double dy = points[i].northM - centroids[c].northM;
                double d2 = dx * dx + dy * dy;

                if (d2 < bestDist)
                {
                    bestDist = d2;
                    bestId = c;
                }
            }
            labels[i] = bestId;
        }

        // Recompute
        std::vector<double> sumEast(k, 0.0), sumNorth(k, 0.0);
        std::vector<uint32_t> count(k, 0);

        for (uint32_t i = 0; i < points.size(); ++i)
        {
            uint32_t c = labels[i];
            sumEast[c] += points[i].eastM;
            sumNorth[c] += points[i].northM;
            count[c] += 1;
        }

        for (uint32_t c = 0; c < k; ++c)
        {
            if (count[c] > 0)
            {
                centroids[c].eastM  = sumEast[c] / count[c];
                centroids[c].northM = sumNorth[c] / count[c];
            }
        }
    }

    return centroids;
}

static void
SetUavTargetToCurrentPosition(GeoWaypointState* st)
{
    Vector geo = st->mob->GetGeographicPosition();
    LocalPoint2d cur = GeoToLocal(geo, st->centerLatDeg, st->centerLonDeg);
    st->targetEastM = cur.eastM;
    st->targetNorthM = cur.northM;
    st->hasTarget = true;
}

static void
UpdateUavTargetsFromUnderservedUes(NodeContainer groundUeNodesS1,
                                   NodeContainer groundUeNodesS2,
                                   NetDeviceContainer ntnGnbNrDevs,
                                   double rsrpThreshDbm,
                                   double controlPeriodSec)
{
    std::vector<LocalPoint2d> underservedPts;

    // 1) collect underserved UEs
    for (uint32_t i = 0; i < groundUeNodesS1.GetN(); ++i)
    {
        if (IsUeUnderserved(groundUeNodesS1.Get(i), rsrpThreshDbm))
        {
            auto mob = DynamicCast<GeocentricConstantPositionMobilityModel>(
                groundUeNodesS1.Get(i)->GetObject<MobilityModel>());
            if (mob)
            {
                underservedPts.push_back(
                    GeoToLocal(mob->GetGeographicPosition(), g_refLat, g_refLon));
            }
        }
    }

    for (uint32_t i = 0; i < groundUeNodesS2.GetN(); ++i)
    {
        if (IsUeUnderserved(groundUeNodesS2.Get(i), rsrpThreshDbm))
        {
            auto mob = DynamicCast<GeocentricConstantPositionMobilityModel>(
                groundUeNodesS2.Get(i)->GetObject<MobilityModel>());
            if (mob)
            {
                underservedPts.push_back(
                    GeoToLocal(mob->GetGeographicPosition(), g_refLat, g_refLon));
            }
        }
    }

    // 2) build available-UAV list
    std::vector<uint32_t> availableUavs;
    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
        if (!gnb || !gnb->GetRrc())
        {
            continue;
        }

        uint32_t load = gnb->GetRrc()->GetUeCount();
        uint32_t cap  = maxUesPerCellNtn;

        if (load < cap)
        {
            availableUavs.push_back(i);
        }
        else
        {
            // Full UAV: hover where it is
            SetUavTargetToCurrentPosition(g_uavStates[i]);
        }
    }

    // Nothing to do
    if (underservedPts.empty() || availableUavs.empty())
    {
        Simulator::Schedule(Seconds(controlPeriodSec),
                            &UpdateUavTargetsFromUnderservedUes,
                            groundUeNodesS1,
                            groundUeNodesS2,
                            ntnGnbNrDevs,
                            rsrpThreshDbm,
                            controlPeriodSec);
        return;
    }

    uint32_t k = std::min<uint32_t>(availableUavs.size(), underservedPts.size());
    std::vector<LocalPoint2d> centroids = RunKMeans(underservedPts, k, 10);
    for (auto& centroid : centroids)
    {
        centroid.eastM *= g_uavMissionTargetScale;
        centroid.northM *= g_uavMissionTargetScale;
    }

    // 3) assign each available UAV to one centroid (greedy nearest)
    std::vector<bool> centroidUsed(centroids.size(), false);

    for (uint32_t a = 0; a < availableUavs.size(); ++a)
    {
        uint32_t uavIdx = availableUavs[a];
        GeoWaypointState* st = g_uavStates[uavIdx];

        Vector geo = st->mob->GetGeographicPosition();
        LocalPoint2d cur = GeoToLocal(geo, st->centerLatDeg, st->centerLonDeg);

        double bestDist = std::numeric_limits<double>::max();
        int32_t bestCentroid = -1;

        for (uint32_t c = 0; c < centroids.size(); ++c)
        {
            if (centroidUsed[c])
            {
                continue;
            }

            double dx = centroids[c].eastM - cur.eastM;
            double dy = centroids[c].northM - cur.northM;
            double d2 = dx * dx + dy * dy;

            if (d2 < bestDist)
            {
                bestDist = d2;
                bestCentroid = static_cast<int32_t>(c);
            }
        }

        if (bestCentroid >= 0)
        {
            st->targetEastM = centroids[bestCentroid].eastM;
            st->targetNorthM = centroids[bestCentroid].northM;
            st->hasTarget = true;
            centroidUsed[bestCentroid] = true;
        }
        else
        {
            SetUavTargetToCurrentPosition(st);
        }
    }

    Simulator::Schedule(Seconds(controlPeriodSec),
                        &UpdateUavTargetsFromUnderservedUes,
                        groundUeNodesS1,
                        groundUeNodesS2,
                        ntnGnbNrDevs,
                        rsrpThreshDbm,
                        controlPeriodSec);
}
///

///
void
install_mobility_geocentric(NodeContainer staticNodes,
                            NodeContainer tnGnbNodes,
                            NodeContainer ntnGnbNodes,
                            NodeContainer groundUeNodesS1,
                            NodeContainer groundUeNodesS2)
{
    auto setFixedGeo = [&](Ptr<Node> n, double lat, double lon, double alt)
    {
        auto mob = CreateObject<GeocentricConstantPositionMobilityModel>();
        mob->SetGeographicPosition(Vector(lat, lon, alt));
        mob->SetCoordinateTranslationReferencePoint(Vector(g_refLat, g_refLon, 0.0));
        n->AggregateObject(mob);
    };

    for (uint32_t i = 0; i < staticNodes.GetN(); ++i)
    {
        setFixedGeo(staticNodes.Get(i), g_refLat, g_refLon, 10.0);
    }

    Ptr<UniformRandomVariable> tnEast = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> tnNorth = CreateObject<UniformRandomVariable>();

    for (uint32_t i = 0; i < tnGnbNodes.GetN(); ++i)
    {
        Vector geo = RandomGeoFromCenter(g_refLat,
                                         g_refLon,
                                         25.0,
                                         TN_GNB_HALF_W_M,
                                         TN_GNB_HALF_H_M,
                                         tnEast,
                                         tnNorth);

        setFixedGeo(tnGnbNodes.Get(i), geo.x, geo.y, geo.z);
    }

    for (uint32_t i = 0; i < ntnGnbNodes.GetN(); ++i)
    {
        Ptr<UniformRandomVariable> eastRv = CreateObject<UniformRandomVariable>();
        Ptr<UniformRandomVariable> northRv = CreateObject<UniformRandomVariable>();

        double uavAlt = 120.0 + 5.0 * i;
        Vector initGeo = RandomGeoFromCenter(g_refLat,
                                             g_refLon,
                                             uavAlt,
                                             UAV_AREA_HALF_W_M,
                                             UAV_AREA_HALF_H_M,
                                             eastRv,
                                             northRv);

        auto mob = CreateObject<GeocentricConstantPositionMobilityModel>();
        mob->SetGeographicPosition(initGeo);
        mob->SetCoordinateTranslationReferencePoint(Vector(g_refLat, g_refLon, 0.0));
        ntnGnbNodes.Get(i)->AggregateObject(mob);

        auto st = std::make_unique<GeoWaypointState>();
        st->mob = mob;
        st->centerLatDeg = g_refLat;
        st->centerLonDeg = g_refLon;
        st->fixedAltM = uavAlt;
        st->halfWidthM = UAV_AREA_HALF_W_M;
        st->halfHeightM = UAV_AREA_HALF_H_M;
        st->speedMps = g_uavSpeedMps;

        LocalPoint2d initLocal = GeoToLocal(initGeo, g_refLat, g_refLon);
        st->targetEastM = initLocal.eastM;
        st->targetNorthM = initLocal.northM;
        st->hasTarget = true;

        g_uavStates.push_back(st.get());
        g_geoStates.push_back(std::move(st));

        Simulator::Schedule(MilliSeconds(g_mobilityUpdateMs),
                            &MoveGeoNodeToAssignedTarget,
                            g_uavStates.back(),
                            g_mobilityUpdateMs);
    }

    for (uint32_t i = 0; i < groundUeNodesS1.GetN(); ++i)
    {
        Ptr<UniformRandomVariable> eastRv = CreateObject<UniformRandomVariable>();
        Ptr<UniformRandomVariable> northRv = CreateObject<UniformRandomVariable>();

        Vector initGeo = RandomGeoFromCenter(g_refLat,
                                             g_refLon,
                                             1.5,
                                             UE_AREA_HALF_W_M,
                                             UE_AREA_HALF_H_M,
                                             eastRv,
                                             northRv);

        auto mob = CreateObject<GeocentricConstantPositionMobilityModel>();
        mob->SetGeographicPosition(initGeo);
        mob->SetCoordinateTranslationReferencePoint(Vector(g_refLat, g_refLon, 0.0));
        groundUeNodesS1.Get(i)->AggregateObject(mob);

        auto st = std::make_unique<GeoWaypointState>();
        st->mob = mob;
        st->centerLatDeg = g_refLat;
        st->centerLonDeg = g_refLon;
        st->fixedAltM = 1.5;
        st->halfWidthM = UE_AREA_HALF_W_M;
        st->halfHeightM = UE_AREA_HALF_H_M;
        st->speedMps = 5.0;

        SelectNewGeoWaypoint(st.get(), eastRv, northRv);

        Simulator::Schedule(MilliSeconds(g_mobilityUpdateMs),
                            &MoveGeoNodeRandomWaypoint,
                            st.get(),
                            eastRv,
                            northRv,
                            g_mobilityUpdateMs);

        g_geoStates.push_back(std::move(st));
    }

    for (uint32_t i = 0; i < groundUeNodesS2.GetN(); ++i)
    {
        Ptr<UniformRandomVariable> eastRv = CreateObject<UniformRandomVariable>();
        Ptr<UniformRandomVariable> northRv = CreateObject<UniformRandomVariable>();

        Vector initGeo = RandomGeoFromCenter(g_refLat,
                                             g_refLon,
                                             1.5,
                                             UE_AREA_HALF_W_M,
                                             UE_AREA_HALF_H_M,
                                             eastRv,
                                             northRv);

        auto mob = CreateObject<GeocentricConstantPositionMobilityModel>();
        mob->SetGeographicPosition(initGeo);
        mob->SetCoordinateTranslationReferencePoint(Vector(g_refLat, g_refLon, 0.0));
        groundUeNodesS2.Get(i)->AggregateObject(mob);

        auto st = std::make_unique<GeoWaypointState>();
        st->mob = mob;
        st->centerLatDeg = g_refLat;
        st->centerLonDeg = g_refLon;
        st->fixedAltM = 1.5;
        st->halfWidthM = UE_AREA_HALF_W_M;
        st->halfHeightM = UE_AREA_HALF_H_M;
        st->speedMps = 4.0;

        SelectNewGeoWaypoint(st.get(), eastRv, northRv);

        Simulator::Schedule(MilliSeconds(g_mobilityUpdateMs),
                            &MoveGeoNodeRandomWaypoint,
                            st.get(),
                            eastRv,
                            northRv,
                            g_mobilityUpdateMs);

        g_geoStates.push_back(std::move(st));
    }
}
///

Ptr<SpectrumValue>
CreateTxPowerSpectralDensity(double fcHz, double pwrDbm, double bwHz, double rbWidthHz)
{
    unsigned int numRbs = std::floor(bwHz / rbWidthHz);
    double f = fcHz - (numRbs * rbWidthHz / 2.0);

    Bands rbs;
    for (uint32_t numrb = 0; numrb < numRbs; ++numrb)
    {
        BandInfo rb;
        rb.fl = f;
        f += rbWidthHz / 2.0;
        rb.fc = f;
        f += rbWidthHz / 2.0;
        rb.fh = f;
        rbs.push_back(rb);
    }

    Ptr<SpectrumModel> model = Create<SpectrumModel>(rbs);
    Ptr<SpectrumValue> txPsd = Create<SpectrumValue>(model);

    double powerTxW = std::pow(10.0, (pwrDbm - 30.0) / 10.0);
    double txPowerDensity = powerTxW / bwHz;

    for (auto it = txPsd->ValuesBegin(); it != txPsd->ValuesEnd(); ++it)
    {
        *it = txPowerDensity;
    }

    return txPsd;
}

Ptr<SpectrumValue>
CreateNoisePowerSpectralDensity(double fcHz, double noiseFigureDb, double bwHz, double rbWidthHz)
{
    unsigned int numRbs = std::floor(bwHz / rbWidthHz);
    double f = fcHz - (numRbs * rbWidthHz / 2.0);

    Bands rbs;
    std::vector<int> rbIds;

    for (uint32_t numrb = 0; numrb < numRbs; ++numrb)
    {
        BandInfo rb;
        rb.fl = f;
        f += rbWidthHz / 2.0;
        rb.fc = f;
        f += rbWidthHz / 2.0;
        rb.fh = f;
        rbs.push_back(rb);
        rbIds.push_back(numrb);
    }

    Ptr<SpectrumModel> model = Create<SpectrumModel>(rbs);
    Ptr<SpectrumValue> noisePsd = Create<SpectrumValue>(model);

    const double ktDbmHz = -174.0;
    double ktWHz = std::pow(10.0, (ktDbmHz - 30.0) / 10.0);
    double noiseFigureLinear = std::pow(10.0, noiseFigureDb / 10.0);
    double noisePowerSpectralDensity = ktWHz * noiseFigureLinear;

    for (int rbId : rbIds)
    {
        (*noisePsd)[rbId] = noisePowerSpectralDensity;
    }

    return noisePsd;
}

static void
DoBeamforming(Ptr<NetDevice> thisDevice,
              Ptr<PhasedArrayModel> thisAntenna,
              Ptr<NetDevice> otherDevice)
{
    Vector aPos = thisDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();
    Vector bPos = otherDevice->GetNode()->GetObject<MobilityModel>()->GetPosition();

    Angles completeAngle(bPos, aPos);
    double hAngleRadian = completeAngle.GetAzimuth();
    double vAngleRadian = completeAngle.GetInclination();

    uint64_t totNoArrayElements = thisAntenna->GetNumElems();
    PhasedArrayModel::ComplexVector antennaWeights(totNoArrayElements);

    double power = 1.0 / std::sqrt(static_cast<double>(totNoArrayElements));

    const double sinV = std::sin(vAngleRadian);
    const double cosV = std::cos(vAngleRadian);
    const double sinH = std::sin(hAngleRadian);
    const double cosH = std::cos(hAngleRadian);

    for (uint64_t ind = 0; ind < totNoArrayElements; ind++)
    {
        Vector loc = thisAntenna->GetElementLocation(ind);
        double phase = -2.0 * M_PI *
                       (sinV * cosH * loc.x + sinV * sinH * loc.y + cosV * loc.z);
        antennaWeights[ind] = std::exp(std::complex<double>(0.0, phase)) * power;
    }

    thisAntenna->SetBeamformingVector(antennaWeights);
}

struct SatNtnLink
{
    Ptr<ThreeGppPropagationLossModel> propagation;
    Ptr<ThreeGppSpectrumPropagationLossModel> spectrum;

    Ptr<PhasedArrayModel> txAntenna;
    Ptr<PhasedArrayModel> rxAntenna;

    Ptr<NetDevice> txDev;
    Ptr<NetDevice> rxDev;

    double frequencyHz = 20e9;
    double bandwidthHz = 400e6;
    double rbBandwidthHz = 120e3;
    double txPowerDbm = 0.0;
    double rxNoiseFigureDb = 1.2;
};

static double
ComputeNtnSnrDb(SatNtnLink& link,
                Ptr<MobilityModel> txMob,
                Ptr<MobilityModel> rxMob,
                bool refreshBeamforming)
{
    if (refreshBeamforming)
    {
        DoBeamforming(link.txDev, link.txAntenna, link.rxDev);
        DoBeamforming(link.rxDev, link.rxAntenna, link.txDev);
    }

    Ptr<SpectrumValue> txPsd = CreateTxPowerSpectralDensity(link.frequencyHz,
                                                            link.txPowerDbm,
                                                            link.bandwidthHz,
                                                            link.rbBandwidthHz);
    Ptr<SpectrumValue> rxPsd = txPsd->Copy();

    Ptr<SpectrumValue> noisePsd = CreateNoisePowerSpectralDensity(link.frequencyHz,
                                                                  link.rxNoiseFigureDb,
                                                                  link.bandwidthHz,
                                                                  link.rbBandwidthHz);

    double propagationGainDb = link.propagation->CalcRxPower(0.0, txMob, rxMob);
    double propagationGainLinear = std::pow(10.0, propagationGainDb / 10.0);
    (*rxPsd) *= propagationGainLinear;

    Ptr<SpectrumSignalParameters> rxSsp = Create<SpectrumSignalParameters>();
    rxSsp->psd = rxPsd;
    rxSsp->txAntenna =
        ConstCast<AntennaModel, const AntennaModel>(link.txAntenna->GetAntennaElement());

    rxSsp = link.spectrum->CalcRxPowerSpectralDensity(rxSsp,
                                                      txMob,
                                                      rxMob,
                                                      link.txAntenna,
                                                      link.rxAntenna);

    return 10.0 * std::log10(Sum(*rxSsp->psd) / Sum(*noisePsd));
}

struct UavSatBackhaulCell
{
    uint16_t cellId = 0;
    Ptr<GeocentricConstantPositionMobilityModel> uavMob;
    SatNtnLink serviceDl; // SAT -> UAV
    SatNtnLink serviceUl; // UAV -> SAT
};

struct SatBackhaulContext
{
    std::vector<UavSatBackhaulCell> cells;

    SatNtnLink feederDl; // SAT -> GW
    SatNtnLink feederUl; // GW -> SAT

    Ptr<GeocentricConstantPositionMobilityModel> satMob;
    Ptr<GeocentricConstantPositionMobilityModel> gwMob;

    std::ofstream* file = nullptr;
    double logPeriodMs = 500.0;
    double minAcceptableBackhaulSnrDb = 0.0;
    uint32_t healthyNtnCapacity = 10;

    // NEW: also update initial-attach capacities inside NrHelper
    Ptr<NrHelper> initAttachNrHelper = nullptr;
};

static void
ApplySatelliteBackhaulState(SatBackhaulContext* ctx, bool writeLog)
{
    double t = Simulator::Now().GetSeconds();

    double feederDlSnrDb = ComputeNtnSnrDb(ctx->feederDl, ctx->satMob, ctx->gwMob, true);
    double feederUlSnrDb = ComputeNtnSnrDb(ctx->feederUl, ctx->gwMob, ctx->satMob, true);

    for (auto& cell : ctx->cells)
    {
        double serviceDlSnrDb = ComputeNtnSnrDb(cell.serviceDl, ctx->satMob, cell.uavMob, true);
        double serviceUlSnrDb = ComputeNtnSnrDb(cell.serviceUl, cell.uavMob, ctx->satMob, true);

        double backhaulDlSnrDb = std::min(serviceDlSnrDb, feederDlSnrDb);
        double backhaulUlSnrDb = std::min(serviceUlSnrDb, feederUlSnrDb);

        g_backhaulDlSnrDb[cell.cellId] = backhaulDlSnrDb;
        g_backhaulUlSnrDb[cell.cellId] = backhaulUlSnrDb;

        bool backhaulHealthy =
            (std::min(backhaulDlSnrDb, backhaulUlSnrDb) >= ctx->minAcceptableBackhaulSnrDb);

        uint32_t newCap = backhaulHealthy ? ctx->healthyNtnCapacity : 0;

        // 1) update xApp / HO side
        if (g_rsrpLm)
        {
            g_rsrpLm->SetCellCapacity(cell.cellId, newCap);
            g_rsrpLm->SetCellBackhaulDlSnrDb(cell.cellId, backhaulDlSnrDb);
            g_rsrpLm->SetCellBackhaulUlSnrDb(cell.cellId, backhaulUlSnrDb);
        }

        // 2) update initial-attach / retry-attach side
        if (ctx->initAttachNrHelper)
        {
            ctx->initAttachNrHelper->SetCellCapacity(cell.cellId, newCap);
        }

        if (writeLog && ctx->file && ctx->file->is_open())
        {
            (*ctx->file) << std::fixed << std::setprecision(6)
                         << t << ","
                         << cell.cellId << ","
                         << serviceDlSnrDb << ","
                         << serviceUlSnrDb << ","
                         << feederDlSnrDb << ","
                         << feederUlSnrDb << ","
                         << backhaulDlSnrDb << ","
                         << backhaulUlSnrDb << ","
                         << (backhaulHealthy ? 1 : 0)
                         << "\n";
        }
    }
}

static void
LogSatelliteBackhaul(SatBackhaulContext* ctx)
{
    ApplySatelliteBackhaulState(ctx, true);

    Simulator::Schedule(Seconds(ctx->logPeriodMs / 1000.0),
                        &LogSatelliteBackhaul,
                        ctx);
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
    double e2SendInterval = 2.0;
    double maxWaitTime = 0.010;
    double txDelay = 0.1;
    bool enableFlowMonitor = false;
    bool enableRsrpTrace = false;
    bool enablePositionTrace = true;
    bool enableHandoverTrace = true;
    bool enableHandoverFailureTrace = true;
    bool enableOranInfoLog = true;
    bool enableNrHelperInfoLog = false;
    bool enableSetupPrints = false;
    bool enableProgress = true;
    bool quietTiming = false;
    bool enableDecisionCsv = true;
    bool enableOranAppLossReports = true;
    bool enableOranCellLoadReports = true;
    bool enableDedicatedQosFlows = false;
    bool enablePdcpDiscarding = true;
    uint32_t discardTimerMs = 100;
    uint32_t reorderingTimerMs = 100;
    uint32_t maxRlcTxBufferSize = 10 * 1024 * 1024;
    std::string rlcMode = "um";
    double stopTailSeconds = 0.0;
    double progressIntervalSec = 1.0;
    bool enableFading = true;
    int channelUpdatePeriodMs = 100;
    int channelConditionUpdatePeriodMs = 200;
    bool enableFhControl = true;
    bool useFixedMcs = false;
    uint8_t fixedMcs = 0;
    bool enableSrsInUlSlots = true;
    bool enableSrsInFSlots = true;
    double txTnPower = 83.0;
    double txNtnPower = 78.0;
    double ueTxPower = 43.0;
    double initMinRsrpDbm = -160.0;
    double initRetryIntervalSec = 2.0;
    bool remMode = false; // [0]: REM disabled; [1]: generate REM
    int32_t remRbId = -1; // kept for compatibility (not used by this REM helper)
    // Important: keep the native NR handover algorithm disabled.
    // Handover decisions are made by the O-RAN logic module below. If an NR A3/A2-A4
    // algorithm is enabled at the same time, the UE may receive conflicting handover
    // decisions from two independent controllers.
    std::string handoverAlgorithm = "ns3::NrNoOpHandoverAlgorithm";
    Time simTime = Seconds(40);
    std::string dbFileName = "oran-repository-tn-ntn.db";
    std::string lateCommandPolicy = "DROP";
    // Deployment baselines used for the article comparison:
    //   tn-only          : UE + terrestrial cells only. No UAV cell is installed.
    //   tn-uav          : UE + terrestrial cells + UAV cell nodes. No satellite monitor.
    //   tn-uav-satellite: UE + terrestrial cells + UAV cell nodes + satellite backhaul monitor.
    std::string deploymentMode = "tn-uav-satellite"; // tn-only | tn-uav | tn-uav-satellite

    // xHaul monitor parameters. This first version estimates the UAV-to-ground-donor
    // RSRP from geometry and free-space path loss. It is a policy/trace monitor, not a
    // separate physical NR UE device mounted on the UAV. A later version can replace
    // this proxy with a co-located UAV-UE NetDevice attached to a TN donor gNB.
    double xhaulTxPowerDbm = 43.0;
    double xhaulFrequencyHz = 4.0e9;
    double xhaulHealthyRsrpDbm = -95.0;
    double xhaulDegradedRsrpDbm = -115.0;
    double xhaulTraceIntervalSec = 1.0;
    double xhaulMaxDonorDistanceM = 10000.0;

    // Optional synthetic degradation window for controlled experiments.
    // Example: start at 15 s, stop at 30 s, subtract 35 dB from the estimated
    // xHaul RSRP. This lets us compare mode switching and QoS under repeatable
    // xHaul degradation without changing UAV trajectories.
    double xhaulDegradationStartSec = -1.0;
    double xhaulDegradationStopSec = -1.0;
    double xhaulDegradationPenaltyDb = 25.0;
    bool enableXhaulChannelVariation = false;
    double xhaulShadowingStddevDb = 0.0;
    double xhaulFadingStddevDb = 0.0;

    // Optional TN access/infrastructure degradation window.
    // This reduces terrestrial gNB transmit power during the configured period.
    // It is useful for the TN-only baseline because it creates a repeatable
    // "terrestrial network is insufficient" interval for comparison against
    // UAV-assisted and satellite-assisted deployments.
    double tnDegradationStartSec = -1.0;
    double tnDegradationStopSec = -1.0;
    double tnDegradationPenaltyDb = 15.0;

    bool enableSatBackhaulMonitor = true;
    bool enableOnboardUavRic = true;
    double satBackhaulLogStepMs = 500.0;
    double satBackhaulMinSnrDb = 0.0;

    std::string satBackhaulScenario = "NTN-Suburban";
    double satBackhaulFrequencyHz = 20e9;
    double satBackhaulBandwidthHz = 400e6;
    double satBackhaulRbBandwidthHz = 120e3;

    double satEirpDensityDbwPerMHz = 40.0;
    double satAntennaGainDb = 58.5;
    double uavUtAntennaGainDb = 39.7;
    double gwAntennaGainDb = 45.0;

    double uavUtTxPowerDbm = 33.0;
    double gwTxPowerDbm = 46.0;

    double uavUtNoiseFigureDb = 1.2;
    double gwNoiseFigureDb = 1.2;
    double satRxNoiseFigureDb = 1.2;
    
    double groundAttachDelay = 6.0; // seconds
    // Scheduler CLI knobs (safe defaults to a concrete scheduler)
    bool ofdma = true;            // true=OFDMA, false=TDMA
    //In this scenario, BWPs already separate the main service types (voice, UES1 DL, UES1 UL).
    // Therefore, QoS scheduling is less critical than in a mixed-traffic single-BWP setup.
    // QoS scheduler becomes more useful when multiple traffic classes compete within the same BWP.
    std::string schedKind = "PF"; // RR | PF | MR | Qos
    // UES1 UL is configured lighter than UES1 DL: 1 Mbps / 30 fps vs 5 Mbps / 60 fps
    // double ueS1DlVideoRateMbps = 2.0;
    // uint16_t ueS1DlVideoFps = 30;
    // double ueS1UlVideoRateMbps = 0.5;
    // uint16_t ueS1UlVideoFps = 15;
    double hysteresisDb = 2.0; // dB, for RSRP-based handover decisions (if used)

    std::string xrAppType = "VR";  // VR | AR | CG (Cloud Gaming)
    std::string monitoredTraffic = "udp"; // udp | xr
    double monitoredDlRateMbps = 1.0;
    double monitoredUlRateMbps = 0.25;
    uint32_t monitoredPacketSizeBytes = 1000;
    double uavControlStartSec = 7.0;
    double uavControlPeriodSec = 2.0;
    double underservedRsrpThreshDbm = -120.0;

    CommandLine cmd;
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.AddValue("use-oran", "Indicates whether ORAN should be used or not", useOran);
    cmd.AddValue("use-onnx-lm", "Use the ONNX LM", useOnnx);
    cmd.AddValue("use-torch-lm", "Use the PyTorch LM", useTorch);
    cmd.AddValue("use-rsrp-lm", "Use the RSRP-based LM", useRsrp);
    cmd.AddValue("sim-time", "The duration for which traffic should flow", simTime);
    cmd.AddValue("lm-query-interval", "The LM query interval", lmQueryInterval);
    cmd.AddValue("e2-send-interval", "Interval between E2 report transmissions in seconds", e2SendInterval);
    cmd.AddValue("tx-delay", "The E2 terminator's transmission delay", txDelay);
    cmd.AddValue("handover-algorithm", "Specify which handover algorithm to use", handoverAlgorithm);
    cmd.AddValue("db-file", "Specify the DB file to create", dbFileName);
    cmd.AddValue("deployment-mode",
                 "Deployment baseline: tn-only | tn-uav | tn-uav-satellite",
                 deploymentMode);
    cmd.AddValue("xhaul-tx-power-dbm", "TN donor transmit power used by xHaul RSRP monitor", xhaulTxPowerDbm);
    cmd.AddValue("xhaul-frequency-hz", "Carrier frequency used by xHaul RSRP monitor", xhaulFrequencyHz);
    cmd.AddValue("xhaul-healthy-rsrp-dbm", "xHaul RSRP threshold for HEALTHY state", xhaulHealthyRsrpDbm);
    cmd.AddValue("xhaul-degraded-rsrp-dbm", "xHaul RSRP threshold for DEGRADED state", xhaulDegradedRsrpDbm);
    cmd.AddValue("xhaul-trace-interval", "xHaul/autonomy trace interval in seconds", xhaulTraceIntervalSec);
    cmd.AddValue("xhaul-max-donor-distance-m",
                 "Maximum UAV-to-TN donor distance for terrestrial xHaul connectivity",
                 xhaulMaxDonorDistanceM);
    cmd.AddValue("xhaul-degradation-start",
                 "Start time for synthetic xHaul degradation; negative disables it",
                 xhaulDegradationStartSec);
    cmd.AddValue("xhaul-degradation-stop",
                 "Stop time for synthetic xHaul degradation",
                 xhaulDegradationStopSec);
    cmd.AddValue("xhaul-degradation-penalty-db",
                 "RSRP penalty applied during synthetic xHaul degradation",
                 xhaulDegradationPenaltyDb);
    cmd.AddValue("enable-xhaul-channel-variation",
                 "Enable stochastic xHaul shadowing/fading variation in the UAV-to-TN donor RSRP proxy",
                 enableXhaulChannelVariation);
    cmd.AddValue("xhaul-shadowing-stddev-db",
                 "Standard deviation of xHaul log-normal shadowing in dB",
                 xhaulShadowingStddevDb);
    cmd.AddValue("xhaul-fading-stddev-db",
                 "Standard deviation of xHaul fast-fading loss proxy in dB",
                 xhaulFadingStddevDb);
    cmd.AddValue("tn-degradation-start",
                 "Start time for synthetic TN access degradation; negative disables it",
                 tnDegradationStartSec);
    cmd.AddValue("tn-degradation-stop",
                 "Stop time for synthetic TN access degradation",
                 tnDegradationStopSec);
    cmd.AddValue("tn-degradation-penalty-db",
                 "TN gNB TxPower penalty applied during synthetic TN degradation",
                 tnDegradationPenaltyDb);
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
    cmd.AddValue("tn-area-half-w-m", "Half-width of the terrestrial gNB placement area in meters", TN_GNB_HALF_W_M);
    cmd.AddValue("tn-area-half-h-m", "Half-height of the terrestrial gNB placement area in meters", TN_GNB_HALF_H_M);
    cmd.AddValue("uav-area-half-w-m", "Half-width of the UAV initial/mobility area in meters", UAV_AREA_HALF_W_M);
    cmd.AddValue("uav-area-half-h-m", "Half-height of the UAV initial/mobility area in meters", UAV_AREA_HALF_H_M);
    cmd.AddValue("ue-area-half-w-m", "Half-width of the UE mobility area in meters", UE_AREA_HALF_W_M);
    cmd.AddValue("ue-area-half-h-m", "Half-height of the UE mobility area in meters", UE_AREA_HALF_H_M);
    cmd.AddValue("uav-speed-mps", "UAV movement speed in meters per second", g_uavSpeedMps);
    cmd.AddValue("uav-mission-target-scale",
                 "Scale applied to underserved-UE cluster centroids when assigning UAV mission targets",
                 g_uavMissionTargetScale);
    cmd.AddValue("enable-sat-backhaul-monitor",
                 "Enable parallel satellite backhaul monitor for NTN/UAV gNBs",
                 enableSatBackhaulMonitor);
    cmd.AddValue("enable-onboard-uav-ric",
                 "Enable a separate simulated onboard UAV Near-RT RIC for UAV autonomy",
                 enableOnboardUavRic);
    cmd.AddValue("sat-backhaul-log-step-ms",
                 "Satellite backhaul logging period in ms",
                 satBackhaulLogStepMs);
    cmd.AddValue("sat-backhaul-min-snr-db",
                 "Minimum acceptable backhaul SNR (dB) for allowing NTN HO",
                 satBackhaulMinSnrDb);
    cmd.AddValue("sat-backhaul-scenario",
                 "Satellite backhaul scenario: NTN-DenseUrban | NTN-Urban | NTN-Suburban | NTN-Rural",
                 satBackhaulScenario);
    cmd.AddValue("mobility-update-ms", "Waypoint mobility update period in milliseconds", g_mobilityUpdateMs);
    cmd.AddValue("position-trace-interval", "UE/UAV position trace interval in seconds", g_positionTraceIntervalSec);
    cmd.AddValue("enable-flow-monitor", "Enable FlowMonitor and periodic QoS files", enableFlowMonitor);
    cmd.AddValue("enable-rsrp-trace", "Enable per-UE RSRP trace file", enableRsrpTrace);
    cmd.AddValue("enable-position-trace", "Enable periodic UE/UAV position trace files", enablePositionTrace);
    cmd.AddValue("enable-handover-trace", "Enable handover trace file", enableHandoverTrace);
    cmd.AddValue("enable-handover-failure-trace",
                 "Enable NR RRC handover failure trace file",
                 enableHandoverFailureTrace);
    cmd.AddValue("enable-oran-info-log", "Enable verbose INFO logging for the ORAN LM", enableOranInfoLog);
    cmd.AddValue("enable-nr-helper-info-log", "Enable verbose NrHelper INFO logging", enableNrHelperInfoLog);
    cmd.AddValue("enable-setup-prints", "Enable setup-time console prints", enableSetupPrints);
    cmd.AddValue("enable-progress", "Print lightweight simulation-time progress to stdout", enableProgress);
    cmd.AddValue("progress-interval", "Simulation seconds between progress prints", progressIntervalSec);
    cmd.AddValue("enable-decision-csv", "Write per-candidate handover decision CSV", enableDecisionCsv);
    cmd.AddValue("enable-oran-app-loss-reports", "Enable O-RAN app-loss reporters", enableOranAppLossReports);
    cmd.AddValue("enable-oran-cell-load-reports", "Enable O-RAN gNB cell-load reporters", enableOranCellLoadReports);
    cmd.AddValue("enable-dedicated-qos-flows",
                 "Install dedicated QoS flows for monitored/background UDP traffic. "
                 "Leave disabled for robust default-bearer KPI runs.",
                 enableDedicatedQosFlows);
    cmd.AddValue("quiet-timing", "Disable optional logs/traces/prints for wall-clock timing runs", quietTiming);
    cmd.AddValue("enable-pdcp-discarding", "Enable PDCP discarding for bounded UDP/XR queues", enablePdcpDiscarding);
    cmd.AddValue("pdcp-discard-timer-ms", "PDCP discard timer in milliseconds", discardTimerMs);
    cmd.AddValue("rlc-reordering-timer-ms", "RLC UM reordering timer in milliseconds", reorderingTimerMs);
    cmd.AddValue("rlc-max-tx-buffer-size", "Maximum RLC UM TX buffer size in bytes", maxRlcTxBufferSize);
    cmd.AddValue("rlc-mode",
                 "QoS-flow RLC mapping: um uses RLC_UM_ALWAYS; am uses RLC_AM_ALWAYS. "
                 "Use am for robust long handover/load comparison runs.",
                 rlcMode);
    cmd.AddValue("stop-tail", "Extra simulation seconds after sim-time for app/drain events", stopTailSeconds);
    cmd.AddValue("enable-fading", "Enable fast fading channel component", enableFading);
    cmd.AddValue("channel-update-ms", "3GPP channel matrix update period in milliseconds", channelUpdatePeriodMs);
    cmd.AddValue("channel-condition-update-ms", "3GPP LOS/NLOS channel condition update period in milliseconds", channelConditionUpdatePeriodMs);
    cmd.AddValue("enable-fh-control", "Enable 5G-LENA fronthaul control calculations", enableFhControl);
    cmd.AddValue("use-fixed-mcs", "Use fixed DL/UL MCS instead of adaptive AMC", useFixedMcs);
    cmd.AddValue("fixed-mcs", "Fixed MCS index used when --use-fixed-mcs=1", fixedMcs);
    cmd.AddValue("enable-srs-in-ul-slots", "Allow NR SRS scheduling in UL slots", enableSrsInUlSlots);
    cmd.AddValue("enable-srs-in-f-slots", "Allow NR SRS scheduling in flexible slots", enableSrsInFSlots);
    cmd.AddValue("tn-tx-power-dbm", "TN gNB access-link transmit power in dBm", txTnPower);
    cmd.AddValue("uav-tx-power-dbm", "UAV/NTN gNB access-link transmit power in dBm", txNtnPower);
    cmd.AddValue("ue-tx-power-dbm", "UE access-link transmit power in dBm", ueTxPower);
    cmd.AddValue("init-min-rsrp", "Minimum RSRP for initial attach in dBm", initMinRsrpDbm);
    cmd.AddValue("init-retry-interval", "Initial attach retry interval in seconds", initRetryIntervalSec);
    cmd.AddValue("monitored-traffic",
                 "Traffic model for monitored UES1 flows: udp | xr",
                 monitoredTraffic);
    cmd.AddValue("monitored-dl-rate-mbps",
                 "Downlink offered rate per monitored UE when --monitored-traffic=udp",
                 monitoredDlRateMbps);
    cmd.AddValue("monitored-ul-rate-mbps",
                 "Uplink offered rate per monitored UE when --monitored-traffic=udp",
                 monitoredUlRateMbps);
    cmd.AddValue("monitored-packet-size",
                 "UDP packet size in bytes when --monitored-traffic=udp",
                 monitoredPacketSizeBytes);
    cmd.AddValue("uav-control-start",
                 "Time when the UAV autonomy movement policy starts moving UAVs toward underserved UEs",
                 uavControlStartSec);
    cmd.AddValue("uav-control-period",
                 "Period of the UAV autonomy movement policy in seconds",
                 uavControlPeriodSec);
    cmd.AddValue("uav-underserved-rsrp-thresh-dbm",
                 "UE RSRP threshold below which the UAV autonomy policy treats a UE as underserved",
                 underservedRsrpThreshDbm);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(deploymentMode != "tn-only" &&
                        deploymentMode != "tn-uav" &&
                        deploymentMode != "tn-uav-satellite",
                    "Unsupported --deployment-mode. Use tn-only, tn-uav, or tn-uav-satellite.");
    g_deploymentMode = deploymentMode;
    g_satBackhaulMinSnrDb = satBackhaulMinSnrDb;
    g_enableOnboardUavRic = enableOnboardUavRic;
    g_enableXhaulChannelVariation = enableXhaulChannelVariation;
    g_xhaulShadowingStddevDb = xhaulShadowingStddevDb;
    g_xhaulFadingStddevDb = xhaulFadingStddevDb;
    if (deploymentMode == "tn-only")
    {
        numNtnGnbs = 0;
        enableSatBackhaulMonitor = false;
        enableOnboardUavRic = false;
    }
    else if (deploymentMode == "tn-uav")
    {
        enableSatBackhaulMonitor = false;
        enableOnboardUavRic = false;
    }
    else
    {
        enableSatBackhaulMonitor = true;
    }
    g_enableOnboardUavRic = enableOnboardUavRic;

    if (quietTiming)
    {
        verbose = false;
        enableFlowMonitor = false;
        enableRsrpTrace = false;
        enablePositionTrace = false;
        enableHandoverTrace = false;
        enableHandoverFailureTrace = false;
        enableOranInfoLog = false;
        enableNrHelperInfoLog = false;
        enableSetupPrints = false;
        enableProgress = true;
        enableDecisionCsv = false;
        enableOranAppLossReports = false;
        enableOranCellLoadReports = false;
        enableSatBackhaulMonitor = false;
        enableOnboardUavRic = false;
        enableFhControl = false;
        remMode = false;
    }
    g_enableOnboardUavRic = enableOnboardUavRic;

    NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
                    "Cannot use ML LM or RSRP LM without enabling O-RAN.");
    NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1, "Cannot use more than one LM simultaneously.");
    NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NrNoOpHandoverAlgorithm" && (useOnnx || useTorch || useRsrp),
                    "Cannot use non-noop handover algorithm with ML/RSRP LM (avoid conflicts).");
    NS_ABORT_MSG_IF(!enableFading,
                    "This scenario uses NrHelper::AttachToMaxRsrpGnb for initial attach, "
                    "which requires NR channel fading. Remove --enable-fading=0.");
    NS_ABORT_MSG_IF(rlcMode != "um" && rlcMode != "am",
                    "Unsupported --rlc-mode. Use um or am.");
    NS_ABORT_MSG_IF(monitoredTraffic != "udp" && monitoredTraffic != "xr",
                    "Unsupported --monitored-traffic. Use udp or xr.");

    Time simulationStopTime = simTime + Seconds(stopTailSeconds);

    std::ostringstream runTag;
    runTag << deploymentMode << "_ueS1_" << numGroundUesS1 << "_ueS2_" << numGroundUesS2 << "_tnGnb_" << numTnGnbs << "_ntnGnb_" << numNtnGnbs << "_tnCap_" << maxUesPerCellTn << "_ntnCap_" << maxUesPerCellNtn << "_hyst_" << hysteresisDb;

    // Base output folder for this run
    ns3_dir = "results/nr/tn-ntn/" + runTag.str() + "/";

    // Update file paths to be inside ns3_dir
    s_ueS1PositionTraceFile = ns3_dir + "ues1-position-trace.tr";
    s_ueS2PositionTraceFile = ns3_dir + "ues2-position-trace.tr";
    s_uavPositionTraceFile = ns3_dir + "uav-position-trace.tr";
    s_handoverTraceFile = ns3_dir + "handover-trace.tr";
    s_handoverFailureTraceFile = ns3_dir + "handover-failure-trace.tr";
    s_flowStatTraceFile = ns3_dir + "flow-stats.log";
    s_tnInfrastructureTraceFile = ns3_dir + "tn-infrastructure-trace.csv";
    s_satBackhaulTraceFile = ns3_dir + "sat-backhaul-trace.txt";
    s_xhaulAutonomyTraceFile = ns3_dir + "xhaul-autonomy-trace.csv";

    // Ensure results/nr/ directory exists
    std::filesystem::create_directories(ns3_dir);

    Ptr<OutputStreamWrapper> rsrpRsrqSinrTraceStream;
    if (enableRsrpTrace)
    {
        rsrpRsrqSinrTraceStream =
            Create<OutputStreamWrapper>(ns3_dir + "rsrp-trace.tr", std::ios::out);

        *rsrpRsrqSinrTraceStream->GetStream()
            << "Time RNTI CellId CellType RSRP RSRQ Serving CCID\n";
    }

    if (enableOranInfoLog || enableNrHelperInfoLog)
    {
        // Redirect enabled NS_LOG output to a file.
        g_nsLogFile.open(ns3_dir + "ns3-oran-lm.log", std::ios::out | std::ios::trunc);
        g_oldClogBuf = std::clog.rdbuf(g_nsLogFile.rdbuf());
        g_nsLogFile << std::unitbuf;
        std::clog << std::unitbuf;
    }

    // // ---- Redirect NS_LOG_UNCOND (std::cout) to a separate file ----
    // g_uncondFile.open(ns3_dir + "init-attach.log", std::ios::out | std::ios::trunc);
    // g_oldCoutBuf = std::cout.rdbuf(g_uncondFile.rdbuf());

    if (enableOranInfoLog)
    {
        LogComponentEnable("OranLmNr2NrRsrpHandoverWithTnNtn", LOG_LEVEL_INFO);
    }
    if (enableNrHelperInfoLog)
    {
        LogComponentEnable("NrHelper", LOG_LEVEL_INFO);
    }

    // Bound UDP/XR queue growth under overload. Unbounded RLC queues can consume RAM for days.
    Config::SetDefault("ns3::NrRlcUm::EnablePdcpDiscarding", BooleanValue(enablePdcpDiscarding));
    Config::SetDefault("ns3::NrRlcUm::DiscardTimerMs", UintegerValue(discardTimerMs));
    Config::SetDefault("ns3::NrRlcUm::ReorderingTimer", TimeValue(MilliSeconds(reorderingTimerMs)));
    // UM is lower-latency but can expose RLC reassembly/PDCP-header assertions in
    // heavy mobility and handover stress runs. AM is more conservative and is the
    // recommended mode when collecting paper-comparison QoS baselines.
    Config::SetDefault("ns3::NrGnbRrc::QosFlowToRlcMapping",
                       EnumValue(rlcMode == "um" ? NrGnbRrc::RLC_UM_ALWAYS
                                                  : NrGnbRrc::RLC_AM_ALWAYS));

    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(maxRlcTxBufferSize));
    //Config::SetDefault("ns3::NrGnbRrc::MaxUesPerCell", UintegerValue(maxUesPerCell));

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(channelUpdatePeriodMs)));

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
    Ptr<NrChannelHelper> ntnChannelHelper = CreateObject<NrChannelHelper>();
    ntnChannelHelper->ConfigureFactories("NTN-Urban", "Default");
    ntnChannelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(enableShadowing));

    tnChannelHelper->SetChannelConditionModelAttribute(
        "UpdatePeriod", TimeValue(MilliSeconds(channelConditionUpdatePeriodMs)));
    ntnChannelHelper->SetChannelConditionModelAttribute(
        "UpdatePeriod", TimeValue(MilliSeconds(channelConditionUpdatePeriodMs)));
    // }
    //ObjectFactory distanceBasedChannelFactory;
    
    // Create the NR helper
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    
    nrHelper->SetHandoverAlgorithmType(handoverAlgorithm);

    auto setSchedulerIfAvailable = [&](const std::string& name) -> bool {
        TypeId tid;
        if (TypeId::LookupByNameFailSafe(name, &tid))
        {
            if (enableSetupPrints)
            {
                NS_LOG_UNCOND(std::string("NR: trying ") + name);
            }
            nrHelper->SetSchedulerTypeId(tid); 
            if (enableSetupPrints)
            {
                NS_LOG_UNCOND(std::string("NR: using ") + name);
            }
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

    bool enableHarqRetx = false;

    nrHelper->SetSchedulerAttribute("EnableHarqReTx", BooleanValue(enableHarqRetx));
    nrHelper->SetSchedulerAttribute("EnableSrsInUlSlots", BooleanValue(enableSrsInUlSlots));
    nrHelper->SetSchedulerAttribute("EnableSrsInFSlots", BooleanValue(enableSrsInFSlots));
    //nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(txPower));
    //nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPower));

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

    if (enableFhControl)
    {
        nrHelper->EnableFhControl();
        nrHelper->SetFhControlAttribute("FhControlMethod", StringValue("OptimizeRBs"));
        nrHelper->SetFhControlAttribute("FhCapacity", UintegerValue(10000)); // 10 Gbps for XR
        nrHelper->SetFhControlAttribute("OverheadDyn", UintegerValue(32));    // or 100 if you want heavier overhead
    }


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
    // ---------------- Common BWP layout ----------------
    // Global/common indices for ALL nodes:
    //   BWP 0 = TN
    //   BWP 1 = NTN DL
    //   BWP 2 = NTN UL

    BandwidthPartInfoPtrVector allBwps;

    // ---- TDD + FDD setup ----
    uint8_t bandMask = NrChannelHelper::INIT_PROPAGATION |
                    (enableFading ? NrChannelHelper::INIT_FADING : 0);

    CcBwpCreator ccBwpCreator;

    // Frequencies
    double centralFrequencyBand1 = 4.0e9; // TN band
    double bandwidthBand1        = 20e6;

    double centralFrequencyBand2 = 4.2e9; // NTN band
    double bandwidthBand2        = 20e6;

    // TN band: 1 BWP
    CcBwpCreator::SimpleOperationBandConf bandConfTn(centralFrequencyBand1,
                                                    bandwidthBand1,
                                                    1);

    // NTN band: 2 BWPs (DL + UL)
    CcBwpCreator::SimpleOperationBandConf bandConfNtn(centralFrequencyBand2,
                                                    bandwidthBand2,
                                                    1);
    bandConfNtn.m_numBwp = 2;

    OperationBandInfo bandTn  = ccBwpCreator.CreateOperationBandContiguousCc(bandConfTn);
    OperationBandInfo bandNtn = ccBwpCreator.CreateOperationBandContiguousCc(bandConfNtn);

    std::vector<std::reference_wrapper<OperationBandInfo>> tnBands;
    tnBands.emplace_back(std::ref(bandTn));

    std::vector<std::reference_wrapper<OperationBandInfo>> ntnBands;
    ntnBands.emplace_back(std::ref(bandNtn));

    // Assign different channel models to different bands
    tnChannelHelper->AssignChannelsToBands(tnBands, bandMask);
    ntnChannelHelper->AssignChannelsToBands(ntnBands, bandMask);

    // Build ONE common/global BWP list
    std::vector<std::reference_wrapper<OperationBandInfo>> bands;
    bands.emplace_back(std::ref(bandTn));
    bands.emplace_back(std::ref(bandNtn));

    // Global indexing becomes:
    //   allBwps[0] = TN
    //   allBwps[1] = NTN DL
    //   allBwps[2] = NTN UL
    allBwps = CcBwpCreator::GetAllBwps(bands);

    // Global/common BWP indices
    const uint32_t bwpTn    = 0;
    const uint32_t bwpNtnDl = 1;
    const uint32_t bwpNtnUl = 2;
    const uint32_t numBwps  = allBwps.size(); // should be 3

    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
        TypeIdValue(QuasiOmniDirectPathBeamforming::GetTypeId()));
    if (enableFading)
    {
        nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    }

    // The network interface installed on the node (e.g., 5G modem)
    NetDeviceContainer tnGnbNrDevs;
    NetDeviceContainer ntnGnbNrDevs;
    NetDeviceContainer allGnbNrDevs;
    NetDeviceContainer groundNrDevsS1;
    NetDeviceContainer groundNrDevsS2;

    Ptr<HybridSatEpcHelper> epcHelper = CreateObject<HybridSatEpcHelper>();
    epcHelper->SetAttribute("TnS1uLinkDelay", TimeValue(MilliSeconds(0)));
    epcHelper->SetAttribute("NtnGnbSatDelay", TimeValue(MilliSeconds(120)));
    epcHelper->SetAttribute("NtnSatGwDelay", TimeValue(MilliSeconds(120)));
    epcHelper->SetAttribute("NtnGwSgwDelay", TimeValue(MilliSeconds(1)));

    nrHelper->SetEpcHelper(epcHelper);

    // Create remote host
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    // One InternetStackHelper only
    InternetStackHelper internet;
    internet.Install(remoteHostContainer);

    install_mobility_geocentric(remoteHostContainer,
                                tnGnbNodes,
                                ntnGnbNodes,
                                groundUeNodesS1,
                                groundUeNodesS2);

    // ---------------- SAT / GW nodes ----------------
    NodeContainer satNode;
    satNode.Create(1);

    NodeContainer gwNode;
    gwNode.Create(1);

    auto satMob = CreateObject<GeocentricConstantPositionMobilityModel>();
    satMob->SetGeographicPosition(Vector(g_refLat, g_refLon, 35786000.0));
    satMob->SetCoordinateTranslationReferencePoint(Vector(g_refLat, g_refLon, 0.0));
    satNode.Get(0)->AggregateObject(satMob);

    auto gwMob = CreateObject<GeocentricConstantPositionMobilityModel>();
    gwMob->SetGeographicPosition(Vector(g_refLat, g_refLon, 20.0));
    gwMob->SetCoordinateTranslationReferencePoint(Vector(g_refLat, g_refLon, 0.0));
    gwNode.Get(0)->AggregateObject(gwMob);

    NodeContainer backhaulNodes;
    backhaulNodes.Add(satNode);
    backhaulNodes.Add(gwNode);
    internet.Install(backhaulNodes);

    epcHelper->SetSatelliteNodes(satNode.Get(0), gwNode.Get(0));

    for (uint32_t i = 0; i < ntnGnbNodes.GetN(); ++i)
    {
        if (deploymentMode == "tn-uav-satellite")
        {
            // Satellite mode registers UAV gNBs as dual-backhaul nodes:
            // healthy xHaul uses the direct TN/core route, while degraded
            // xHaul can switch the same S1-U endpoint to SAT->GW->core.
            epcHelper->AddNtnGnbNode(ntnGnbNodes.Get(i));
        }
    }

    // Initialize after EPC / NTN registration is ready
    nrHelper->Initialize();

    Ptr<Node> pgw = epcHelper->GetPgwNode();

    // Connect PGW to remote host
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

    // ------------------------------------------------------------
    // Monitor-only point-to-point links for beamforming reference
    // ------------------------------------------------------------
    PointToPointHelper satMonP2p;
    satMonP2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("1Gbps")));
    satMonP2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(1)));

    std::vector<NetDeviceContainer> uavSatMonitorDevs(numNtnGnbs);
    for (uint32_t i = 0; i < numNtnGnbs; ++i)
    {
        uavSatMonitorDevs[i] = satMonP2p.Install(ntnGnbNodes.Get(i), satNode.Get(0));
    }

    NetDeviceContainer satGwMonitorDevs = satMonP2p.Install(satNode.Get(0), gwNode.Get(0));

    // ------------------------------------------------------------
    // Service BWP mapping
    // ------------------------------------------------------------
    // Keep BWP1/BWP2 available for NTN/REM experiments, but put the actual UE
    // service flows on BWP0. The satellite in this scenario is the UAV backhaul
    // fallback path, not the UE-facing NR access carrier. Mapping XR traffic to
    // the NTN access BWPs makes the low-altitude UAV/UE data path unrealistically
    // fragile and can leave the monitored DL flows with zero received packets.
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpTn));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(bwpTn));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(bwpTn));

    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpTn));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(bwpTn));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(bwpTn));

    // Install devices with ONE common BWP layout
    tnGnbNrDevs  = nrHelper->InstallGnbDevice(tnGnbNodes, allBwps);
    ntnGnbNrDevs = nrHelper->InstallGnbDevice(ntnGnbNodes, allBwps);

    // Start after measurements and initial attach have had a little time to appear.
    // For the satellite-assisted natural-degradation scenario this can be set
    // to 15 s, making 15-30 s the mission period where UAVs move toward weak
    // UE clusters and the UAV-to-TN donor xHaul can naturally weaken.
    Simulator::Schedule(Seconds(uavControlStartSec),
                        &UpdateUavTargetsFromUnderservedUes,
                        groundUeNodesS1,
                        groundUeNodesS2,
                        ntnGnbNrDevs,
                        underservedRsrpThreshDbm,
                        uavControlPeriodSec);

    allGnbNrDevs.Add(tnGnbNrDevs);
    allGnbNrDevs.Add(ntnGnbNrDevs);

    groundNrDevsS1 = nrHelper->InstallUeDevice(groundUeNodesS1, allBwps);
    groundNrDevsS2 = nrHelper->InstallUeDevice(groundUeNodesS2, allBwps);

    const uint32_t ueNumBwps = numBwps;

    // ------------------------------------------------------------
    // Tx power per family
    // TN gNBs: only BWP0 active
    // NTN gNBs: only BWP1 active for DL, BWP2 is UL-only
    // ------------------------------------------------------------
    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(tnGnbNrDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(txTnPower));
        nrHelper->GetGnbPhy(tnGnbNrDevs.Get(i), 1)->SetAttribute("TxPower", DoubleValue(txTnPower));
        nrHelper->GetGnbPhy(tnGnbNrDevs.Get(i), 2)->SetAttribute("TxPower", DoubleValue(0.0));
    }

    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(ntnGnbNrDevs.Get(i), 0)->SetAttribute("TxPower", DoubleValue(txNtnPower));
        nrHelper->GetGnbPhy(ntnGnbNrDevs.Get(i), 1)->SetAttribute("TxPower", DoubleValue(txNtnPower));
        nrHelper->GetGnbPhy(ntnGnbNrDevs.Get(i), 2)->SetAttribute("TxPower", DoubleValue(0.0)); // UL-only
    }

    {
        std::ofstream tnInfraOut(s_tnInfrastructureTraceFile, std::ios_base::trunc);
        tnInfraOut << "Time,TnDegradationActive,TnGnbTxPowerDbm,TnPenaltyDb\n";
    }
    ApplyTnInfrastructureDegradation(nrHelper,
                                     tnGnbNrDevs,
                                     txTnPower,
                                     tnDegradationPenaltyDb,
                                     false);
    if (tnDegradationStartSec >= 0.0 && tnDegradationStopSec > tnDegradationStartSec)
    {
        Simulator::Schedule(Seconds(tnDegradationStartSec),
                            &ApplyTnInfrastructureDegradation,
                            nrHelper,
                            tnGnbNrDevs,
                            txTnPower,
                            tnDegradationPenaltyDb,
                            true);
        Simulator::Schedule(Seconds(tnDegradationStopSec),
                            &ApplyTnInfrastructureDegradation,
                            nrHelper,
                            tnGnbNrDevs,
                            txTnPower,
                            tnDegradationPenaltyDb,
                            false);
    }

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

    // Register NTN cell IDs
    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
        NS_ABORT_MSG_IF(!gnb, "NTN device is not NrGnbNetDevice");
        g_ntnCellIds.insert(gnb->GetCellId());
    }

    // ------------------------------------------------------------
    // Parallel satellite backhaul monitor for each NTN/UAV gNB
    // ------------------------------------------------------------
    std::ofstream satBackhaulOut;
    std::unique_ptr<SatBackhaulContext> satBackhaulCtx;

    if (enableSatBackhaulMonitor)
    {
        satBackhaulOut.open(s_satBackhaulTraceFile, std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!satBackhaulOut.is_open(),
                        "Could not create " << s_satBackhaulTraceFile);

        satBackhaulOut << "Time,CellId,"
                    << "ServiceDlSnrDb,ServiceUlSnrDb,"
                    << "FeederDlSnrDb,FeederUlSnrDb,"
                    << "BackhaulDlSnrDb,BackhaulUlSnrDb,"
                    << "BackhaulHealthy\n";

        ObjectFactory propFactory;
        ObjectFactory condFactory;

        if (satBackhaulScenario == "NTN-DenseUrban")
        {
            propFactory.SetTypeId(ThreeGppNTNDenseUrbanPropagationLossModel::GetTypeId());
            condFactory.SetTypeId(ThreeGppNTNDenseUrbanChannelConditionModel::GetTypeId());
        }
        else if (satBackhaulScenario == "NTN-Urban")
        {
            propFactory.SetTypeId(ThreeGppNTNUrbanPropagationLossModel::GetTypeId());
            condFactory.SetTypeId(ThreeGppNTNUrbanChannelConditionModel::GetTypeId());
        }
        else if (satBackhaulScenario == "NTN-Suburban")
        {
            propFactory.SetTypeId(ThreeGppNTNSuburbanPropagationLossModel::GetTypeId());
            condFactory.SetTypeId(ThreeGppNTNSuburbanChannelConditionModel::GetTypeId());
        }
        else if (satBackhaulScenario == "NTN-Rural")
        {
            propFactory.SetTypeId(ThreeGppNTNRuralPropagationLossModel::GetTypeId());
            condFactory.SetTypeId(ThreeGppNTNRuralChannelConditionModel::GetTypeId());
        }
        else
        {
            NS_FATAL_ERROR("Unknown satBackhaulScenario: " << satBackhaulScenario);
        }

        Ptr<ChannelConditionModel> serviceCond =
            condFactory.Create<ThreeGppChannelConditionModel>();
        Ptr<ChannelConditionModel> feederCond =
            condFactory.Create<ThreeGppChannelConditionModel>();

        double satTxPowDbm =
            (satEirpDensityDbwPerMHz +
            10.0 * std::log10(satBackhaulBandwidthHz / 1e6) -
            satAntennaGainDb) + 30.0;

        Ptr<ThreeGppPropagationLossModel> servicePropagation =
            propFactory.Create<ThreeGppPropagationLossModel>();
        servicePropagation->SetAttribute("Frequency", DoubleValue(satBackhaulFrequencyHz));
        servicePropagation->SetAttribute("ShadowingEnabled", BooleanValue(true));
        servicePropagation->SetChannelConditionModel(serviceCond);

        Ptr<ThreeGppSpectrumPropagationLossModel> serviceSpectrum =
            CreateObject<ThreeGppSpectrumPropagationLossModel>();
        serviceSpectrum->SetChannelModelAttribute("Frequency", DoubleValue(satBackhaulFrequencyHz));
        serviceSpectrum->SetChannelModelAttribute("Scenario", StringValue(satBackhaulScenario));
        serviceSpectrum->SetChannelModelAttribute("ChannelConditionModel",
                                                PointerValue(serviceCond));

        Ptr<ThreeGppPropagationLossModel> feederPropagation =
            propFactory.Create<ThreeGppPropagationLossModel>();
        feederPropagation->SetAttribute("Frequency", DoubleValue(satBackhaulFrequencyHz));
        feederPropagation->SetAttribute("ShadowingEnabled", BooleanValue(true));
        feederPropagation->SetChannelConditionModel(feederCond);

        Ptr<ThreeGppSpectrumPropagationLossModel> feederSpectrum =
            CreateObject<ThreeGppSpectrumPropagationLossModel>();
        feederSpectrum->SetChannelModelAttribute("Frequency", DoubleValue(satBackhaulFrequencyHz));
        feederSpectrum->SetChannelModelAttribute("Scenario", StringValue(satBackhaulScenario));
        feederSpectrum->SetChannelModelAttribute("ChannelConditionModel",
                                                PointerValue(feederCond));

        auto MakeArray = [](double gainDb) -> Ptr<PhasedArrayModel>
        {
            return CreateObjectWithAttributes<UniformPlanarArray>(
                "NumColumns", UintegerValue(1),
                "NumRows", UintegerValue(1),
                "AntennaElement",
                PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
                    "Gain", DoubleValue(gainDb))));
        };

        satBackhaulCtx = std::make_unique<SatBackhaulContext>();
        satBackhaulCtx->satMob = satMob;
        satBackhaulCtx->gwMob = gwMob;
        satBackhaulCtx->file = &satBackhaulOut;
        satBackhaulCtx->logPeriodMs = satBackhaulLogStepMs;
        satBackhaulCtx->minAcceptableBackhaulSnrDb = satBackhaulMinSnrDb;
        satBackhaulCtx->healthyNtnCapacity = maxUesPerCellNtn;

        // Feeder DL: SAT -> GW
        satBackhaulCtx->feederDl.propagation = feederPropagation;
        satBackhaulCtx->feederDl.spectrum = feederSpectrum;
        satBackhaulCtx->feederDl.txDev = satGwMonitorDevs.Get(0);
        satBackhaulCtx->feederDl.rxDev = satGwMonitorDevs.Get(1);
        satBackhaulCtx->feederDl.frequencyHz = satBackhaulFrequencyHz;
        satBackhaulCtx->feederDl.bandwidthHz = satBackhaulBandwidthHz;
        satBackhaulCtx->feederDl.rbBandwidthHz = satBackhaulRbBandwidthHz;
        satBackhaulCtx->feederDl.txPowerDbm = satTxPowDbm;
        satBackhaulCtx->feederDl.rxNoiseFigureDb = gwNoiseFigureDb;
        satBackhaulCtx->feederDl.txAntenna = MakeArray(satAntennaGainDb);
        satBackhaulCtx->feederDl.rxAntenna = MakeArray(gwAntennaGainDb);

        // Feeder UL: GW -> SAT
        satBackhaulCtx->feederUl.propagation = feederPropagation;
        satBackhaulCtx->feederUl.spectrum = feederSpectrum;
        satBackhaulCtx->feederUl.txDev = satGwMonitorDevs.Get(1);
        satBackhaulCtx->feederUl.rxDev = satGwMonitorDevs.Get(0);
        satBackhaulCtx->feederUl.frequencyHz = satBackhaulFrequencyHz;
        satBackhaulCtx->feederUl.bandwidthHz = satBackhaulBandwidthHz;
        satBackhaulCtx->feederUl.rbBandwidthHz = satBackhaulRbBandwidthHz;
        satBackhaulCtx->feederUl.txPowerDbm = gwTxPowerDbm;
        satBackhaulCtx->feederUl.rxNoiseFigureDb = satRxNoiseFigureDb;
        satBackhaulCtx->feederUl.txAntenna = MakeArray(gwAntennaGainDb);
        satBackhaulCtx->feederUl.rxAntenna = MakeArray(satAntennaGainDb);

        for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
        {
            Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
            NS_ABORT_MSG_IF(!gnb, "NTN device is not NrGnbNetDevice");

            Ptr<GeocentricConstantPositionMobilityModel> uavMob =
                DynamicCast<GeocentricConstantPositionMobilityModel>(
                    ntnGnbNodes.Get(i)->GetObject<MobilityModel>());
            NS_ABORT_MSG_IF(!uavMob, "NTN node has no Geocentric mobility model");

            UavSatBackhaulCell cell;
            cell.cellId = gnb->GetCellId();
            cell.uavMob = uavMob;

            // Service DL: SAT -> UAV
            cell.serviceDl.propagation = servicePropagation;
            cell.serviceDl.spectrum = serviceSpectrum;
            cell.serviceDl.txDev = uavSatMonitorDevs[i].Get(1);
            cell.serviceDl.rxDev = uavSatMonitorDevs[i].Get(0);
            cell.serviceDl.frequencyHz = satBackhaulFrequencyHz;
            cell.serviceDl.bandwidthHz = satBackhaulBandwidthHz;
            cell.serviceDl.rbBandwidthHz = satBackhaulRbBandwidthHz;
            cell.serviceDl.txPowerDbm = satTxPowDbm;
            cell.serviceDl.rxNoiseFigureDb = uavUtNoiseFigureDb;
            cell.serviceDl.txAntenna = MakeArray(satAntennaGainDb);
            cell.serviceDl.rxAntenna = MakeArray(uavUtAntennaGainDb);

            // Service UL: UAV -> SAT
            cell.serviceUl.propagation = servicePropagation;
            cell.serviceUl.spectrum = serviceSpectrum;
            cell.serviceUl.txDev = uavSatMonitorDevs[i].Get(0);
            cell.serviceUl.rxDev = uavSatMonitorDevs[i].Get(1);
            cell.serviceUl.frequencyHz = satBackhaulFrequencyHz;
            cell.serviceUl.bandwidthHz = satBackhaulBandwidthHz;
            cell.serviceUl.rbBandwidthHz = satBackhaulRbBandwidthHz;
            cell.serviceUl.txPowerDbm = uavUtTxPowerDbm;
            cell.serviceUl.rxNoiseFigureDb = satRxNoiseFigureDb;
            cell.serviceUl.txAntenna = MakeArray(uavUtAntennaGainDb);
            cell.serviceUl.rxAntenna = MakeArray(satAntennaGainDb);

            satBackhaulCtx->cells.push_back(std::move(cell));
        }
    }

    if (enableFhControl)
    {
        nrHelper->ConfigureFhControl(allGnbNrDevs);
    }

    // for (auto it = allGnbNrDevs.Begin(); it != allGnbNrDevs.End(); ++it)
    // {
    //     Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(*it);
    //     NS_ABORT_MSG_IF(!gnb, "Device is not NrGnbNetDevice");

    //     gnb->GetNrFhControl()->TraceConnectWithoutContext(
    //         "RequiredFhDlThroughput",
    //         MakeCallback(&ReportFhTrace));

    //     gnb->GetNrFhControl()->TraceConnectWithoutContext(
    //         "UsedAirRbs",
    //         MakeCallback(&ReportAiTrace));
    // }

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

    // ---------------- Patterns ----------------
    static const std::string tddPattern   = "DL|DL|DL|DL|DL|DL|DL|UL|UL|UL|";
    static const std::string fddDlPattern = "DL|DL|DL|DL|DL|DL|DL|DL|DL|DL|";
    static const std::string fddUlPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";

    // TN gNBs: use BWP0 only
    for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NetDevice> gnbDev = tnGnbNrDevs.Get(i);
        nrHelper->GetGnbPhy(gnbDev, 0)->SetAttribute("Pattern", StringValue(tddPattern));
        nrHelper->GetGnbPhy(gnbDev, 1)->SetAttribute("Pattern", StringValue(fddDlPattern));
        nrHelper->GetGnbPhy(gnbDev, 2)->SetAttribute("Pattern", StringValue(fddUlPattern));
    }

    // NTN gNBs: use BWP1 for DL and BWP2 for UL
    for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
    {
        Ptr<NetDevice> gnbDev = ntnGnbNrDevs.Get(i);
        nrHelper->GetGnbPhy(gnbDev, 0)->SetAttribute("Pattern", StringValue(tddPattern));
        nrHelper->GetGnbPhy(gnbDev, 1)->SetAttribute("Pattern", StringValue(fddDlPattern));
        nrHelper->GetGnbPhy(gnbDev, 2)->SetAttribute("Pattern", StringValue(fddUlPattern));
    }

    // ---------------- Output links ----------------
    // Global/common mapping is safe again: UL BWP2 -> DL BWP1
    for (uint32_t i = 0; i < allGnbNrDevs.GetN(); ++i)
    {
        NrHelper::GetBwpManagerGnb(allGnbNrDevs.Get(i))->SetOutputLink(2, 1);
    }

    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        NrHelper::GetBwpManagerUe(groundNrDevsS1.Get(i))->SetOutputLink(1, 2);
    }

    for (uint32_t i = 0; i < groundNrDevsS2.GetN(); ++i)
    {
        NrHelper::GetBwpManagerUe(groundNrDevsS2.Get(i))->SetOutputLink(1, 2);
    }

    // Apply final configuration after set patterns + output links
    for (auto it = allGnbNrDevs.Begin(); it != allGnbNrDevs.End(); ++it)
    {
        Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(*it);
        if (gnb)
        {
            gnb->UpdateConfig();
        }
    }

    for (auto it = groundNrDevsS1.Begin(); it != groundNrDevsS1.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue)
        {
            ue->UpdateConfig();
        }
    }

    for (auto it = groundNrDevsS2.Begin(); it != groundNrDevsS2.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue)
        {
            ue->UpdateConfig();
        }
    }

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

    // Set global fallback to 0
    nrHelper->SetAttribute("InitMaxUesPerCell", UintegerValue(0));
    nrHelper->SetAttribute("InitMinRsrpDbm",    DoubleValue(initMinRsrpDbm));
    nrHelper->SetAttribute("InitRetryInterval", TimeValue(Seconds(initRetryIntervalSec)));
    nrHelper->SetAttribute("InitAttachLogging", BooleanValue(enableSetupPrints));

    // ------------------------------------------------------------
    // INITIAL backhaul-aware preload for attach/retry-attach
    // This makes t=0 attach and later retries see the current NTN backhaul state.
    // ------------------------------------------------------------
    if (enableSatBackhaulMonitor && satBackhaulCtx)
    {
        satBackhaulCtx->initAttachNrHelper = nrHelper;

        // Apply current backhaul state NOW, but do not write a log line at t=0
        ApplySatelliteBackhaulState(satBackhaulCtx.get(), false);
    }

    Time tLateAttach = Seconds(groundAttachDelay);

    if (enableSetupPrints)
    {
        for (uint32_t i = 0; i < allGnbNodes.GetN(); ++i)
        {
            Ptr<Node> n = allGnbNodes.Get(i);
            std::cout << "Node " << n->GetId() << " devices:\n";
            for (uint32_t d = 0; d < n->GetNDevices(); ++d)
            {
                Ptr<NetDevice> dev = n->GetDevice(d);
                std::cout << "  dev " << d
                        << " type=" << dev->GetInstanceTypeId().GetName() << "\n";
            }
        }
    }

    // X2/Xn-style interface required by 5G-LENA for inter-gNB NR handover.
    // The O-RAN command selects the target; this interface lets the simulated
    // RAN execute the handover between TN and UAV/NTN cells.
    nrHelper->AddX2Interface(allGnbNodes);

    internet.Install(groundUeNodesS1);
    internet.Install(groundUeNodesS2);

    Ipv4InterfaceContainer ueIpIfaceS1;
    Ipv4InterfaceContainer ueIpIfaceS2;
    ueIpIfaceS1 = epcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNrDevsS1));
    ueIpIfaceS2 = epcHelper->AssignUeIpv4Address(NetDeviceContainer(groundNrDevsS2));

    // -------- IP -> IMSI map --------
    g_ipToImsi.clear();

    for (uint32_t i = 0; i < groundNrDevsS1.GetN(); ++i)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(groundNrDevsS1.Get(i));
        NS_ABORT_MSG_IF(!ue, "groundNrDevsS1[" << i << "] not NrUeNetDevice");
        Ipv4Address ip = ueIpIfaceS1.GetAddress(i);
        g_ipToImsi[ip.Get()] = ue->GetImsi();
    }

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

    for (uint32_t u = 0; u < groundUeNodesS2.GetN(); ++u)
    {
        Ptr<Node> ueNode = groundUeNodesS2.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // remoteHost IP address on the PGW-remoteHost point-to-point link
    Ipv4Address remoteHostIp = internetIpIfaces.GetAddress(1);

    // XR Application containers
    ApplicationContainer xrDlSinks;
    ApplicationContainer xrUlSinks;
    ApplicationContainer xrDlSenders;
    ApplicationContainer xrUlSenders;
    ApplicationContainer xrPingApps;

    // XR configuration type
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

        // Seed the EPC/ARP path before XR traffic starts. The 5G-LENA XR
        // examples use the same workaround so the first application packets are
        // not lost while neighbor/ARP state is still cold.
        PingHelper ping(ueIpIfaceS1.GetAddress(i));
        xrPingApps.Add(ping.Install(remoteHostContainer));

        if (monitoredTraffic == "udp")
        {
            UdpServerHelper dlServer(dlPort);
            xrDlSinks.Add(dlServer.Install(groundUeNodesS1.Get(i)));

            UdpClientHelper dlClient(ueIpIfaceS1.GetAddress(i), dlPort);
            dlClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            dlClient.SetAttribute(
                "Interval",
                TimeValue(Seconds(static_cast<double>(monitoredPacketSizeBytes) * 8.0 /
                                  (monitoredDlRateMbps * 1e6))));
            dlClient.SetAttribute("PacketSize", UintegerValue(monitoredPacketSizeBytes));
            xrDlSenders.Add(dlClient.Install(remoteHost));

            UdpServerHelper ulServer(ulPort);
            xrUlSinks.Add(ulServer.Install(remoteHost));

            UdpClientHelper ulClient(remoteHostIp, ulPort);
            ulClient.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
            ulClient.SetAttribute(
                "Interval",
                TimeValue(Seconds(static_cast<double>(monitoredPacketSizeBytes) * 8.0 /
                                  (monitoredUlRateMbps * 1e6))));
            ulClient.SetAttribute("PacketSize", UintegerValue(monitoredPacketSizeBytes));
            xrUlSenders.Add(ulClient.Install(groundUeNodesS1.Get(i)));
        }
        else
        {
            PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), dlPort));
            xrDlSinks.Add(dlSink.Install(groundUeNodesS1.Get(i)));

            XrTrafficMixerHelper xrDlHelper;
            xrDlHelper.ConfigureXr(dlConfig);
            std::vector<Address> dlAddresses;
            dlAddresses.push_back(InetSocketAddress(ueIpIfaceS1.GetAddress(i), dlPort));
            xrDlSenders.Add(
                xrDlHelper.Install("ns3::UdpSocketFactory", dlAddresses, remoteHost));

            PacketSinkHelper ulSink("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), ulPort));
            xrUlSinks.Add(ulSink.Install(remoteHost));

            XrTrafficMixerHelper xrUlHelper;
            xrUlHelper.ConfigureXr(ulConfig);
            std::vector<Address> ulAddresses;
            ulAddresses.push_back(InetSocketAddress(remoteHostIp, ulPort));
            xrUlSenders.Add(
                xrUlHelper.Install("ns3::UdpSocketFactory", ulAddresses, groundUeNodesS1.Get(i)));
        }

        // Global/common BWP mapping
        NrQosFlow xrDlFlow(NrQosFlow::GBR_CONV_VIDEO);
        Ptr<NrQosRule> xrDlRule = Create<NrQosRule>();
        NrQosRule::PacketFilter dlpf;
        dlpf.localPortStart = dlPort;
        dlpf.localPortEnd   = dlPort;
        dlpf.direction      = NrQosRule::DOWNLINK;
        xrDlRule->Add(dlpf);

        NrQosFlow xrUlFlow(NrQosFlow::GBR_LIVE_UL_71);
        Ptr<NrQosRule> xrUlRule = Create<NrQosRule>();
        NrQosRule::PacketFilter ulpf;
        ulpf.remotePortStart = ulPort;
        ulpf.remotePortEnd   = ulPort;
        ulpf.direction       = NrQosRule::UPLINK;
        xrUlRule->Add(ulpf);

        Ptr<NetDevice> ueDev = groundNrDevsS1.Get(i);
        if (enableDedicatedQosFlows)
        {
            nrHelper->ActivateDedicatedQosFlow(ueDev, xrDlFlow, xrDlRule);
            nrHelper->ActivateDedicatedQosFlow(ueDev, xrUlFlow, xrUlRule);
        }
    }

    // Ground UEs traffic
    uint16_t groundBasePort = 20000;
    ApplicationContainer ueAppsS2;
    ApplicationContainer groundRemoteAppsS2;

    for (uint16_t i = 0; i < groundUeNodesS2.GetN(); i++)
    {
        uint16_t port = groundBasePort + i;

        Ptr<NetDevice> gUeDev = groundNrDevsS2.Get(i);
        NrQosFlow voiceFlow(NrQosFlow::GBR_CONV_VOICE);
        Ptr<NrQosRule> voiceRule = Create<NrQosRule>();
        NrQosRule::PacketFilter gdl;
        gdl.localPortStart = port;
        gdl.localPortEnd   = port;
        voiceRule->Add(gdl);

        if (enableDedicatedQosFlows)
        {
            nrHelper->ActivateDedicatedQosFlow(gUeDev, voiceFlow, voiceRule);
        }

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
        ueAppsS2.Add(dlSink.Install(groundUeNodesS2.Get(i)));

        TrafficGeneratorHelper voiceHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(ueIpIfaceS2.GetAddress(i), port),
                                        TrafficGeneratorNgmnVoip::GetTypeId());

        ApplicationContainer voiceApps = voiceHelper.Install(remoteHost);
        groundRemoteAppsS2.Add(voiceApps);
    }

    // ---------------------------------------------------------------------
    // Initial association
    // ---------------------------------------------------------------------
    // Dedicated QoS flows must be installed before initial attach because this
    // NR stack does not implement activating new NAS QoS flows after the
    // initial UE context has already been established.
    nrHelper->AttachToMaxRsrpGnb(groundNrDevsS1, allGnbNrDevs);

    // UES2 is attached later to create background load after the monitored UES1
    // flows have started. This helps stress handover and load-aware decisions.
    Simulator::Schedule(tLateAttach, [nrHelper, groundNrDevsS2, allGnbNrDevs]() {
        nrHelper->AttachToMaxRsrpGnb(groundNrDevsS2, allGnbNrDevs);
    });

    // Start/stop times
    xrDlSinks.Start(Seconds(1.0));
    xrDlSinks.Stop(simulationStopTime);

    xrPingApps.Start(Seconds(1.0));
    xrPingApps.Stop(Seconds(3.8));

    xrDlSenders.Start(Seconds(4.0));
    xrDlSenders.Stop(simulationStopTime);

    xrUlSinks.Start(Seconds(1.0));
    xrUlSinks.Stop(simulationStopTime);

    xrUlSenders.Start(Seconds(4.5));
    xrUlSenders.Stop(simulationStopTime);

    ueAppsS2.Start(Seconds(1));
    groundRemoteAppsS2.Start(tLateAttach + Seconds(0.5));

    groundRemoteAppsS2.Stop(simulationStopTime);
    ueAppsS2.Stop(simulationStopTime);


    g_uavAutonomyXappCtx.reset();
    if (ntnGnbNrDevs.GetN() > 0)
    {
        g_uavAutonomyXappCtx = std::make_unique<UavAutonomyXappContext>();
        g_uavAutonomyXappCtx->tnGnbs = tnGnbNodes;
        g_uavAutonomyXappCtx->uavs = ntnGnbNodes;
        g_uavAutonomyXappCtx->tnGnbNrDevs = tnGnbNrDevs;
        g_uavAutonomyXappCtx->uavGnbNrDevs = ntnGnbNrDevs;
        g_uavAutonomyXappCtx->epcHelper = epcHelper;
        g_uavAutonomyXappCtx->xhaulTxPowerDbm = xhaulTxPowerDbm;
        g_uavAutonomyXappCtx->xhaulFrequencyHz = xhaulFrequencyHz;
        g_uavAutonomyXappCtx->xhaulMaxDonorDistanceM = xhaulMaxDonorDistanceM;
        g_uavAutonomyXappCtx->healthyThresholdDbm = xhaulHealthyRsrpDbm;
        g_uavAutonomyXappCtx->degradedThresholdDbm = xhaulDegradedRsrpDbm;
        g_uavAutonomyXappCtx->degradationStartSec = xhaulDegradationStartSec;
        g_uavAutonomyXappCtx->degradationStopSec = xhaulDegradationStopSec;
        g_uavAutonomyXappCtx->degradationPenaltyDb = xhaulDegradationPenaltyDb;
        g_uavAutonomyXappCtx->e2TxDelaySec = txDelay;
        g_uavAutonomyXappCtx->e2SendIntervalSec = e2SendInterval;
        g_uavAutonomyXappCtx->lmQueryIntervalSec = lmQueryInterval;
        g_uavAutonomyXappCtx->enableXhaulChannelVariation = enableXhaulChannelVariation;
        g_uavAutonomyXappCtx->xhaulShadowingStddevDb = xhaulShadowingStddevDb;
        g_uavAutonomyXappCtx->xhaulFadingStddevDb = xhaulFadingStddevDb;
        g_uavAutonomyXappCtx->enableOnboardUavRic = enableOnboardUavRic;
    }


    // ORAN BEGIN
    if (useOran == true)
    {
        // -----------------------------------------------------------------
        // Near-RT RIC / xApp-like control loop
        // -----------------------------------------------------------------
        // In this example, the O-RAN logic module acts like a simple xApp:
        //   - E2 node terminators collect UE/gNB reports.
        //   - The Near-RT RIC periodically invokes the logic module.
        //   - The logic module selects a target cell.
        //   - The CMM sends an NR-to-NR handover command for execution.
        //
        // This is why the native NR handover algorithm is left as no-op.
        // We want the O-RAN loop to be the only runtime handover controller.
        if (!dbFileName.empty())
        {
            ::remove(dbFileName.c_str());
        }

        // Default to a no-op LM, then replace it below if RSRP/ONNX/Torch mode
        // is requested. For the article baseline, useRsrp=true is the expected
        // setting because it gives an explainable RSRP/hysteresis handover.
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
            // Main handover policy used by this scenario.
            // It is RSRP-driven, but it also understands TN/NTN cell labels and
            // per-cell capacity maps. Satellite backhaul monitoring can update
            // NTN cell capacity to 0 when satellite backhaul is unhealthy.
            defaultLmTid = TypeId::LookupByName("ns3::OranLmNr2NrRsrpHandoverWithTnNtn");
        }

        if (useTorch)
        {
            auto path = StringValue(ns3_dir + "saved_trained_classification_pytorch.pt");
            Config::SetDefault("ns3::OranLmNr2NrTorchHandover::TorchModelPath", StringValue(path));
        }
        ObjectFactory defaultLmFactory;
        defaultLmFactory.SetTypeId(defaultLmTid);
        defaultLm = defaultLmFactory.Create<OranLm>();

        // Disable one global capacity value because this scenario has separate
        // TN and UAV/NTN capacity maps. Each cell receives its own limit below.
        defaultLm->SetAttribute("MaxUesPerCell", UintegerValue(0)); // per-cell map takes over

        if (useRsrp)
        {
            Ptr<OranLmNr2NrRsrpHandoverWithTnNtn> rsrpLm =
                DynamicCast<OranLmNr2NrRsrpHandoverWithTnNtn>(defaultLm);

            if (rsrpLm)
            {
                g_rsrpLm = rsrpLm;

                if (enableDecisionCsv)
                {
                    // Candidate-level handover decision trace. Despite the old
                    // filename, this is useful for both ML dataset generation
                    // and plain rule-based baseline analysis.
                    g_rsrpLm->SetDecisionCsvFilename(ns3_dir + "ml-ho-dataset.csv");
                }

                // Tell the LM which cells are terrestrial donor cells and which
                // cells are UAV/NTN cells. This supports separate TN/UAV capacity
                // limits and lets later analysis distinguish TN->UAV, UAV->TN,
                // and UAV->UAV handovers.
                for (uint32_t i = 0; i < tnGnbNrDevs.GetN(); ++i)
                {
                    Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(tnGnbNrDevs.Get(i));
                    rsrpLm->SetCellCapacity(gnb->GetCellId(), maxUesPerCellTn);
                    rsrpLm->SetCellIsNtn(gnb->GetCellId(), false);
                }

                for (uint32_t i = 0; i < ntnGnbNrDevs.GetN(); ++i)
                {
                    Ptr<NrGnbNetDevice> gnb = DynamicCast<NrGnbNetDevice>(ntnGnbNrDevs.Get(i));
                    rsrpLm->SetCellCapacity(gnb->GetCellId(), maxUesPerCellNtn);
                    rsrpLm->SetCellIsNtn(gnb->GetCellId(), true);
                }
            }
        }

        // If the best RSRP target is full or disallowed, try another candidate
        // instead of immediately keeping the current serving cell.
        defaultLm->SetAttribute("TryNextBest", BooleanValue(true)); // try next best otherwise keep current

        // Reject very weak handover targets even if they are technically the
        // strongest candidate. This prevents moving a UE to an unusable cell.
        defaultLm->SetAttribute("MinAcceptableRsrpDbm", DoubleValue(-120.0)); // default is -120 dbm

        // Hysteresis protects against ping-pong handovers by requiring the
        // target cell to be better than the current cell by this margin.
        defaultLm->SetAttribute("HysteresisDb", DoubleValue(hysteresisDb)); // default is 2 dbm

        dataRepository->SetAttribute("DatabaseFile", StringValue(dbFileName));
        defaultLm->SetName("UE_MOBILITY_XAPP");
        defaultLm->SetAttribute("Verbose", BooleanValue(verbose));
        defaultLm->SetAttribute("NearRtRic", PointerValue(nearRtRic));

        const bool useSeparateOnboardUavRic =
            enableOnboardUavRic && deploymentMode == "tn-uav-satellite" && ntnGnbNrDevs.GetN() > 0;

        Ptr<OranLmUavAutonomyControl> uavAutonomyLm = CreateObject<OranLmUavAutonomyControl>();
        uavAutonomyLm->SetAttribute("Verbose", BooleanValue(verbose));

        cmm->SetAttribute("NearRtRic", PointerValue(nearRtRic));

        nearRtRicE2Terminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
        nearRtRicE2Terminator->SetAttribute("DataRepository", PointerValue(dataRepository));
        nearRtRicE2Terminator->SetAttribute(
            "TransmissionDelayRv",
            StringValue("ns3::ConstantRandomVariable[Constant=" + std::to_string(txDelay) + "]"));

        // Terrestrial Near-RT RIC. In the satellite-assisted case this RIC
        // keeps the UE Mobility xApp, while a separate onboard-UAV RIC below
        // runs UAV autonomy. In non-satellite UAV scenarios, both xApps remain
        // in this RIC and autonomy runs first.
        if (useSeparateOnboardUavRic)
        {
            nearRtRic->SetAttribute("DefaultLogicModule", PointerValue(defaultLm));
        }
        else
        {
            uavAutonomyLm->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nearRtRic->SetAttribute("DefaultLogicModule", PointerValue(uavAutonomyLm));
            nearRtRic->AddLogicModule(defaultLm);
        }
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


        if (useSeparateOnboardUavRic)
        {
            std::string onboardUavRicDbFile = ns3_dir + "onboard-uav-ric-repository.db";
            ::remove(onboardUavRicDbFile.c_str());

            Ptr<OranDataRepository> onboardUavDataRepository =
                CreateObject<OranDataRepositorySqlite>();
            Ptr<OranCmm> onboardUavCmm = CreateObject<OranCmmHandover>();
            Ptr<OranNearRtRic> onboardUavRic = CreateObject<OranNearRtRic>();
            Ptr<OranNearRtRicE2Terminator> onboardUavE2Terminator =
                CreateObject<OranNearRtRicE2Terminator>();

            onboardUavDataRepository->SetAttribute("DatabaseFile",
                                                   StringValue(onboardUavRicDbFile));
            uavAutonomyLm->SetAttribute("NearRtRic", PointerValue(onboardUavRic));
            onboardUavCmm->SetAttribute("NearRtRic", PointerValue(onboardUavRic));
            onboardUavE2Terminator->SetAttribute("NearRtRic", PointerValue(onboardUavRic));
            onboardUavE2Terminator->SetAttribute("DataRepository",
                                                 PointerValue(onboardUavDataRepository));
            onboardUavE2Terminator->SetAttribute(
                "TransmissionDelayRv",
                StringValue("ns3::ConstantRandomVariable[Constant=" +
                            std::to_string(txDelay) + "]"));

            onboardUavRic->SetAttribute("DefaultLogicModule", PointerValue(uavAutonomyLm));
            onboardUavRic->SetAttribute("E2Terminator", PointerValue(onboardUavE2Terminator));
            onboardUavRic->SetAttribute("DataRepository",
                                        PointerValue(onboardUavDataRepository));
            onboardUavRic->SetAttribute("LmQueryInterval",
                                        TimeValue(Seconds(lmQueryInterval)));
            onboardUavRic->SetAttribute("ConflictMitigationModule",
                                        PointerValue(onboardUavCmm));
            onboardUavRic->SetAttribute("E2NodeInactivityThreshold", TimeValue(Seconds(2)));
            onboardUavRic->SetAttribute("E2NodeInactivityIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=2]"));
            onboardUavRic->SetAttribute("LmQueryMaxWaitTime",
                                        TimeValue(Seconds(maxWaitTime)));
            onboardUavRic->SetAttribute("LmQueryLateCommandPolicy",
                                        StringValue(lateCommandPolicy));

            // Start the onboard UAV RIC slightly before the terrestrial RIC so
            // UAV availability is updated before UE mobility decisions in the
            // same nominal RIC control cycle.
            Simulator::Schedule(Seconds(2.4), &OranNearRtRic::Start, onboardUavRic);
        }

        Simulator::Schedule(Seconds(2.5), &OranNearRtRic::Start, nearRtRic);

        for (uint32_t idx = 0; idx < groundUeNodesS1.GetN(); idx++)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrUeCellInfo> nrUeCellInfoReporter = CreateObject<OranReporterNrUeCellInfo>();
            Ptr<OranReporterAppLoss> appLossReporter =
                enableOranAppLossReports ? CreateObject<OranReporterAppLoss>() : nullptr;
            Ptr<OranReporterNrUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterNrUeRsrpRsrq>();
            Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));

            if (enableOranAppLossReports)
            {
                appLossReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
                xrDlSenders.Get(idx)->TraceConnectWithoutContext(
                    "Tx", MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
                xrDlSinks.Get(idx)->TraceConnectWithoutContext(
                    "Rx", MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));
            }
          
            //The UES1’s physical layer (NrUePhy) periodically measures:RSRP (signal strength),
                                                                    //RSRQ (signal quality),
                                                                    //SINR (interference/noise level).
            for (uint32_t netDevIdx = 0; netDevIdx < groundUeNodesS1.Get(idx)->GetNDevices(); ++netDevIdx)
            {
                Ptr<NrUeNetDevice> nrUeDevice =
                    groundUeNodesS1.Get(idx)->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();

                if (nrUeDevice)
                {
                    for (uint32_t b = 0; b < ueNumBwps; ++b)
                    {
                        Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(b);
                        if (uePhy)
                        {
                            uePhy->TraceConnectWithoutContext(
                                "ReportUeMeasurements",
                                MakeCallback(&ns3::OranReporterNrUeRsrpRsrq::ReportRsrpRsrq,
                                            rsrpRsrqReporter));
                        }
                    }
                }
            }

            nrUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrUeTerminator->SetAttribute("RegistrationIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrUeTerminator->SetAttribute("SendIntervalRv",
                                         StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                     std::to_string(e2SendInterval) + "]"));

            nrUeTerminator->AddReporter(locationReporter);
            nrUeTerminator->AddReporter(nrUeCellInfoReporter);
            nrUeTerminator->AddReporter(rsrpRsrqReporter);
            if (enableOranAppLossReports)
            {
                nrUeTerminator->AddReporter(appLossReporter);
            }
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
            Ptr<OranReporterAppLoss> appLossReporter =
                enableOranAppLossReports ? CreateObject<OranReporterAppLoss>() : nullptr;
            Ptr<OranReporterNrUeRsrpRsrq> rsrpRsrqReporter = CreateObject<OranReporterNrUeRsrpRsrq>();
            Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));

            if (enableOranAppLossReports)
            {
                appLossReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
                groundRemoteAppsS2.Get(idx)->TraceConnectWithoutContext(
                    "Tx", MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
                ueAppsS2.Get(idx)->TraceConnectWithoutContext(
                    "Rx", MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));
            }

            // RSRP/RSRQ measurements from the ground UE PHY
            for (uint32_t netDevIdx = 0; netDevIdx < groundUeNodesS2.Get(idx)->GetNDevices(); ++netDevIdx)
            {
                Ptr<NrUeNetDevice> nrUeDevice =
                    groundUeNodesS2.Get(idx)->GetDevice(netDevIdx)->GetObject<NrUeNetDevice>();

                if (nrUeDevice)
                {
                    for (uint32_t b = 0; b < ueNumBwps; ++b)
                    {
                        Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(b);
                        if (uePhy)
                        {
                            uePhy->TraceConnectWithoutContext(
                                "ReportUeMeasurements",
                                MakeCallback(&ns3::OranReporterNrUeRsrpRsrq::ReportRsrpRsrq,
                                            rsrpRsrqReporter));
                        }
                    }
                }
            }

            nrUeTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrUeTerminator->SetAttribute("RegistrationIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrUeTerminator->SetAttribute("SendIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(e2SendInterval) + "]"));

            nrUeTerminator->AddReporter(locationReporter);
            nrUeTerminator->AddReporter(nrUeCellInfoReporter);
            nrUeTerminator->AddReporter(rsrpRsrqReporter);
            if (enableOranAppLossReports)
            {
                nrUeTerminator->AddReporter(appLossReporter);
            }

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

        // TN gNB reporters
        for (uint32_t idx = 0; idx < tnGnbNodes.GetN(); ++idx)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrCellLoad> nrCellLoadReporter =
                enableOranCellLoadReports ? CreateObject<OranReporterNrCellLoad>() : nullptr;
            Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            if (enableOranCellLoadReports)
            {
                nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            }

            auto dev = tnGnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();

            if (enableOranCellLoadReports)
            {
                dev->GetMac(0)->TraceConnectWithoutContext(
                    "DlScheduling",
                    MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));
            }

            nrGnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrGnbTerminator->SetAttribute("RegistrationIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrGnbTerminator->SetAttribute("SendIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(e2SendInterval) + "]"));

            nrGnbTerminator->AddReporter(locationReporter);
            if (enableOranCellLoadReports)
            {
                nrGnbTerminator->AddReporter(nrCellLoadReporter);
            }
            nrGnbTerminator->Attach(tnGnbNodes.Get(idx));
            nrGnbTerminator->SetAttribute("TransmissionDelayRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(txDelay) + "]"));

            Simulator::Schedule(Seconds(1.5),
                                &OranE2NodeTerminatorNrGnb::Activate,
                                nrGnbTerminator);
        }

        // NTN gNB reporters
        for (uint32_t idx = 0; idx < ntnGnbNodes.GetN(); ++idx)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrCellLoad> nrCellLoadReporter =
                enableOranCellLoadReports ? CreateObject<OranReporterNrCellLoad>() : nullptr;
            Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            if (enableOranCellLoadReports)
            {
                nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            }

            auto dev = ntnGnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();

            if (enableOranCellLoadReports)
            {
                dev->GetMac(1)->TraceConnectWithoutContext(
                    "DlScheduling",
                    MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));
            }

            nrGnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrGnbTerminator->SetAttribute("RegistrationIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrGnbTerminator->SetAttribute("SendIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(e2SendInterval) + "]"));

            nrGnbTerminator->AddReporter(locationReporter);
            if (enableOranCellLoadReports)
            {
                nrGnbTerminator->AddReporter(nrCellLoadReporter);
            }
            nrGnbTerminator->Attach(ntnGnbNodes.Get(idx));
            nrGnbTerminator->SetAttribute("TransmissionDelayRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=" +
                                                    std::to_string(txDelay) + "]"));

            Simulator::Schedule(Seconds(1.5),
                                &OranE2NodeTerminatorNrGnb::Activate,
                                nrGnbTerminator);
        }

    }
    // ORAN END

    if (enableSatBackhaulMonitor && satBackhaulCtx)
    {
        // Satellite backhaul monitor:
        // periodically computes service-link and feeder-link SNR for each UAV
        // cell. When SNR is below the configured threshold, the UAV/NTN cell can
        // be marked unavailable for handover by reducing its allowed capacity.
        Simulator::Schedule(Seconds(0.1),
                            &LogSatelliteBackhaul,
                            satBackhaulCtx.get());
    }

    {
        // Cross-layer autonomy trace header.
        // This file is the main new output for the article idea. It should be
        // compared against qos-vs-time.txt and handover traces for:
        //   1. TN only,
        //   2. TN + UAV,
        //   3. TN + UAV + satellite.
        std::ofstream xhaulOut(s_xhaulAutonomyTraceFile, std::ios_base::trunc);
        xhaulOut << "Time,DeploymentMode,UavIndex,UavCellId,BestDonorCellId,"
                 << "BestDonorDistanceM,XhaulConnected,"
                 << "XhaulChannelVariationDb,XhaulRsrpDbm,XhaulState,XhaulDegradationActive,"
                 << "SatBackhaulDlSnrDb,SatBackhaulUlSnrDb,SatBackhaulHealthy,"
                 << "OnboardUavRicAvailable,OnboardUavRicState,"
                 << "BackhaulMode,ControlPath,ActiveUavRic,"
                 << "NormalUeHandoverAllowed,"
                 << "E2TxDelaySec,E2SendIntervalSec,LmQueryIntervalSec,UavMode\n";
    }
    if (ntnGnbNrDevs.GetN() > 0 && !useOran)
    {
        // Start the UAV-to-TN xHaul health monitor only when UAV cells exist.
        // With O-RAN enabled, the UAV Autonomy xApp writes the same trace from
        // the RIC query loop. This timer is the non-O-RAN fallback path.
        Simulator::Schedule(Seconds(0.5),
                            &TraceXhaulAutonomy,
                            tnGnbNodes,
                            ntnGnbNodes,
                            tnGnbNrDevs,
                            ntnGnbNrDevs,
                            epcHelper,
                            xhaulTxPowerDbm,
                            xhaulFrequencyHz,
                            xhaulMaxDonorDistanceM,
                            xhaulHealthyRsrpDbm,
                            xhaulDegradedRsrpDbm,
                            xhaulDegradationStartSec,
                            xhaulDegradationStopSec,
                            xhaulDegradationPenaltyDb,
                            txDelay,
                            e2SendInterval,
                            lmQueryInterval,
                            xhaulTraceIntervalSec);
    }

    if (enablePositionTrace)
    {
        // Erase the position trace files if they exist.
        std::ofstream posOutFile1(s_ueS1PositionTraceFile, std::ios_base::trunc);
        posOutFile1.close();

        std::ofstream posOutFile2(s_uavPositionTraceFile, std::ios_base::trunc);
        posOutFile2.close();

        std::ofstream posOutFile3(s_ueS2PositionTraceFile, std::ios_base::trunc);
        posOutFile3.close();

        // Start tracing node locations
        Simulator::Schedule(Seconds(g_positionTraceIntervalSec), &TraceUeS1Positions, groundUeNodesS1);
        Simulator::Schedule(Seconds(g_positionTraceIntervalSec), &TraceUavPositions, ntnGnbNodes);

        /* Start tracing UE Set 2 only when they actually start */
        Simulator::Schedule(tLateAttach, &TraceUeS2Positions, groundUeNodesS2);
    }

    if (enableHandoverTrace)
    {
        std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::trunc);
        hoOutFile.close();

        // Connect to handover trace so we know when a handover is successfully performed
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                        MakeCallback(&NotifyHandoverEndOkGnb));
    }

    if (enableHandoverFailureTrace)
    {
        std::ofstream hoFailOutFile(s_handoverFailureTraceFile, std::ios_base::trunc);
        hoFailOutFile.close();

        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureNoPreamble",
                        MakeBoundCallback(&NotifyHandoverFailureGnb, std::string("NoPreamble")));
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureMaxRach",
                        MakeBoundCallback(&NotifyHandoverFailureGnb, std::string("MaxRach")));
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureLeaving",
                        MakeBoundCallback(&NotifyHandoverFailureGnb, std::string("LeavingTimeout")));
        Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverFailureJoining",
                        MakeBoundCallback(&NotifyHandoverFailureGnb, std::string("JoiningTimeout")));
        Config::Connect("/NodeList/*/DeviceList/*/NrUeRrc/HandoverEndError",
                        MakeCallback(&NotifyHandoverEndErrorUe));
    }

    if (enableRsrpTrace)
    {
        for (NetDeviceContainer::Iterator it = groundNrDevsS1.Begin(); it != groundNrDevsS1.End(); ++it)
        {
            Ptr<NetDevice> device = *it;
            Ptr<NrUeNetDevice> nrUeDevice = device->GetObject<NrUeNetDevice>();

            if (nrUeDevice)
            {
                for (uint32_t b = 0; b < ueNumBwps; ++b)
                {
                    Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(b);
                    if (uePhy)
                    {
                        uePhy->TraceConnectWithoutContext(
                            "ReportUeMeasurements",
                            MakeBoundCallback(&TraceRsrpRsrqSinr, rsrpRsrqSinrTraceStream));
                    }
                }
            }
        }

        for (NetDeviceContainer::Iterator it = groundNrDevsS2.Begin(); it != groundNrDevsS2.End(); ++it)
        {
            Ptr<NetDevice> device = *it;
            Ptr<NrUeNetDevice> nrUeDevice = device->GetObject<NrUeNetDevice>();

            if (nrUeDevice)
            {
                for (uint32_t b = 0; b < ueNumBwps; ++b)
                {
                    Ptr<NrUePhy> uePhy = nrUeDevice->GetPhy(b);
                    if (uePhy)
                    {
                        uePhy->TraceConnectWithoutContext(
                            "ReportUeMeasurements",
                            MakeBoundCallback(&TraceRsrpRsrqSinr, rsrpRsrqSinrTraceStream));
                    }
                }
            }
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
    Ptr<FlowMonitor> flowMonitor;

    if (enableFlowMonitor)
    {
        NodeContainer nodesToMonitor;
        nodesToMonitor.Add(remoteHostContainer);   // include remote host
        nodesToMonitor.Add(groundUeNodesS1);
        nodesToMonitor.Add(groundUeNodesS2);

        flowMonitor = flowHelper.Install(nodesToMonitor);

        std::ofstream qos_vs_time;
        qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::trunc);
        qos_vs_time << "Time,UE,Dir,Delay,Jitter,Throughput,PDR" << std::endl;

        g_prevFlowStats.clear();
        g_lastQosSampleTime = Seconds(0);
        Simulator::Schedule(Seconds(4.0), ThroughputMonitor, &flowHelper, flowMonitor);
    }


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

    // std::ofstream flowOutFile(s_flowStatTraceFile, std::ios_base::trunc);
    // flowOutFile << "Time,Role,IMSI\n";
    // flowOutFile.close();

    // Tell the simulator how long to run
    if (enableProgress)
    {
        Simulator::ScheduleNow(&PrintSimulationProgress,
                               Seconds(std::max(0.1, progressIntervalSec)),
                               simulationStopTime);
    }
    Simulator::Stop(simulationStopTime);
    Simulator::Run();
    if (enableFlowMonitor)
    {
        WriteFlowReportToFile(flowMonitor, &flowHelper, ns3_dir + "final-flow-report.txt");
    }

    if (g_oldClogBuf) { std::clog.rdbuf(g_oldClogBuf); }
    if (g_nsLogFile.is_open()) { g_nsLogFile.close(); }

    // if (g_oldCoutBuf) { std::cout.rdbuf(g_oldCoutBuf); }
    // if (g_uncondFile.is_open()) { g_uncondFile.close(); }
    // if (g_fhTraceFile.is_open()) { g_fhTraceFile.close(); }
    // if (g_airTraceFile.is_open()) { g_airTraceFile.close(); }   

    if (satBackhaulOut.is_open()) { satBackhaulOut.close(); }

    Simulator::Destroy();
    return 0;
}
