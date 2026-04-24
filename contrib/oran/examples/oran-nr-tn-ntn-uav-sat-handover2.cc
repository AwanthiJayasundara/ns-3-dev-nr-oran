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

NS_LOG_COMPONENT_DEFINE("OranNrTnNtnUavSatHandover2");

/**
 * Example of ORAN-driven NR multi-cell UES1 handover with QoS monitoring (5G-LENA).
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
 * Two set of ue traffic added (No HO> we can change if want) and connect
 * with RIC too so load-aware handover decisions can be made based on the total number of 
 * UEs (UES1 + UE2) connected to each gNB. 
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
 */

const static float TN_GNB_HEIGHT = 25;


// Variables
uint32_t numGroundUesS1 = 120; // UES1 
uint32_t numGroundUesS2 = 120; // UES2

uint32_t numTnGnbs = 8;
uint32_t numNtnGnbs = 6; // e.g., uav gnbs for ntn area

static const double g_refLat = 53.3498;   // Dublin city centre
static const double g_refLon = -6.2603;

static const double TN_GNB_HALF_W_M  = 2500.0;
static const double TN_GNB_HALF_H_M  = 1200.0;

static const double UAV_AREA_HALF_W_M = 3000.0;
static const double UAV_AREA_HALF_H_M = 1500.0;

static const double UE_AREA_HALF_W_M  = 3000.0;
static const double UE_AREA_HALF_H_M  = 1500.0;

//uint32_t maxUesPerCell = 20; // ORAN LM parameter: maximum number of UEs per cell (for load-aware handover decisions)
uint32_t maxUesPerCellTn  = 20;
uint32_t maxUesPerCellNtn = 10;

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

struct FlowIntervalSnapshot
{
    uint64_t txPackets = 0;
    uint64_t rxPackets = 0;
    uint64_t rxBytes = 0;
    Time delaySum = Seconds(0);
    Time jitterSum = Seconds(0);
};

static std::map<uint32_t, FlowIntervalSnapshot> g_prevFlowStats;

// // static std::string s_trafficTraceFile;
static std::string s_ueS1PositionTraceFile;
static std::string s_ueS2PositionTraceFile;
static std::string s_uavPositionTraceFile;
static std::string s_handoverTraceFile;
static std::string s_flowStatTraceFile;
static std::string ns3_dir;
//fh control trace files
// static std::ofstream g_fhTraceFile;
// static std::ofstream g_airTraceFile;

static std::string s_satBackhaulTraceFile;

static std::map<uint16_t, double> g_backhaulDlSnrDb;
static std::map<uint16_t, double> g_backhaulUlSnrDb;

static std::set<uint16_t> g_ntnCellIds;
static std::map<std::pair<uint16_t,uint16_t>, double> g_latestRsrp;

static Ptr<OranLmNr2NrRsrpHandoverWithTnNtn> g_rsrpLm = nullptr;

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
                  double rsrq,
                  bool servingCell,
                  uint8_t componentCarrierId)
{
    g_latestRsrp[{rnti,cellId}] = rsrp;
}

// void
// ReportFhTrace(const SfnSf& sfn, uint16_t physCellId, uint16_t bwpId, uint64_t reqFh)
// {
//     if (!g_fhTraceFile.is_open())
//     {
//         g_fhTraceFile.open(ns3_dir + "fh-trace.txt", std::ios::out | std::ios::trunc);
//         g_fhTraceFile << "Time,CellId,BwpId,RequiredFhDlThroughput\n";
//     }

//     g_fhTraceFile << Simulator::Now().GetSeconds() << ","
//                   << physCellId << ","
//                   << bwpId << ","
//                   << reqFh << "\n";
// }

// void
// ReportAiTrace(const SfnSf& sfn, uint16_t physCellId, uint16_t bwpId, uint32_t airRbs)
// {
//     if (!g_airTraceFile.is_open())
//     {
//         g_airTraceFile.open(ns3_dir + "air-rbs-trace.txt", std::ios::out | std::ios::trunc);
//         g_airTraceFile << "Time,CellId,BwpId,UsedAirRbs\n";
//     }

//     g_airTraceFile << Simulator::Now().GetSeconds() << ","
//                    << physCellId << ","
//                    << bwpId << ","
//                    << airRbs << "\n";
// }

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
    // std::ofstream flowLog(s_flowStatTraceFile, std::ios_base::app);

    auto flowStats = flowMon->GetFlowStats();
    auto ue_network = Ipv4Address("7.0.0.0");
    auto ue_network_mask = Ipv4Mask("255.0.0.0");
    Ptr<Ipv4FlowClassifier> classing =
        DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());

    const double intervalSec = management_interval.GetSeconds();

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

        // -------- interval metrics --------
        double pdr = (dTxPackets > 0) ? (100.0 * static_cast<double>(dRxPackets) /
                                         static_cast<double>(dTxPackets))
                                      : 0.0;

        double delay = (dRxPackets > 0) ? (dDelaySum.GetSeconds() /
                                           static_cast<double>(dRxPackets))
                                        : 0.0;

        double jitter = (dRxPackets > 0) ? (dJitterSum.GetSeconds() /
                                            static_cast<double>(dRxPackets))
                                         : 0.0;

        double throughput = (intervalSec > 0.0)
                                ? (static_cast<double>(dRxBytes) * 8.0 /
                                   intervalSec / 1024.0 / 1024.0)
                                : 0.0;

        // flowLog << Simulator::Now().GetSeconds() << ","
        //         << role << ","
        //         << imsi << "\n";

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
            else if (isUl)
            {
                user_delay_ul[receiver_id] = delay;
                user_jitter_ul[receiver_id] = jitter;
                user_throughput_ul[receiver_id] = throughput;
                user_pdr_ul[receiver_id] = pdr;
            }
        }
    }

    std::ofstream qos_vs_time;
    qos_vs_time.open(ns3_dir + "qos-vs-time.txt", std::ofstream::out | std::ofstream::app);
    double t = Simulator::Now().GetSeconds();

    for (uint32_t ue = 0; ue < numGroundUesS1; ++ue)
    {
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

    Simulator::Schedule(Seconds(1), &TraceUeS1Positions, ues);
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

    Simulator::Schedule(Seconds(1), &TraceUeS2Positions, ues);
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

    Simulator::Schedule(Seconds(1), &TraceUavPositions, uavs);
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
        st->speedMps = 10.0;

        LocalPoint2d initLocal = GeoToLocal(initGeo, g_refLat, g_refLon);
        st->targetEastM = initLocal.eastM;
        st->targetNorthM = initLocal.northM;
        st->hasTarget = true;

        g_uavStates.push_back(st.get());
        g_geoStates.push_back(std::move(st));

        Simulator::Schedule(MilliSeconds(10),
                            &MoveGeoNodeToAssignedTarget,
                            g_uavStates.back(),
                            10.0);
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

        Simulator::Schedule(MilliSeconds(10),
                            &MoveGeoNodeRandomWaypoint,
                            st.get(),
                            eastRv,
                            northRv,
                            10.0);

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

        Simulator::Schedule(MilliSeconds(10),
                            &MoveGeoNodeRandomWaypoint,
                            st.get(),
                            eastRv,
                            northRv,
                            10.0);

        g_geoStates.push_back(std::move(st));
    }
}
///

// void install_mobility(NodeContainer staticNodes,
//                       NodeContainer tnGnbNodes,
//                       NodeContainer ntnGnbNodes,
//                       NodeContainer groundUeNodesS1,
//                       NodeContainer groundUeNodesS2)
// {
//     // --------------------------------------------------
//     // 0) Remote host / static node
//     // --------------------------------------------------
//     Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
//     allocator->Add(Vector(0, 0, 0));

//     MobilityHelper staticNodesHelper;
//     staticNodesHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//     staticNodesHelper.SetPositionAllocator(allocator);
//     staticNodesHelper.Install(staticNodes);

//     // ==================================================
//     // 1) TN gNBs: fixed terrestrial gNBs in TN area
//     //    TN area: x = [0,1000], y = [-1000,1000]
//     // ==================================================
//     Ptr<ListPositionAllocator> tnGnbPosition = CreateObject<ListPositionAllocator>();

//     const double TN_X_MIN = 0.0;
//     const double TN_X_MAX = 2000.0;
//     const double TN_Y_MIN = -1000.0;
//     const double TN_Y_MAX = 1000.0;
//     const double TN_Z     = TN_GNB_HEIGHT;

//     const double minDist = 200.0;
//     const uint32_t maxAttempts = 20000;

//     Ptr<UniformRandomVariable> tnUx = CreateObject<UniformRandomVariable>();
//     Ptr<UniformRandomVariable> tnUy = CreateObject<UniformRandomVariable>();
//     tnUx->SetAttribute("Min", DoubleValue(TN_X_MIN));
//     tnUx->SetAttribute("Max", DoubleValue(TN_X_MAX));
//     tnUy->SetAttribute("Min", DoubleValue(TN_Y_MIN));
//     tnUy->SetAttribute("Max", DoubleValue(TN_Y_MAX));

//     std::vector<Vector> tnPlaced;
//     tnPlaced.reserve(tnGnbNodes.GetN());

//     auto TnFarEnough = [&](const Vector& cand) -> bool
//     {
//         for (const auto& p : tnPlaced)
//         {
//             const double dx = cand.x - p.x;
//             const double dy = cand.y - p.y;
//             const double d  = std::sqrt(dx * dx + dy * dy);
//             if (d < minDist)
//             {
//                 return false;
//             }
//         }
//         return true;
//     };

//     for (uint32_t i = 0; i < tnGnbNodes.GetN(); ++i)
//     {
//         bool ok = false;
//         for (uint32_t a = 0; a < maxAttempts; ++a)
//         {
//             Vector cand(tnUx->GetValue(), tnUy->GetValue(), TN_Z);
//             if (TnFarEnough(cand))
//             {
//                 tnGnbPosition->Add(cand);
//                 tnPlaced.push_back(cand);
//                 ok = true;
//                 break;
//             }
//         }

//         NS_ABORT_MSG_IF(!ok,
//                         "Could not place TN gNB " << i
//                         << " in TN area with minDist=" << minDist);
//     }

//     MobilityHelper tnGnbHelper;
//     tnGnbHelper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//     tnGnbHelper.SetPositionAllocator(tnGnbPosition);
//     tnGnbHelper.Install(tnGnbNodes);

//     // ==================================================
//     // 2) NTN/UAV gNBs: moving aerial gNBs in separate area
//     //    NTN area: x = [2000,6000], y = [-1000,1000]
//     //    initial altitude z = [100,200]
//     // ==================================================
//     Ptr<RandomBoxPositionAllocator> ntnGnbPosition = CreateObject<RandomBoxPositionAllocator>();
//     ntnGnbPosition->SetAttribute("X",
//         StringValue("ns3::UniformRandomVariable[Min=2100.0|Max=9900.0]"));
//     ntnGnbPosition->SetAttribute("Y",
//         StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
//     ntnGnbPosition->SetAttribute("Z",
//         StringValue("ns3::UniformRandomVariable[Min=100.0|Max=200.0]")); // UAV altitude between 100 and 200 meters

//     MobilityHelper ntnGnbHelper;
//     ntnGnbHelper.SetPositionAllocator(ntnGnbPosition);
//     ntnGnbHelper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
//                                   "Bounds",
//                                   StringValue("2000|10000|-1000|1000"),
//                                   "Speed",
//                                   StringValue("ns3::UniformRandomVariable[Min=8.0|Max=16.0]"),
//                                   "Pause",
//                                   StringValue("ns3::UniformRandomVariable[Min=0.5|Max=2.0]"));
//     ntnGnbHelper.Install(ntnGnbNodes);

//     // ==================================================
//     // 3) UE Set 1: moving across whole TN+NTN area
//     //    whole area: x = [0,6000], y = [-1000,1000]
//     // ==================================================
//     Ptr<RandomBoxPositionAllocator> boxS1 = CreateObject<RandomBoxPositionAllocator>();
//     boxS1->SetAttribute("X",
//         StringValue("ns3::UniformRandomVariable[Min=100.0|Max=9900.0]"));
//     boxS1->SetAttribute("Y",
//         StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
//     boxS1->SetAttribute("Z",
//         StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

//     MobilityHelper ueS1Helper;
//     ueS1Helper.SetPositionAllocator(boxS1);
//     ueS1Helper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
//                                 "Bounds",
//                                 StringValue("0|10000|-1000|1000"),
//                                 "Speed",
//                                 StringValue("ns3::UniformRandomVariable[Min=2.0|Max=10.0]"),
//                                 "Pause",
//                                 StringValue("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
//     ueS1Helper.Install(groundUeNodesS1);

//     // ==================================================
//     // 4) UE Set 2: static across whole TN+NTN area
//     // ==================================================
//     Ptr<RandomBoxPositionAllocator> boxS2 = CreateObject<RandomBoxPositionAllocator>();
//     boxS2->SetAttribute("X",
//         StringValue("ns3::UniformRandomVariable[Min=100.0|Max=9900.0]"));
//     boxS2->SetAttribute("Y",
//         StringValue("ns3::UniformRandomVariable[Min=-900.0|Max=900.0]"));
//     boxS2->SetAttribute("Z",
//         StringValue("ns3::ConstantRandomVariable[Constant=1.5]"));

//     // MobilityHelper ueS2Helper;
//     // ueS2Helper.SetPositionAllocator(boxS2);
//     // ueS2Helper.SetMobilityModel("ns3::ConstantPositionMobilityModel");
//     // ueS2Helper.Install(groundUeNodesS2);
//     MobilityHelper ueS2Helper;
//     ueS2Helper.SetPositionAllocator(boxS2);
//     ueS2Helper.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
//                                 "Bounds",
//                                 StringValue("0|10000|-1000|1000"),
//                                 "Speed",
//                                 StringValue("ns3::UniformRandomVariable[Min=2.0|Max=10.0]"),
//                                 "Pause",
//                                 StringValue("ns3::UniformRandomVariable[Min=1.0|Max=6.0]"));
//     ueS2Helper.Install(groundUeNodesS2);
// }

// ============================================================
// Satellite backhaul monitor helpers
// ============================================================

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
    double maxWaitTime = 0.010;
    double txDelay = 0.1;
    bool remMode = false; // [0]: REM disabled; [1]: generate REM
    int32_t remRbId = -1; // kept for compatibility (not used by this REM helper)
    std::string handoverAlgorithm = "ns3::NrNoOpHandoverAlgorithm";
    Time simTime = Seconds(40);
    std::string dbFileName = "oran-repository-tn-ntn.db";
    std::string lateCommandPolicy = "DROP";

    bool enableSatBackhaulMonitor = true;
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
    cmd.AddValue("enable-sat-backhaul-monitor",
                 "Enable parallel satellite backhaul monitor for NTN/UAV gNBs",
                 enableSatBackhaulMonitor);
    cmd.AddValue("sat-backhaul-log-step-ms",
                 "Satellite backhaul logging period in ms",
                 satBackhaulLogStepMs);
    cmd.AddValue("sat-backhaul-min-snr-db",
                 "Minimum acceptable backhaul SNR (dB) for allowing NTN HO",
                 satBackhaulMinSnrDb);
    cmd.AddValue("sat-backhaul-scenario",
                 "Satellite backhaul scenario: NTN-DenseUrban | NTN-Urban | NTN-Suburban | NTN-Rural",
                 satBackhaulScenario);
    cmd.Parse(argc, argv);

    NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
                    "Cannot use ML LM or RSRP LM without enabling O-RAN.");
    NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1, "Cannot use more than one LM simultaneously.");
    NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NrNoOpHandoverAlgorithm" && (useOnnx || useTorch || useRsrp),
                    "Cannot use non-noop handover algorithm with ML/RSRP LM (avoid conflicts).");

    std::ostringstream runTag;
    runTag << "ueS1_" << numGroundUesS1 << "_ueS2_" << numGroundUesS2 << "_tnGnb_" << numTnGnbs << "_ntnGnb_" << numNtnGnbs << "_tnCap_" << maxUesPerCellTn << "_ntnCap_" << maxUesPerCellNtn << "_hyst_" << hysteresisDb;

    // Base output folder for this run
    ns3_dir = "results/nr/tn-ntn/" + runTag.str() + "/";

    // Update file paths to be inside ns3_dir
    s_ueS1PositionTraceFile = ns3_dir + "ues1-position-trace.tr";
    s_ueS2PositionTraceFile = ns3_dir + "ues2-position-trace.tr";
    s_uavPositionTraceFile = ns3_dir + "uav-position-trace.tr";
    s_handoverTraceFile = ns3_dir + "handover-trace.tr";
    s_flowStatTraceFile = ns3_dir + "flow-stats.log";
    s_satBackhaulTraceFile = ns3_dir + "sat-backhaul-trace.txt";

    // Ensure results/nr/ directory exists
    std::filesystem::create_directories(ns3_dir);

    Ptr<OutputStreamWrapper> rsrpRsrqSinrTraceStream =
    Create<OutputStreamWrapper>(ns3_dir + "rsrp-trace.tr", std::ios::out);

    *rsrpRsrqSinrTraceStream->GetStream()
        << "Time RNTI CellId CellType RSRP RSRQ Serving CCID\n";
    // ---- Redirect NS_LOG (std::clog) to a file ----
    g_nsLogFile.open(ns3_dir + "ns3-oran-lm.log", std::ios::out | std::ios::trunc);
    g_oldClogBuf = std::clog.rdbuf(g_nsLogFile.rdbuf());

    // // ---- Redirect NS_LOG_UNCOND (std::cout) to a separate file ----
    // g_uncondFile.open(ns3_dir + "init-attach.log", std::ios::out | std::ios::trunc);
    // g_oldCoutBuf = std::cout.rdbuf(g_uncondFile.rdbuf());

    LogComponentEnable("OranLmNr2NrRsrpHandoverWithTnNtn", LOG_LEVEL_INFO);
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
    Ptr<NrChannelHelper> ntnChannelHelper = CreateObject<NrChannelHelper>();
    ntnChannelHelper->ConfigureFactories("NTN-Urban", "Default");
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
    nrHelper->SetFhControlAttribute("FhCapacity", UintegerValue(10000)); // 10 Gbps for XR
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
    // ---------------- Common BWP layout ----------------
    // Global/common indices for ALL nodes:
    //   BWP 0 = TN
    //   BWP 1 = NTN DL
    //   BWP 2 = NTN UL

    BandwidthPartInfoPtrVector allBwps;

    // ---- TDD + FDD setup ----
    bool enableFading = true;
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
        epcHelper->AddNtnGnbNode(ntnGnbNodes.Get(i));
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
    // Global BWP manager mapping is now safe again
    // ------------------------------------------------------------
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpTn));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(bwpNtnDl));
    nrHelper->SetGnbBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(bwpNtnUl));

    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VOICE", UintegerValue(bwpTn));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_CONV_VIDEO", UintegerValue(bwpNtnDl));
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_LIVE_UL_71", UintegerValue(bwpNtnUl));

    // Install devices with ONE common BWP layout
    tnGnbNrDevs  = nrHelper->InstallGnbDevice(tnGnbNodes, allBwps);
    ntnGnbNrDevs = nrHelper->InstallGnbDevice(ntnGnbNodes, allBwps);

    double uavControlPeriodSec = 2.0;
    double underservedRsrpThreshDbm = -120.0;

    // Start after measurements and initial attach have had a little time to appear
    Simulator::Schedule(Seconds(7.0),
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

    nrHelper->ConfigureFhControl(allGnbNrDevs);

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
    nrHelper->SetAttribute("InitMinRsrpDbm",    DoubleValue(-120.0));
    nrHelper->SetAttribute("InitRetryInterval", TimeValue(Seconds(2.0)));

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

    // S1 attach
    nrHelper->AttachToMaxRsrpGnb(groundNrDevsS1, allGnbNrDevs);

    // S2 late attach
    Time tLateAttach = Seconds(groundAttachDelay);
    Simulator::Schedule(tLateAttach, [nrHelper, groundNrDevsS2, allGnbNrDevs]() {
        nrHelper->AttachToMaxRsrpGnb(groundNrDevsS2, allGnbNrDevs);
    });

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

        // DL sink on UE
        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), dlPort));
        xrDlSinks.Add(dlSink.Install(groundUeNodesS1.Get(i)));

        // XR DL traffic generator
        XrTrafficMixerHelper xrDlHelper;
        xrDlHelper.ConfigureXr(dlConfig);

        std::vector<Address> dlAddresses;
        dlAddresses.push_back(InetSocketAddress(ueIpIfaceS1.GetAddress(i), dlPort));

        ApplicationContainer dlApps =
            xrDlHelper.Install("ns3::UdpSocketFactory",
                            dlAddresses,
                            remoteHost);

        xrDlSenders.Add(dlApps);

        // UL sink on remote host
        PacketSinkHelper ulSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), ulPort));
        xrUlSinks.Add(ulSink.Install(remoteHost));

        // XR UL traffic generator
        XrTrafficMixerHelper xrUlHelper;
        xrUlHelper.ConfigureXr(ulConfig);

        std::vector<Address> ulAddresses;
        ulAddresses.push_back(InetSocketAddress(remoteHostIp, ulPort));

        ApplicationContainer ulApps =
            xrUlHelper.Install("ns3::UdpSocketFactory",
                            ulAddresses,
                            groundUeNodesS1.Get(i));

        xrUlSenders.Add(ulApps);

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

        nrHelper->ActivateDedicatedQosFlow(groundNrDevsS1.Get(i), xrDlFlow, xrDlRule);
        nrHelper->ActivateDedicatedQosFlow(groundNrDevsS1.Get(i), xrUlFlow, xrUlRule);
    }

    // Start/stop times
    xrDlSinks.Start(Seconds(1.0));
    xrDlSinks.Stop(simTime + Seconds(15));

    xrDlSenders.Start(Seconds(2.0));
    xrDlSenders.Stop(simTime + Seconds(10));

    xrUlSinks.Start(Seconds(1.0));
    xrUlSinks.Stop(simTime + Seconds(15));

    xrUlSenders.Start(Seconds(3.0));
    xrUlSenders.Stop(simTime + Seconds(15));

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

        Simulator::Schedule(tLateAttach,
            [nrHelper, gUeDev, voiceFlow, voiceRule]() {
                nrHelper->ActivateDedicatedQosFlow(gUeDev, voiceFlow, voiceRule);
            });

        PacketSinkHelper dlSink("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
        ueAppsS2.Add(dlSink.Install(groundUeNodesS2.Get(i)));

        TrafficGeneratorHelper voiceHelper("ns3::UdpSocketFactory",
                                        InetSocketAddress(ueIpIfaceS2.GetAddress(i), port),
                                        TrafficGeneratorNgmnVoip::GetTypeId());

        ApplicationContainer voiceApps = voiceHelper.Install(remoteHost);
        groundRemoteAppsS2.Add(voiceApps);
    }

    ueAppsS2.Start(Seconds(1));
    groundRemoteAppsS2.Start(tLateAttach + Seconds(0.5));

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

        // defaultLm->SetAttribute("MaxUesPerCell", UintegerValue(0)); 

        // defaultLm->SetAttribute("TryNextBest", BooleanValue(true)); // try next best otherwise keep current
        defaultLm->SetAttribute("MaxUesPerCell", UintegerValue(0)); // per-cell map takes over

        if (useRsrp)
        {
            Ptr<OranLmNr2NrRsrpHandoverWithTnNtn> rsrpLm =
                DynamicCast<OranLmNr2NrRsrpHandoverWithTnNtn>(defaultLm);

            if (rsrpLm)
            {
                g_rsrpLm = rsrpLm;

                g_rsrpLm->SetDecisionCsvFilename(ns3_dir + "ml-ho-dataset.csv");

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

        defaultLm->SetAttribute("TryNextBest", BooleanValue(true)); // try next best otherwise keep current

        defaultLm->SetAttribute("MinAcceptableRsrpDbm", DoubleValue(-120.0)); // default is -120 dbm

        defaultLm->SetAttribute("HysteresisDb", DoubleValue(hysteresisDb)); // default is 2 dbm

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
            xrDlSenders.Get(idx)->TraceConnectWithoutContext("Tx",
                                                            MakeCallback(&ns3::OranReporterAppLoss::AddTx, appLossReporter));
            xrDlSinks.Get(idx)->TraceConnectWithoutContext("Rx",
                                                        MakeCallback(&ns3::OranReporterAppLoss::AddRx, appLossReporter));
          
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

        // TN gNB reporters
        for (uint32_t idx = 0; idx < tnGnbNodes.GetN(); ++idx)
        {
            Ptr<OranReporterLocation> locationReporter = CreateObject<OranReporterLocation>();
            Ptr<OranReporterNrCellLoad> nrCellLoadReporter = CreateObject<OranReporterNrCellLoad>();
            Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));

            auto dev = tnGnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();

            dev->GetMac(0)->TraceConnectWithoutContext(
                "DlScheduling",
                MakeCallback(&ns3::OranReporterNrCellLoad::DlScheduled, nrCellLoadReporter));

            nrGnbTerminator->SetAttribute("NearRtRic", PointerValue(nearRtRic));
            nrGnbTerminator->SetAttribute("RegistrationIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            nrGnbTerminator->SetAttribute("SendIntervalRv",
                                        StringValue("ns3::ConstantRandomVariable[Constant=1]"));

            nrGnbTerminator->AddReporter(locationReporter);
            nrGnbTerminator->AddReporter(nrCellLoadReporter);
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
            Ptr<OranReporterNrCellLoad> nrCellLoadReporter = CreateObject<OranReporterNrCellLoad>();
            Ptr<OranE2NodeTerminatorNrGnb> nrGnbTerminator = CreateObject<OranE2NodeTerminatorNrGnb>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));
            nrCellLoadReporter->SetAttribute("Terminator", PointerValue(nrGnbTerminator));

            auto dev = ntnGnbNrDevs.Get(idx)->GetObject<NrGnbNetDevice>();

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
        Simulator::Schedule(Seconds(0.1),
                            &LogSatelliteBackhaul,
                            satBackhaulCtx.get());
    }

    // Erase the trace files if they exist
    std::ofstream posOutFile1(s_ueS1PositionTraceFile, std::ios_base::trunc);
    posOutFile1.close();

    std::ofstream posOutFile2(s_uavPositionTraceFile, std::ios_base::trunc);
    posOutFile2.close();

    std::ofstream posOutFile3(s_ueS2PositionTraceFile, std::ios_base::trunc);
    posOutFile3.close();

    std::ofstream hoOutFile(s_handoverTraceFile, std::ios_base::trunc);
    hoOutFile.close();

    // Start tracing node locations
    Simulator::Schedule(Seconds(1), &TraceUeS1Positions, groundUeNodesS1);
    Simulator::Schedule(Seconds(1), &TraceUavPositions, ntnGnbNodes);

    /* Start tracing UE Set 2 only when they actually start */
    Simulator::Schedule(tLateAttach, &TraceUeS2Positions, groundUeNodesS2);

    // Connect to handover trace so we know when a handover is successfully performed
    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkGnb));

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
    g_prevFlowStats.clear();
    Simulator::Schedule(Seconds(4.0), ThroughputMonitor, &flowHelper, flowMonitor);


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
    Simulator::Stop(simTime + Seconds(15));
    Simulator::Run();
    WriteFlowReportToFile(flowMonitor, &flowHelper, ns3_dir + "final-flow-report.txt");

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
