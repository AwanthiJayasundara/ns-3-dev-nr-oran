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

#include <cctype>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("OranNrTnNtnSimulationWithSecrecy");

/**
 * OranNrTnNtnSimulationWithSecrecy
 *
 * End-to-end ns-3 (5G-LENA NR) + ns-O-RAN scenario for evaluating TN–NTN integration
 * with an optional Physical-Layer Security (PLS) overlay driven by *measured SINR*.
 *
 * ---------------------------------------------------------------------------
 * 1) Topology / Mobility
 * ---------------------------------------------------------------------------
 * - Two disjoint service regions:
 *   (A) TN area centered at (0,0) with bounds [-TN_HALF .. TN_HALF] (meters).
 *       • TN gNBs are static at fixed coordinates.
 *       • TN UEs move with RandomDirection2dMobilityModel within TN bounds.
 *
 *   (B) NTN area shifted by +AREA_B_OFFSET_X meters on the X-axis, with bounds
 *       [AREA_B_OFFSET_X-NTN_HALF .. AREA_B_OFFSET_X+NTN_HALF] and Y ∈ [-NTN_HALF .. NTN_HALF].
 *       • "NTN gNBs" are modeled as mobile aerial gNBs (e.g., UAV-BS) moving inside Area B.
 *       • NTN UEs move within the Area B bounds (same mobility model).
 *
 * - Optional passive eavesdroppers ("EVE") are modeled as additional NR UEs:
 *   • EVE nodes attach like normal UEs (AttachToMaxRsrpGnb) and *only listen* (no apps),
 *     providing a practical way to obtain PHY measurements (RSRP/RSRQ/SINR) for leakage risk.
 *
 * ---------------------------------------------------------------------------
 * 2) NR / PHY configuration (high level)
 * ---------------------------------------------------------------------------
 * - Example carrier settings:
 *   • DL center ~ 4.0 GHz, UL center ~ 3.9 GHz, bandwidth ~ 20 MHz
 *   • Configurable scheduler (TDMA/OFDMA and RR/PF/MR/QoS variants)
 *   • FDD patterns configured via "Pattern" attributes on gNB PHY
 * - Channel configured via NrChannelHelper (scenario "UMa" here), optional fading enabled.
 *
 * ---------------------------------------------------------------------------
 * 3) O-RAN control loop (Near-RT RIC + LM)
 * ---------------------------------------------------------------------------
 * - When O-RAN is enabled (useOran=true):
 *   • Near-RT RIC collects periodic measurements through E2 terminators & reporters
 *   • A Logic Module (LM/xApp) can control handover decisions
 *   • When LM controls HO, the native HO algorithm is set to NoOp to avoid conflicts.
 *
 * - LM options include:
 *   • No-op / baseline, ONNX LM, Torch LM, and secrecy-aware LM (OranLmNrSecrecyAwareHandover).
 *
 * ---------------------------------------------------------------------------
 * 4) Security overlay (PLS): SINR-based secrecy monitoring
 * ---------------------------------------------------------------------------
 * Measurements:
 * - Legitimate UEs log:
 *   • RSRP/RSRQ via "ReportUeMeasurements"
 *   • Downlink SINR via "DlDataSinr" and "DlCtrlSinr"
 * - Eavesdroppers log the same traces (because they are implemented as NR UEs).
 *
 * Data handling:
 * - This script stores the *latest* SINR sample per {UE,eve} per cell in a map, with a maxAge
 *   filter (e.g., 1s) so stale values are ignored.
 *   This avoids the common bug of tracking a "max forever" eaves value that never decreases.
 *
 * Secrecy metric (computed every management_interval seconds):
 *   C_s = [ log2(1 + SINR_UE) - log2(1 + max_e SINR_EVE(e)) ]^+
 * - Outage is flagged when: C_s < secrecyTargetBitsPerHz
 *
 * Notes:
 * - SINR values from DlDataSinr/DlCtrlSinr callbacks are treated as *linear* (not dB).
 * - dB conversions are used for logging/inspection only.
 * - A legacy RSRP→SNR approximation monitor is kept for reference, but SINR-based secrecy is
 *   the preferred/cleaner PLS metric here.
 *
 * ---------------------------------------------------------------------------
 * 5) KPI logging (QoS + traces)
 * ---------------------------------------------------------------------------
 * - FlowMonitor periodically records per-UE throughput, delay, jitter, PDR, PLR.
 * - Additional traces: traffic TX/RX, UE positions, handover events, UE/EVE RSRP and SINR,
 *   secrecy-vs-time outputs.
 *
 * Output directory (default):
 *   results/nr/tn-ntn/withsecrecylm/
 * Main outputs include:
 *   - nr-traffic-trace.tr
 *   - nr-position-trace.tr
 *   - nr-handover-trace.tr
 *   - nr-rsrp-ue.tr / nr-rsrp-eve.tr
 *   - nr-sinr-ue.tr / nr-sinr-eve.tr
 *   - secrecy-sinr-vs-time.txt   (SINR-based secrecy + outage)
 *   - secrecy-vs-time.txt        (legacy RSRP-based approximation, if enabled)
 *   - qos-vs-time.txt
 *
 * Typical comparisons enabled by toggles:
 * - TN-only:        --scenario=tn
 * - NTN-only:       --scenario=ntn
 * - TN+NTN:         --scenario=tn-ntn
 * - TN+NTN (RSRP):  --scenario=tn-ntn --use-rsrp-lm=1 --enable-eves=0
 * - TN+NTN (Secrecy-aware): --scenario=tn-ntn --use-rsrp-lm=1 --enable-eves=1
 */

// =========================
//  CONFIG CONSTANTS
// =========================
static constexpr double AREA_B_OFFSET_X = 6000.0;   // NTN area is shifted +6000m on X
static constexpr double TN_HALF = 1000.0;           // TN bounds: [-1000..1000]
static constexpr double NTN_HALF = 2000.0;          // NTN bounds: [-2000..2000] around offset
static constexpr double GNB_HEIGHT_TN = 25.0;

// Output dirs (set at runtime in main() after --scenario parsing)
static std::string ns3_dir;
static std::string s_trafficTraceFile;
static std::string s_positionTraceFile;
static std::string s_handoverTraceFile;
// static std::string s_rsrpUeTraceFile;
//static std::string s_rsrpEveTraceFile;
//static std::string s_secrecyTraceFile;
// static std::string s_sinrUeTraceFile;
static std::string s_sinrEveTraceFile;
static std::string s_secrecySinrTraceFile;

// =========================
//  GLOBALS (QoS like your code)
// =========================
Time management_interval = Seconds(2);

std::vector<Ipv4Address> user_ip;
std::vector<double> user_delay;
std::vector<double> user_jitter;
std::vector<double> user_throughput;
std::vector<double> user_pdr;
std::vector<double> user_plr;

static std::unordered_map<uint32_t, uint64_t> g_prevTxPackets;
static std::unordered_map<uint32_t, uint64_t> g_prevRxPackets;
static std::unordered_map<uint32_t, uint64_t> g_prevRxBytes;

// =========================
//  GLOBALS (Security overlay)
// =========================
// Store each UE's latest serving cell and RSRP (dBm)
static std::vector<uint16_t> g_ueServingCell;
static std::vector<double>   g_ueServingRsrpDbm;

static Ptr<OranDataRepository> g_repo;
static std::vector<uint64_t> g_eveIds; // index -> eve node id

// Channel scenario selection
std::string channelScenario = "auto";   // auto | UMa | RMa | NTN-Rural | NTN-Urban | ...
std::string channelCondition = "Default"; // Default | LOS | NLOS | Buildings
std::string channelModel = "ThreeGpp";    // ThreeGpp | TwoRay | NYU



// =========================
// NEW (SINR): store "latest" SINR samples per UE/eve per cell
// 5G-LENA NrUePhy DlDataSinr/DlCtrlSinr traced callbacks are:
//   (uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
// =========================
struct SinrSample
{
    double sinrLin = 0.0;     // we assume this is LINEAR (not dB)
    Time   t = Seconds(0);
};

// g_ueLastSinr[ueIdx][cellId]  = { last SINR, time }
static std::vector<std::unordered_map<uint16_t, SinrSample>> g_ueLastSinr;

// g_eveLastSinr[eveIdx][cellId] = { last SINR, time }
static std::vector<std::unordered_map<uint16_t, SinrSample>> g_eveLastSinr;

// Also keep a "serving SINR" scalar if you want quick access (optional)
static std::vector<double> g_ueServingSinrLin; // linear

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
static std::vector<std::unordered_map<uint16_t, EveSample>> g_eveLast;

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
    flowMon->CheckForLostPackets();

    auto statsMap = flowMon->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(fmhelper->GetClassifier());

    const double intervalSec = management_interval.GetSeconds();
    auto ue_network = Ipv4Address("7.0.0.0");
    auto ue_network_mask = Ipv4Mask("255.0.0.0");

    std::fill(user_delay.begin(), user_delay.end(), 0.0);
    std::fill(user_jitter.begin(), user_jitter.end(), 0.0);
    std::fill(user_throughput.begin(), user_throughput.end(), 0.0);
    std::fill(user_pdr.begin(), user_pdr.end(), 0.0);
    std::fill(user_plr.begin(), user_plr.end(), 0.0);

    for (const auto& kv : statsMap)
    {
        uint32_t flowId = kv.first;
        const auto& st = kv.second;

        if (st.txPackets == 0 && st.rxPackets == 0)
            continue;

        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flowId);
        if (!ue_network_mask.IsMatch(ue_network, t.destinationAddress))
            continue;

        uint64_t prevTx = g_prevTxPackets[flowId];
        uint64_t prevRx = g_prevRxPackets[flowId];
        uint64_t prevRb = g_prevRxBytes[flowId];

        uint64_t dTx = (st.txPackets >= prevTx) ? (st.txPackets - prevTx) : 0;
        uint64_t dRx = (st.rxPackets >= prevRx) ? (st.rxPackets - prevRx) : 0;
        uint64_t dRb = (st.rxBytes   >= prevRb) ? (st.rxBytes   - prevRb) : 0;

        g_prevTxPackets[flowId] = st.txPackets;
        g_prevRxPackets[flowId] = st.rxPackets;
        g_prevRxBytes[flowId]   = st.rxBytes;

        double thrMbps = (dRb * 8.0) / intervalSec / 1e6;

        double pdr = (dTx > 0) ? (100.0 * (double)dRx / (double)dTx) : 0.0;
        if (pdr > 100.0) pdr = 100.0;

        double plr = (dTx > 0) ? (100.0 * (double)(dTx - std::min(dTx, dRx)) / (double)dTx) : 0.0;

        double delay  = (st.rxPackets > 0) ? (st.delaySum.GetSeconds()  / st.rxPackets) : 0.0;
        double jitter = (st.rxPackets > 0) ? (st.jitterSum.GetSeconds() / st.rxPackets) : 0.0;

        int ueId = get_user_id_from_ipv4(t.destinationAddress);
        if (ueId >= 0)
        {
            user_throughput[ueId] = thrMbps;
            user_pdr[ueId] = pdr;
            user_plr[ueId] = plr;
            user_delay[ueId] = delay;
            user_jitter[ueId] = jitter;
        }
    }

    std::ofstream qos_vs_time(ns3_dir + "qos-vs-time2.txt", std::ios::app);
    double now = Simulator::Now().GetSeconds();
    for (uint32_t ue = 0; ue < user_delay.size(); ++ue)
    {
        qos_vs_time << now << "," << ue << "," << user_delay[ue] << ","
                    << user_jitter[ue] << "," << user_throughput[ue] << ","
                    << user_pdr[ue] << "," << user_plr[ue] << "\n";
    }

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

// void NotifyHandoverStartGnb(std::string context, uint64_t imsi,
//                             uint16_t sourceCellId, uint16_t targetCellId)
// {
//     std::ofstream f(s_handoverTraceFile, std::ios_base::app);
//     f << Simulator::Now().GetSeconds()
//       << " START imsi=" << imsi
//       << " " << sourceCellId << "->" << targetCellId
//       << std::endl;
// }

// void NotifyHandoverEndErrorGnb(std::string context, uint64_t imsi,
//                                uint16_t cellid, uint16_t rnti)
// {
//     std::ofstream f(s_handoverTraceFile, std::ios_base::app);
//     f << Simulator::Now().GetSeconds()
//       << " ERROR imsi=" << imsi
//       << " cell=" << cellid
//       << " rnti=" << rnti
//       << std::endl;
// }

// =========================
//  SECURITY: UE + EVE MEASUREMENT TRACES (RSRP/RSRQ) - unchanged
// =========================
// static void
// UeMeasTraceCb(uint32_t ueIdx,
//               Ptr<OutputStreamWrapper> stream,
//               uint16_t rnti,
//               uint16_t cellId,
//               double rsrp,
//               double rsrq,
//               bool servingCell,
//               uint8_t componentCarrierId)
// {
//     *stream->GetStream() << Simulator::Now().GetSeconds() << " UE " << ueIdx
//                          << " rnti " << rnti
//                          << " cell " << cellId
//                          << " rsrp " << rsrp
//                          << " rsrq " << rsrq
//                          << " serving " << servingCell
//                          << " cc " << (uint32_t)componentCarrierId
//                          << std::endl;

//     if (servingCell && ueIdx < g_ueServingCell.size())
//     {
//         g_ueServingCell[ueIdx] = cellId;
//         g_ueServingRsrpDbm[ueIdx] = rsrp;

//         // NEW (SINR): if we already have a recent SINR sample for this serving cell, refresh g_ueServingSinrLin
//         if (ueIdx < g_ueServingSinrLin.size() && ueIdx < g_ueLastSinr.size())
//         {
//             auto it = g_ueLastSinr[ueIdx].find(cellId);
//             if (it != g_ueLastSinr[ueIdx].end())
//             {
//                 g_ueServingSinrLin[ueIdx] = it->second.sinrLin;
//             }
//         }
//     }
// }

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

//     // store latest sample for THIS eaves + THIS cell
//     if (eveIdx < g_eveLast.size())
//     {
//         g_eveLast[eveIdx][cellId] = { rsrp, Simulator::Now() };
//     }
// }

// =========================
// NEW (SINR): DlDataSinr / DlCtrlSinr callbacks
// =========================
static inline double SafeLinearToDb(double xLin)
{
    if (xLin <= 0.0) return -std::numeric_limits<double>::infinity();
    return 10.0 * std::log10(xLin);
}

static void UeMeasCb(uint32_t ueIdx, uint16_t rnti, uint16_t cellId,
                    double rsrp, double rsrq, bool servingCell, uint8_t ccId)
{
    if (servingCell && ueIdx < g_ueServingCell.size())
        g_ueServingCell[ueIdx] = cellId;
}

static void UeDlSinrCb(uint32_t ueIdx, uint16_t cellId, uint16_t rnti,
                      double sinr, uint16_t bwpId)
{
    if (ueIdx < g_ueLastSinr.size())
        g_ueLastSinr[ueIdx][cellId] = { sinr, Simulator::Now() };
}

static void EveDlSinrCb(uint32_t eveIdx, bool isCtrl,
                        uint16_t cellId, uint16_t rnti,
                        double sinr, uint16_t bwpId)
{
    if (eveIdx < g_eveLastSinr.size())
        g_eveLastSinr[eveIdx][cellId] = { sinr, Simulator::Now() };

    if (g_repo && eveIdx < g_eveIds.size())
    {
        double sinrDb = SafeLinearToDb(sinr);
        g_repo->LogNrEveSinr(g_eveIds[eveIdx], cellId, bwpId, sinr, sinrDb, isCtrl);
    }
}

// static void
// UeDlDataSinrCb(uint32_t ueIdx,
//                Ptr<OutputStreamWrapper> stream,
//                uint16_t cellId,
//                uint16_t rnti,
//                double sinr,
//                uint16_t bwpId)
// {
//     *stream->GetStream() << Simulator::Now().GetSeconds()
//                          << " UE " << ueIdx
//                          << " cell " << cellId
//                          << " rnti " << rnti
//                          << " bwp " << bwpId
//                          << " DlDataSinrLin " << sinr
//                          << " DlDataSinrDb " << SafeLinearToDb(sinr)
//                          << std::endl;

//     if (ueIdx < g_ueLastSinr.size())
//     {
//         g_ueLastSinr[ueIdx][cellId] = { sinr, Simulator::Now() };
//     }

//     if (ueIdx < g_ueServingCell.size() && ueIdx < g_ueServingSinrLin.size())
//     {
//         if (g_ueServingCell[ueIdx] == cellId)
//         {
//             g_ueServingSinrLin[ueIdx] = sinr;
//         }
//     }
// }

// static void
// UeDlCtrlSinrCb(uint32_t ueIdx,
//                Ptr<OutputStreamWrapper> stream,
//                uint16_t cellId,
//                uint16_t rnti,
//                double sinr,
//                uint16_t bwpId)
// {
//     *stream->GetStream() << Simulator::Now().GetSeconds()
//                          << " UE " << ueIdx
//                          << " cell " << cellId
//                          << " rnti " << rnti
//                          << " bwp " << bwpId
//                          << " DlCtrlSinrLin " << sinr
//                          << " DlCtrlSinrDb " << SafeLinearToDb(sinr)
//                          << std::endl;

//     if (ueIdx < g_ueLastSinr.size())
//     {
//         g_ueLastSinr[ueIdx][cellId] = { sinr, Simulator::Now() };
//     }

//     if (ueIdx < g_ueServingCell.size() && ueIdx < g_ueServingSinrLin.size())
//     {
//         if (g_ueServingCell[ueIdx] == cellId)
//         {
//             g_ueServingSinrLin[ueIdx] = sinr;
//         }
//     }
// }

// static void
// EveDlDataSinrCb(uint32_t eveIdx,
//                 Ptr<OutputStreamWrapper> stream,
//                 uint16_t cellId,
//                 uint16_t rnti,
//                 double sinr,
//                 uint16_t bwpId)
// {
//     const double sinrDb = SafeLinearToDb(sinr);

//     *stream->GetStream() << Simulator::Now().GetSeconds() << " EVE " << eveIdx
//                          << " cell " << cellId << " bwp " << bwpId
//                          << " DlDataSinrLin " << sinr << " DlDataSinrDb " << sinrDb
//                          << std::endl;

//     if (eveIdx < g_eveLastSinr.size())
//     {
//         g_eveLastSinr[eveIdx][cellId] = { sinr, Simulator::Now() };
//     }

//     if (g_repo && eveIdx < g_eveIds.size())
//     {
//         g_repo->LogNrEveSinr(g_eveIds[eveIdx], cellId, bwpId, sinr, sinrDb, false);
//     }
// }

// static void
// EveDlCtrlSinrCb(uint32_t eveIdx,
//                 Ptr<OutputStreamWrapper> stream,
//                 uint16_t cellId,
//                 uint16_t rnti,
//                 double sinr,
//                 uint16_t bwpId)
// {
//     const double sinrDb = SafeLinearToDb(sinr);

//     *stream->GetStream() << Simulator::Now().GetSeconds() << " EVE " << eveIdx
//                          << " cell " << cellId << " bwp " << bwpId
//                          << " DlCtrlSinrLin " << sinr << " DlCtrlSinrDb " << sinrDb
//                          << std::endl;

//     if (eveIdx < g_eveLastSinr.size())
//     {
//         g_eveLastSinr[eveIdx][cellId] = { sinr, Simulator::Now() };
//     }

//     if (g_repo && eveIdx < g_eveIds.size())
//     {
//         g_repo->LogNrEveSinr(g_eveIds[eveIdx], cellId, bwpId, sinr, sinrDb, true);
//     }
// }

// =========================
//  SECURITY: periodic secrecy computation
// =========================
//static double DbToLinear(double x_db) { return std::pow(10.0, x_db / 10.0); }
static double Log2(double x) { return std::log(x) / std::log(2.0); }

// ---------------------------------------------------------
// NEW (SINR-based) secrecy monitor
// ---------------------------------------------------------
void SecrecyMonitorSinr(double secrecyTargetBitsPerHz)
{
    std::ofstream out(s_secrecySinrTraceFile, std::ios_base::app);

    Time maxAge = Seconds(1.0);

    for (uint32_t u = 0; u < g_ueServingCell.size(); ++u)
    {
        uint16_t cellId = g_ueServingCell[u];
        if (cellId == 0)
            continue;

        double ueSinrLin = 0.0;
        bool haveUeSinr = false;

        if (u < g_ueLastSinr.size())
        {
            auto it = g_ueLastSinr[u].find(cellId);
            if (it != g_ueLastSinr[u].end())
            {
                if (Simulator::Now() - it->second.t <= maxAge)
                {
                    ueSinrLin = it->second.sinrLin;
                    haveUeSinr = true;
                }
            }
        }

        if (!haveUeSinr && u < g_ueServingSinrLin.size())
        {
            if (g_ueServingSinrLin[u] > 0.0)
            {
                ueSinrLin = g_ueServingSinrLin[u];
                haveUeSinr = true;
            }
        }

        if (!haveUeSinr)
            continue;

        double seUe = Log2(1.0 + std::max(ueSinrLin, 0.0));

        double eveSinrLin = 0.0;
        bool haveEveSinr = false;

        for (uint32_t e = 0; e < g_eveLastSinr.size(); ++e)
        {
            auto it2 = g_eveLastSinr[e].find(cellId);
            if (it2 == g_eveLastSinr[e].end())
                continue;

            if (Simulator::Now() - it2->second.t > maxAge)
                continue;

            eveSinrLin = std::max(eveSinrLin, it2->second.sinrLin);
            haveEveSinr = true;
        }

        double seEve = haveEveSinr ? Log2(1.0 + std::max(eveSinrLin, 0.0)) : 0.0;
        double secrecy = std::max(seUe - seEve, 0.0);
        bool outage = (secrecy < secrecyTargetBitsPerHz);

        out << Simulator::Now().GetSeconds()
            << ",UE," << u
            << ",cell," << cellId
            << ",ueSinrLin," << ueSinrLin
            << ",ueSinrDb," << SafeLinearToDb(ueSinrLin)
            << ",eveSinrLin," << (haveEveSinr ? eveSinrLin : 0.0)
            << ",eveSinrDb," << (haveEveSinr ? SafeLinearToDb(eveSinrLin) : -std::numeric_limits<double>::infinity())
            << ",seUe," << seUe
            << ",seEve," << seEve
            << ",secrecy," << secrecy
            << ",outage," << outage
            << std::endl;
    }

    out.close();

    Simulator::Schedule(management_interval,
                        &SecrecyMonitorSinr,
                        secrecyTargetBitsPerHz);
}

// =========================
//  MOBILITY INSTALL (unchanged)
// =========================
void InstallMobilityTnNtn(NodeContainer staticNodes,
                          NodeContainer tnGnbs,
                          NodeContainer ntnGnbs,
                          NodeContainer ueA,
                          NodeContainer ueB,
                          NodeContainer eveA,
                          NodeContainer eveB,
                          bool enableNtn)
{
    {
        const double minDist = 400.0;  // meters
        const double z = GNB_HEIGHT_TN;

        auto Dist2 = [](const Vector& a, const Vector& b) {
            const double dx = a.x - b.x;
            const double dy = a.y - b.y;
            return dx*dx + dy*dy;
        };

        auto FarEnough = [&](const Vector& cand, const std::vector<Vector>& placed) {
            const double minD2 = minDist * minDist;
            for (const auto& p : placed)
            {
                if (Dist2(cand, p) < minD2)
                    return false;
            }
            return true;
        };

        Ptr<ListPositionAllocator> gnbPosition = CreateObject<ListPositionAllocator>();
        std::vector<Vector> placed;
        placed.reserve(tnGnbs.GetN());

        auto AddFixed = [&](double x, double y) {
            Vector v(x, y, z);
            gnbPosition->Add(v);
            placed.push_back(v);
        };

        // --- First 5: your fixed TN gNBs (only add as many as exist)
        if (tnGnbs.GetN() >= 1) AddFixed(0,    0);
        if (tnGnbs.GetN() >= 2) AddFixed(500,  500);
        if (tnGnbs.GetN() >= 3) AddFixed(-500, 500);
        if (tnGnbs.GetN() >= 4) AddFixed(500,  -500);
        if (tnGnbs.GetN() >= 5) AddFixed(-500, -500);

        uint32_t fixedCount = std::min<uint32_t>(tnGnbs.GetN(), 5);

        // --- Remaining: random TN gNBs in [-TN_HALF, TN_HALF] x [-TN_HALF, TN_HALF]
        Ptr<UniformRandomVariable> ux = CreateObject<UniformRandomVariable>();
        Ptr<UniformRandomVariable> uy = CreateObject<UniformRandomVariable>();

        // Optional (recommended): keep a small margin from the border to reduce rejections
        const double margin = 0.0; // e.g., margin = minDist/2.0;

        ux->SetAttribute("Min", DoubleValue(-TN_HALF + margin));
        ux->SetAttribute("Max", DoubleValue( TN_HALF - margin));
        uy->SetAttribute("Min", DoubleValue(-TN_HALF + margin));
        uy->SetAttribute("Max", DoubleValue( TN_HALF - margin));

        const uint32_t maxAttempts = 20000;

        for (uint32_t i = fixedCount; i < tnGnbs.GetN(); ++i)
        {
            bool ok = false;
            for (uint32_t a = 0; a < maxAttempts; ++a)
            {
                Vector cand(ux->GetValue(), uy->GetValue(), z);
                if (FarEnough(cand, placed))
                {
                    gnbPosition->Add(cand);
                    placed.push_back(cand);
                    ok = true;
                    break;
                }
            }

            NS_ABORT_MSG_IF(!ok,
                "Could not place TN gNB " << i
                << " with minDist=" << minDist << "m. "
                << "Try reducing minDist or increasing TN area (TN_HALF).");
        }

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
                                   "Speed",  StringValue(UniformRv(10.0, 15.0)),
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

    {
        Ptr<ListPositionAllocator> pa = CreateObject<ListPositionAllocator>();
        pa->Add(Vector(900.0, 0.0, 1.5));
        pa->Add(Vector(-900.0, 0.0, 1.5));
        pa->Add(Vector(0.0, 900.0, 1.5));

        MobilityHelper h;
        h.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        h.SetPositionAllocator(pa);
        h.Install(eveA);
    }

    {
        Ptr<ListPositionAllocator> pb = CreateObject<ListPositionAllocator>();
        pb->Add(Vector(AREA_B_OFFSET_X + 1900.0, 0.0, 1.5));
        pb->Add(Vector(AREA_B_OFFSET_X - 1900.0, 0.0, 1.5));
        pb->Add(Vector(AREA_B_OFFSET_X, 1900.0, 1.5));

        MobilityHelper h;
        h.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        h.SetPositionAllocator(pb);
        h.Install(eveB);
    }
}

int main(int argc, char* argv[])
{
    bool verbose = false;
    bool useOran = true;
    bool useOnnx = false;
    bool useTorch = false;
    bool useRsrp = true;

    // ------------------------------------------------------------------
    // NEW: Scenario selector
    //   --scenario=tn     => TN-only
    //   --scenario=ntn    => NTN-only
    //   --scenario=tn-ntn => TN+NTN (default)
    //
    // IMPORTANT: we do NOT remove your old flags; we override them safely
    // based on scenario after parsing.
    // ------------------------------------------------------------------
    std::string scenario = "ntn";  // NEW

    bool enableNtn = true;
    // NEW: enableTn flag (TN can be disabled for NTN-only runs)
    bool enableTn = false;

    bool enableEves = true;
    uint32_t numUesA = 50;
    uint32_t numUesB = 50;
    uint32_t numTnGnbs = 10;
    uint32_t numNtnGnbs = 10;

    uint32_t nEveA = 3;
    uint32_t nEveB = 3;

    double lmQueryInterval = 2;
    double maxWaitTime = 0.010;
    double txDelay = 0.1;
    bool remMode = false;
    int32_t remRbId = -1;
    std::string handoverAlgorithm = "ns3::NrNoOpHandoverAlgorithm";
    Time simTime = Seconds(25);
    std::string dbFileName = "tn-ntn-oran-lm-with-secrecy.db";
    std::string lateCommandPolicy = "DROP";

    bool ofdma = true;
    std::string schedKind = "PF";

    double secrecyTargetBitsPerHz = 0.10;

    std::string leakageModel = "oracle"; // "oracle" | "riskmap" | "hybrid" | "fixed"

    std::string riskMapFile = "";
    double riskMinEavSinrDb = -15.0;
    double riskMaxEavSinrDb = 5.0;

    CommandLine cmd;
    cmd.AddValue("verbose", "Enable printing SQL queries results", verbose);
    cmd.AddValue("use-oran", "Enable O-RAN", useOran);
    cmd.AddValue("use-onnx-lm", "Use ONNX LM", useOnnx);
    cmd.AddValue("use-torch-lm", "Use Torch LM", useTorch);
    cmd.AddValue("use-rsrp-lm", "Use RSRP LM", useRsrp);

    // NEW
    //cmd.AddValue("scenario", "Scenario: tn | ntn | tn-ntn", scenario);

    //cmd.AddValue("enable-ntn", "Enable NTN gNBs (TN-only vs TN+NTN)", enableNtn);
    cmd.AddValue("enable-eves", "Enable eavesdroppers + secrecy logging", enableEves);

    cmd.AddValue("num-ues-a", "UEs in Area A (TN)", numUesA);
    cmd.AddValue("num-ues-b", "UEs in Area B (NTN offset)", numUesB);

    cmd.AddValue("n-eve-a", "Eavesdroppers in Area A", nEveA);
    cmd.AddValue("n-eve-b", "Eavesdroppers in Area B", nEveB);

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

    cmd.AddValue("channel-scenario","Channel scenario: auto | UMa | RMa | UMi | NTN-Rural | NTN-Urban | NTN-Suburban | NTN-DenseUrban",channelScenario);
    cmd.AddValue("channel-condition", "Channel condition: Default | LOS | NLOS | Buildings", channelCondition);
    cmd.AddValue("channel-model", "Channel model: ThreeGpp | TwoRay | NYU", channelModel);

    cmd.AddValue("leakage-model", "oracle|riskmap|hybrid|fixed", leakageModel);

    cmd.AddValue("riskmap-file", "Path to risk map file: 'cellId riskScore'", riskMapFile);
    cmd.AddValue("risk-min-eav-sinr-db", "RiskScore=0 Eve SINR (dB)", riskMinEavSinrDb);
    cmd.AddValue("risk-max-eav-sinr-db", "RiskScore=1 Eve SINR (dB)", riskMaxEavSinrDb);


    cmd.Parse(argc, argv);

    // ------------------------------------------------------------------
    // NEW: apply scenario overrides *after* parsing.
    // We do NOT delete anything: only override flags/counts.
    // ------------------------------------------------------------------
    std::string scenarioLower = scenario;
    std::transform(scenarioLower.begin(), scenarioLower.end(), scenarioLower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (scenarioLower == "tn")
    {
        enableTn = true;
        enableNtn = false; // overrides your --enable-ntn
        // Force B-side node counts to zero (so they are not created/added)
    }
    else if (scenarioLower == "ntn")
    {
        enableTn = false;
        enableNtn = true; // ensures B-side is active
        // Force A-side node counts to zero
    }
    else if (scenarioLower == "tn-ntn")
    {
        enableTn = true;
        enableNtn = true;
        // keep your counts as provided
    }
    else
    {
        NS_ABORT_MSG("Unknown --scenario value. Use: tn | ntn | tn-ntn");
    }

    std::string propScenario;

    if (channelScenario != "auto")
    {
        propScenario = channelScenario;
    }
    else
    {
        if (scenarioLower == "tn")
        {
            propScenario = "UMa";
        }
        else if (scenarioLower == "ntn")
        {
            // Option A: rural terrestrial macro
            propScenario = "RMa";

            // Option B (often better “NTN flavored”): uncomment instead
            // propScenario = "NTN-Rural";
        }
        else // tn-ntn
        {
            // Single channel model must be shared. Pick one:
            propScenario = "UMa";      // common choice if your TN narrative is “urban”
            // propScenario = "RMa";   // alternative if you want “rural-ish everywhere”
        }
    }


    // ------------------------------------------------------------------
    // NEW: update output directory per scenario (so you can run separately
    // without overwriting).
    // ------------------------------------------------------------------

    // Build a folder name for UE counts
    std::string uesFolder;
    if (numUesA == numUesB)
    {
        uesFolder = std::to_string(numUesA) + "_ues";           // "50_ues"
    }
    else
    {
        uesFolder = std::to_string(numUesA) + "A_" +
                    std::to_string(numUesB) + "B_ues";          // e.g., "50A_40B_ues"
    }

    {
        std::string tag = scenarioLower;
        if (tag == "tn-ntn") tag = "tn-ntn"; // optional aliases

        // Make directory ABSOLUTE to avoid "build/" vs demonstrated directory confusion
        auto outDir = std::filesystem::absolute(
            std::filesystem::path("results") / "nr" / "tn-ntn" / "withsecrecylm" / tag / uesFolder
        );

        ns3_dir = outDir.string() + "/";

        s_trafficTraceFile      = ns3_dir + "nr-traffic-trace2.tr";
        s_positionTraceFile     = ns3_dir + "nr-position-trace2.tr";
        s_handoverTraceFile     = ns3_dir + "nr-handover-trace2.tr";
        // s_rsrpUeTraceFile       = ns3_dir + "nr-rsrp-ue.tr";
        // s_rsrpEveTraceFile      = ns3_dir + "nr-rsrp-eve.tr";
        // s_secrecyTraceFile      = ns3_dir + "secrecy-vs-time.tr";
        // s_sinrUeTraceFile       = ns3_dir + "nr-sinr-ue.tr";
        // s_sinrEveTraceFile      = ns3_dir + "nr-sinr-eve.tr";
        s_secrecySinrTraceFile  = ns3_dir + "secrecy-sinr-vs-time2.txt";

        NS_LOG_UNCOND("CWD      = " << std::filesystem::current_path());
        NS_LOG_UNCOND("scenario = " << scenarioLower);
        NS_LOG_UNCOND("ns3_dir  = " << ns3_dir);
    }


    NS_ABORT_MSG_IF(useOran == false && (useOnnx || useTorch || useRsrp),
                    "Cannot use LM without enabling O-RAN.");
    NS_ABORT_MSG_IF((useOnnx + useTorch + useRsrp) > 1, "Cannot use more than one LM simultaneously.");
    NS_ABORT_MSG_IF(handoverAlgorithm != "ns3::NrNoOpHandoverAlgorithm" && (useOnnx || useTorch || useRsrp),
                    "Avoid conflicts: use NoOp HO when LM controls HO.");

    std::filesystem::create_directories(ns3_dir);

    auto TouchOrAbort = [](const std::string& p) {
    std::ofstream f(p, std::ios::trunc);
    NS_ABORT_MSG_IF(!f.is_open(), "Cannot open file for writing: " << p);
    };

    TouchOrAbort(s_trafficTraceFile);
    TouchOrAbort(s_positionTraceFile);
    TouchOrAbort(s_handoverTraceFile);
    // TouchOrAbort(s_rsrpUeTraceFile);
    // TouchOrAbort(s_rsrpEveTraceFile);
    // TouchOrAbort(s_sinrUeTraceFile);
    // TouchOrAbort(s_sinrEveTraceFile);
    //TouchOrAbort(s_secrecyTraceFile);
    TouchOrAbort(s_secrecySinrTraceFile);
    TouchOrAbort(ns3_dir + "qos-vs-time2.txt");


    { std::ofstream f(s_trafficTraceFile, std::ios_base::trunc); }
    { std::ofstream f(s_positionTraceFile, std::ios_base::trunc); }
    { std::ofstream f(s_handoverTraceFile, std::ios_base::trunc); }
    // { std::ofstream f(s_rsrpUeTraceFile, std::ios_base::trunc); }
    // { std::ofstream f(s_rsrpEveTraceFile, std::ios_base::trunc); }

    // { std::ofstream f(s_sinrUeTraceFile, std::ios_base::trunc); }
    // { std::ofstream f(s_sinrEveTraceFile, std::ios_base::trunc); }

    // {
    //     std::ofstream f(s_secrecyTraceFile, std::ios_base::trunc);
    //     f << "time,type,id,cell,rsrpUeDbm,rsrpEveDbm,seUe,seEve,secrecy,covered,outage\n";
    // }

    {
        std::ofstream f(s_secrecySinrTraceFile, std::ios_base::trunc);
        f << "time,type,id,cell,ueSinrLin,ueSinrDb,eveSinrLin,eveSinrDb,seUe,seEve,secrecy,outage\n";
    }

    {
        std::ofstream f(ns3_dir + "qos-vs-time2.txt", std::ios_base::trunc);
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

    // These Create() calls now reflect scenario overrides (counts may be 0)
    tnGnbNodes.Create(numTnGnbs);
    ntnGnbNodes.Create(numNtnGnbs);

    ueAreaA.Create(numUesA);
    ueAreaB.Create(numUesB);

    // OLD (always added both) - keep but comment
    // ueNodes.Add(ueAreaA);
    // ueNodes.Add(ueAreaB);

    // NEW: add only active areas
    if (enableTn)
    {
        ueNodes.Add(ueAreaA);
    }
    if (enableNtn)
    {
        ueNodes.Add(ueAreaB);
    }

    remoteHostContainer.Create(1);
    Ptr<Node> remoteHost = remoteHostContainer.Get(0);

    NodeContainer eveA, eveB, eveNodes;
    if (enableEves)
    {
        // These Create() counts were already scenario-adjusted (nEveA or nEveB may be 0)
        eveA.Create(nEveA);
        eveB.Create(nEveB);

        // OLD (always add both) - keep but comment
        // eveNodes.Add(eveA);
        // eveNodes.Add(eveB);

        // NEW: add only active areas
        if (enableTn)
        {
            eveNodes.Add(eveA);
        }
        if (enableNtn)
        {
            eveNodes.Add(eveB);
        }

        g_eveLast.clear();
        g_eveLast.resize(eveNodes.GetN());

        g_eveLastSinr.clear();
        g_eveLastSinr.resize(eveNodes.GetN());
    }

    // OLD (always add TN; add NTN if enableNtn) - keep but comment
    // allGnbsActive.Add(tnGnbNodes);
    // if (enableNtn)
    // {
    //     allGnbsActive.Add(ntnGnbNodes);
    // }

    // NEW: add only enabled domains
    if (enableTn)
    {
        allGnbsActive.Add(tnGnbNodes);
    }
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
                         eveA,
                         eveB,
                         enableNtn);

    // =========================
    // NR + Channel setup
    // =========================
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();

    //std::string propScenario = "UMa";
    bool enableShadowing = false;
    channelHelper->ConfigureFactories(propScenario, channelCondition, channelModel);
    NS_LOG_UNCOND("Channel: scenario=" << propScenario
               << " condition=" << channelCondition
               << " model=" << channelModel);

    //channelHelper->ConfigureFactories(propScenario, "Default");
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
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(DataRate("10Gb/s")));
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
    NetDeviceContainer eveNrDevs;

    gnbNrDevs = nrHelper->InstallGnbDevice(allGnbsActive, Bwps);
    ueNrDevs  = nrHelper->InstallUeDevice(ueNodes, Bwps);

    if (enableEves && eveNodes.GetN() > 0)
    {
        eveNrDevs = nrHelper->InstallUeDevice(eveNodes, Bwps);
    }

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
    for (auto it = eveNrDevs.Begin(); it != eveNrDevs.End(); ++it)
    {
        Ptr<NrUeNetDevice> ue = DynamicCast<NrUeNetDevice>(*it);
        if (ue) ue->UpdateConfig();
    }

    // ---------------------------------------------------------
    // X2 interface
    // OLD (single call):
    // nrHelper->AddX2Interface(allGnbsActive);
    //
    // NEW: do X2 per-domain (safe for separated TN/NTN runs and safe if later you
    //      decide to configure TN and NTN with different BWPs)
    // ---------------------------------------------------------
    // nrHelper->AddX2Interface(allGnbsActive); // OLD - keep but comment
    if (enableTn && tnGnbNodes.GetN() > 0)
    {
        nrHelper->AddX2Interface(tnGnbNodes);
    }
    if (enableNtn && ntnGnbNodes.GetN() > 0)
    {
        nrHelper->AddX2Interface(ntnGnbNodes);
    }

    // =========================================================
    // Install IPv4 stack on UEs BEFORE AttachToMaxRsrpGnb()
    // =========================================================
    internet.Install(ueNodes);

    if (enableEves && eveNodes.GetN() > 0)
    {
        internet.Install(eveNodes);
    }

    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNrDevs));

    Ipv4InterfaceContainer eveIpIface;
    if (enableEves && eveNrDevs.GetN() > 0)
    {
        eveIpIface = epcHelper->AssignUeIpv4Address(NetDeviceContainer(eveNrDevs));
    }

    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        Ptr<Ipv4StaticRouting> ueStaticRouting =
            ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    if (enableEves && eveNodes.GetN() > 0)
    {
        for (uint32_t e = 0; e < eveNodes.GetN(); ++e)
        {
            Ptr<Node> eveNode = eveNodes.Get(e);
            Ptr<Ipv4StaticRouting> eveStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(eveNode->GetObject<Ipv4>());
            eveStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }
    }

    nrHelper->AttachToMaxRsrpGnb(ueNrDevs, gnbNrDevs);
    if (enableEves && eveNrDevs.GetN() > 0)
    {
        nrHelper->AttachToMaxRsrpGnb(eveNrDevs, gnbNrDevs);
    }

    // =========================
    // Apps
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
        streamingServer->SetAttribute("DataRate", DataRateValue(DataRate("200000bps")));///////// changed from 1000000bps to 200000bps to increase outage probability and better see secrecy effects
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
    // O-RAN
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
            defaultLmTid = TypeId::LookupByName("ns3::OranLmNrSecrecyAwareHandover");
        }

        ObjectFactory defaultLmFactory;
        defaultLmFactory.SetTypeId(defaultLmTid);
        defaultLm = defaultLmFactory.Create<OranLm>();

        defaultLm->SetAttribute("LeakageModel", StringValue(leakageModel));
        defaultLm->SetAttribute("RiskMapFile", StringValue(riskMapFile));
        defaultLm->SetAttribute("RiskMinEavSinrDb", DoubleValue(riskMinEavSinrDb));
        defaultLm->SetAttribute("RiskMaxEavSinrDb", DoubleValue(riskMaxEavSinrDb));


        dataRepository->SetAttribute("DatabaseFile", StringValue(dbFileName));

        g_repo = dataRepository;

        g_eveIds.clear();
        for (uint32_t i = 0; i < eveNodes.GetN(); ++i)
        {
            uint64_t eveId = eveNodes.Get(i)->GetId();
            g_eveIds.push_back(eveId);
            g_repo->LogNrEve(eveId, "EVE_" + std::to_string(i));
        }

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
            Ptr<OranReporterNrUeSinr> sinrReporter = CreateObject<OranReporterNrUeSinr>();
            Ptr<OranE2NodeTerminatorNrUe> nrUeTerminator = CreateObject<OranE2NodeTerminatorNrUe>();

            locationReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            nrUeCellInfoReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            rsrpRsrqReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));
            sinrReporter->SetAttribute("Terminator", PointerValue(nrUeTerminator));

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

                    uePhy->TraceConnectWithoutContext(
                        "DlDataSinr",
                        MakeCallback(&ns3::OranReporterNrUeSinr::ReportDlDataSinr, sinrReporter));

                    uePhy->TraceConnectWithoutContext(
                        "DlCtrlSinr",
                        MakeCallback(&ns3::OranReporterNrUeSinr::ReportDlCtrlSinr, sinrReporter));
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
            nrUeTerminator->AddReporter(sinrReporter);
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
    // Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverStart",
    //             MakeCallback(&NotifyHandoverStartGnb));

    Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndOk",
                    MakeCallback(&NotifyHandoverEndOkGnb));

    // Config::Connect("/NodeList/*/DeviceList/*/NrGnbRrc/HandoverEndError",
    //                 MakeCallback(&NotifyHandoverEndErrorGnb));


    // =========================
    // SECURITY TRACES + Secrecy monitor scheduling
    // =========================
    g_ueServingCell.assign(ueNodes.GetN(), 0);
    g_ueServingRsrpDbm.assign(ueNodes.GetN(), -300.0);

    g_ueLastSinr.clear();
    g_ueLastSinr.resize(ueNodes.GetN());
    g_ueServingSinrLin.assign(ueNodes.GetN(), 0.0);

    // Ptr<OutputStreamWrapper> ueMeasStream  = Create<OutputStreamWrapper>(s_rsrpUeTraceFile, std::ios::out);
    // Ptr<OutputStreamWrapper> eveMeasStream = Create<OutputStreamWrapper>(s_rsrpEveTraceFile, std::ios::out);

    // Ptr<OutputStreamWrapper> ueSinrStream  = Create<OutputStreamWrapper>(s_sinrUeTraceFile, std::ios::out);
    // Ptr<OutputStreamWrapper> eveSinrStream = Create<OutputStreamWrapper>(s_sinrEveTraceFile, std::ios::out);

    // for (uint32_t i = 0; i < ueNrDevs.GetN(); ++i)
    // {
    //     Ptr<NrUeNetDevice> d = ueNrDevs.Get(i)->GetObject<NrUeNetDevice>();
    //     if (!d) continue;
    //     Ptr<NrUePhy> phy = d->GetPhy(0);
    //     if (!phy) continue;

    //     // phy->TraceConnectWithoutContext("ReportUeMeasurements",
    //     //     MakeBoundCallback(&UeMeasTraceCb, i, ueMeasStream));

    //     phy->TraceConnectWithoutContext("DlDataSinr",
    //         MakeBoundCallback(&UeDlDataSinrCb, i, ueSinrStream));

    //     phy->TraceConnectWithoutContext("DlCtrlSinr",
    //         MakeBoundCallback(&UeDlCtrlSinrCb, i, ueSinrStream));
    // }

    // if (enableEves && eveNrDevs.GetN() > 0)
    // {
    //     for (uint32_t i = 0; i < eveNrDevs.GetN(); ++i)
    //     {
    //         Ptr<NrUeNetDevice> d = eveNrDevs.Get(i)->GetObject<NrUeNetDevice>();
    //         if (!d) continue;
    //         Ptr<NrUePhy> phy = d->GetPhy(0);
    //         if (!phy) continue;

    //         // phy->TraceConnectWithoutContext("ReportUeMeasurements",
    //         //     MakeBoundCallback(&EveMeasTraceCb, i, eveMeasStream));

    //         // phy->TraceConnectWithoutContext("DlDataSinr",
    //         //     MakeBoundCallback(&EveDlDataSinrCb, i, eveSinrStream));

    //         // phy->TraceConnectWithoutContext("DlCtrlSinr",
    //         //     MakeBoundCallback(&EveDlCtrlSinrCb, i, eveSinrStream));
    //     }

    //     Simulator::Schedule(management_interval,
    //                         &SecrecyMonitorSinr,
    //                         secrecyTargetBitsPerHz);
    // }

    // UE connects (needed so g_ueServingCell + g_ueLastSinr get updated)
    for (uint32_t i = 0; i < ueNrDevs.GetN(); ++i)
    {
        auto d = ueNrDevs.Get(i)->GetObject<NrUeNetDevice>();
        if (!d) continue;
        auto phy = d->GetPhy(0);
        if (!phy) continue;

        phy->TraceConnectWithoutContext("ReportUeMeasurements",
            MakeBoundCallback(&UeMeasCb, i));

        phy->TraceConnectWithoutContext("DlDataSinr",
            MakeBoundCallback(&UeDlSinrCb, i));

        phy->TraceConnectWithoutContext("DlCtrlSinr",
            MakeBoundCallback(&UeDlSinrCb, i));
    }

    // EVE connects (needed so g_eveLastSinr updates)
    if (enableEves && eveNrDevs.GetN() > 0)
    {
        for (uint32_t i = 0; i < eveNrDevs.GetN(); ++i)
        {
            auto d = eveNrDevs.Get(i)->GetObject<NrUeNetDevice>();
            if (!d) continue;
            auto phy = d->GetPhy(0);
            if (!phy) continue;

            phy->TraceConnectWithoutContext("DlDataSinr",
                MakeBoundCallback(&EveDlSinrCb, i, false));

            phy->TraceConnectWithoutContext("DlCtrlSinr",
                MakeBoundCallback(&EveDlSinrCb, i, true));
        }

        Simulator::Schedule(management_interval, &SecrecyMonitorSinr, secrecyTargetBitsPerHz);
    }


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