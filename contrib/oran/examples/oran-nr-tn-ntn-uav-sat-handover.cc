/*
 * Full satellite-backhauled UAV path
 *
 * Real packet path:
 *   UL: UE -> UAV -> SAT -> GW -> Server
 *   DL: Server -> GW -> SAT -> UAV -> UE
 *
 * Packet forwarding is real at IP level using PointToPoint links.
 * Radio/SAT channel quality is monitored in parallel using:
 *   - Friis access monitor (UE <-> UAV)
 *   - 3GPP NTN service monitor (UAV <-> SAT)
 *   - 3GPP NTN feeder monitor (SAT <-> GW)
 *
 * Outputs:
 *   - uav_sat_backhaul_channel_trace.txt
 *   - uav_sat_backhaul_flow_trace.txt
 *   - uav_sat_backhaul_flow_summary.txt
 */

#include "ns3/applications-module.h"
#include "ns3/antenna-model.h"
#include "ns3/channel-condition-model.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/geocentric-constant-position-mobility-model.h"
#include "ns3/internet-module.h"
#include "ns3/isotropic-antenna-model.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/node.h"
#include "ns3/point-to-point-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/spectrum-signal-parameters.h"
#include "ns3/three-gpp-channel-model.h"
#include "ns3/three-gpp-propagation-loss-model.h"
#include "ns3/three-gpp-spectrum-propagation-loss-model.h"
#include "ns3/uniform-planar-array.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("UeUavSatGatewayCoreFullPath");

// ============================================================
// Utility: PSD creation
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

// ============================================================
// Random geographic helpers
// ============================================================

struct RandomWaypointState
{
    double speedMps = 2.0;
    double centerLatDeg = 53.3015;
    double centerLonDeg = -6.1778;
    double halfWidthM = 50.0;
    double halfHeightM = 50.0;
    double fixedAltM = 1.5;

    double targetEastM = 0.0;
    double targetNorthM = 0.0;
    bool hasTarget = false;
    double reachThresholdM = 0.5;
};

static Vector
RandomGeoPoint(double centerLatDeg,
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

static void
SelectNewWaypoint(RandomWaypointState* st,
                  Ptr<UniformRandomVariable> eastRv,
                  Ptr<UniformRandomVariable> northRv)
{
    st->targetEastM = eastRv->GetValue(-st->halfWidthM, st->halfWidthM);
    st->targetNorthM = northRv->GetValue(-st->halfHeightM, st->halfHeightM);
    st->hasTarget = true;
}

static void
MoveNodeRandomWaypoint(Ptr<GeocentricConstantPositionMobilityModel> mob,
                       RandomWaypointState* st,
                       Ptr<UniformRandomVariable> eastRv,
                       Ptr<UniformRandomVariable> northRv,
                       double stepMs)
{
    double dt = stepMs / 1000.0;

    Vector geo = mob->GetGeographicPosition();
    double latDeg = geo.x;
    double lonDeg = geo.y;
    double altM = st->fixedAltM;

    double centerLatRad = st->centerLatDeg * M_PI / 180.0;
    double metersPerDegLat = 111320.0;
    double metersPerDegLon = 111320.0 * std::cos(centerLatRad);

    double eastOffsetM = (lonDeg - st->centerLonDeg) * metersPerDegLon;
    double northOffsetM = (latDeg - st->centerLatDeg) * metersPerDegLat;

    if (!st->hasTarget)
    {
        SelectNewWaypoint(st, eastRv, northRv);
    }

    double dEast = st->targetEastM - eastOffsetM;
    double dNorth = st->targetNorthM - northOffsetM;
    double distanceToTarget = std::sqrt(dEast * dEast + dNorth * dNorth);

    if (distanceToTarget <= st->reachThresholdM)
    {
        SelectNewWaypoint(st, eastRv, northRv);
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

    mob->SetGeographicPosition(Vector(latDeg, lonDeg, altM));

    Simulator::Schedule(Seconds(stepMs / 1000.0),
                        &MoveNodeRandomWaypoint,
                        mob,
                        st,
                        eastRv,
                        northRv,
                        stepMs);
}

// ============================================================
// Beamforming helper
// ============================================================

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
        double phase = -2.0 * M_PI * (sinV * cosH * loc.x + sinV * sinH * loc.y + cosV * loc.z);
        antennaWeights[ind] = std::exp(std::complex<double>(0.0, phase)) * power;
    }

    thisAntenna->SetBeamformingVector(antennaWeights);
}

// ============================================================
// Channel models
// ============================================================

struct NtnLink
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
ComputeNtnSnrDb(NtnLink& link, Ptr<MobilityModel> txMob, Ptr<MobilityModel> rxMob, bool refreshBeamforming)
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

struct AccessLink
{
    Ptr<FriisPropagationLossModel> propagation;
    double frequencyHz = 3.5e9;
    double bandwidthHz = 20e6;
    double txPowerDbm = 30.0;
    double txAntennaGainDb = 5.0;
    double rxAntennaGainDb = 0.0;
    double rxNoiseFigureDb = 7.0;
};

static double
ComputeAccessSnrDb(AccessLink& link, Ptr<MobilityModel> txMob, Ptr<MobilityModel> rxMob)
{
    double rxPowerDbm =
        link.propagation->CalcRxPower(link.txPowerDbm, txMob, rxMob) +
        link.txAntennaGainDb +
        link.rxAntennaGainDb;

    double noiseDbm = -174.0 + 10.0 * std::log10(link.bandwidthHz) + link.rxNoiseFigureDb;
    return rxPowerDbm - noiseDbm;
}

// ============================================================
// Channel logger
// ============================================================

struct LogContext
{
    Ptr<GeocentricConstantPositionMobilityModel> ueMob;
    Ptr<GeocentricConstantPositionMobilityModel> uavMob;
    Ptr<GeocentricConstantPositionMobilityModel> satMob;
    Ptr<GeocentricConstantPositionMobilityModel> gwMob;

    AccessLink* accessDlLink = nullptr; // UAV -> UE
    AccessLink* accessUlLink = nullptr; // UE -> UAV

    NtnLink* serviceDlLink = nullptr;   // SAT -> UAV
    NtnLink* serviceUlLink = nullptr;   // UAV -> SAT

    NtnLink* feederDlLink = nullptr;    // SAT -> GW
    NtnLink* feederUlLink = nullptr;    // GW -> SAT

    std::ofstream* file = nullptr;
};

static void
LogScenario(LogContext* ctx, double stepMs)
{
    double t = Simulator::Now().GetSeconds();

    double accessDlSnrDb = ComputeAccessSnrDb(*ctx->accessDlLink, ctx->uavMob, ctx->ueMob);
    double accessUlSnrDb = ComputeAccessSnrDb(*ctx->accessUlLink, ctx->ueMob, ctx->uavMob);

    double serviceDlSnrDb = ComputeNtnSnrDb(*ctx->serviceDlLink, ctx->satMob, ctx->uavMob, true);
    double serviceUlSnrDb = ComputeNtnSnrDb(*ctx->serviceUlLink, ctx->uavMob, ctx->satMob, true);

    double feederDlSnrDb = ComputeNtnSnrDb(*ctx->feederDlLink, ctx->satMob, ctx->gwMob, true);
    double feederUlSnrDb = ComputeNtnSnrDb(*ctx->feederUlLink, ctx->gwMob, ctx->satMob, true);

    double backhaulDlBottleneckDb = std::min(serviceDlSnrDb, feederDlSnrDb);
    double backhaulUlBottleneckDb = std::min(serviceUlSnrDb, feederUlSnrDb);

    double e2eDlBottleneckDb = std::min(accessDlSnrDb, backhaulDlBottleneckDb);
    double e2eUlBottleneckDb = std::min(accessUlSnrDb, backhaulUlBottleneckDb);

    Vector ueGeo = ctx->ueMob->GetGeographicPosition();
    Vector uavGeo = ctx->uavMob->GetGeographicPosition();

    (*ctx->file) << std::fixed << std::setprecision(8)
                 << t << " "
                 << ueGeo.x << " " << ueGeo.y << " " << ueGeo.z << " "
                 << uavGeo.x << " " << uavGeo.y << " " << uavGeo.z << " "
                 << std::setprecision(4)
                 << accessDlSnrDb << " "
                 << accessUlSnrDb << " "
                 << serviceDlSnrDb << " "
                 << serviceUlSnrDb << " "
                 << feederDlSnrDb << " "
                 << feederUlSnrDb << " "
                 << backhaulDlBottleneckDb << " "
                 << backhaulUlBottleneckDb << " "
                 << e2eDlBottleneckDb << " "
                 << e2eUlBottleneckDb
                 << std::endl;

    Simulator::Schedule(Seconds(stepMs / 1000.0), &LogScenario, ctx, stepMs);
}

// ============================================================
// Periodic flow logger
// ============================================================

static void
LogFlowMetrics(Ptr<FlowMonitor> monitor,
               FlowMonitorHelper* helper,
               std::ofstream* file,
               double intervalSec)
{
    monitor->CheckForLostPackets();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(helper->GetClassifier());

    double now = Simulator::Now().GetSeconds();

    for (const auto& flow : monitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        double rxDuration = 0.0;
        if (flow.second.timeLastRxPacket > flow.second.timeFirstTxPacket)
        {
            rxDuration =
                flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds();
        }

        double throughputBps = (rxDuration > 0.0) ? (flow.second.rxBytes * 8.0 / rxDuration) : 0.0;
        double meanDelaySec =
            (flow.second.rxPackets > 0)
                ? (flow.second.delaySum.GetSeconds() / flow.second.rxPackets)
                : 0.0;

        (*file) << std::fixed << std::setprecision(6)
                << now << ","
                << flow.first << ","
                << t.sourceAddress << ","
                << t.destinationAddress << ","
                << flow.second.txPackets << ","
                << flow.second.rxPackets << ","
                << flow.second.lostPackets << ","
                << flow.second.txBytes << ","
                << flow.second.rxBytes << ","
                << meanDelaySec << ","
                << throughputBps
                << std::endl;
    }

    Simulator::Schedule(Seconds(intervalSec),
                        &LogFlowMetrics,
                        monitor,
                        helper,
                        file,
                        intervalSec);
}

// ============================================================
// Main
// ============================================================

int
main(int argc, char* argv[])
{
    uint32_t simTimeMs = 20000;
    double moveStepMs = 10.0;
    double logStepMs = 100.0;
    double flowLogStepSec = 1.0;

    double centerLat = 53.3015;
    double centerLon = -6.1778;

    double ueAltM = 1.5;
    double uavAltM = 120.0;
    double gwAltM = 20.0;
    double geoSatAltM = 35786000.0;

    double ueSpeedMps = 2.0;
    double uavSpeedMps = 10.0;

    double ueHalfWidthM = 60.0;
    double ueHalfHeightM = 60.0;

    double uavHalfWidthM = 150.0;
    double uavHalfHeightM = 100.0;

    std::string ntnScenario = "NTN-Suburban";
    double ntnFrequencyHz = 20e9;
    double ntnBandwidthHz = 400e6;
    double ntnRbBandwidthHz = 120e3;

    double satEIRPDensity = 40.0;
    double satAntennaGainDb = 58.5;
    double uavUtAntennaGainDb = 39.7;
    double gwAntennaGainDb = 45.0;
    double uavUtNoiseFigureDb = 1.2;
    double gwNoiseFigureDb = 1.2;
    double satRxNoiseFigureDb = 1.2;

    double accessFrequencyHz = 3.5e9;
    double accessBandwidthHz = 20e6;

    // DL access
    double uavBsTxPowerDbm = 30.0;
    double uavBsGainDb = 5.0;
    double ueGainDb = 0.0;
    double ueNoiseFigureDb = 7.0;

    // UL access
    double ueTxPowerDbm = 23.0;
    double uavAccessNoiseFigureDb = 7.0;

    // UL backhaul placeholders
    double uavUtTxPowerDbm = 33.0;
    double gwTxPowerDbm = 46.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("simTimeMs", "Simulation time in ms", simTimeMs);
    cmd.AddValue("moveStepMs", "Mobility update period in ms", moveStepMs);
    cmd.AddValue("logStepMs", "Channel logging period in ms", logStepMs);
    cmd.AddValue("flowLogStepSec", "Flow logging period in seconds", flowLogStepSec);
    cmd.AddValue("centerLat", "Reference latitude", centerLat);
    cmd.AddValue("centerLon", "Reference longitude", centerLon);
    cmd.AddValue("ueSpeedMps", "UE speed in m/s", ueSpeedMps);
    cmd.AddValue("uavSpeedMps", "UAV speed in m/s", uavSpeedMps);
    cmd.AddValue("ueHalfWidthM", "UE patrol half width in m", ueHalfWidthM);
    cmd.AddValue("ueHalfHeightM", "UE patrol half height in m", ueHalfHeightM);
    cmd.AddValue("uavHalfWidthM", "UAV patrol half width in m", uavHalfWidthM);
    cmd.AddValue("uavHalfHeightM", "UAV patrol half height in m", uavHalfHeightM);
    cmd.AddValue("ntnScenario", "NTN scenario", ntnScenario);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod",
                       TimeValue(MilliSeconds(10)));
    Config::SetDefault("ns3::ThreeGppChannelConditionModel::UpdatePeriod",
                       TimeValue(MilliSeconds(0)));

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(1);

    // ------------------------------------------------------------
    // Nodes
    // ------------------------------------------------------------
    NodeContainer nodes;
    nodes.Create(5);

    Ptr<Node> ueNode = nodes.Get(0);
    Ptr<Node> uavNode = nodes.Get(1);
    Ptr<Node> satNode = nodes.Get(2);
    Ptr<Node> gwNode = nodes.Get(3);
    Ptr<Node> serverNode = nodes.Get(4);

    // ------------------------------------------------------------
    // Internet stack
    // ------------------------------------------------------------
    InternetStackHelper internet;
    internet.Install(nodes);

    // ------------------------------------------------------------
    // Packet-level path
    // ------------------------------------------------------------
    PointToPointHelper accessP2p;
    accessP2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("100Mbps")));
    accessP2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(5)));

    PointToPointHelper serviceP2p;
    serviceP2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("50Mbps")));
    serviceP2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(120)));

    PointToPointHelper feederP2p;
    feederP2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("200Mbps")));
    feederP2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(120)));

    PointToPointHelper coreP2p;
    coreP2p.SetDeviceAttribute("DataRate", DataRateValue(DataRate("1Gbps")));
    coreP2p.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));

    NetDeviceContainer ueUavDevs = accessP2p.Install(ueNode, uavNode);
    NetDeviceContainer uavSatDevs = serviceP2p.Install(uavNode, satNode);
    NetDeviceContainer satGwDevs = feederP2p.Install(satNode, gwNode);
    NetDeviceContainer gwSrvDevs = coreP2p.Install(gwNode, serverNode);

    Ipv4AddressHelper addr;
    addr.SetBase("10.0.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifUeUav = addr.Assign(ueUavDevs);

    addr.SetBase("10.0.2.0", "255.255.255.0");
    Ipv4InterfaceContainer ifUavSat = addr.Assign(uavSatDevs);

    addr.SetBase("10.0.3.0", "255.255.255.0");
    Ipv4InterfaceContainer ifSatGw = addr.Assign(satGwDevs);

    addr.SetBase("10.0.4.0", "255.255.255.0");
    Ipv4InterfaceContainer ifGwSrv = addr.Assign(gwSrvDevs);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    // ------------------------------------------------------------
    // Geocentric mobility
    // ------------------------------------------------------------
    Ptr<GeocentricConstantPositionMobilityModel> ueMob =
        CreateObject<GeocentricConstantPositionMobilityModel>();
    Ptr<GeocentricConstantPositionMobilityModel> uavMob =
        CreateObject<GeocentricConstantPositionMobilityModel>();
    Ptr<GeocentricConstantPositionMobilityModel> satMob =
        CreateObject<GeocentricConstantPositionMobilityModel>();
    Ptr<GeocentricConstantPositionMobilityModel> gwMob =
        CreateObject<GeocentricConstantPositionMobilityModel>();

    Ptr<UniformRandomVariable> ueEastRv = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> ueNorthRv = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> uavEastRv = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> uavNorthRv = CreateObject<UniformRandomVariable>();

    Vector ueInit = RandomGeoPoint(centerLat,
                                   centerLon,
                                   ueAltM,
                                   ueHalfWidthM,
                                   ueHalfHeightM,
                                   ueEastRv,
                                   ueNorthRv);

    Vector uavInit = RandomGeoPoint(centerLat,
                                    centerLon,
                                    uavAltM,
                                    uavHalfWidthM,
                                    uavHalfHeightM,
                                    uavEastRv,
                                    uavNorthRv);

    ueMob->SetGeographicPosition(ueInit);
    uavMob->SetGeographicPosition(uavInit);
    satMob->SetGeographicPosition(Vector(centerLat, centerLon, geoSatAltM));
    gwMob->SetGeographicPosition(Vector(centerLat, centerLon, gwAltM));

    ueMob->SetCoordinateTranslationReferencePoint(Vector(centerLat, centerLon, 0.0));
    uavMob->SetCoordinateTranslationReferencePoint(Vector(centerLat, centerLon, 0.0));
    satMob->SetCoordinateTranslationReferencePoint(Vector(centerLat, centerLon, 0.0));
    gwMob->SetCoordinateTranslationReferencePoint(Vector(centerLat, centerLon, 0.0));

    ueNode->AggregateObject(ueMob);
    uavNode->AggregateObject(uavMob);
    satNode->AggregateObject(satMob);
    gwNode->AggregateObject(gwMob);

    // ------------------------------------------------------------
    // Random waypoint mobility
    // ------------------------------------------------------------
    RandomWaypointState* ueState = new RandomWaypointState();
    ueState->speedMps = ueSpeedMps;
    ueState->centerLatDeg = centerLat;
    ueState->centerLonDeg = centerLon;
    ueState->halfWidthM = ueHalfWidthM;
    ueState->halfHeightM = ueHalfHeightM;
    ueState->fixedAltM = ueAltM;
    ueState->hasTarget = false;

    RandomWaypointState* uavState = new RandomWaypointState();
    uavState->speedMps = uavSpeedMps;
    uavState->centerLatDeg = centerLat;
    uavState->centerLonDeg = centerLon;
    uavState->halfWidthM = uavHalfWidthM;
    uavState->halfHeightM = uavHalfHeightM;
    uavState->fixedAltM = uavAltM;
    uavState->hasTarget = false;

    SelectNewWaypoint(ueState, ueEastRv, ueNorthRv);
    SelectNewWaypoint(uavState, uavEastRv, uavNorthRv);

    Simulator::Schedule(Seconds(moveStepMs / 1000.0),
                        &MoveNodeRandomWaypoint,
                        ueMob,
                        ueState,
                        ueEastRv,
                        ueNorthRv,
                        moveStepMs);

    Simulator::Schedule(Seconds(moveStepMs / 1000.0),
                        &MoveNodeRandomWaypoint,
                        uavMob,
                        uavState,
                        uavEastRv,
                        uavNorthRv,
                        moveStepMs);

    // ------------------------------------------------------------
    // Real end-to-end application traffic
    // ------------------------------------------------------------
    uint16_t echoPort = 9;

    UdpEchoServerHelper echoServer(echoPort);
    ApplicationContainer serverApps = echoServer.Install(serverNode);
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(MilliSeconds(simTimeMs - 1000));

    UdpEchoClientHelper echoClient(ifGwSrv.GetAddress(1), echoPort);
    echoClient.SetAttribute("MaxPackets", UintegerValue(15));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(512));

    ApplicationContainer clientApps = echoClient.Install(ueNode);
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(MilliSeconds(simTimeMs - 1000));

    // ------------------------------------------------------------
    // Flow monitor
    // ------------------------------------------------------------
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll();

    // ------------------------------------------------------------
    // Access link monitors
    // ------------------------------------------------------------
    AccessLink accessDlLink;
    accessDlLink.propagation = CreateObject<FriisPropagationLossModel>();
    accessDlLink.propagation->SetAttribute("Frequency", DoubleValue(accessFrequencyHz));
    accessDlLink.frequencyHz = accessFrequencyHz;
    accessDlLink.bandwidthHz = accessBandwidthHz;
    accessDlLink.txPowerDbm = uavBsTxPowerDbm;
    accessDlLink.txAntennaGainDb = uavBsGainDb;
    accessDlLink.rxAntennaGainDb = ueGainDb;
    accessDlLink.rxNoiseFigureDb = ueNoiseFigureDb;

    AccessLink accessUlLink;
    accessUlLink.propagation = CreateObject<FriisPropagationLossModel>();
    accessUlLink.propagation->SetAttribute("Frequency", DoubleValue(accessFrequencyHz));
    accessUlLink.frequencyHz = accessFrequencyHz;
    accessUlLink.bandwidthHz = accessBandwidthHz;
    accessUlLink.txPowerDbm = ueTxPowerDbm;
    accessUlLink.txAntennaGainDb = ueGainDb;
    accessUlLink.rxAntennaGainDb = uavBsGainDb;
    accessUlLink.rxNoiseFigureDb = uavAccessNoiseFigureDb;

    // ------------------------------------------------------------
    // NTN channel models
    // ------------------------------------------------------------
    ObjectFactory propFactory;
    ObjectFactory condFactory;

    if (ntnScenario == "NTN-DenseUrban")
    {
        propFactory.SetTypeId(ThreeGppNTNDenseUrbanPropagationLossModel::GetTypeId());
        condFactory.SetTypeId(ThreeGppNTNDenseUrbanChannelConditionModel::GetTypeId());
    }
    else if (ntnScenario == "NTN-Urban")
    {
        propFactory.SetTypeId(ThreeGppNTNUrbanPropagationLossModel::GetTypeId());
        condFactory.SetTypeId(ThreeGppNTNUrbanChannelConditionModel::GetTypeId());
    }
    else if (ntnScenario == "NTN-Suburban")
    {
        propFactory.SetTypeId(ThreeGppNTNSuburbanPropagationLossModel::GetTypeId());
        condFactory.SetTypeId(ThreeGppNTNSuburbanChannelConditionModel::GetTypeId());
    }
    else if (ntnScenario == "NTN-Rural")
    {
        propFactory.SetTypeId(ThreeGppNTNRuralPropagationLossModel::GetTypeId());
        condFactory.SetTypeId(ThreeGppNTNRuralChannelConditionModel::GetTypeId());
    }
    else
    {
        NS_FATAL_ERROR("Unknown NTN scenario: " << ntnScenario);
    }

    Ptr<ChannelConditionModel> serviceCond = condFactory.Create<ThreeGppChannelConditionModel>();
    Ptr<ChannelConditionModel> feederCond = condFactory.Create<ThreeGppChannelConditionModel>();

    double satTxPowDbm =
        (satEIRPDensity + 10.0 * std::log10(ntnBandwidthHz / 1e6) - satAntennaGainDb) + 30.0;

    Ptr<ThreeGppPropagationLossModel> servicePropagation =
        propFactory.Create<ThreeGppPropagationLossModel>();
    servicePropagation->SetAttribute("Frequency", DoubleValue(ntnFrequencyHz));
    servicePropagation->SetAttribute("ShadowingEnabled", BooleanValue(true));
    servicePropagation->SetChannelConditionModel(serviceCond);

    Ptr<ThreeGppSpectrumPropagationLossModel> serviceSpectrum =
        CreateObject<ThreeGppSpectrumPropagationLossModel>();
    serviceSpectrum->SetChannelModelAttribute("Frequency", DoubleValue(ntnFrequencyHz));
    serviceSpectrum->SetChannelModelAttribute("Scenario", StringValue(ntnScenario));
    serviceSpectrum->SetChannelModelAttribute("ChannelConditionModel", PointerValue(serviceCond));

    Ptr<ThreeGppPropagationLossModel> feederPropagation =
        propFactory.Create<ThreeGppPropagationLossModel>();
    feederPropagation->SetAttribute("Frequency", DoubleValue(ntnFrequencyHz));
    feederPropagation->SetAttribute("ShadowingEnabled", BooleanValue(true));
    feederPropagation->SetChannelConditionModel(feederCond);

    Ptr<ThreeGppSpectrumPropagationLossModel> feederSpectrum =
        CreateObject<ThreeGppSpectrumPropagationLossModel>();
    feederSpectrum->SetChannelModelAttribute("Frequency", DoubleValue(ntnFrequencyHz));
    feederSpectrum->SetChannelModelAttribute("Scenario", StringValue(ntnScenario));
    feederSpectrum->SetChannelModelAttribute("ChannelConditionModel", PointerValue(feederCond));

    // Service DL: SAT -> UAV
    NtnLink serviceDlLink;
    serviceDlLink.propagation = servicePropagation;
    serviceDlLink.spectrum = serviceSpectrum;
    serviceDlLink.txDev = uavSatDevs.Get(1);
    serviceDlLink.rxDev = uavSatDevs.Get(0);
    serviceDlLink.frequencyHz = ntnFrequencyHz;
    serviceDlLink.bandwidthHz = ntnBandwidthHz;
    serviceDlLink.rbBandwidthHz = ntnRbBandwidthHz;
    serviceDlLink.txPowerDbm = satTxPowDbm;
    serviceDlLink.rxNoiseFigureDb = uavUtNoiseFigureDb;

    serviceDlLink.txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(satAntennaGainDb))));

    serviceDlLink.rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(uavUtAntennaGainDb))));

    // Service UL: UAV -> SAT
    NtnLink serviceUlLink;
    serviceUlLink.propagation = servicePropagation;
    serviceUlLink.spectrum = serviceSpectrum;
    serviceUlLink.txDev = uavSatDevs.Get(0);
    serviceUlLink.rxDev = uavSatDevs.Get(1);
    serviceUlLink.frequencyHz = ntnFrequencyHz;
    serviceUlLink.bandwidthHz = ntnBandwidthHz;
    serviceUlLink.rbBandwidthHz = ntnRbBandwidthHz;
    serviceUlLink.txPowerDbm = uavUtTxPowerDbm;
    serviceUlLink.rxNoiseFigureDb = satRxNoiseFigureDb;

    serviceUlLink.txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(uavUtAntennaGainDb))));

    serviceUlLink.rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(satAntennaGainDb))));

    // Feeder DL: SAT -> GW
    NtnLink feederDlLink;
    feederDlLink.propagation = feederPropagation;
    feederDlLink.spectrum = feederSpectrum;
    feederDlLink.txDev = satGwDevs.Get(0);
    feederDlLink.rxDev = satGwDevs.Get(1);
    feederDlLink.frequencyHz = ntnFrequencyHz;
    feederDlLink.bandwidthHz = ntnBandwidthHz;
    feederDlLink.rbBandwidthHz = ntnRbBandwidthHz;
    feederDlLink.txPowerDbm = satTxPowDbm;
    feederDlLink.rxNoiseFigureDb = gwNoiseFigureDb;

    feederDlLink.txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(satAntennaGainDb))));

    feederDlLink.rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(gwAntennaGainDb))));

    // Feeder UL: GW -> SAT
    NtnLink feederUlLink;
    feederUlLink.propagation = feederPropagation;
    feederUlLink.spectrum = feederSpectrum;
    feederUlLink.txDev = satGwDevs.Get(1);
    feederUlLink.rxDev = satGwDevs.Get(0);
    feederUlLink.frequencyHz = ntnFrequencyHz;
    feederUlLink.bandwidthHz = ntnBandwidthHz;
    feederUlLink.rbBandwidthHz = ntnRbBandwidthHz;
    feederUlLink.txPowerDbm = gwTxPowerDbm;
    feederUlLink.rxNoiseFigureDb = satRxNoiseFigureDb;

    feederUlLink.txAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(gwAntennaGainDb))));

    feederUlLink.rxAntenna = CreateObjectWithAttributes<UniformPlanarArray>(
        "NumColumns", UintegerValue(1),
        "NumRows", UintegerValue(1),
        "AntennaElement",
        PointerValue(CreateObjectWithAttributes<IsotropicAntennaModel>(
            "Gain", DoubleValue(satAntennaGainDb))));

    // ------------------------------------------------------------
    // Output files
    // ------------------------------------------------------------
    std::ofstream channelTrace("uav_sat_backhaul_channel_trace.txt", std::ios::out);
    NS_ASSERT_MSG(channelTrace.is_open(), "Could not create uav_sat_backhaul_channel_trace.txt");
    channelTrace << "#time_s ue_lat ue_lon ue_alt uav_lat uav_lon uav_alt "
                    "access_dl_snr_db access_ul_snr_db "
                    "service_dl_snr_db service_ul_snr_db "
                    "feeder_dl_snr_db feeder_ul_snr_db "
                    "backhaul_dl_bottleneck_snr_db backhaul_ul_bottleneck_snr_db "
                    "e2e_dl_bottleneck_snr_db e2e_ul_bottleneck_snr_db"
                 << std::endl;

    std::ofstream flowTrace("uav_sat_backhaul_flow_trace.txt", std::ios::out);
    NS_ASSERT_MSG(flowTrace.is_open(), "Could not create uav_sat_backhaul_flow_trace.txt");
    flowTrace << "time_s,flow_id,src,dst,tx_packets,rx_packets,lost_packets,tx_bytes,rx_bytes,"
                 "mean_delay_s,throughput_bps"
              << std::endl;

    // ------------------------------------------------------------
    // Channel logging context
    // ------------------------------------------------------------
    LogContext ctx;
    ctx.ueMob = ueMob;
    ctx.uavMob = uavMob;
    ctx.satMob = satMob;
    ctx.gwMob = gwMob;
    ctx.accessDlLink = &accessDlLink;
    ctx.accessUlLink = &accessUlLink;
    ctx.serviceDlLink = &serviceDlLink;
    ctx.serviceUlLink = &serviceUlLink;
    ctx.feederDlLink = &feederDlLink;
    ctx.feederUlLink = &feederUlLink;
    ctx.file = &channelTrace;

    Simulator::Schedule(Seconds(0.0), &LogScenario, &ctx, logStepMs);
    Simulator::Schedule(Seconds(flowLogStepSec),
                        &LogFlowMetrics,
                        flowMonitor,
                        &flowmonHelper,
                        &flowTrace,
                        flowLogStepSec);

    // ------------------------------------------------------------
    // Run
    // ------------------------------------------------------------
    Simulator::Stop(MilliSeconds(simTimeMs));
    Simulator::Run();

    // ------------------------------------------------------------
    // Final flow summary
    // ------------------------------------------------------------
    flowMonitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

    std::ofstream flowSummary("uav_sat_backhaul_flow_summary.txt", std::ios::out);
    NS_ASSERT_MSG(flowSummary.is_open(), "Could not create uav_sat_backhaul_flow_summary.txt");

    flowSummary << "#FlowId SrcAddr DstAddr TxPackets RxPackets LostPackets "
                   "TxBytes RxBytes Throughput_bps MeanDelay_s"
                << std::endl;

    for (const auto& flow : flowMonitor->GetFlowStats())
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);

        double duration = 0.0;
        if (flow.second.timeLastRxPacket > flow.second.timeFirstTxPacket)
        {
            duration =
                flow.second.timeLastRxPacket.GetSeconds() - flow.second.timeFirstTxPacket.GetSeconds();
        }

        double throughput = (duration > 0.0) ? (flow.second.rxBytes * 8.0 / duration) : 0.0;
        double meanDelay =
            (flow.second.rxPackets > 0)
                ? (flow.second.delaySum.GetSeconds() / flow.second.rxPackets)
                : 0.0;

        flowSummary << flow.first << " "
                    << t.sourceAddress << " "
                    << t.destinationAddress << " "
                    << flow.second.txPackets << " "
                    << flow.second.rxPackets << " "
                    << flow.second.lostPackets << " "
                    << flow.second.txBytes << " "
                    << flow.second.rxBytes << " "
                    << throughput << " "
                    << meanDelay
                    << std::endl;
    }

    flowSummary.close();
    flowTrace.close();
    channelTrace.close();

    Simulator::Destroy();

    delete ueState;
    delete uavState;

    return 0;
}