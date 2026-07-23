#include "nr-hybrid-sat-epc-helper.h"

#include "ns3/boolean.h"
#include "ns3/data-rate.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

namespace ns3
{

namespace
{

void
RemoveHostRoutesTo(Ptr<Node> node, Ipv4Address destination)
{
    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4StaticRouting> routing = routingHelper.GetStaticRouting(node->GetObject<Ipv4>());

    for (int32_t i = static_cast<int32_t>(routing->GetNRoutes()) - 1; i >= 0; --i)
    {
        Ipv4RoutingTableEntry route = routing->GetRoute(static_cast<uint32_t>(i));
        if (route.IsHost() && route.GetDest() == destination)
        {
            routing->RemoveRoute(static_cast<uint32_t>(i));
        }
    }
}

void
AddDirectHostRoute(Ptr<Node> node, Ipv4Address destination, uint32_t interface)
{
    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4StaticRouting> routing = routingHelper.GetStaticRouting(node->GetObject<Ipv4>());
    routing->AddHostRouteTo(destination, interface);
}

void
AddGatewayHostRoute(Ptr<Node> node,
                    Ipv4Address destination,
                    Ipv4Address nextHop,
                    uint32_t interface)
{
    Ipv4StaticRoutingHelper routingHelper;
    Ptr<Ipv4StaticRouting> routing = routingHelper.GetStaticRouting(node->GetObject<Ipv4>());
    routing->AddHostRouteTo(destination, nextHop, interface);
}

} // namespace

NS_LOG_COMPONENT_DEFINE("HybridSatEpcHelper");

NS_OBJECT_ENSURE_REGISTERED(HybridSatEpcHelper);

HybridSatEpcHelper::HybridSatEpcHelper()
    : NrNoBackhaulEpcHelper(),
      m_satNode(nullptr),
      m_gwNode(nullptr),
      m_tnS1uLinkDataRate(DataRate("10Gb/s")),
      m_tnS1uLinkDelay(Seconds(0)),
      m_ntnGnbSatRate(DataRate("1Gb/s")),
      m_ntnGnbSatDelay(MilliSeconds(120)),
      m_ntnSatGwRate(DataRate("1Gb/s")),
      m_ntnSatGwDelay(MilliSeconds(120)),
      m_ntnGwSgwRate(DataRate("10Gb/s")),
      m_ntnGwSgwDelay(MilliSeconds(1)),
      m_s1uLinkMtu(2000)
{
    NS_LOG_FUNCTION(this);
}

HybridSatEpcHelper::~HybridSatEpcHelper()
{
    NS_LOG_FUNCTION(this);
}

TypeId
HybridSatEpcHelper::GetTypeId()
{
    NS_LOG_FUNCTION_NOARGS();

    static TypeId tid =
        TypeId("ns3::HybridSatEpcHelper")
            .SetParent<NrNoBackhaulEpcHelper>()
            .SetGroupName("Nr")
            .AddConstructor<HybridSatEpcHelper>()
            .AddAttribute("TnS1uLinkDataRate",
                          "Direct TN gNB <-> SGW data rate",
                          DataRateValue(DataRate("10Gb/s")),
                          MakeDataRateAccessor(&HybridSatEpcHelper::m_tnS1uLinkDataRate),
                          MakeDataRateChecker())
            .AddAttribute("TnS1uLinkDelay",
                          "Direct TN gNB <-> SGW delay",
                          TimeValue(Seconds(0)),
                          MakeTimeAccessor(&HybridSatEpcHelper::m_tnS1uLinkDelay),
                          MakeTimeChecker())
            .AddAttribute("NtnGnbSatRate",
                          "NTN UAV gNB <-> SAT data rate",
                          DataRateValue(DataRate("1Gb/s")),
                          MakeDataRateAccessor(&HybridSatEpcHelper::m_ntnGnbSatRate),
                          MakeDataRateChecker())
            .AddAttribute("NtnGnbSatDelay",
                          "NTN UAV gNB <-> SAT propagation delay",
                          TimeValue(MilliSeconds(120)),
                          MakeTimeAccessor(&HybridSatEpcHelper::m_ntnGnbSatDelay),
                          MakeTimeChecker())
            .AddAttribute("NtnSatGwRate",
                          "SAT <-> GW data rate",
                          DataRateValue(DataRate("1Gb/s")),
                          MakeDataRateAccessor(&HybridSatEpcHelper::m_ntnSatGwRate),
                          MakeDataRateChecker())
            .AddAttribute("NtnSatGwDelay",
                          "SAT <-> GW propagation delay",
                          TimeValue(MilliSeconds(120)),
                          MakeTimeAccessor(&HybridSatEpcHelper::m_ntnSatGwDelay),
                          MakeTimeChecker())
            .AddAttribute("NtnGwSgwRate",
                          "GW <-> SGW data rate",
                          DataRateValue(DataRate("10Gb/s")),
                          MakeDataRateAccessor(&HybridSatEpcHelper::m_ntnGwSgwRate),
                          MakeDataRateChecker())
            .AddAttribute("NtnGwSgwDelay",
                          "GW <-> SGW propagation delay",
                          TimeValue(MilliSeconds(1)),
                          MakeTimeAccessor(&HybridSatEpcHelper::m_ntnGwSgwDelay),
                          MakeTimeChecker())
            .AddAttribute("S1uLinkMtu",
                          "MTU used on created S1-U style links",
                          UintegerValue(2000),
                          MakeUintegerAccessor(&HybridSatEpcHelper::m_s1uLinkMtu),
                          MakeUintegerChecker<uint16_t>());

    return tid;
}

void
HybridSatEpcHelper::NotifyConstructionCompleted()
{
    NrNoBackhaulEpcHelper::NotifyConstructionCompleted();

    NS_LOG_FUNCTION(this);

    // /30 subnets for each leg
    m_tnIpv4AddressHelper.SetBase("10.0.0.0", "255.255.255.252");
    m_gnbSatIpv4AddressHelper.SetBase("10.1.0.0", "255.255.255.252");
    m_satGwIpv4AddressHelper.SetBase("10.2.0.0", "255.255.255.252");
    m_gwSgwIpv4AddressHelper.SetBase("10.3.0.0", "255.255.255.252");
}

void
HybridSatEpcHelper::DoDispose()
{
    NS_LOG_FUNCTION(this);
    m_satNode = nullptr;
    m_gwNode = nullptr;
    NrNoBackhaulEpcHelper::DoDispose();
}

void
HybridSatEpcHelper::SetSatelliteNodes(Ptr<Node> satNode, Ptr<Node> gwNode)
{
    NS_LOG_FUNCTION(this << satNode << gwNode);
    m_satNode = satNode;
    m_gwNode = gwNode;
}

void
HybridSatEpcHelper::AddNtnGnbNode(Ptr<Node> gnbNode)
{
    NS_LOG_FUNCTION(this << gnbNode);
    m_ntnNodeIds.insert(gnbNode->GetId());
}

void
HybridSatEpcHelper::AddGnb(Ptr<Node> gnb, Ptr<NetDevice> nrGnbNetDevice, uint16_t cellId)
{
    NS_LOG_FUNCTION(this << gnb << nrGnbNetDevice << cellId);

    // Register basic EPC-side state first
    NrNoBackhaulEpcHelper::AddGnb(gnb, nrGnbNetDevice, cellId);

    if (m_ntnNodeIds.find(gnb->GetId()) == m_ntnNodeIds.end())
    {
        AddTnBackhaul(gnb, cellId);
    }
    else
    {
        if (m_satNode == nullptr || m_gwNode == nullptr)
        {
            NS_FATAL_ERROR("HybridSatEpcHelper: satellite nodes not set before AddGnb()");
        }
        AddNtnDualBackhaul(gnb, cellId);
    }
}

void
HybridSatEpcHelper::AddTnBackhaul(Ptr<Node> gnb, uint16_t cellId)
{
    NS_LOG_FUNCTION(this << gnb << cellId);

    Ptr<Node> sgw = GetSgwNode();

    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute("DataRate", DataRateValue(m_tnS1uLinkDataRate));
    p2ph.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    p2ph.SetChannelAttribute("Delay", TimeValue(m_tnS1uLinkDelay));

    NetDeviceContainer gnbSgwDevices = p2ph.Install(gnb, sgw);

    m_tnIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer ifaces = m_tnIpv4AddressHelper.Assign(gnbSgwDevices);

    Ipv4Address gnbS1uAddress = ifaces.GetAddress(0);
    Ipv4Address sgwS1uAddress = ifaces.GetAddress(1);

    NrNoBackhaulEpcHelper::AddS1Interface(gnb, gnbS1uAddress, sgwS1uAddress, cellId);
}

void
HybridSatEpcHelper::AddNtnSatelliteBackhaul(Ptr<Node> gnb, uint16_t cellId)
{
    NS_LOG_FUNCTION(this << gnb << cellId);

    Ptr<Node> sgw = GetSgwNode();

    // Enable forwarding on transit nodes
    m_satNode->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    m_gwNode->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    // gNB <-> SAT
    PointToPointHelper gnbSat;
    gnbSat.SetDeviceAttribute("DataRate", DataRateValue(m_ntnGnbSatRate));
    gnbSat.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    gnbSat.SetChannelAttribute("Delay", TimeValue(m_ntnGnbSatDelay));
    NetDeviceContainer gnbSatDevs = gnbSat.Install(gnb, m_satNode);

    // SAT <-> GW
    PointToPointHelper satGw;
    satGw.SetDeviceAttribute("DataRate", DataRateValue(m_ntnSatGwRate));
    satGw.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    satGw.SetChannelAttribute("Delay", TimeValue(m_ntnSatGwDelay));
    NetDeviceContainer satGwDevs = satGw.Install(m_satNode, m_gwNode);

    // GW <-> SGW
    PointToPointHelper gwSgw;
    gwSgw.SetDeviceAttribute("DataRate", DataRateValue(m_ntnGwSgwRate));
    gwSgw.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    gwSgw.SetChannelAttribute("Delay", TimeValue(m_ntnGwSgwDelay));
    NetDeviceContainer gwSgwDevs = gwSgw.Install(m_gwNode, sgw);

    // Address each leg
    m_gnbSatIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer gnbSatIf = m_gnbSatIpv4AddressHelper.Assign(gnbSatDevs);

    m_satGwIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer satGwIf = m_satGwIpv4AddressHelper.Assign(satGwDevs);

    m_gwSgwIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer gwSgwIf = m_gwSgwIpv4AddressHelper.Assign(gwSgwDevs);

    Ipv4Address gnbAddr  = gnbSatIf.GetAddress(0);
    Ipv4Address satToGnb = gnbSatIf.GetAddress(1);

    Ipv4Address satToGw  = satGwIf.GetAddress(0);
    Ipv4Address gwToSat  = satGwIf.GetAddress(1);

    Ipv4Address gwToSgw  = gwSgwIf.GetAddress(0);
    Ipv4Address sgwAddr  = gwSgwIf.GetAddress(1);

    Ptr<Ipv4> gnbIpv4 = gnb->GetObject<Ipv4>();
    Ptr<Ipv4> satIpv4 = m_satNode->GetObject<Ipv4>();
    Ptr<Ipv4> gwIpv4 = m_gwNode->GetObject<Ipv4>();
    Ptr<Ipv4> sgwIpv4 = sgw->GetObject<Ipv4>();

    int32_t gnbIf = gnbIpv4->GetInterfaceForDevice(gnbSatDevs.Get(0));
    int32_t satIfToGw = satIpv4->GetInterfaceForDevice(satGwDevs.Get(0));
    int32_t gwIfToSat = gwIpv4->GetInterfaceForDevice(satGwDevs.Get(1));
    int32_t sgwIf = sgwIpv4->GetInterfaceForDevice(gwSgwDevs.Get(1));

    Ipv4StaticRoutingHelper routingHelper;

    Ptr<Ipv4StaticRouting> gnbRt = routingHelper.GetStaticRouting(gnbIpv4);
    Ptr<Ipv4StaticRouting> satRt = routingHelper.GetStaticRouting(satIpv4);
    Ptr<Ipv4StaticRouting> gwRt = routingHelper.GetStaticRouting(gwIpv4);
    Ptr<Ipv4StaticRouting> sgwRt = routingHelper.GetStaticRouting(sgwIpv4);

    // gNB reaches SGW via SAT
    gnbRt->AddHostRouteTo(sgwAddr, satToGnb, gnbIf);

    // SAT reaches SGW via GW
    satRt->AddHostRouteTo(sgwAddr, gwToSat, satIfToGw);

    // SGW reaches gNB via GW
    sgwRt->AddHostRouteTo(gnbAddr, gwToSgw, sgwIf);

    // GW reaches gNB via SAT
    gwRt->AddHostRouteTo(gnbAddr, satToGw, gwIfToSat);

    // Register S1 endpoint pair as (gNB address, SGW address)
    NrNoBackhaulEpcHelper::AddS1Interface(gnb, gnbAddr, sgwAddr, cellId);
}

void
HybridSatEpcHelper::AddNtnDualBackhaul(Ptr<Node> gnb, uint16_t cellId)
{
    NS_LOG_FUNCTION(this << gnb << cellId);

    Ptr<Node> sgw = GetSgwNode();

    m_satNode->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));
    m_gwNode->GetObject<Ipv4>()->SetAttribute("IpForward", BooleanValue(true));

    // Direct terrestrial xHaul leg: UAV gNB <-> SGW/core.
    PointToPointHelper tnDirect;
    tnDirect.SetDeviceAttribute("DataRate", DataRateValue(m_tnS1uLinkDataRate));
    tnDirect.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    tnDirect.SetChannelAttribute("Delay", TimeValue(m_tnS1uLinkDelay));
    NetDeviceContainer gnbSgwDevs = tnDirect.Install(gnb, sgw);

    // Satellite fallback legs: UAV gNB <-> SAT <-> GW <-> SGW/core.
    PointToPointHelper gnbSat;
    gnbSat.SetDeviceAttribute("DataRate", DataRateValue(m_ntnGnbSatRate));
    gnbSat.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    gnbSat.SetChannelAttribute("Delay", TimeValue(m_ntnGnbSatDelay));
    NetDeviceContainer gnbSatDevs = gnbSat.Install(gnb, m_satNode);

    PointToPointHelper satGw;
    satGw.SetDeviceAttribute("DataRate", DataRateValue(m_ntnSatGwRate));
    satGw.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    satGw.SetChannelAttribute("Delay", TimeValue(m_ntnSatGwDelay));
    NetDeviceContainer satGwDevs = satGw.Install(m_satNode, m_gwNode);

    PointToPointHelper gwSgw;
    gwSgw.SetDeviceAttribute("DataRate", DataRateValue(m_ntnGwSgwRate));
    gwSgw.SetDeviceAttribute("Mtu", UintegerValue(m_s1uLinkMtu));
    gwSgw.SetChannelAttribute("Delay", TimeValue(m_ntnGwSgwDelay));
    NetDeviceContainer gwSgwDevs = gwSgw.Install(m_gwNode, sgw);

    m_tnIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer tnIf = m_tnIpv4AddressHelper.Assign(gnbSgwDevs);

    m_gnbSatIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer gnbSatIf = m_gnbSatIpv4AddressHelper.Assign(gnbSatDevs);

    m_satGwIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer satGwIf = m_satGwIpv4AddressHelper.Assign(satGwDevs);

    m_gwSgwIpv4AddressHelper.NewNetwork();
    Ipv4InterfaceContainer gwSgwIf = m_gwSgwIpv4AddressHelper.Assign(gwSgwDevs);

    NtnDualBackhaulRouteInfo info;
    info.cellId = cellId;
    info.gnb = gnb;
    info.sat = m_satNode;
    info.gw = m_gwNode;
    info.sgw = sgw;

    info.gnbTnAddr = tnIf.GetAddress(0);
    info.sgwTnAddr = tnIf.GetAddress(1);
    info.gnbSatAddr = gnbSatIf.GetAddress(0);
    info.satToGnbAddr = gnbSatIf.GetAddress(1);
    info.satToGwAddr = satGwIf.GetAddress(0);
    info.gwToSatAddr = satGwIf.GetAddress(1);
    info.gwToSgwAddr = gwSgwIf.GetAddress(0);
    info.sgwGwAddr = gwSgwIf.GetAddress(1);

    Ptr<Ipv4> gnbIpv4 = gnb->GetObject<Ipv4>();
    Ptr<Ipv4> satIpv4 = m_satNode->GetObject<Ipv4>();
    Ptr<Ipv4> gwIpv4 = m_gwNode->GetObject<Ipv4>();
    Ptr<Ipv4> sgwIpv4 = sgw->GetObject<Ipv4>();

    info.gnbTnIf = gnbIpv4->GetInterfaceForDevice(gnbSgwDevs.Get(0));
    info.sgwTnIf = sgwIpv4->GetInterfaceForDevice(gnbSgwDevs.Get(1));
    info.gnbSatIf = gnbIpv4->GetInterfaceForDevice(gnbSatDevs.Get(0));
    info.satIfToGnb = satIpv4->GetInterfaceForDevice(gnbSatDevs.Get(1));
    info.satIfToGw = satIpv4->GetInterfaceForDevice(satGwDevs.Get(0));
    info.gwIfToSat = gwIpv4->GetInterfaceForDevice(satGwDevs.Get(1));
    info.gwIfToSgw = gwIpv4->GetInterfaceForDevice(gwSgwDevs.Get(0));
    info.sgwGwIf = sgwIpv4->GetInterfaceForDevice(gwSgwDevs.Get(1));

    m_ntnDualBackhaulRoutes[cellId] = info;

    // The S1-U endpoint pair remains stable. Runtime mode changes only alter
    // routes to these endpoint addresses, so the RAN/xApp handover path is not
    // touched by backhaul switching.
    NrNoBackhaulEpcHelper::AddS1Interface(gnb, info.gnbTnAddr, info.sgwTnAddr, cellId);

    ApplyNtnDualBackhaulMode(m_ntnDualBackhaulRoutes[cellId], "tn");
}

void
HybridSatEpcHelper::SetNtnBackhaulMode(uint16_t cellId, const std::string& mode)
{
    auto it = m_ntnDualBackhaulRoutes.find(cellId);
    if (it == m_ntnDualBackhaulRoutes.end())
    {
        return;
    }
    ApplyNtnDualBackhaulMode(it->second, mode);
}

void
HybridSatEpcHelper::ApplyNtnDualBackhaulMode(NtnDualBackhaulRouteInfo& info,
                                             const std::string& mode)
{
    const std::string normalizedMode =
        (mode == "satellite") ? "satellite" : ((mode == "unavailable" || mode == "blocked") ? "unavailable" : "tn");
    if (info.mode == normalizedMode)
    {
        return;
    }

    RemoveHostRoutesTo(info.gnb, info.sgwTnAddr);
    RemoveHostRoutesTo(info.sgw, info.gnbTnAddr);
    RemoveHostRoutesTo(info.sat, info.sgwTnAddr);
    RemoveHostRoutesTo(info.sat, info.gnbTnAddr);
    RemoveHostRoutesTo(info.gw, info.sgwTnAddr);
    RemoveHostRoutesTo(info.gw, info.gnbTnAddr);

    if (normalizedMode == "satellite")
    {
        AddGatewayHostRoute(info.gnb, info.sgwTnAddr, info.satToGnbAddr, info.gnbSatIf);
        AddGatewayHostRoute(info.sat, info.sgwTnAddr, info.gwToSatAddr, info.satIfToGw);
        AddGatewayHostRoute(info.gw, info.sgwTnAddr, info.sgwGwAddr, info.gwIfToSgw);

        AddGatewayHostRoute(info.sgw, info.gnbTnAddr, info.gwToSgwAddr, info.sgwGwIf);
        AddGatewayHostRoute(info.gw, info.gnbTnAddr, info.satToGwAddr, info.gwIfToSat);
        AddGatewayHostRoute(info.sat, info.gnbTnAddr, info.gnbSatAddr, info.satIfToGnb);
    }
    else if (normalizedMode == "tn")
    {
        AddDirectHostRoute(info.gnb, info.sgwTnAddr, info.gnbTnIf);
        AddDirectHostRoute(info.sgw, info.gnbTnAddr, info.sgwTnIf);
    }
    // unavailable: leave the S1-U endpoint routes removed. This models a UAV
    // donor/backhaul outage when no satellite fallback path is allowed.

    info.mode = normalizedMode;
    NS_LOG_INFO("NTN_BACKHAUL_MODE cell=" << info.cellId
                                          << " gnbS1u=" << info.gnbTnAddr
                                          << " sgwS1u=" << info.sgwTnAddr
                                          << " mode=" << info.mode);
}

} // namespace ns3
