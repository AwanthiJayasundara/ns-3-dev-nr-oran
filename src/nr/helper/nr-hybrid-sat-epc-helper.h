#ifndef NR_HYBRID_SAT_EPC_HELPER_H
#define NR_HYBRID_SAT_EPC_HELPER_H

#include "nr-no-backhaul-epc-helper.h"

#include "ns3/ipv4-address.h"
#include "ns3/ptr.h"

#include <map>
#include <set>
#include <string>

namespace ns3
{

class HybridSatEpcHelper : public NrNoBackhaulEpcHelper
{
  public:
    HybridSatEpcHelper();
    ~HybridSatEpcHelper() override;

    static TypeId GetTypeId();

    void DoDispose() override;
    void AddGnb(Ptr<Node> gnbNode, Ptr<NetDevice> nrGnbNetDevice, uint16_t cellId) override;

    void SetSatelliteNodes(Ptr<Node> satNode, Ptr<Node> gwNode);
    void AddNtnGnbNode(Ptr<Node> gnbNode);
    void SetNtnBackhaulMode(uint16_t cellId, const std::string& mode);

  protected:
    void NotifyConstructionCompleted() override;

  private:
    void AddTnBackhaul(Ptr<Node> gnb, uint16_t cellId);
    void AddNtnSatelliteBackhaul(Ptr<Node> gnb, uint16_t cellId);
    void AddNtnDualBackhaul(Ptr<Node> gnb, uint16_t cellId);

    struct NtnDualBackhaulRouteInfo;
    void ApplyNtnDualBackhaulMode(NtnDualBackhaulRouteInfo& info, const std::string& mode);

  private:
    Ptr<Node> m_satNode;
    Ptr<Node> m_gwNode;

    std::set<uint32_t> m_ntnNodeIds;

    struct NtnDualBackhaulRouteInfo
    {
        uint16_t cellId = 0;
        Ptr<Node> gnb;
        Ptr<Node> sat;
        Ptr<Node> gw;
        Ptr<Node> sgw;

        Ipv4Address gnbTnAddr;
        Ipv4Address sgwTnAddr;
        Ipv4Address gnbSatAddr;
        Ipv4Address satToGnbAddr;
        Ipv4Address satToGwAddr;
        Ipv4Address gwToSatAddr;
        Ipv4Address gwToSgwAddr;
        Ipv4Address sgwGwAddr;

        int32_t gnbTnIf = -1;
        int32_t sgwTnIf = -1;
        int32_t gnbSatIf = -1;
        int32_t satIfToGnb = -1;
        int32_t satIfToGw = -1;
        int32_t gwIfToSat = -1;
        int32_t gwIfToSgw = -1;
        int32_t sgwGwIf = -1;

        std::string mode;
    };

    std::map<uint16_t, NtnDualBackhaulRouteInfo> m_ntnDualBackhaulRoutes;

    // TN direct backhaul
    DataRate m_tnS1uLinkDataRate;
    Time m_tnS1uLinkDelay;

    // NTN routed legs
    DataRate m_ntnGnbSatRate;
    Time m_ntnGnbSatDelay;

    DataRate m_ntnSatGwRate;
    Time m_ntnSatGwDelay;

    DataRate m_ntnGwSgwRate;
    Time m_ntnGwSgwDelay;

    uint16_t m_s1uLinkMtu;

    // Address helpers
    Ipv4AddressHelper m_tnIpv4AddressHelper;
    Ipv4AddressHelper m_gnbSatIpv4AddressHelper;
    Ipv4AddressHelper m_satGwIpv4AddressHelper;
    Ipv4AddressHelper m_gwSgwIpv4AddressHelper;
};

} // namespace ns3

#endif // NR_HYBRID_SAT_EPC_HELPER_H
