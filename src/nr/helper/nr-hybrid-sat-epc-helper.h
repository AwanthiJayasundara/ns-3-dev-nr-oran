#ifndef NR_HYBRID_SAT_EPC_HELPER_H
#define NR_HYBRID_SAT_EPC_HELPER_H

#include "nr-no-backhaul-epc-helper.h"

#include <set>

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

  protected:
    void NotifyConstructionCompleted() override;

  private:
    void AddTnBackhaul(Ptr<Node> gnb, uint16_t cellId);
    void AddNtnSatelliteBackhaul(Ptr<Node> gnb, uint16_t cellId);

  private:
    Ptr<Node> m_satNode;
    Ptr<Node> m_gwNode;

    std::set<uint32_t> m_ntnNodeIds;

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