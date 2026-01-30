#ifndef ORAN_LM_NR_SECRECY_AWARE_HANDOVER_H
#define ORAN_LM_NR_SECRECY_AWARE_HANDOVER_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include "ns3/nstime.h"
#include "ns3/type-id.h"
#include "ns3/vector.h"

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * @ingroup oran
 *
 * Secrecy-aware RSRP handover LM:
 * - Selects best cell by RSRP (like OranLmNr2NrRsrpHandover),
 * - BUT issues HO only if secrecy is not in outage:
 *     Cs = [log2(1+gb) - log2(1+ge)]+  >= SecrecyRateThr
 *
 * gb is estimated from serving SINR + RSRP delta (heuristic),
 * ge is modeled by a constant EavSinrDb (simple attacker model).
 */
class OranLmNrSecrecyAwareHandover : public OranLm
{
  protected:
    struct UeInfo
    {
        uint64_t nodeId;
        uint16_t cellId;
        uint16_t rnti;
        Vector position;
    };

    struct GnbInfo
    {
        uint64_t nodeId;
        uint16_t cellId;
        Vector position;
    };

  public:
    static TypeId GetTypeId(void);

    OranLmNrSecrecyAwareHandover(void);
    ~OranLmNrSecrecyAwareHandover(void) override;

    std::vector<Ptr<OranCommand>> Run(void) override;

  private:
    std::vector<UeInfo> GetUeInfos(Ptr<OranDataRepository> data) const;
    std::vector<GnbInfo> GetGnbInfos(Ptr<OranDataRepository> data) const;

    std::vector<Ptr<OranCommand>> GetHandoverCommands(Ptr<OranDataRepository> data,
                                                      const std::vector<UeInfo>& ueInfos,
                                                      const std::vector<GnbInfo>& gnbInfos) const;

    // Read serving DlDataSinr (bwpId=0, isCtrl=false) from DB
    bool GetServingDlDataSinrLin(Ptr<OranDataRepository> data,
                                 uint64_t ueE2NodeId,
                                 uint16_t servingCellId,
                                 uint16_t servingRnti,
                                 double& outSinrLin) const;

    bool GetWorstEveSinrLinForCell(Ptr<OranDataRepository> data,
                                uint16_t cellId,
                                double& worstSinrLin) const;

    // Secrecy math helpers
    static double DbToLin(double db);
    static double SecrecyCapacity(double gbLin, double geLin);

  private:
    // RSRP HO parameters
    double m_hysteresisDb;
    double m_rsrpThresholdDb;

    Time m_warmup;
    Time m_minHoInterval;
    Time m_hoAttemptTimeout;

    // Secrecy parameters
    double m_secrecyRateThr;
    double m_eavSinrDb;

    bool m_requireSinr;
};

} // namespace ns3

#endif /* ORAN_LM_NR_SECRECY_AWARE_HANDOVER_H */
