#ifndef ORAN_LM_NR_2_NR_RSRP_HANDOVER_H
#define ORAN_LM_NR_2_NR_RSRP_HANDOVER_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include "ns3/nstime.h"
#include "ns3/vector.h"

namespace ns3
{

/**
 * @ingroup oran
 *
 * Logic Module for the Near-RT RIC that issues Commands to handover from
 * an NR cell to another based on the RSRP from the UE to the gNBs (NO secrecy).
 *
 * -------------------------------------------------------------------------
 * REQUIRED STEPS (build + use):
 *  1) Add this .cc to your ns-oran module build (wscript/CMakeLists.txt).
 *  2) Include this header in your scenario and create the LM:
 *        Ptr<OranLmNr2NrRsrpHandover> lm = CreateObject<OranLmNr2NrRsrpHandover>();
 *        lm->SetAttribute("Active", BooleanValue(true));   // inherited from OranLm
 *     then attach it to the Near-RT RIC (same way you did for secrecy LM).
 *  3) Ensure RSRP/RSRQ reports are being logged into the OranDataRepository
 *     (otherwise GetNrUeRsrpRsrq() returns empty and no HO will happen).
 *
 * NOTE:
 *  - This is RSRP-only HO logic (no secrecy checks, no SINR requirement).
 *  - If you still hit an SRS-related assert in 5G-LENA, disable SRS in your
 *    simulation script (scheduler attributes), not inside this LM.
 * -------------------------------------------------------------------------
 */
class OranLmNr2NrRsrpHandover : public OranLm
{
  protected:
    /**
     * UE related information.
     */
    struct UeInfo
    {
        uint64_t nodeId; //!< The node ID.
        uint16_t cellId; //!< The cell ID (from GetNrUeCellInfo()).
        uint16_t rnti;   //!< The RNTI ID (from GetNrUeCellInfo()).
        Vector position; //!< The physical position.
    };

    /**
     * gNB related information.
     */
    struct GnbInfo
    {
        uint64_t nodeId; //!< The node ID.
        uint16_t cellId; //!< The cell ID.
        Vector position; //!< The physical position.
    };

  public:
    static TypeId GetTypeId(void);

    OranLmNr2NrRsrpHandover(void);

    ~OranLmNr2NrRsrpHandover(void) override;

    std::vector<Ptr<OranCommand>> Run(void) override;

  private:
    std::vector<OranLmNr2NrRsrpHandover::UeInfo> GetUeInfos(Ptr<OranDataRepository> data) const;

    std::vector<OranLmNr2NrRsrpHandover::GnbInfo> GetGnbInfos(Ptr<OranDataRepository> data) const;

    std::vector<Ptr<OranCommand>> GetHandoverCommands(
        Ptr<OranDataRepository> data,
        std::vector<OranLmNr2NrRsrpHandover::UeInfo> ueInfos,
        std::vector<OranLmNr2NrRsrpHandover::GnbInfo> gnbInfos) const;

  private:
    /**
     * RSRP hysteresis margin in dB.
     * (legacy name kept: m_rsrpThreshold)
     */
    double m_rsrpThreshold;

    /**
     * HO pacing/robustness knobs.
     */
    Time m_warmup;
    Time m_minHoInterval;
    Time m_hoAttemptTimeout;
};

} // namespace ns3

#endif /* ORAN_LM_NR_2_NR_RSRP_HANDOVER_H */
