#ifndef ORAN_LM_NR_2_NR_RSRP_HANDOVER_WITH_TN_NTN_H
#define ORAN_LM_NR_2_NR_RSRP_HANDOVER_WITH_TN_NTN_H

#include "ns3/oran-lm.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

#include <cstdint>
#include <map>
#include <vector>
#include <fstream>
#include <string>

namespace ns3
{

class OranDataRepository;
class OranCommand;

class OranLmNr2NrRsrpHandoverWithTnNtn : public OranLm
{
  public:
    static TypeId GetTypeId(void);

    OranLmNr2NrRsrpHandoverWithTnNtn(void);
    ~OranLmNr2NrRsrpHandoverWithTnNtn(void) override;

    void SetCellCapacity(uint16_t cellId, uint32_t maxUes);
    void SetCellIsNtn(uint16_t cellId, bool isNtn);

    void SetCellBackhaulDlSnrDb(uint16_t cellId, double snrDb);
    void SetCellBackhaulUlSnrDb(uint16_t cellId, double snrDb);
    void SetDecisionCsvFilename(const std::string& filename);

    std::vector<Ptr<OranCommand>> Run(void) override;

  private:
    struct UeInfo
    {
        uint64_t nodeId{0};
        uint16_t cellId{0};
        uint16_t rnti{0};
        Vector position{0.0, 0.0, 0.0};
    };

    struct GnbInfo
    {
        uint64_t nodeId{0};
        uint16_t cellId{0};
        Vector position{0.0, 0.0, 0.0};
    };

    std::vector<UeInfo> GetUeInfos(Ptr<OranDataRepository> data) const;
    std::vector<GnbInfo> GetGnbInfos(Ptr<OranDataRepository> data) const;

    std::vector<Ptr<OranCommand>> GetHandoverCommands(Ptr<OranDataRepository> data,
                                                      std::vector<UeInfo> ueInfos,
                                                      std::vector<GnbInfo> gnbInfos) const;

  private:
    double m_rsrpThreshold;
    Time m_warmup;
    Time m_minHoInterval;
    Time m_hoAttemptTimeout;
    Time m_timeToTrigger;
    Time m_lowRsrpRecheck;

    uint32_t m_maxUesPerCell;
    bool m_tryNextBest;
    double m_minAcceptableRsrpDbm;

    std::map<uint16_t, uint32_t> m_cellCapacityMap;
    std::map<uint16_t, bool> m_cellIsNtnMap;

    std::map<uint16_t, double> m_cellBackhaulDlSnrDb;
std::map<uint16_t, double> m_cellBackhaulUlSnrDb;

std::string m_decisionCsvFilename;
mutable std::ofstream m_decisionCsv;

void EnsureDecisionCsvOpen(void) const;

void LogDecisionRow(uint64_t ueId,
                    uint16_t rnti,
                    uint16_t servingCellId,
                    double servingRsrp,
                    uint16_t candidateCellId,
                    double candidateRsrp,
                    uint32_t candidateLoad,
                    uint32_t candidateEffLoad,
                    uint32_t candidateCap,
                    bool candidateIsNtn,
                    double backhaulDlSnr,
                    double backhaulUlSnr,
                    uint16_t finalChosenCell) const;

    double m_tnMinRsrpDbm;
    double m_ntnEnterMarginDb;
    double m_tnReturnMarginDb;
};

} // namespace ns3

#endif // ORAN_LM_NR_2_NR_RSRP_HANDOVER_WITH_TN_NTN_H