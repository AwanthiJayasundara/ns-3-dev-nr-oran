#ifndef ORAN_REPORT_NR_UE_SINR_H
#define ORAN_REPORT_NR_UE_SINR_H

#include "oran-report.h"

#include <string>

namespace ns3
{

/**
 * @ingroup oran
 *
 * Report with NR UE downlink SINR measurement.
 */
class OranReportNrUeSinr : public OranReport
{
  public:
    static TypeId GetTypeId();

    OranReportNrUeSinr();
    ~OranReportNrUeSinr() override;

    std::string ToString() const override;

    uint16_t GetRnti() const;
    uint16_t GetCellId() const;
    uint16_t GetBwpId() const;
    double GetSinrLin() const;
    double GetSinrDb() const;
    bool GetIsCtrl() const;

  private:
    uint16_t m_rnti;
    uint16_t m_cellId;
    uint16_t m_bwpId;
    double   m_sinrLin;
    double   m_sinrDb;
    bool     m_isCtrl;
};

} // namespace ns3

#endif // ORAN_REPORT_NR_UE_SINR_H
