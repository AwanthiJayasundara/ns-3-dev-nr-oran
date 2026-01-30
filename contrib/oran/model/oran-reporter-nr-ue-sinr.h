#ifndef ORAN_REPORTER_NR_UE_SINR_H
#define ORAN_REPORTER_NR_UE_SINR_H

#include "oran-reporter.h"
#include "oran-report.h"

#include <vector>

namespace ns3
{

/**
 * @ingroup oran
 *
 * Reporter that captures NR UE DL SINR (DlDataSinr / DlCtrlSinr) and sends it to RIC DB.
 */
class OranReporterNrUeSinr : public OranReporter
{
  public:
    static TypeId GetTypeId();

    OranReporterNrUeSinr();
    ~OranReporterNrUeSinr() override;

    // Matches NrUePhy traces:
    // DlDataSinr(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
    void ReportDlDataSinr(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId);

    // DlCtrlSinr(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
    void ReportDlCtrlSinr(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId);

  protected:
    std::vector<Ptr<OranReport>> GenerateReports() override;

  private:
    void AddSinrReport(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId, bool isCtrl);

    std::vector<Ptr<OranReport>> m_reports;
};

} // namespace ns3

#endif // ORAN_REPORTER_NR_UE_SINR_H
