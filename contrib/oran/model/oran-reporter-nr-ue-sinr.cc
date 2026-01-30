#include "oran-reporter-nr-ue-sinr.h"

#include "oran-report-nr-ue-sinr.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReporterNrUeSinr");
NS_OBJECT_ENSURE_REGISTERED(OranReporterNrUeSinr);

TypeId
OranReporterNrUeSinr::GetTypeId()
{
    static TypeId tid = TypeId("ns3::OranReporterNrUeSinr")
                            .SetParent<OranReporter>()
                            .AddConstructor<OranReporterNrUeSinr>();
    return tid;
}

OranReporterNrUeSinr::OranReporterNrUeSinr()
{
    NS_LOG_FUNCTION(this);
}

OranReporterNrUeSinr::~OranReporterNrUeSinr()
{
    NS_LOG_FUNCTION(this);
}

void
OranReporterNrUeSinr::ReportDlDataSinr(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    NS_LOG_FUNCTION(this << +cellId << +rnti << sinr << +bwpId);
    AddSinrReport(cellId, rnti, sinr, bwpId, false);
}

void
OranReporterNrUeSinr::ReportDlCtrlSinr(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId)
{
    NS_LOG_FUNCTION(this << +cellId << +rnti << sinr << +bwpId);
    AddSinrReport(cellId, rnti, sinr, bwpId, true);
}

void
OranReporterNrUeSinr::AddSinrReport(uint16_t cellId, uint16_t rnti, double sinr, uint16_t bwpId, bool isCtrl)
{
    if (!m_active)
    {
        return;
    }

    NS_ABORT_MSG_IF(m_terminator == nullptr,
                    "Attempting to generate reports in reporter with NULL E2 Terminator");

    const double sinrDb =
        (sinr > 0.0) ? (10.0 * std::log10(sinr)) : (-std::numeric_limits<double>::infinity());

    Ptr<OranReportNrUeSinr> report = CreateObject<OranReportNrUeSinr>();
    report->SetAttribute("ReporterE2NodeId", UintegerValue(m_terminator->GetE2NodeId()));
    report->SetAttribute("Time", TimeValue(Simulator::Now()));
    report->SetAttribute("Rnti", UintegerValue(rnti));
    report->SetAttribute("CellId", UintegerValue(cellId));
    report->SetAttribute("BwpId", UintegerValue(bwpId));
    report->SetAttribute("SinrLin", DoubleValue(sinr));
    report->SetAttribute("SinrDb", DoubleValue(std::isfinite(sinrDb) ? sinrDb : -1e9));
    report->SetAttribute("IsCtrl", BooleanValue(isCtrl));

    m_reports.push_back(report);
}

std::vector<Ptr<OranReport>>
OranReporterNrUeSinr::GenerateReports()
{
    NS_LOG_FUNCTION(this);

    std::vector<Ptr<OranReport>> reports;
    if (m_active)
    {
        reports = m_reports;
        m_reports.clear();
    }
    return reports;
}

} // namespace ns3
