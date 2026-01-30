#include "oran-report-nr-ue-sinr.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"

#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranReportNrUeSinr");
NS_OBJECT_ENSURE_REGISTERED(OranReportNrUeSinr);

TypeId
OranReportNrUeSinr::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::OranReportNrUeSinr")
            .SetParent<OranReport>()
            .AddConstructor<OranReportNrUeSinr>()
            .AddAttribute("Rnti",
                          "The RNTI.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&OranReportNrUeSinr::m_rnti),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("CellId",
                          "The cell ID.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&OranReportNrUeSinr::m_cellId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("BwpId",
                          "The BWP ID from SINR trace.",
                          UintegerValue(0),
                          MakeUintegerAccessor(&OranReportNrUeSinr::m_bwpId),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("SinrLin",
                          "Downlink SINR (linear).",
                          DoubleValue(0.0),
                          MakeDoubleAccessor(&OranReportNrUeSinr::m_sinrLin),
                          MakeDoubleChecker<double>())
            .AddAttribute("SinrDb",
                          "Downlink SINR (dB).",
                          DoubleValue(-1e9),
                          MakeDoubleAccessor(&OranReportNrUeSinr::m_sinrDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("IsCtrl",
                          "True if this was DlCtrlSinr, false if DlDataSinr.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranReportNrUeSinr::m_isCtrl),
                          MakeBooleanChecker());

    return tid;
}

OranReportNrUeSinr::OranReportNrUeSinr()
    : OranReport(),
      m_rnti(0),
      m_cellId(0),
      m_bwpId(0),
      m_sinrLin(0.0),
      m_sinrDb(-1e9),
      m_isCtrl(false)
{
    NS_LOG_FUNCTION(this);
}

OranReportNrUeSinr::~OranReportNrUeSinr()
{
    NS_LOG_FUNCTION(this);
}

std::string
OranReportNrUeSinr::ToString() const
{
    NS_LOG_FUNCTION(this);

    std::stringstream ss;
    Time time = GetTime();

    ss << "OranReportNrUeSinr("
       << "E2NodeId=" << GetReporterE2NodeId()
       << ";Time=" << time.As(Time::S)
       << ";RNTI=" << +m_rnti
       << ";CellId=" << +m_cellId
       << ";BwpId=" << +m_bwpId
       << ";SinrLin=" << m_sinrLin
       << ";SinrDb=" << m_sinrDb
       << ";IsCtrl=" << m_isCtrl
       << ")";

    return ss.str();
}

uint16_t OranReportNrUeSinr::GetRnti() const { return m_rnti; }
uint16_t OranReportNrUeSinr::GetCellId() const { return m_cellId; }
uint16_t OranReportNrUeSinr::GetBwpId() const { return m_bwpId; }
double   OranReportNrUeSinr::GetSinrLin() const { return m_sinrLin; }
double   OranReportNrUeSinr::GetSinrDb() const { return m_sinrDb; }
bool     OranReportNrUeSinr::GetIsCtrl() const { return m_isCtrl; }

} // namespace ns3
