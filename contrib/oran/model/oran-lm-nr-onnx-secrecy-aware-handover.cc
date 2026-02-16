#include "oran-lm-nr-onnx-secrecy-aware-handover.h"

#include "oran-command-nr-2-nr-handover.h"
#include "oran-data-repository.h"

#include "ns3/abort.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <tuple>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("OranLmNrOnnxSecrecyAwareHandover");
NS_OBJECT_ENSURE_REGISTERED(OranLmNrOnnxSecrecyAwareHandover);

static inline double LinToDbSafe(double x)
{
    if (x <= 0.0) return -1e9;
    return 10.0 * std::log10(x);
}

static inline void SetUintegerAttrIfExists(Ptr<Object> obj, const std::string& name, uint64_t v)
{
    TypeId tid = obj->GetInstanceTypeId();
    TypeId::AttributeInformation info;
    if (tid.LookupAttributeByName(name, &info))
    {
        obj->SetAttribute(name, UintegerValue(v));
    }
}

TypeId
OranLmNrOnnxSecrecyAwareHandover::GetTypeId(void)
{
    static TypeId tid =
        TypeId("ns3::OranLmNrOnnxSecrecyAwareHandover")
            .SetParent<OranLm>()
            .AddConstructor<OranLmNrOnnxSecrecyAwareHandover>()

            .AddAttribute("HysteresisDb",
                          "RSRP HO hysteresis in dB.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_hysteresisDb),
                          MakeDoubleChecker<double>())

            .AddAttribute("RsrpThresholdDb",
                          "Optional min RSRP gain (dB) to allow HO.",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_rsrpThresholdDb),
                          MakeDoubleChecker<double>())

            .AddAttribute("Warmup",
                          "Warm-up time before HO decisions start.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_warmup),
                          MakeTimeChecker())

            .AddAttribute("MinHoInterval",
                          "Minimum time between HO commands for same UE.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_minHoInterval),
                          MakeTimeChecker())

            .AddAttribute("HoAttemptTimeout",
                          "If HO is pending, wait this long before allowing a retry.",
                          TimeValue(Seconds(2.0)),
                          MakeTimeAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_hoAttemptTimeout),
                          MakeTimeChecker())

            .AddAttribute("SecrecyRateThr",
                          "Minimum secrecy capacity (bits/s/Hz).",
                          DoubleValue(0.10),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_secrecyRateThr),
                          MakeDoubleChecker<double>())

            .AddAttribute("EavSinrDb",
                          "Assumed eavesdropper SINR (dB) for simple secrecy model.",
                          DoubleValue(-5.0),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_eavSinrDb),
                          MakeDoubleChecker<double>())

            .AddAttribute("RequireSinr",
                          "If true, block HO until serving SINR exists in DB.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_requireSinr),
                          MakeBooleanChecker())

            .AddAttribute("LeakageModel",
                          "Leakage model: oracle | riskmap | hybrid | fixed",
                          StringValue("oracle"),
                          MakeStringAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_leakageModel),
                          MakeStringChecker())

            .AddAttribute("RiskMapFile",
                          "File: lines 'cellId riskScore' (0..1)",
                          StringValue(""),
                          MakeStringAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_riskMapFile),
                          MakeStringChecker())

            .AddAttribute("RiskMinEavSinrDb",
                          "Eve SINR (dB) used when riskScore=0.",
                          DoubleValue(-15.0),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_riskMinEavSinrDb),
                          MakeDoubleChecker<double>())

            .AddAttribute("RiskMaxEavSinrDb",
                          "Eve SINR (dB) used when riskScore=1.",
                          DoubleValue(5.0),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_riskMaxEavSinrDb),
                          MakeDoubleChecker<double>())

            // ---- ML trigger gate ----
            .AddAttribute("EnableMlTrigger",
                          "If true, use ONNX model to decide WHEN to attempt HO.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_enableMlTrigger),
                          MakeBooleanChecker())

            .AddAttribute("OnnxModelPathTn",
                          "ONNX model path for TN (WHEN-to-HO trigger).",
                          StringValue(""),
                          MakeStringAccessor(&OranLmNrOnnxSecrecyAwareHandover::SetOnnxModelPathTn),
                          MakeStringChecker())

            .AddAttribute("OnnxModelPathNtn",
                          "ONNX model path for NTN (WHEN-to-HO trigger).",
                          StringValue(""),
                          MakeStringAccessor(&OranLmNrOnnxSecrecyAwareHandover::SetOnnxModelPathNtn),
                          MakeStringChecker())

            .AddAttribute("TnCellIdMax",
                          "Cells with cellId <= TnCellIdMax are treated as TN for model selection.",
                          UintegerValue(10),
                          MakeUintegerAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_tnCellIdMax),
                          MakeUintegerChecker<uint16_t>())

            .AddAttribute("HoProbThr",
                          "Trigger HO logic only if P(HO_next) > HoProbThr (unless outage).",
                          DoubleValue(0.20),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_hoProbThr),
                          MakeDoubleChecker<double>())

            .AddAttribute("MlSecrecyTarget",
                          "Used to compute outage feature for ML vector (must match training).",
                          DoubleValue(0.10),
                          MakeDoubleAccessor(&OranLmNrOnnxSecrecyAwareHandover::m_mlSecrecyTarget),
                          MakeDoubleChecker<double>());

    return tid;
}

OranLmNrOnnxSecrecyAwareHandover::OranLmNrOnnxSecrecyAwareHandover(void)
    : OranLm(),
      m_hysteresisDb(2.0),
      m_rsrpThresholdDb(2.0),
      m_warmup(Seconds(2.0)),
      m_minHoInterval(Seconds(2.0)),
      m_hoAttemptTimeout(Seconds(2.0)),
      m_secrecyRateThr(0.10),
      m_eavSinrDb(-5.0),
      m_requireSinr(true),
      m_riskMapLoaded(false),
      m_riskMinEavSinrDb(-15.0),
      m_riskMaxEavSinrDb(5.0),
      m_enableMlTrigger(false),
      m_hoProbThr(0.60),
      m_mlSecrecyTarget(0.10),
      m_tnCellIdMax(10),
      m_env(ORT_LOGGING_LEVEL_WARNING, "OranLmNrOnnxSecrecyAwareHandover"),
      m_memoryInfo(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      m_sessionTn(nullptr),
      m_sessionNtn(nullptr)
{
    NS_LOG_FUNCTION(this);
    m_name = "OranLmNrOnnxSecrecyAwareHandover";
}

OranLmNrOnnxSecrecyAwareHandover::~OranLmNrOnnxSecrecyAwareHandover(void)
{
    NS_LOG_FUNCTION(this);
}

void
OranLmNrOnnxSecrecyAwareHandover::SetOnnxModelPathTn(const std::string& path)
{
    if (path.empty())
    {
        m_sessionTn.reset();
        return;
    }

    std::ifstream f(path.c_str());
    NS_ABORT_MSG_IF(!f.good(), "TN ONNX model file not found: " << path);
    f.close();

    Ort::SessionOptions opts;
    m_sessionTn = std::make_unique<Ort::Session>(m_env, path.c_str(), opts);
    NS_LOG_UNCOND("Loaded TN ONNX model: " << path);

    for (size_t i = 0; i < m_sessionTn->GetOutputCount(); ++i)
    {
        auto outName = m_sessionTn->GetOutputNameAllocated(i, m_allocator);
        NS_LOG_UNCOND("TN ONNX output[" << i << "] = " << outName.get());
    }
}

void
OranLmNrOnnxSecrecyAwareHandover::SetOnnxModelPathNtn(const std::string& path)
{
    if (path.empty())
    {
        m_sessionNtn.reset();
        return;
    }

    std::ifstream f(path.c_str());
    NS_ABORT_MSG_IF(!f.good(), "NTN ONNX model file not found: " << path);
    f.close();

    Ort::SessionOptions opts;
    m_sessionNtn = std::make_unique<Ort::Session>(m_env, path.c_str(), opts);
    NS_LOG_UNCOND("Loaded NTN ONNX model: " << path);

    for (size_t i = 0; i < m_sessionNtn->GetOutputCount(); ++i)
    {
        auto outName = m_sessionNtn->GetOutputNameAllocated(i, m_allocator);
        NS_LOG_UNCOND("NTN ONNX output[" << i << "] = " << outName.get());
    }

}

Ort::Session*
OranLmNrOnnxSecrecyAwareHandover::PickSessionForCell(uint16_t servingCellId) const
{
    // Simple domain split: cellId <= m_tnCellIdMax => TN; else NTN
    if (servingCellId != 0 && servingCellId <= m_tnCellIdMax)
    {
        return m_sessionTn.get();
    }
    return m_sessionNtn.get();
}

float
OranLmNrOnnxSecrecyAwareHandover::LinToDbForMl(double xLin)
{
    if (xLin <= 0.0) return -100.0f;
    return static_cast<float>(10.0 * std::log10(xLin));
}

static inline void Softmax2(float a, float b, float& pa, float& pb)
{
    float m = std::max(a, b);
    float ea = std::exp(a - m);
    float eb = std::exp(b - m);
    float s = ea + eb;
    pa = ea / s;
    pb = eb / s;
}

bool
OranLmNrOnnxSecrecyAwareHandover::InferHoProbability(Ort::Session& sess,
                                                     const std::array<float, 9>& x,
                                                     float& pHo) const
{
    pHo = 0.0f;

    std::array<int64_t, 2> shape{1, 9};

    auto inputTensor = Ort::Value::CreateTensor<float>(
        m_memoryInfo,
        const_cast<float*>(x.data()),
        x.size(),
        shape.data(),
        shape.size());

    auto inputName = sess.GetInputNameAllocated(0, m_allocator);
    std::array<const char*, 1> inputNames{inputName.get()};

    // IMPORTANT: this output name may change after re-export (often "probabilities")
    static const char* kProbOutName = "probabilities";
    std::array<const char*, 1> outputNames{kProbOutName};

    auto outs = sess.Run(Ort::RunOptions{},
                         inputNames.data(), &inputTensor, 1,
                         outputNames.data(), outputNames.size());

    NS_ABORT_MSG_IF(outs.size() != 1, "ONNX did not return expected output");

    Ort::Value& v = outs[0];
    NS_ABORT_MSG_IF(!v.IsTensor(), "Requested output is not a tensor");

    auto info = v.GetTensorTypeAndShapeInfo();
    size_t n = info.GetElementCount();
    const float* d = v.GetTensorData<float>();

    if (n == 1)
    {
        pHo = d[0];
    }
    else if (n == 2)
    {
        // binary probs: [P0, P1]
        pHo = d[1];
    }
    else
    {
        NS_ABORT_MSG("Unexpected probability tensor size: " << n);
    }

    // Clamp
    if (pHo < 0.0f) pHo = 0.0f;
    if (pHo > 1.0f) pHo = 1.0f;

    return true;
}

// -------------------- Existing parts (secrecy HO logic) --------------------

std::vector<Ptr<OranCommand>>
OranLmNrOnnxSecrecyAwareHandover::Run(void)
{
    std::vector<Ptr<OranCommand>> commands;

    if (!m_active)
    {
        return commands;
    }

    NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                    "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");

    Ptr<OranDataRepository> data = m_nearRtRic->Data();
    LoadRiskMapIfNeeded();

    auto ueInfos = GetUeInfos(data);
    auto gnbInfos = GetGnbInfos(data);

    return GetHandoverCommands(data, ueInfos, gnbInfos);
}

std::vector<OranLmNrOnnxSecrecyAwareHandover::UeInfo>
OranLmNrOnnxSecrecyAwareHandover::GetUeInfos(Ptr<OranDataRepository> data) const
{
    std::vector<UeInfo> ueInfos;
    for (auto ueId : data->GetNrUeE2NodeIds())
    {
        UeInfo ueInfo;
        ueInfo.nodeId = ueId;

        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetNrUeCellInfo(ueInfo.nodeId);
        if (!found)
        {
            continue;
        }

        auto nodePositions = data->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());
        if (nodePositions.empty())
        {
            continue;
        }

        ueInfo.position = nodePositions.rbegin()->second;
        ueInfos.push_back(ueInfo);
    }
    return ueInfos;
}

std::vector<OranLmNrOnnxSecrecyAwareHandover::GnbInfo>
OranLmNrOnnxSecrecyAwareHandover::GetGnbInfos(Ptr<OranDataRepository> data) const
{
    std::vector<GnbInfo> gnbInfos;
    for (auto gnbId : data->GetNrGnbE2NodeIds())
    {
        GnbInfo gnbInfo;
        gnbInfo.nodeId = gnbId;

        bool found;
        std::tie(found, gnbInfo.cellId) = data->GetNrGnbCellInfo(gnbInfo.nodeId);
        if (!found)
        {
            continue;
        }

        auto nodePositions = data->GetNodePositions(gnbInfo.nodeId, Seconds(0), Simulator::Now());
        if (nodePositions.empty())
        {
            continue;
        }

        gnbInfo.position = nodePositions.rbegin()->second;
        gnbInfos.push_back(gnbInfo);
    }
    return gnbInfos;
}

void
OranLmNrOnnxSecrecyAwareHandover::LoadRiskMapIfNeeded()
{
    if (m_riskMapLoaded) return;
    m_riskMapLoaded = true;
    m_cellRisk.clear();

    if (m_riskMapFile.empty()) return;

    std::ifstream in(m_riskMapFile.c_str());
    if (!in.is_open())
    {
        NS_LOG_UNCOND("RiskMapFile cannot be opened: " << m_riskMapFile
                      << " (riskmap will assume worst for unknown cells)");
        return;
    }

    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream iss(line);

        uint16_t cellId;
        double risk;
        if (!(iss >> cellId >> risk)) continue;

        risk = std::max(0.0, std::min(1.0, risk));
        m_cellRisk[cellId] = risk;
    }
}

double
OranLmNrOnnxSecrecyAwareHandover::DbToLin(double db)
{
    return std::pow(10.0, db / 10.0);
}

double
OranLmNrOnnxSecrecyAwareHandover::SecrecyCapacity(double gbLin, double geLin)
{
    gbLin = std::max(0.0, gbLin);
    geLin = std::max(0.0, geLin);

    const double cb = std::log2(1.0 + gbLin);
    const double ce = std::log2(1.0 + geLin);
    return std::max(0.0, cb - ce);
}

bool
OranLmNrOnnxSecrecyAwareHandover::GetServingDlDataSinrLin(Ptr<OranDataRepository> data,
                                                          uint64_t ueE2NodeId,
                                                          uint16_t servingCellId,
                                                          uint16_t servingRnti,
                                                          double& outSinrLin) const
{
    outSinrLin = -1.0;
    auto sinrRecs = data->GetNrUeSinr(ueE2NodeId);

    for (const auto& rec : sinrRecs)
    {
        uint16_t rnti, cellId, bwpId;
        double sinrLin, sinrDb;
        bool isCtrl;
        std::tie(rnti, cellId, bwpId, sinrLin, sinrDb, isCtrl) = rec;

        if (!isCtrl && bwpId == 0 && cellId == servingCellId && rnti == servingRnti)
        {
            outSinrLin = sinrLin;
            return (outSinrLin > 0.0);
        }
    }
    return false;
}

bool
OranLmNrOnnxSecrecyAwareHandover::GetWorstEveSinrLinForCell(Ptr<OranDataRepository> data,
                                                            uint16_t cellId,
                                                            double& outGeLin) const
{
    outGeLin = 0.0;
    bool foundData = false;
    bool foundCtrl = false;

    double bestDataLin = 0.0;
    double bestCtrlLin = 0.0;

    for (auto eveId : data->GetNrEveNodeIds())
    {
        auto recs = data->GetNrEveSinr(eveId);

        bool localFoundData = false;
        bool localFoundCtrl = false;
        double localDataLin = 0.0;
        double localCtrlLin = 0.0;

        for (const auto& rec : recs)
        {
            uint16_t recCellId, bwpId;
            double sinrLin, sinrDb;
            bool isCtrl;
            std::tie(recCellId, bwpId, sinrLin, sinrDb, isCtrl) = rec;

            if (bwpId != 0 || recCellId != cellId) continue;

            if (!isCtrl)
            {
                localFoundData = true;
                localDataLin = sinrLin;
                break; // prefer data
            }
            else if (!localFoundCtrl)
            {
                localFoundCtrl = true;
                localCtrlLin = sinrLin;
            }
        }

        if (localFoundData)
        {
            bestDataLin = std::max(bestDataLin, localDataLin);
            foundData = true;
        }
        else if (localFoundCtrl)
        {
            bestCtrlLin = std::max(bestCtrlLin, localCtrlLin);
            foundCtrl = true;
        }
    }

    if (foundData)
    {
        outGeLin = bestDataLin;
        return true;
    }
    if (foundCtrl)
    {
        outGeLin = bestCtrlLin;
        return true;
    }
    return false;
}

bool
OranLmNrOnnxSecrecyAwareHandover::GetLeakageSinrLinForCell(Ptr<OranDataRepository> data,
                                                           uint16_t cellId,
                                                           double& geLin) const
{
    if (m_leakageModel == "fixed")
    {
        geLin = DbToLin(m_eavSinrDb);
        return true;
    }

    auto riskToGe = [&](uint16_t cId, double& outLin) -> bool {
        double risk = 1.0; // default worst
        auto it = m_cellRisk.find(cId);
        if (it != m_cellRisk.end()) risk = it->second;

        const double geDb =
            m_riskMinEavSinrDb + risk * (m_riskMaxEavSinrDb - m_riskMinEavSinrDb);
        outLin = DbToLin(geDb);
        return true;
    };

    if (m_leakageModel == "riskmap")
    {
        return riskToGe(cellId, geLin);
    }

    if (m_leakageModel == "oracle")
    {
        bool ok = GetWorstEveSinrLinForCell(data, cellId, geLin);
        if (!ok)
        {
            geLin = DbToLin(m_eavSinrDb);
            return false;
        }
        return true;
    }

    if (m_leakageModel == "hybrid")
    {
        double geFixed = DbToLin(m_eavSinrDb);
        double geRisk = 0.0;
        riskToGe(cellId, geRisk);

        double geOracle = 0.0;
        bool haveOracle = GetWorstEveSinrLinForCell(data, cellId, geOracle);

        geLin = std::max(geFixed, geRisk);
        if (haveOracle) geLin = std::max(geLin, geOracle);

        return haveOracle;
    }

    geLin = DbToLin(m_eavSinrDb);
    return true;
}

std::vector<Ptr<OranCommand>>
OranLmNrOnnxSecrecyAwareHandover::GetHandoverCommands(Ptr<OranDataRepository> data,
                                                      const std::vector<UeInfo>& ueInfos,
                                                      const std::vector<GnbInfo>& gnbInfos) const
{
    static std::map<uint64_t, Time> lastHoCmdTime;
    static std::map<uint64_t, uint16_t> pendingHoTarget;

    std::vector<Ptr<OranCommand>> commands;

    if (Simulator::Now() < m_warmup)
    {
        return commands;
    }

    const double kEps = 1e-6;
    const double kOutageMinRsrpGainDb = 0.1;

    for (const auto& ueInfo : ueInfos)
    {
        auto rsrpMeasurements = data->GetNrUeRsrpRsrq(ueInfo.nodeId);

        double servingRsrp = -DBL_MAX;
        uint16_t servingCellId = 0;
        uint16_t servingRnti = 0;

        struct Cand { uint16_t cellId; double rsrp; };
        std::vector<Cand> cands;

        for (const auto& meas : rsrpMeasurements)
        {
            uint16_t rnti;
            uint16_t cellId;
            double rsrp;
            double rsrq;
            bool isServingCell;
            uint8_t componentCarrierId;

            std::tie(rnti, cellId, rsrp, rsrq, isServingCell, componentCarrierId) = meas;

            if (componentCarrierId != 0) continue;

            cands.push_back({cellId, rsrp});

            if (isServingCell)
            {
                servingRsrp = rsrp;
                servingCellId = cellId;
                servingRnti = rnti;
            }
        }

        if (servingCellId == 0 || servingRnti == 0) continue;

        const uint16_t currentCellId = servingCellId;
        const uint16_t currentRnti = servingRnti;

        // Clear pending if reached target
        auto pit = pendingHoTarget.find(ueInfo.nodeId);
        if (pit != pendingHoTarget.end() && pit->second == currentCellId)
        {
            pendingHoTarget.erase(pit);
        }

        // Pending HO => wait timeout
        auto tit = lastHoCmdTime.find(ueInfo.nodeId);
        if (pendingHoTarget.count(ueInfo.nodeId) && tit != lastHoCmdTime.end())
        {
            if (Simulator::Now() - tit->second < m_hoAttemptTimeout)
            {
                continue;
            }
        }

        // Map serving cell -> serving gNB E2 node id
        uint64_t oldCellNodeId = 0;
        for (const auto& g : gnbInfos)
        {
            if (g.cellId == currentCellId)
            {
                oldCellNodeId = g.nodeId;
                break;
            }
        }
        if (oldCellNodeId == 0) continue;

        // Get serving SINR
        double servingSinrLin = -1.0;
        const bool hasSinr =
            GetServingDlDataSinrLin(data, ueInfo.nodeId, currentCellId, currentRnti, servingSinrLin);

        if (!hasSinr && m_requireSinr) continue;
        if (!hasSinr) servingSinrLin = DbToLin(-10.0);

        // Eve SINR (leakage) for current cell
        double currentGeLin = 0.0;
        bool haveCurrentEve = GetLeakageSinrLinForCell(data, currentCellId, currentGeLin);
        if (!haveCurrentEve) currentGeLin = DbToLin(m_eavSinrDb);

        const double currentCs = SecrecyCapacity(servingSinrLin, currentGeLin);

        // Threshold for secrecy gating
        const double thr = (m_secrecyRateThr > 0.0) ? m_secrecyRateThr : m_mlSecrecyTarget;
        const bool currentOutage = (currentCs < thr);

        // -------- ML trigger gate (WHEN) --------
        if (m_enableMlTrigger && !currentOutage)
        {
            Ort::Session* sess = PickSessionForCell(currentCellId);

            if (sess != nullptr)
            {
                float ueSinrDb  = LinToDbForMl(servingSinrLin);
                float eveSinrDb = LinToDbForMl(currentGeLin);
                float secrecy   = static_cast<float>(currentCs);
                float outage    = (secrecy < static_cast<float>(m_mlSecrecyTarget)) ? 1.0f : 0.0f;

                auto& ps = m_prev[ueInfo.nodeId];

                if (ps.hasPrev)
                {
                    float dUeSinrDb = ueSinrDb - ps.ueSinrDb_prev;
                    float dSecrecy  = secrecy  - ps.secrecy_prev;

                    std::array<float, 9> x = {
                        ueSinrDb,
                        eveSinrDb,
                        secrecy,
                        outage,
                        ps.ueSinrDb_prev,
                        ps.secrecy_prev,
                        ps.outage_prev,
                        dUeSinrDb,
                        dSecrecy
                    };

                    float pHo = 0.0f;
                    bool ok = InferHoProbability(*sess, x, pHo);

                    // update prev
                    ps.ueSinrDb_prev = ueSinrDb;
                    ps.secrecy_prev  = secrecy;
                    ps.outage_prev   = outage;

                    if (ok && !(pHo > static_cast<float>(m_hoProbThr)))
                    {
                        NS_LOG_INFO("ML gate blocks HO attempt UE=" << ueInfo.nodeId
                                    << " cell=" << currentCellId
                                    << " pHo=" << pHo << " thr=" << m_hoProbThr
                                    << " Cs=" << currentCs);
                        continue;
                    }
                }
                else
                {
                    ps.hasPrev = true;
                    ps.ueSinrDb_prev = ueSinrDb;
                    ps.secrecy_prev  = secrecy;
                    ps.outage_prev   = outage;
                }
            }
            // If model not loaded, we simply fall back to normal behavior (no freeze).
        }

        // Cooldown only if not in outage
        if (!currentOutage)
        {
            if (tit != lastHoCmdTime.end() && (Simulator::Now() - tit->second) < m_minHoInterval)
            {
                continue;
            }
        }

        // Sort candidates by RSRP desc
        std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
            return a.rsrp > b.rsrp;
        });

        // Default: stay
        uint16_t chosenCellId = currentCellId;
        double chosenRsrp = servingRsrp;
        double chosenCs = currentCs;
        double chosenGbLin = servingSinrLin;
        double chosenGeLin = currentGeLin;
        bool chosenHaveEve = haveCurrentEve;

        if (!currentOutage)
        {
            // NORMAL MODE
            for (const auto& c : cands)
            {
                if (c.cellId == currentCellId) continue;

                const double gainDb = c.rsrp - servingRsrp;
                if (!(gainDb > std::max(m_hysteresisDb, m_rsrpThresholdDb))) continue;

                const double gbLin = servingSinrLin * DbToLin(gainDb);

                double geLinCand = 0.0;
                bool haveEve = GetLeakageSinrLinForCell(data, c.cellId, geLinCand);

                if (!haveEve)
                {
                    geLinCand = std::max(currentGeLin, DbToLin(m_eavSinrDb));
                }

                const double cs = SecrecyCapacity(gbLin, geLinCand);
                if (cs < thr) continue;

                chosenCellId = c.cellId;
                chosenRsrp = c.rsrp;
                chosenGbLin = gbLin;
                chosenGeLin = geLinCand;
                chosenHaveEve = haveEve;
                chosenCs = cs;
                break;
            }
        }
        else
        {
            // OUTAGE MODE: maximize Cs, relax RSRP
            uint16_t bestCellId = currentCellId;
            double bestRsrp = servingRsrp;

            double bestCs = currentCs;
            double bestGbLin = servingSinrLin;
            double bestGeLin = currentGeLin;
            bool bestHaveEve = haveCurrentEve;

            for (const auto& c : cands)
            {
                if (c.cellId == currentCellId) continue;

                const double gainDb = c.rsrp - servingRsrp;
                const double gbLin = servingSinrLin * DbToLin(gainDb);

                double geLinCand = 0.0;
                bool haveEve = GetWorstEveSinrLinForCell(data, c.cellId, geLinCand);

                if (!haveEve)
                {
                    geLinCand = std::max(currentGeLin, DbToLin(m_eavSinrDb));
                }

                const double cs = SecrecyCapacity(gbLin, geLinCand);

                const bool betterCs = (cs > bestCs + kEps);
                const bool tieBetterRsrp =
                    (std::fabs(cs - bestCs) <= kEps) && (c.rsrp > bestRsrp + kEps);

                if (betterCs || tieBetterRsrp)
                {
                    bestCellId = c.cellId;
                    bestRsrp = c.rsrp;
                    bestCs = cs;
                    bestGbLin = gbLin;
                    bestGeLin = geLinCand;
                    bestHaveEve = haveEve;
                }
            }

            const bool secrecyImproves = (bestCs > currentCs + kEps);
            const bool rsrpImproves = (bestRsrp > servingRsrp + kOutageMinRsrpGainDb);

            if (bestCellId != currentCellId && (secrecyImproves || rsrpImproves))
            {
                chosenCellId = bestCellId;
                chosenRsrp = bestRsrp;
                chosenCs = bestCs;
                chosenGbLin = bestGbLin;
                chosenGeLin = bestGeLin;
                chosenHaveEve = bestHaveEve;
            }
        }

        if (chosenCellId == currentCellId) continue;

        // Map chosen cell -> target gNB E2 node id
        uint64_t targetCellNodeId = 0;
        for (const auto& g : gnbInfos)
        {
            if (g.cellId == chosenCellId)
            {
                targetCellNodeId = g.nodeId;
                break;
            }
        }
        NS_ABORT_MSG_IF(targetCellNodeId == 0,
                        "Chosen target cellId has no gNB mapping: cellId=" << chosenCellId);

        Ptr<OranCommandNr2NrHandover> cmd = CreateObject<OranCommandNr2NrHandover>();

        cmd->SetAttribute("TargetRnti", UintegerValue(currentRnti));
        cmd->SetAttribute("TargetCellId", UintegerValue(chosenCellId));

        // Compatible attribute setting across ns-oran variants
        SetUintegerAttrIfExists(cmd, "SourceE2NodeId", oldCellNodeId);
        SetUintegerAttrIfExists(cmd, "ServingE2NodeId", oldCellNodeId);

        SetUintegerAttrIfExists(cmd, "TargetGnbE2NodeId", targetCellNodeId);
        SetUintegerAttrIfExists(cmd, "DestinationE2NodeId", targetCellNodeId);

        // Compatibility: keep executor/source behavior (as in your previous code)
        cmd->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));

        NS_LOG_UNCOND("LM HO (Secrecy+ONNX) UE=" << ueInfo.nodeId
                      << " rnti=" << currentRnti
                      << " " << currentCellId << "->" << chosenCellId
                      << " Cs=" << chosenCs
                      << " thr=" << thr
                      << " outageMode=" << (currentOutage ? 1 : 0)
                      << " enableML=" << (m_enableMlTrigger ? 1 : 0)
                      << " haveEve=" << (chosenHaveEve ? 1 : 0)
                      << " gbDb=" << LinToDbSafe(chosenGbLin)
                      << " geDb=" << LinToDbSafe(chosenGeLin)
                      << " rsrpFrom=" << servingRsrp
                      << " rsrpTo=" << chosenRsrp);

        data->LogCommandLm(m_name, cmd);
        commands.push_back(cmd);

        lastHoCmdTime[ueInfo.nodeId] = Simulator::Now();
        pendingHoTarget[ueInfo.nodeId] = chosenCellId;
    }

    return commands;
}

} // namespace ns3
