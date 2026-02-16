#ifndef ORAN_LM_NR_ONNX_SECRECY_AWARE_HANDOVER_H
#define ORAN_LM_NR_ONNX_SECRECY_AWARE_HANDOVER_H

#include "oran-data-repository.h"
#include "oran-lm.h"

#include "ns3/nstime.h"
#include "ns3/type-id.h"
#include "ns3/vector.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// ONNX Runtime (ns-O-RAN already uses it in the ONNX LM example)
#include <onnxruntime_cxx_api.h>

namespace ns3
{

class OranLmNrOnnxSecrecyAwareHandover : public OranLm
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

    // Previous state per UE for your 9-feature vector
    struct PrevState
    {
        bool hasPrev = false;
        float ueSinrDb_prev = -100.0f;
        float secrecy_prev  = 0.0f;
        float outage_prev   = 0.0f;
    };

  public:
    static TypeId GetTypeId(void);

    OranLmNrOnnxSecrecyAwareHandover(void);
    ~OranLmNrOnnxSecrecyAwareHandover(void) override;

    std::vector<Ptr<OranCommand>> Run(void) override;

  private:
    // Data extraction
    std::vector<UeInfo> GetUeInfos(Ptr<OranDataRepository> data) const;
    std::vector<GnbInfo> GetGnbInfos(Ptr<OranDataRepository> data) const;

    // Main HO logic
    std::vector<Ptr<OranCommand>> GetHandoverCommands(Ptr<OranDataRepository> data,
                                                      const std::vector<UeInfo>& ueInfos,
                                                      const std::vector<GnbInfo>& gnbInfos) const;

    // SINR and Eve
    bool GetServingDlDataSinrLin(Ptr<OranDataRepository> data,
                                 uint64_t ueE2NodeId,
                                 uint16_t servingCellId,
                                 uint16_t servingRnti,
                                 double& outSinrLin) const;

    bool GetWorstEveSinrLinForCell(Ptr<OranDataRepository> data,
                                   uint16_t cellId,
                                   double& worstSinrLin) const;

    void LoadRiskMapIfNeeded();

    bool GetLeakageSinrLinForCell(Ptr<OranDataRepository> data,
                                  uint16_t cellId,
                                  double& geLin) const;

    // Secrecy helpers
    static double DbToLin(double db);
    static double SecrecyCapacity(double gbLin, double geLin);

    // ---- ONNX / ML ----
    void SetOnnxModelPathTn(const std::string& path);
    void SetOnnxModelPathNtn(const std::string& path);

    bool InferHoProbability(Ort::Session& sess,
                            const std::array<float, 9>& x,
                            float& pHo) const;

    Ort::Session* PickSessionForCell(uint16_t servingCellId) const;


    static float LinToDbForMl(double xLin);

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

    // Leakage model
    std::string m_leakageModel;
    std::string m_riskMapFile;
    std::unordered_map<uint16_t, double> m_cellRisk;
    bool m_riskMapLoaded;
    double m_riskMinEavSinrDb;
    double m_riskMaxEavSinrDb;

    // ---- ML trigger gate ----
    bool m_enableMlTrigger;
    double m_hoProbThr;
    double m_mlSecrecyTarget;

    // Domain split for TN vs NTN model selection (simple + works well in your setup)
    uint16_t m_tnCellIdMax;

    // Previous per-UE state (mutable because GetHandoverCommands is const style)
    mutable std::unordered_map<uint64_t, PrevState> m_prev;

    // ONNX runtime objects
    Ort::Env m_env;
    Ort::AllocatorWithDefaultOptions m_allocator;
    Ort::MemoryInfo m_memoryInfo;

    std::unique_ptr<Ort::Session> m_sessionTn;
    std::unique_ptr<Ort::Session> m_sessionNtn;
};

} // namespace ns3

#endif // ORAN_LM_NR_ONNX_SECRECY_AWARE_HANDOVER_H