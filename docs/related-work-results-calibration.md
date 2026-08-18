# Related-work evidence used for synthetic figure calibration

This note records the literature evidence used to design the temporary result
figures. It does **not** turn the synthetic values into measured results. Values
from these papers cannot be copied into this paper because their targets,
channels, traffic, objectives, and evaluation platforms differ from ours.

## Relevant methods and reported outcomes

| Work | What it does | Reported outcome relevant to our presentation | How it informs our figures |
|---|---|---|---|
| Lu *et al.*, [Machine Learning for Predictive Deployment of UAVs with Multiple Access](https://arxiv.org/abs/2003.02631) | Predicts cellular traffic with an LSTM, estimates UAV service regions using joint K-means/EM/GMM, and optimises UAV positions. | Up to 24% lower total transmit power than deployment without traffic prediction. | Prediction should improve a reactive method by a moderate amount, not create an implausible order-of-magnitude gain. |
| Zhang *et al.*, [Predictive Deployment of UAV Base Stations in Wireless Networks](https://arxiv.org/abs/1811.01149) | Learns uneven spatial traffic with weighted EM and deploys a UAV to an overload hotspot. | About 10% prediction error; improvements are reported in capacity, energy, and service delay over event-driven baselines. | Motivates an imperfect probability surface and explicit prediction uncertainty. |
| Fotouhi *et al.*, [Dynamic Base Station Repositioning to Improve Spectral Efficiency of Drone Small Cells](https://arxiv.org/abs/1704.01244) | Repositions a drone small cell using local user-position information. | Nearly 100% spectral-efficiency gain in its single-cell traffic model. | Supports a clear static-versus-moving gap, but this large gain is treated as an upper contextual result, not copied into our KPIs. |
| Mirzaeinia *et al.*, [Placement of UAV-Mounted Mobile Base Station through User Load-Feature K-means Clustering](https://arxiv.org/abs/2010.01236) | Adds requested traffic as a K-means feature for UAV-BS placement. | UAVs are placed closer to high-traffic users and outperform position-only placement; the abstract does not provide a transferable percentage. | Supports load-aware spatial clustering and irregular, unequal hotspot populations. |
| Zhou *et al.*, [Integrated Sensing and Communication in UAV Swarms for Cooperative Multiple Targets Tracking](https://pure.tudelft.nl/ws/portalfiles/portal/160230907/Integrated_Sensing_and_Communication_in_UAV_Swarms_for_Cooperative_Multiple_Targets_Tracking.pdf) | Uses cyber twins and distributed UAV cooperation for multi-target sensing/tracking. | Reports 65.7% communication-energy saving and about 20% sensing improvement; later results report 21.0% and 26.3% tracking-ratio gains over two baselines. | Supports showing per-run variability, sensing misses, and a moderate predictive/cooperative gain rather than perfect sensing. |
| Jiang *et al.*, [UAV-enabled ISAC: Tracking Design and Optimization](https://arxiv.org/abs/2401.03726) | Uses EKF tracking and trajectory optimisation to minimise position/velocity PCRBs under a communication constraint. | Shows trajectory-dependent sensing/communication trade-offs and improvement over a right-above baseline. | Supports reporting localisation error jointly with the movement/control response; its PCRB is not used as our RMSE. |
| Meng *et al.*, [UAV Trajectory and Beamforming Optimization for Integrated Periodic Sensing and Communication](https://arxiv.org/abs/2203.10223) | Jointly optimises trajectory, precoding, and sensing instants. | The proposed design is close to its upper bound, and achievable rate changes with sensing frequency, sensing gain, distance, and maximum speed. | Supports sensitivity studies and warns against presenting one smooth operating point as general. |
| Saur *et al.*, [Reliable UAV Detection with ISAC](https://arxiv.org/abs/2605.23561) | Demonstrates monostatic OFDM sensing using commercial 5G hardware. | Reports reliable sub-metre UAV detection/localisation beyond 500 m in clutter. | Confirms that hundreds-of-metres sensing is physically plausible, but its UAV target and hardware do not calibrate a 1 m² ground-UE model directly. |

## Calibration decisions for the temporary figures

- End-to-end throughput is capped by the configured 0.2 Mbit/s downlink
  offered load per monitored UE.
- Ten synthetic seed outcomes are shown as individual points plus mean 95%
  confidence intervals, instead of perfectly smooth deterministic curves.
- The oracle-reactive method remains the best reactive localisation benchmark.
  RF prediction provides a moderate improvement over ISAC-reactive control but
  does not automatically dominate the oracle in every metric.
- The spatial example uses 60 central UEs and 30 outer UEs in three unequal,
  anisotropic clusters inside the configured 2 km by 2 km area.
- RF scores use an imbalanced positive class and deteriorate with the 2, 5, and
  10 s horizons. Precision--recall curves are included because accuracy is
  misleading for sparse hotspot labels.
- The sensing curve is sampled from the paper's radar, logistic detection, and
  localisation-error equations. A temporary **30 dB effective aggregate loss**
  is used to create a measurable transition across the compact area. With the
  current configured loss `L_s=1`, the model predicts essentially 100%
  detection and the minimum localisation deviation over most of the area. The
  loss/effective sensing power therefore requires a calibration sweep before
  final simulation claims.

## What must be replaced

Every CSV row is marked
`SYNTHETIC_LITERATURE_INFORMED_NOT_NS3_RESULTS`. Replace the method, sensing,
RF, dynamics, spatial, and confidence-interval values using aligned ns-3 and
offline-RF traces before submission. Related-work results should be discussed
as context only, never merged with the proposed method's results.
