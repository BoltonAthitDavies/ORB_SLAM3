# ORB-SLAM3 — formulation, configuration, and what a CARLA port would take

> **Scope and status.** This directory is **vanilla upstream ORB-SLAM3 V1.0** (commit `4452a3c`, 22 Dec 2021)
> — no CARLA integration, no ROS 2 node, nothing built (`Vocabulary/ORBvoc.txt` is still a `.tar.gz`).
> This document is written in the same shape as **§4 SLAM (VINS-Fusion)** of the main
> [`README.md`](../vins_fusion_ros2/README.md): formulation → configuration → experiment setup →
> results → analysis. The difference in provenance matters and is marked throughout:
>
> | Section | Source |
> |---------|--------|
> | §1 formulation | [`ORBSLAM3.pdf`](ORBSLAM3.pdf) + this source tree |
> | §2 configuration | [`Calibration_Tutorial.pdf`](Calibration_Tutorial.pdf) + `Examples/*/*.yaml`; the CARLA column is **derived here, not measured** |
> | §4 results | **published numbers from `ORBSLAM3.pdf`** — *not* our runs. No ORB-SLAM3 experiment has been run in this project. |
> | §5 analysis | paper + the CARLA findings already established in the main README §4.6 |

---

## Table of contents

1. [Formulation — the equations](#1-formulation--the-equations)
2. [Configuration — the calibration file](#2-configuration--the-calibration-file)
3. [Sensor configurations & experiment setup](#3-sensor-configurations--experiment-setup)
4. [Results (from the paper)](#4-results-from-the-paper)
5. [Analysis — key findings](#5-analysis--key-findings)
6. [Commands](#6-commands)
7. [Gaps for this project](#7-gaps-for-this-project)
8. [Appendix — code map](#appendix--code-map)

---

## 1. Formulation — the equations

ORB-SLAM3 is a **feature-based, tightly-coupled, keyframe-and-BA** SLAM system. Like VINS-Fusion it is
an optimization (MAP) estimator, not a filter — but where VINS-Fusion keeps a **fixed sliding window of
10 keyframes and marginalizes everything older into a prior**, ORB-SLAM3 keeps a **persistent map** and
revisits arbitrarily old keyframes through place recognition. That single difference drives everything
below.

**Three (four) kinds of data association.** The paper's organizing idea (§I) is that accuracy comes from
how far back in time you can still match:

| Association | Span | Mechanism | VINS-Fusion equivalent |
|-------------|------|-----------|------------------------|
| **short-term** | last few frames | KLT-style tracking of map points | KLT optical flow — yes |
| **mid-term** | within accumulated-drift range | reproject *map* points into the frame, match ORB descriptors | **none** — points leaving the window are gone |
| **long-term** | any past time | DBoW2 place recognition → loop closure / relocalization | none in the estimator (`global_fusion` GPS is a different mechanism) |
| **multi-map** | across maps/sessions | Atlas + place recognition → **map merging** | none |

Mid-term association is the one worth dwelling on: it is what lets ORB-SLAM3 reach **zero drift when
re-entering a mapped area**, and it is structurally unavailable to a sliding-window estimator. The paper
attributes most of its accuracy edge over VINS-Fusion to this, not to the IMU handling.

### 1.1 Pipeline — three threads over one Atlas

```
        Frame ──▶ Extract ORB ─┐
                               ├──▶ initial pose (last frame / relocalization) ──▶ track local map ──▶ new-KF decision
        IMU   ──▶ integration ─┘                                                                            │
                                                                                                        KeyFrame
   ┌────────────────────── ATLAS ──────────────────────┐                                                    │
   │  active map  │  non-active maps  │  DBoW2 KF DB   │                                        ┌───────────▼───────────┐
   │  MapPoints, KeyFrames, covisibility, spanning tree│                                        │  LOCAL MAPPING        │
   └───────────────────────────────────────────────────┘                                        │  KF insert, MP cull,  │
                          ▲                                                                     │  new points, LOCAL BA,│
                          │                                                                     │  IMU init, KF cull,   │
   ┌──────────────────────┴──────────────────────────────────────┐                              │  IMU scale refinement │
   │  LOOP & MAP MERGING                                          │◀─────────────────────────────┘
   │  DB query → Sim3/SE3 → { loop: fuse + essential graph + full BA }
   │                        { different map: merge maps, welding BA, essential graph }
   └──────────────────────────────────────────────────────────────┘
```

The paper draws three threads. **The code spawns only two** — `Tracking` runs in the *caller's* thread
(`System.h:234` says so explicitly), so `TrackStereo()` blocks until that frame is tracked:

| Component | Thread | Rate | Spawn |
|-----------|--------|------|-------|
| `Tracking` | **caller's** — not spawned | frame | `System::TrackStereo` `System.cc:244` → `Tracking::GrabImageStereo` → `Track()` |
| `LocalMapping::Run` | own | keyframe | `System.cc:197` |
| `LoopClosing::Run` | own | keyframe | `System.cc:214` |
| `RunGlobalBundleAdjustment` | **transient**, per correction | on loop/merge | `LoopClosing.cc:1163` / `:1769` |
| `Viewer::Run` | own, optional | — | `System.cc:233` |

That matters for a closed-loop CARLA port: tracking cost lands directly in your control cycle, while
mapping and loop closure are genuinely asynchronous.

Data moves through three queues, each with its own mutex:

| Edge | Queue | Enqueue |
|------|-------|---------|
| caller → Tracking | `mlQueueImuData` | `Tracking::GrabImuData` `Tracking.cc:1618` |
| Tracking → LocalMapping | `mlNewKeyFrames` | `LocalMapping::InsertKeyFrame` `LocalMapping.cc:284` (also sets `mbAbortBA`) |
| LocalMapping → LoopClosing | `mlpLoopKeyFrameQueue` | `LoopClosing::InsertKeyFrame` `LoopClosing.cc:311` |

The big lock is **`Map::mMutexMapUpdate`** — Tracking holds it for the whole of `Track()`
(`Tracking.cc:1886`), and the other threads take it before mutating map geometry. Compare VINS-Fusion,
which runs a single `processMeasurements()` worker under `processingMutex` and has no loop-closure
thread at all in this project's build.

Sensor mode is one enum, `System.h:87`:
`MONOCULAR=0, STEREO=1, RGBD=2, IMU_MONOCULAR=3, IMU_STEREO=4, IMU_RGBD=5`.

### 1.2 State

Per-keyframe state is the same five blocks as VINS-Fusion — pose, velocity, and two biases — but
expressed as an SE(3) element rather than separate position/quaternion (paper eq. 1):

$$
\mathcal{S}_i \doteq \{\mathbf{T}_i,\ \mathbf{v}_i,\ \mathbf{b}^g_i,\ \mathbf{b}^a_i\}, \qquad
\mathbf{T}_i = [\mathbf{R}_i,\ \mathbf{p}_i] \in SE(3)
$$

Map points are **3-D points $\mathbf{x}_j$ in world coordinates**, not inverse depths anchored to a host
frame as in VINS-Fusion. This is a direct consequence of the persistent map: a point must outlive the
keyframe that first saw it.

### 1.3 Cost

Visual-inertial MAP estimation over $k{+}1$ keyframes and $l$ points (paper eq. 4):

$$
\min_{\bar{\mathcal{S}}_k,\ \mathcal{X}} \left(
\sum_{i=1}^{k} \left\| \mathbf{r}_{\mathcal{I}_{i-1,i}} \right\|^2_{\mathbf{\Sigma}^{-1}_{\mathcal{I}_{i-1,i}}}
+ \sum_{j=0}^{l-1} \sum_{i \in \mathcal{K}^j} \rho_{\text{Hub}}\!\left( \left\| \mathbf{r}_{ij} \right\|_{\mathbf{\Sigma}^{-1}_{ij}} \right)
\right)
$$

Note what is **absent** relative to VINS-Fusion's cost: there is **no marginalization-prior term**
$\|r_p - H_p\mathcal{X}\|^2$. ORB-SLAM3 fixes gauge by holding non-optimized covisible keyframes
**constant** in local BA instead of marginalizing them. The paper is explicit that this is a deliberate
trade (§VIII: "performing keyframe marginalization for local BA, instead of assuming an outer set of
static keyframes as we do" is listed as a possible improvement) — it avoids fill-in and linearization
lock-in at the cost of some information.

The Huber kernel $\rho_{\text{Hub}}$ is applied to **reprojection residuals only**; inertial residuals
get no robust kernel, because IMU mis-associations do not exist.

**Inertial residual** (paper eq. 2) — structurally the same preintegration residual as VINS-Fusion's
$r_\mathcal{B}$, written on the manifold:

$$
\mathbf{r}_{\Delta \mathbf{R}_{i,i+1}} = \mathrm{Log}\!\left( \Delta \mathbf{R}^T_{i,i+1} \mathbf{R}^T_i \mathbf{R}_{i+1} \right)
$$

$$
\mathbf{r}_{\Delta \mathbf{v}_{i,i+1}} = \mathbf{R}^T_i \left( \mathbf{v}_{i+1} - \mathbf{v}_i - \mathbf{g}\,\Delta t_{i,i+1} \right) - \Delta \mathbf{v}_{i,i+1}
$$

$$
\mathbf{r}_{\Delta \mathbf{p}_{i,i+1}} = \mathbf{R}^T_i \left( \mathbf{p}_j - \mathbf{p}_i - \mathbf{v}_i \Delta t_{i,i+1} - \tfrac{1}{2}\mathbf{g}\,\Delta t^2 \right) - \Delta \mathbf{p}_{i,i+1}
$$

**Reprojection residual** (paper eq. 3), with $\Pi$ the camera projection and $\mathbf{T}_{CB}$ the
body→camera extrinsic:

$$
\mathbf{r}_{ij} = \mathbf{u}_{ij} - \Pi\!\left( \mathbf{T}_{CB}\, \mathbf{T}^{-1}_i \oplus \mathbf{x}_j \right)
$$

### 1.4 IMU initialization — the three-stage MAP estimate

This is ORB-SLAM3's headline contribution and the part most relevant to this project, because **CARLA
driving is exactly the motion profile where it is documented to struggle** (§5). Rather than solve a
set of algebraic equations (VINS-Fusion's `LinearAlignment` / `RefineGravity` approach), ORB-SLAM3
poses initialization as MAP estimation in three steps (paper §V-B):

**Stage 1 — vision-only MAP.** Run pure monocular SLAM for ~2 s at 4 Hz keyframe rate, giving
$k = 10$ up-to-scale poses $\bar{\mathbf{T}}_{0:k}$ refined by visual-only BA.

**Stage 2 — inertial-only MAP.** Holding the vision trajectory *fixed*, solve for the inertial state:

$$
\mathcal{Y}_k = \{\, s,\ \mathbf{R}_{wg},\ \mathbf{b},\ \bar{\mathbf{v}}_{0:k} \,\}, \qquad
\mathbf{g} = \mathbf{R}_{wg}\,\mathbf{g}_I,\ \ \mathbf{g}_I = (0,0,G)^T
$$

$$
\mathcal{Y}^*_k = \arg\min_{\mathcal{Y}_k} \left( \left\| \mathbf{b} \right\|^2_{\mathbf{\Sigma}_b} + \sum_{i=1}^{k} \left\| \mathbf{r}_{\mathcal{I}_{i-1,i}} \right\|^2_{\mathbf{\Sigma}^{-1}_{\mathcal{I}_{i-1,i}}} \right)
$$

Two design points carry the accuracy claim. **Scale is an explicit optimization variable** updated
multiplicatively, $s^{\text{new}} = s^{\text{old}} \exp(\delta s)$ (eq. 10), which keeps it positive and
converges far faster than letting BA discover scale implicitly. **Gravity is parameterized on the
sphere** by two angles, $\mathbf{R}^{\text{new}}_{wg} = \mathbf{R}^{\text{old}}_{wg}\,\mathrm{Exp}(\delta\alpha_g, \delta\beta_g, 0)$
(eq. 9) — rotation about gravity is unobservable, so it is simply not parameterized. And unlike the
closed-form methods, sensor **uncertainties are respected**: the paper's stated motivation is that
"ignoring sensor uncertainties during IMU initialization produces large unpredictable errors."

**Stage 3 — joint visual-inertial MAP.** A full VI optimization with common biases across keyframes,
re-run at **5 s and 15 s** after initialization.

**In the code, all three stages are the same function** — `LocalMapping::InitializeIMU(priorG, priorA,
bFIBA)` (`LocalMapping.cc:1173`), dispatched from `LocalMapping::Run` (`:181-242`). What distinguishes
them is only the **bias prior weights**, which decay to zero:

| Stage | Trigger | Call |
|-------|---------|------|
| inertial-only MAP init | `!map->isImuInitialized()`, ≥10 KFs and ≥2.0 s (mono) / 1.0 s (stereo) | `InitializeIMU(1e2, 1e10, true)` mono, `(1e2, 1e5, true)` stereo |
| **VIBA1** | `mTinit > 5.0 s` | `InitializeIMU(1.f, 1e5, true)` |
| **VIBA2** | `mTinit > 15.0 s` | `InitializeIMU(0.f, 0.f, true)` — priors fully released |

Internally each call seeds gravity from accumulated $\Delta v$, runs
`Optimizer::InertialOptimization(map, mRwg, mScale, mbg, mba, ...)` (the inertial-only MAP estimate of
eq. 8), rejects the result if `mScale < 0.1`, applies it with `Map::ApplyScaledRotation` +
`Tracking::UpdateFrameIMU`, then runs `Optimizer::FullInertialBA(map, 100, ...)`.

**Scale refinement is monocular-only.** `LocalMapping::ScaleRefinement()` (`:1429`) is guarded by
`mbMonocular` and runs at $t \approx$ 25/35/45/55/65/75 s while the map has ≤200 keyframes. It calls the
two-parameter overload `InertialOptimization(map, mRwg, mScale)` — gravity and scale only — and applies
the result if $|s - 1| > 0.002$. **This is a correction worth flagging: the paper presents scale
refinement as the answer to poor observability under slow motion, but a stereo-inertial CARLA run would
never reach it** (see §5.2).

**Stereo-inertial** fixes $s = 1$ and drops it from the optimization variables, which the paper notes
"enhanc[es] its convergence."

**One hard safety valve:** if `mTinit < 10 s` and the map has moved less than **2 cm**, LocalMapping
sets `mbBadImu` and requests an active-map reset (`LocalMapping.cc:138-145`, seen by Tracking at
`Tracking.cc:1805`). This is the ORB-SLAM3 analogue of the stationary-start failure documented in main
README §4.3 — both systems require a moving start, and CARLA datasets here already do that.

Reported convergence: **5 % scale error in 2 s, 1 % in 15 s** — against 15 s for ORB-SLAM-VI and 20–30 s
for VI-DSO.

### 1.5 Robustness to tracking loss — the Atlas

Where a sliding-window VIO diverges or resets, ORB-SLAM3 changes maps. Visual-inertial tracking enters
`visually lost` below 15 tracked points and degrades in two stages:

- **short-term lost** — pose propagated from IMU, map points reprojected and searched in a wide window; usually recovers. After **5 s**, escalate.
- **long-term lost** — a **new active map** is initialized; the old one is kept in the Atlas as non-active and can be **merged back** later by place recognition.

One guard worth knowing: **if the system is lost within 15 s of IMU initialization, the map is
discarded**, to avoid accumulating a map built on unconverged inertial parameters.

### 1.6 Place recognition, loop closing, and map merging

DBoW2 alone runs at 100 % precision but only **30–40 % recall**, because it requires temporal
consistency across three consecutive keyframes — a delay the paper found "resulted too often in
duplicated areas or in different maps." ORB-SLAM3 replaces temporal consistency with **geometric plus
local consistency**, in six steps:

1. **DBoW2 candidates** — query the Atlas database for the 3 most similar keyframes $K_m$, excluding those covisible with the active keyframe $K_a$.
2. **Local window** — $K_m$ plus its best covisible keyframes and all their map points.
3. **3-D aligning transform** — RANSAC + Horn's algorithm on 3-D↔3-D matches. **Sim(3) for pure monocular or immature monocular-inertial; SE(3) otherwise** — i.e. once scale is observable, the 7th DOF is dropped.
4. **Guided matching refinement** — reproject through $\mathbf{T}_{am}$ both ways, refine by non-linear optimization of bidirectional reprojection error under a Huber kernel.
5. **Verification in three covisible keyframes** — the key recall win: instead of *waiting* for three consecutive keyframes, look for the corroborating evidence **already in the map**.
6. **Gravity-direction check** — in the visual-inertial case with a mature map, reject hypotheses whose pitch and roll are implausible. Gravity is observable, so a loop candidate that tilts the world is wrong.

In the code this is `LoopClosing::NewDetectCommonRegions()` (`LoopClosing.cc:324`) calling
`KeyFrameDatabase::DetectNBestCandidates(pKF, vpLoopCand, vpMergeCand, 3)` — candidates are split into
**loop** (same map) and **merge** (different map) right at the database, which is what makes the two
paths below one mechanism. Verification thresholds are literal constants at `LoopClosing.cc:581`:
20 BoW matches → 15 RANSAC inliers → 20 Sim3 inliers → 50 projected matches → **80** after the tighter
second projection. A hypothesis is confirmed only after **3 consecutive keyframes** agree
(`mnLoopNumCoincidences >= 3`); 2 consecutive misses discard it. The gravity check is quantified: an
inertial loop is rejected outright if $|\phi_x| > 0.008$ or $|\phi_y| > 0.008$ rad (`:240-260`).

> **Gate worth knowing before running this on CARLA.** `NewDetectCommonRegions()` returns immediately —
> adding the keyframe to the database but detecting nothing — when an inertial map has not reached
> **VIBA2** (`!GetIniertialBA2()`, `LoopClosing.cc:341`), when a `STEREO` map has fewer than 5 keyframes,
> or when any map has fewer than 12. So in `IMU_STEREO`, **loop closure does not exist until IMU
> initialization completes all three stages** — the two capabilities are not independent (§5.3).

**Loop closing vs map merging** is the distinction to keep straight: the geometry is nearly identical,
and the branch is decided by *which map the matched keyframe lives in*. Merge is checked **first**, and
a detected merge discards any pending loop hypothesis (`LoopClosing.cc:122`, `:208-218`).

| | Loop closing | Map merging |
|---|---|---|
| Function | `CorrectLoop()` `:969` | `MergeLocal()` `:1215` (visual) / **`MergeLocal2()` `:1783` (inertial)** |
| Match found in | the **active** map | a **different** map in the Atlas |
| Correction | fuse duplicated points, then pose-graph: `OptimizeEssentialGraph4DoF` if inertial + IMU-initialized, else 7-DoF `OptimizeEssentialGraph` | assemble a **welding window** (25 KFs visual / 11 inertial), transform one map into the other's frame, **welding BA** (`MergeInertialBA` or the multi-KF `LocalBundleAdjustment` overload), then essential graph |
| Then | **full BA** in a transient thread — only if `!isImuInitialized() \|\| (KFs < 200 && maps == 1)` | `MergeLocal`: optional GBA thread + `RemoveBadMaps()`. `MergeLocal2`: **neither** |
| Result | drift removed | the two maps become one, which becomes active |

Two details with no VINS-Fusion counterpart. **`OptimizeEssentialGraph4DoF`** (`Optimizer.cc:5292`)
corrects only $x, y, z$ and **yaw** on inertial maps — roll and pitch are observable from gravity and
must not be bent by a loop correction. And `MergeLocal2` **donates the mature map's IMU
initialization**: if the active map had not finished initializing, the merge force-sets
`SetIniertialBA1/BA2/SetImuInitialized` from the map it merged into (`LoopClosing.cc:1855-1870`).

---

## 2. Configuration — the calibration file

Where VINS-Fusion splits configuration across four *pipeline stages* (VO / VINS / global_fusion / RTAB),
ORB-SLAM3 puts everything in **one YAML**, layered by *sensor*. V1.0 introduced a new format
(`File.version: "1.0"`); the old format still parses from `Examples_old/`.

| Layer | Keys | Required for |
|-------|------|--------------|
| **(a) general** | `File.version`, `Camera.type`, `Camera.width/height`, `Camera.fps`, `Camera.RGB` | everything |
| **(b) intrinsics** | `Camera1.fx/fy/cx/cy` + distortion; `Camera2.*` if stereo | everything |
| **(c) stereo** | `Stereo.ThDepth`, and **either** `Stereo.b` (rectified) **or** `Stereo.T_c1_c2` | stereo |
| **(d) inertial** | `IMU.NoiseGyro/NoiseAcc/GyroWalk/AccWalk/Frequency`, `IMU.T_b_c1` | `*_INERTIAL` modes |
| **(e) ORB** | `ORBextractor.nFeatures/scaleFactor/nLevels/iniThFAST/minThFAST` | everything |
| **(f) Atlas** | `System.LoadAtlasFromFile`, `System.SaveAtlasToFile` | multi-session |

### 2.1 Camera type — three, and the choice is not cosmetic

Per `Calibration_Tutorial.pdf` §3.1:

- **`PinHole`** — $f_x, f_y, c_x, c_y$ + radial-tangential $k_1, k_2, [k_3], p_1, p_2$. Stereo pinhole pairs are **rectified internally** by OpenCV `stereorectify`.
- **`KannalaBrandt8`** — fisheye, $k_1..k_4$ equidistant. **Deliberately not rectified**, to keep the FOV. Requires `Camera{1,2}.overlappingBegin/End`.
- **`Rectified`** — already-rectified pair: only $f_x, f_y, c_x, c_y$ and `Stereo.b` in metres.

The non-rectified stereo handling (paper §IV-B) is a real architectural choice: the rig is treated as
**two monocular cameras with a constant relative SE(3)** plus an optional common region. Features in the
overlap are triangulated on first sight; features outside it still contribute as monocular observations
across multiple views. Nothing is thrown away.

### 2.2 IMU noise model

Identical in form to VINS-Fusion's `acc_n / gyr_n / acc_w / gyr_w` (tutorial §3.2), with one trap:

$$
\tilde{\mathbf{a}} = \mathbf{a} + \boldsymbol{\eta}^a + \mathbf{b}^a, \qquad
\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega} + \boldsymbol{\eta}^g + \mathbf{b}^g
$$

with biases as Brownian motion, $\mathbf{b}_{i+1} = \mathbf{b}_i + \boldsymbol{\eta}_{\text{rw}}$.
`NoiseGyro`/`NoiseAcc` are **continuous-time noise densities** in $\mathrm{rad}/s/\sqrt{\mathrm{Hz}}$ and
$m/s^2/\sqrt{\mathrm{Hz}}$; ORB-SLAM3 discretizes internally as $\sigma_{a,f} = \sigma_a / \sqrt{f}$,
which is why **`IMU.Frequency` is mandatory and must be correct** — get it wrong and every preintegration
covariance is mis-scaled.

The tutorial's own advice is worth quoting verbatim, because it runs against the instinct to describe a
noise-free simulator honestly:

> "It is common practice to increase the random walk standard deviations provided by the IMU
> manufacturer (say multiplying them by 10) to account for unmodelled effects and **improving the IMU
> initialization convergence**."

### 2.3 A CARLA config, derived

**This is derivation, not measurement — nothing below has been run.** The rig is the one documented in
main README §3: 960×720, FOV 90°, $f = 480$, $(c_x, c_y) = (480, 360)$, cameras at $x = 1.5$, $z = 1.5$,
$y = \mp 0.25$ (CARLA), IMU at CoG height, 20 Hz stereo / 200 Hz IMU.

**Extrinsics carry over directly.** ORB-SLAM3's `IMU.T_b_c1` is defined as "the transformation that takes
a point from Camera1 to IMU" — which is **exactly** VINS-Fusion's `body_T_cam0`. So:

$$
\texttt{IMU.T\_b\_c1} = {}^{b}T_{c_0} =
\begin{bmatrix} 0 & 0 & 1 & 1.5 \\ -1 & 0 & 0 & 0.25 \\ 0 & -1 & 0 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

And the stereo extrinsic follows, $\mathbf{T}_{c_1 c_2} = ({}^{b}T_{c_0})^{-1}\, {}^{b}T_{c_1}$:

$$
\mathbf{T}_{c_1 c_2} =
\begin{bmatrix} 1 & 0 & 0 & 0.5 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

**Identity rotation, pure $+0.5$ m translation along the optical $x$ axis.** That is the definition of a
rectified pair — which means the CARLA rig should be declared **`Camera.type: "Rectified"` with
`Stereo.b: 0.5`**, skipping ORB-SLAM3's internal `stereorectify` entirely. (KITTI, the other car dataset
in `Examples/`, is configured exactly this way.) A `PinHole` declaration with the full `T_c1_c2` would
also be correct but would pay for a rectification that is a no-op.

Now the parameter table, in the same form as main README §4.2 — stock EuRoC values from
[`Examples/Stereo-Inertial/EuRoC.yaml`](Examples/Stereo-Inertial/EuRoC.yaml), and the reason column
grounded in a property of the CARLA rig:

| Parameter | Stock (EuRoC) | Proposed (CARLA) | Reason — what we know about the rig |
|-----------|---------------|------------------|--------------------------------------|
| `Camera.type` | `PinHole` | **`Rectified`** | $\mathbf{T}_{c_1c_2}$ is identity-rotation + pure-$x$ — the pair is already rectified (derived above) |
| `Camera1.fx/fy` | 458.65 / 457.30 | **480.0 / 480.0** | exact: $f = \frac{W}{2\tan(\text{FOV}/2)} = \frac{960}{2\tan 45^\circ} = 480$ |
| `Camera1.cx/cy` | 367.2 / 248.4 | **480.0 / 360.0** | image centre — CARLA renders an ideal pinhole |
| `Camera1.k1..p2` | nonzero | **omitted** (`Rectified` takes none) | no lens distortion exists to model |
| `Camera.width/height` | 752 × 480 | **960 × 720** | sensor config |
| `Camera.fps` | 20 | **20** | matches the CARLA camera rate |
| `Stereo.b` | — (uses `T_c1_c2`) | **0.5** | baseline, exact by construction |
| `Stereo.ThDepth` | 60.0 | **≈ 35** | baselines-of-depth for the close/far split. EuRoC's 60 × 0.11 m ≈ 6.6 m suits a room; 35 × 0.5 m ≈ 17.5 m suits a street. KITTI uses 35. |
| `IMU.T_b_c1` | measured drone extrinsic | **exact matrix above** | we *place* the cameras — known to machine precision |
| `IMU.NoiseGyro` | 1.7e-04 | 0.000244 → **~0.0024** | VINS uses the BNO055 floor; ORB-SLAM3's tutorial explicitly recommends inflating (×10) for init convergence |
| `IMU.NoiseAcc` | 2.0e-03 | 0.00147 → **~0.0147** | same |
| `IMU.GyroWalk` | 1.94e-05 | 0.00002 → **~0.0002** | same — random walk is where the tutorial's ×10 advice specifically applies |
| `IMU.AccWalk` | 3.0e-03 | 0.0005 → **~0.005** | same |
| `IMU.Frequency` | 200.0 | **200.0** | one IMU sample per 200 Hz world tick |
| `ORBextractor.nFeatures` | 1200 | **2000** | KITTI's value: wide, feature-sparse streets at higher resolution need a bigger budget than a 752×480 indoor rig |
| `ORBextractor.iniThFAST` / `minThFAST` | 20 / 7 | **20 / 7** | keep, unless CARLA's rendered contrast proves low |
| `System.thFarPoints` | unset | **consider ~20 m** | the paper discards points beyond 20 m for TUM-VI *outdoors* because distant sky/cloud points drift; CARLA skyboxes pose the same risk |

> **The one genuinely uncertain row is the IMU noise.** VINS-Fusion's config takes the honest position —
> a noise-free sim IMU gets datasheet-floor values — and main README §4.2 already flags that this
> "does **not** rescue stereo+IMU on smooth driving." ORB-SLAM3's tutorial takes the opposite position for
> a different reason: inflated random walk helps *initialization converge*. Since initialization is
> precisely where the CARLA motion profile is expected to hurt (§5), the inflated values are the ones
> worth trying first, and the gap between the two is a real experiment, not a formatting choice.

---

## 3. Sensor configurations & experiment setup

ORB-SLAM3's four configurations map cleanly onto the main README §4.3 variant table, with one
column that has no VINS-Fusion counterpart:

| Variant | Sensors | Scale from | Notes |
|---------|---------|-----------|-------|
| `MONOCULAR` | one camera | — (up to scale) | evaluated with Sim(3) alignment, 7 DOF |
| `STEREO` | stereo pair | triangulation, 0.5 m baseline | metric; the closest analogue to this project's best VINS variant |
| `IMU_MONOCULAR` | camera + IMU | IMU (estimated) | needs the 3-stage init to converge |
| `IMU_STEREO` | stereo + IMU | baseline ($s \equiv 1$) | paper's most accurate & most robust configuration |
| `RGBD` / `IMU_RGBD` | depth camera | sensor | not applicable to this rig |

**What is missing versus this project's matrix.** There is no GPS variant — ORB-SLAM3 has no global
sensor input at all, so the `*+gps` column of main README §4.5 has no counterpart. In this project GPS
was "the single biggest accuracy win"; ORB-SLAM3's substitute for bounding long-run drift is
**loop closure**, which only pays off on a route that revisits itself. The town10 loop (~442 m, closes)
is a fair test of that; the town01 segment (~190 m, does not close) is not.

**A like-for-like experiment**, mirroring §4.3–4.4, would be: the same three scenes
(`town01_normal`, `town10_normal`, `town10_alwaysrun`), the same 5 runs per variant, APE RMSE against
`/carla/ego_vehicle/odometry`, in the offline (bag-reader) path first — since that is this project's
established accuracy ceiling and removes DDS frame-drop as a variable. Two ORB-SLAM3-specific metrics
should be added that VINS-Fusion has no analogue for: **number of maps in the Atlas at the end** (a
direct count of tracking losses) and **number of loop closures / merges detected**.

### 3.1 ROS 2 topic graph (to be built)

There is **no ROS 2 node upstream** (§7, gap 1) — the graph below is what one *would* wire up, using the
exact CARLA streams this project already records for VINS. Two ingestion paths exist, and only **Path A**
involves ROS 2 topics at all; **Path B** (the recommended one, §7 gap 2) bypasses the DDS layer entirely
and hands each message straight into the C++ API, which is where this project's deterministic accuracy
ceiling is measured (§4.4 of the main README).

The only real reference is the **ROS 1** node
[`ros_stereo_inertial.cc`](Examples_old/ROS/ORB_SLAM3/src/ros_stereo_inertial.cc); the solid box below
is what it *actually* does, the dashed box is what a ROS 2 port would still have to **add** — nothing in
the dashed box exists upstream.

```
 ── PATH A · online ROS node ──────────────────────────────────────────────────────────────────────────
    Real reference (ROS 1, rosbuild): Examples_old/ROS/ORB_SLAM3/src/ros_stereo_inertial.cc

    SUBSCRIBES — 3 plain ros::Subscriber, no message_filters (:141-143)
    /camera/left/image_raw   sensor_msgs/Image ─┐   per-topic          ╔══════════════════════════════╗
    /camera/right/image_raw  sensor_msgs/Image ─┤   buffers + mutexes  ║ ros_stereo_inertial          ║
    /imu                     sensor_msgs/Imu   ─┘ → SyncWithImu() thread║  rectify → TrackStereo(      ║
                                                     (:145, :196)       ║    imLeft, imRight, t,       ║
                                                                        ║    vImuMeas)                 ║
    PUBLISHES — NONE.  No advertise() anywhere; no SaveTrajectory.      ║  output: Pangolin viewer     ║
    The node's only output is the live Pangolin window.                 ║  only (ros::spin → return)   ║
                                                                        ╚══════════════════════════════╝
    To feed it CARLA data, REMAP the bag topics onto the node's names:
       /carla/ego_vehicle/cam_front_left/image   → /camera/left/image_raw
       /carla/ego_vehicle/cam_front_right/image  → /camera/right/image_raw
       /carla/ego_vehicle/imu                    → /imu
       /carla/ego_vehicle/gnss      ─ dropped   (ORB-SLAM3 has no global input, §3 / §5.4)
       /carla/ego_vehicle/odometry  ─ ground truth → APE evaluator only, never enters the node (§3)

    ┌ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┐
      A ROS 2 port must (a) move to rclcpp (gap 1) and (b) ADD publishers — DESIGN CHOICE, not upstream.
    │ A useful set: /orbslam3/pose (PoseStamped) · /odometry (Odometry) · /map_points (PointCloud2)     │
      · /tf (world→body).  Get the pose from TrackStereo's returned Sophus::SE3f (Tcw); the map-side
    │ topics expose the Atlas that §5.5 says to log (map count / merge count).                          │
    └ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ┘

 ── PATH B · offline bag-reader (recommended, deterministic, ~10× faster) ─────────────────────────────

   rosbag2 ──SequentialReader──▶ every msg in header.stamp order ──▶ SLAM.TrackStereo(imL, imR, t, vImu) ──▶ trajectory CSV
                                     NO ROS 2 topics — messages handed straight into the ORB-SLAM3 API
```

Three things about this graph are ORB-SLAM3-specific and worth stating explicitly:

- **`/carla/ego_vehicle/gnss` has no consumer.** Unlike VINS-Fusion's `global_fusion` stage, ORB-SLAM3
  takes no global input — the `*+gps` variants of main README §4.5 have no counterpart here (§3), and
  drift is bounded by loop closure instead (§5.4).
- **The reference node does its own thread-based sync, not `message_filters`.** `TrackStereo` takes one
  timestamp for both images, and the stereo-inertial node satisfies that with the manual `SyncWithImu()`
  thread above — buffering the 200 Hz IMU and passing the samples between consecutive frames as
  `vImuMeas`. (The *non-inertial* [`ros_stereo.cc`](Examples_old/ROS/ORB_SLAM3/src/ros_stereo.cc) is the
  one that uses `message_filters`.)
- **Every output topic is a port-time design choice, not an upstream fact.** The reference node publishes
  nothing; the persistent Atlas (and the map count / merge count §5.5 argues for logging) is reachable
  through the C++ API but is not exposed on any topic until a port adds one.

---

## 4. Results (from the paper)

**These are the authors' published numbers on EuRoC and TUM-VI, reproduced from
[`ORBSLAM3.pdf`](ORBSLAM3.pdf) Tables II, III and VI. They are not measurements from this project, and
neither dataset resembles CARLA driving.** They are included because they set the expectation any CARLA
run would be judged against.

**EuRoC, RMS ATE (m)** — median of 10 executions, from paper Table II. VINS-Fusion rows are the paper's
reported values for the same sequences:

| Configuration | System | MH01 | MH03 | V101 | V103 | V203 | Avg |
|---------------|--------|-----:|-----:|-----:|-----:|-----:|----:|
| Monocular | ORB-SLAM3 | 0.016 | 0.028 | 0.033 | 0.033 | — | 0.041* |
| Stereo | ORB-SLAM2 | 0.035 | 0.028 | 0.035 | 0.048 | — | 0.044* |
| Stereo | **VINS-Fusion** | 0.540 | 0.330 | 0.550 | — | — | 0.424* |
| Stereo | **ORB-SLAM3** | **0.029** | **0.024** | **0.035** | **0.061** | **0.521** | **0.084** |
| Mono-Inertial | VINS-Mono | 0.084 | 0.074 | 0.047 | 0.180 | 0.244 | 0.110 |
| Mono-Inertial | **ORB-SLAM3** | 0.062 | 0.046 | 0.057 | 0.037 | 0.027 | **0.043** |
| Stereo-Inertial | VINS-Fusion | 0.166 | 0.125 | 0.076 | 0.114 | 0.096 | 0.138 |
| Stereo-Inertial | Kimera | 0.080 | 0.110 | 0.050 | 0.120 | 0.190 | 0.119 |
| Stereo-Inertial | **ORB-SLAM3** | **0.036** | **0.035** | **0.038** | **0.024** | **0.024** | **0.035** |

\* averaged over successful sequences only; systems that did not complete all sequences are marked and
not bolded in the original.

The paper's own framing: stereo-inertial ORB-SLAM3 is **"three to four times more accurate than Kimera
and VINS-Fusion"**, and mono-inertial is "five to ten times more accurate than MCSKF, OKVIS and ROVIO."
Headline accuracy figures: **3.5 cm average on EuRoC** (drone), **9 mm under quick hand-held motion in
the TUM-VI room**.

**TUM-VI outdoors** is the more instructive block for a vehicle, because it is the regime where
ORB-SLAM3 itself degrades (Table III, stereo-inertial, RMS ATE in m):

| Sequence | Length | ORB-SLAM3 | Loop closure available |
|----------|-------:|----------:|:---:|
| `outdoors1` | 2656 m | 32.23 | — |
| `outdoors4` | 928 m | 11.61 | — |
| `outdoors5` | 1168 m | 8.95 | ✓ |
| `outdoors7` | 1748 m | 4.58 | ✓ |
| `room1..6` | ~130 m | 0.01–0.02 | ✓ |

Two orders of magnitude between the room and the outdoor sequences, and the paper names the cause:
"in some long *outdoors* sequences, the scarcity of close visual features may cause drift of the
inertial parameters, notably scale and accelerometer bias, which leads to errors in the order of 10 to
70 meters."

**Timing** (paper Table VI, EuRoC V202, Intel i7-7700 @ 3.6 GHz, CPU only, 752×480):

| Thread / stage | Stereo | Stereo-Inertial |
|----------------|-------:|----------------:|
| ORB extraction | 15.68 ms | 15.22 ms |
| stereo rectification | 1.32 ms | 1.60 ms |
| IMU integration | — | 0.22 ms |
| track local map | 6.31 ms | 11.51 ms |
| **tracking total** | **31.48 ms** | **33.05 ms** |
| local BA (mapping) | 134.60 ms | 152.70 ms |
| **mapping total / KF** | **158.84 ms** | **196.61 ms** |

Place recognition costs ~10 ms per keyframe; merges and loop closures stay under 1 s (pose-graph only),
while a full BA after loop closure runs 1.1–4.1 s — in a separate thread, so it does not block tracking.

Context for this project: VINS-Fusion here measures **~3.3 ms/frame** front-end on CPU. ORB-SLAM3's
tracking thread is **~10× that** — ORB extraction alone (15.7 ms) is 5× the entire VINS front end. At
20 Hz (50 ms budget) tracking still fits comfortably; the thing to watch in a closed loop is that
mapping takes ~160–200 ms per keyframe, so keyframes must be rate-limited or the map lags the vehicle.

---

## 5. Analysis — key findings

### 5.1 The paper's own conclusion warns against this project's exact use case

From §VIII, verbatim:

> "In applications with slow motions, or without roll and pitch rotations, **such as a car in a flat
> road, IMU sensors can be difficult to initialize. In those cases, if possible, use stereo SLAM.**"

This is the same observability degeneracy that main README §4.6 established empirically for
VINS-Fusion on CARLA — smooth, planar, low-rotation motion leaves velocity, biases and gravity
unobservable. Two independent VI systems failing the same way on the same motion profile is strong
evidence the finding is about **the motion, not the pipeline**, which is exactly what §4.6 claims.
Running ORB-SLAM3 `IMU_STEREO` on `town10_normal` is therefore a **falsifiable prediction**, and the
single most valuable experiment available: if it diverges too, §4.6 is confirmed cross-system; if it
holds, then ORB-SLAM3's MAP initialization is doing something VINS-Fusion's linear alignment cannot,
and the finding needs narrowing.

### 5.2 The documented remedy for slow motion does not apply to stereo-inertial

Reading the paper alone, ORB-SLAM3 appears to have a recovery mechanism VINS-Fusion lacks: the periodic
**scale refinement**, introduced precisely because "slow motion does not provide good observability of
the inertial parameters."

**In the source it is monocular-only.** `LocalMapping::ScaleRefinement()` (`LocalMapping.cc:1429`) is
reached only under `mbMonocular` (`:231`). An `IMU_STEREO` run on CARLA would therefore get **no**
periodic re-estimation of gravity and scale — which is also self-consistent, since stereo scale is fixed
at $s \equiv 1$ by construction and there is nothing left to refine.

The consequence is that for the configuration this project would actually use, ORB-SLAM3's inertial
safety net is **thinner than the paper reads**, not thicker. If `IMU_STEREO` degrades on smooth CARLA
driving, the mechanism that would have caught it is not in that code path. `IMU_MONOCULAR` is the only
variant that gets it — which makes it an interesting control, despite being the weaker configuration
otherwise.

### 5.3 Inertial failure would remove loop closure too, not just the IMU

`LoopClosing::NewDetectCommonRegions()` returns immediately for an inertial map that has not reached
**VIBA2** (`LoopClosing.cc:341`). The two capabilities are therefore coupled: in `IMU_STEREO`, if IMU
initialization stalls before its 15 s third stage, the system loses **place recognition, loop closure
and map merging as well** — the exact mechanisms §5.4 relies on to bound drift.

This is a compounding failure, and it has a clean design implication: **`STEREO` may outperform
`IMU_STEREO` on CARLA by more than the IMU's own contribution**, because pure stereo has no such gate
(it needs only 5 keyframes). That mirrors main README §4.6's finding that stereo-only was bounded where
stereo+IMU diverged — but predicts a *larger* gap for ORB-SLAM3 than for VINS-Fusion.

### 5.4 On a smooth road, the advantage should come from mid-term association and loop closure

Main README §4.6 found GPS to be the biggest accuracy win because it bounds absolute drift. ORB-SLAM3
bounds drift differently — by *re-recognizing* places. On `town10` (a closed ~442 m loop) that should
work and is the fair comparison against `stereo+gps`. On `town01` (an open ~190 m segment) it cannot
help, and pure `STEREO` should behave much like this project's stereo VINS.

Note the asymmetry in what each system needs from the route: GPS fusion works everywhere and needs no
revisit; loop closure is free but only pays on a closed route. The town10/town01 split in the existing
scene set happens to separate these cleanly.

### 5.5 The Atlas changes what "divergence" means

VINS-Fusion's IMU variants exploded to ~1160 m and were recorded as `DIV`; note also that this project's
`failureDetection()` is disabled (`return false`), so nothing ever resets. ORB-SLAM3 in the same
situation loses tracking and — above 10 keyframes — calls `Tracking::CreateMapInAtlas()`
(`Tracking.cc:2662`), starting a **new map** while keeping the old one; below 10 keyframes it throws the
map away via `ResetActiveMap()`.

So a CARLA comparison must report **map count and merge count** alongside APE. A run that "succeeded"
with 6 maps in the Atlas is a very different result from one that succeeded with 1, and a naive ATE over
a multi-map run is not comparable to a single-map one. `Atlas::CountMaps()` and the merge counters make
this cheap to log.

### 5.6 The 15-second discard rule interacts badly with a short scene

A map is discarded if tracking is lost within 15 s of IMU initialization. `town01_normal` is only 39.5 s
long. Combined with a slow-to-converge initialization on smooth motion — and note that VIBA2 itself does
not fire until $t > 15$ s — a meaningful fraction of that sequence could be spent in
initialize→lose→discard cycles, never reaching a mature map at all.

`town10_alwaysrun` (110 s, continuously maneuvering) is the sequence most likely to give ORB-SLAM3 a
fair inertial run — and it is also the scene where main README §4.6 found varied motion *restored* IMU
observability for VINS-Fusion (stereo+imu+gps → 0.25 m). That agreement is worth checking, and it is the
sequence to run first if the goal is to see ORB-SLAM3's inertial path at its best.

### 5.7 The architectural difference in one line

VINS-Fusion marginalizes and forgets; ORB-SLAM3 keeps a map and re-matches against it. Everything else —
the persistent 3-D map points instead of anchored inverse depths, the absence of a marginalization prior
in the cost, the Atlas, the DBoW2 database, the ~10× tracking cost — follows from that one choice.

### 5.8 Two upstream oddities worth knowing before debugging

Both are in V1.0 as shipped, and both will waste your time otherwise:

- **`Tracking::ResetFrameIMU()` is an empty stub** (`Tracking.cc:1788`, marked `// TODO`) despite being
  called from `Track()` at `:2186`. The real bias/scale propagation lives in
  `Tracking::UpdateFrameIMU()` (`:3980`). This is a close cousin of this project's own disabled
  `failureDetection()` — a safety hook that looks live and isn't.
- **`Optimizer::GlobalBundleAdjustemnt` is misspelled** in the source. That is the actual symbol; grep
  for it accordingly.

---

## 6. Commands

Nothing here is built yet. Bring-up:

```bash
cd ORB_SLAM3
tar -xzf Vocabulary/ORBvoc.txt.tar.gz -C Vocabulary/   # 43 MB packed; required at run time
chmod +x build.sh && ./build.sh                        # builds Thirdparty (DBoW2, g2o, Sophus) + libORB_SLAM3.so
```

Dependencies (`CMakeLists.txt`): **C++11**, **OpenCV ≥ 4.4**, **Eigen ≥ 3.1**, **Pangolin**, optional
`realsense2`. This project already builds OpenCV 4.10 in `~/local`, which satisfies the minimum — but
note the same `LD_LIBRARY_PATH` hazard documented for RTAB-Map in main README §2/§4.2(d): link and run
ORB-SLAM3 against **one** OpenCV consistently.

Run a stereo-inertial sequence (the shape a CARLA runner would copy):

```bash
./Examples/Stereo-Inertial/stereo_inertial_euroc \
    Vocabulary/ORBvoc.txt \
    Examples/Stereo-Inertial/EuRoC.yaml \
    "$SEQ_PATH" Examples/Stereo-Inertial/EuRoC_TimeStamps/MH01.txt \
    dataset-MH01_stereoi
```

Stereo-only, KITTI-style (the closest existing example to a road vehicle):

```bash
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTI00-02.yaml "$KITTI_SEQ"
```

Multi-session — uncomment in the YAML:

```yaml
System.LoadAtlasFromFile: "carla_town10_session1"
System.SaveAtlasToFile:   "carla_town10_session1"
```

---

## 7. Gaps for this project

Concrete work between this directory and a result comparable with main README §4.5:

| # | Gap | Detail |
|---|-----|--------|
| 1 | **ROS 1 only** | `Examples_old/ROS/ORB_SLAM3/manifest.xml` declares `roscpp` and uses rosbuild — this is **ROS 1 (Melodic)**. This project is ROS 2 Humble. There is no ROS 2 node upstream; one must be written, or the offline path used. |
| 2 | **No bag reader** | The natural route, and the one that avoids gap 1 entirely: a `SequentialReader`-based runner mirroring `vins_bag_reader.cpp`, feeding `SLAM.TrackStereo(imLeft, imRight, t, vImuMeas)` directly. That also lands in this project's deterministic offline path, which is where its accuracy ceiling is measured. |
| 3 | **No CARLA YAML** | §2.3 derives the values; they still need writing to a file and sanity-checking against a real bag. |
| 4 | **Frame convention** | ORB-SLAM3 defines world $z$ **opposite gravity** and body ≡ IMU. This project's CARLA→ROS/ENU convention (negate $y$, yaw, $v_y$, $\omega_z$) must be applied before comparison, and the left/right camera assignment double-checked — a swapped pair was the root cause of early live divergence for VINS. |
| 5 | **Evaluation alignment** | Monocular results need **Sim(3)** (7 DOF) alignment; stereo and inertial need **SE(3)** (6 DOF). This project's APE tooling uses Umeyama alignment already, but the DOF must be selected per variant or monocular numbers will be wrong. |
| 6 | **Vocabulary not extracted** | `Vocabulary/ORBvoc.txt.tar.gz` — load takes several seconds at every startup; a binary vocabulary is the usual fix if startup time matters. |
| 7 | **Determinism** | Main README §4.4 treats run-to-run spread as a first-class metric. ORB-SLAM3 uses RANSAC in place recognition and MLPnP relocalization, and runs three concurrent threads — it should be assumed **non-deterministic** unless demonstrated otherwise, so the 5-run spread column matters more here, not less. |

---

## Appendix — code map

Enough to navigate the source without re-deriving it. All paths relative to this directory.

**Tracking state machine** — `Tracking.h:121`:
`SYSTEM_NOT_READY=-1, NO_IMAGES_YET=0, NOT_INITIALIZED=1, OK=2, RECENTLY_LOST=3, LOST=4` (`OK_KLT=5`
is unused). `Track()` is `Tracking.cc:1794`. Pose is obtained by, in order of preference:
`TrackWithMotionModel()` (`:2854`) → `TrackReferenceKeyFrame()` (`:2720`) → `Relocalization()` (`:3609`,
BoW candidates restricted to the **active map**, solved with **MLPnP** RANSAC), then always
`TrackLocalMap()` (`:2949`).

Note what the IMU does to the motion model: once initialized and past the relocalization window,
`TrackWithMotionModel` calls `PredictStateIMU()` and returns immediately (`:2862-2867`) — **the IMU
replaces the constant-velocity model outright** rather than seeding it.

**Which optimizer runs when** — the branch in `TrackLocalMap` (`Tracking.cc:2970-2993`) is worth
knowing, because it silently selects between three different problems:

| Condition | Optimizer |
|-----------|-----------|
| IMU not initialized | `PoseOptimization` (visual motion-only BA) |
| within `mnFramesToResetIMU` of a relocalization | `PoseOptimization` |
| `!mbMapUpdated` | `PoseInertialOptimizationLastFrame` |
| map was updated since last frame | `PoseInertialOptimizationLastKeyFrame` |

**All g2o optimizations** (`src/Optimizer.cc`), inertial ones in bold:

| Function | Def | Called from |
|----------|----:|-------------|
| `PoseOptimization` | `:814` | tracking, relocalization |
| **`PoseInertialOptimizationLastKeyFrame`** | `:4491` | `TrackLocalMap` |
| **`PoseInertialOptimizationLastFrame`** | `:4875` | `TrackLocalMap` |
| `LocalBundleAdjustment` | `:1116` | `LocalMapping::Run:154` |
| `LocalBundleAdjustment` (merge overload) | `:3498` | `MergeLocal` welding BA |
| **`LocalInertialBA`** | `:2383` | `LocalMapping::Run:149` |
| `GlobalBundleAdjustemnt` *(sic)* | `:52` | mono map init; post-loop GBA |
| **`FullInertialBA`** | `:392` | `InitializeIMU`; post-loop GBA on inertial maps |
| `OptimizeEssentialGraph` | `:1501` | `CorrectLoop` |
| `OptimizeEssentialGraph` (merge overload) | `:1785` | `MergeLocal` |
| **`OptimizeEssentialGraph4DoF`** | `:5292` | `CorrectLoop` on inertial maps |
| `OptimizeSim3` | `:2115` | place-recognition verification |
| **`InertialOptimization`** (gravity + scale + biases) | `:3042` | `InitializeIMU` |
| **`InertialOptimization`** (biases only) | `:3227` | `MergeLocal2` |
| **`InertialOptimization`** (gravity + scale only) | `:3389` | `ScaleRefinement` |
| **`MergeInertialBA`** | `:3948` | `MergeLocal`, `MergeLocal2` |
| `Marginalize` | `:2960` | Schur helper for the `LocalInertialBA` prior |

**Preintegration** — `IMU::Preintegrated` (`include/ImuTypes.h:143`). Bias updates are **first-order**:
`SetNewBias()` stores the delta and `GetDeltaRotation/Velocity/Position(b)` apply the stored Jacobians
`JRg, JVg, JVa, JPg, JPa`; full `Reintegrate()` is available because `mvMeasurements` is retained, and
`MergePrevious()` handles keyframe culling. `GRAVITY_VALUE = 9.81` (`:43`) — a **hardcoded constant**,
not a config key, so the CARLA `g_norm: 9.81007` from the VINS config has no equivalent here.

**Config parsing** — two paths, chosen at `System.cc:77`: with `File.version: "1.0"` a `Settings` object
is built (`src/Settings.cc:127`, which `exit(-1)`s on any missing required key); without it each
component parses the old flat yaml itself (`Tracking::ParseCamParamFile` `:619`, `ParseORBParamFile`
`:1217`, `ParseIMUParamFile` `:1301`).

---

## References

- Campos, Elvira, Gómez Rodríguez, Montiel, Tardós. **ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM.** *IEEE T-RO* 37(6):1874–1890, 2021. — [`ORBSLAM3.pdf`](ORBSLAM3.pdf)
- Gómez Rodríguez, Campos, Tardós. **Calibration Tutorial for ORB-SLAM3 v1.0.** Dec 2021. — [`Calibration_Tutorial.pdf`](Calibration_Tutorial.pdf)
- Campos, Montiel, Tardós. **Inertial-Only Optimization for Visual-Inertial Initialization.** *ICRA* 2020. — the three-stage init of §1.4
- Elvira, Montiel, Tardós. **ORBSLAM-Atlas: a robust and accurate multi-map system.** *IROS* 2019. — the Atlas of §1.5
- Gálvez-López, Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences.** *IEEE T-RO* 28(5), 2012. — DBoW2
- Kannala, Brandt. **A generic camera model and calibration method for conventional, wide-angle, and fish-eye lenses.** *IEEE TPAMI* 28(8), 2006.
