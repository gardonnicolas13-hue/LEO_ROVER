## Title
`try_zupt: true` diverges the χ² gate to ~1e17 (IMU path) or literal `inf` (disparity-only path) within seconds on ARM/single-precision build — filter accepts residuals many orders of magnitude past threshold, in *both* ZUPT sub-modes

## Summary
On a Raspberry Pi 4B build (`USE_FLOAT=1`, confirmed in the compiler invocation), enabling zero-velocity updates (`try_zupt: true`) with the robot stationary diverges the filter within seconds, in **both** of the two ZUPT sub-modes this package exposes:

- **IMU-velocity path** (`zupt_chi2_multipler: 1`): estimated body velocity grows monotonically from ~1780 m/s to ~6000+ m/s across a handful of updates, χ² printed around **1.65×10¹⁷** against a threshold near 65, every update logged as *accepted*.
- **Disparity-only path** (`zupt_chi2_multipler: 0`, `zupt_max_disparity: 0.5`) — tried specifically as a hoped-for safer middle ground, since the code comments describe it as gating on visual disparity rather than IMU-derived velocity: **worse, not better**. Velocity reaches ~2×10¹⁷ m/s within roughly a dozen updates (well under a second), and the printed χ² is no longer merely huge, it is literal `inf`, against a printed threshold of `0.000`:
  ```
  [ZUPT]: passed disparity (0.032 < 0.500, 175 features)
  [ZUPT]: accepted |v_IinG| = 143531553559937024.000 (chi2 inf < 0.000)
  [ZUPT]: passed disparity (0.033 < 0.500, 175 features)
  [ZUPT]: accepted |v_IinG| = 171007696303030272.000 (chi2 inf < 0.000)
  ```
  The threshold collapsing to exactly `0.000` alongside `chi2 inf` suggests the consistency-check denominator itself underflows/vanishes in this path, not just an overflow on the numerator.

Neither ZUPT sub-mode is a near-miss gate failure — both report *acceptance* on residuals many orders of magnitude past the threshold they are supposed to enforce, which reads as a numerical-representation defect rather than a tuning issue, and rules out "use the other ZUPT mode instead" as a workaround.

Disabling ZUPT entirely (`try_zupt: false`) removes the crash but leaves the filter with no velocity anchor, and it subsequently diverges (differently, more slowly, but still severely) under real driven motion — see below. So this is not "just turn ZUPT off," it's a real gap in the deployable configuration space on this platform: neither ZUPT mode is usable, and going without it is not sufficient either.

## Environment
- Platform: Raspberry Pi 4B (`aarch64`), Ubuntu 20.04.6 LTS, kernel `5.4.0-1106-raspi`
- ROS: Noetic
- Build: `catkin build`, package `ov_srvins`, compiled with `-DUSE_FLOAT=1` (visible directly in the `cc1plus` invocation during build — single precision throughout, not just claimed by the README)
- Dependencies at their Ubuntu 20.04 apt versions: `libceres-dev 1.14.0`, `libopencv-contrib-dev 4.2.0`, `libeigen3-dev 3.3.7`
- Sensor input: Intel RealSense D455 stereo IR pair (640×480 @ ~15 Hz), onboard IMU sanitized/rescaled upstream (accel scale correction, gyro bias subtraction — both zeroed relative to stock defaults for this test, see config below) at ~80–90 Hz
- Isolated `catkin` workspace, built cleanly, 0 errors, `run_subscribe_msckf` launched via `roslaunch ov_srvins subscribe.launch config:=leo`

## Reproduction

**IMU-velocity path:**
1. Bring up `ov_srvins`'s `run_subscribe_msckf` against a live stereo+IMU feed, robot completely stationary.
2. Config: `try_zupt: true`, `zupt_chi2_multipler: 1`, `zupt_max_velocity: 0.1`, `zupt_noise_multiplier: 10`, `zupt_max_disparity: 0.5`, `zupt_only_at_beginning: false`. Every other field left at the package's own `config/euroc_mav/estimator_config.yaml` defaults except calibration values specific to our camera/IMU pair (intrinsics/extrinsics/noise densities), which were confirmed **not** the cause by bisection (see below).
3. Within a few seconds, console output shows (verbatim):
   ```
   [ZUPT]: accepted |v_IinG| = 1937.014 (chi2 9982107438809088.000 < 65.171)
   [ZUPT]: passed disparity (0.037 < 0.500, 196 features)
   [ZUPT]: accepted |v_IinG| = 2339.450 (chi2 15946744344870912.000 < 58.124)
   ...
   [ZUPT]: accepted |v_IinG| = 6015.597 (chi2 165137575521026048.000 < 65.171)
   ```

**Disparity-only path**, tested separately, otherwise byte-identical config:
1. Same setup, robot stationary. Only change from the stable no-ZUPT baseline: `try_zupt: false` → `true`. `zupt_chi2_multipler` was already `0` in our deployed config, so this single-field flip is the entire delta.
2. Within roughly a dozen updates (well under one second), console output shows (verbatim):
   ```
   [ZUPT]: passed disparity (0.036 < 0.500, 175 features)
   [ZUPT]: accepted |v_IinG| = 24884928843874304.000 (chi2 inf < 0.000)
   [ZUPT]: passed disparity (0.032 < 0.500, 175 features)
   [ZUPT]: accepted |v_IinG| = 29652645337628672.000 (chi2 inf < 0.000)
   ...
   [ZUPT]: accepted |v_IinG| = 203734831702474752.000 (chi2 inf < 0.000)
   ```

In both cases `|v_IinG|` grows monotonically and roughly geometrically call over call, and every single update is logged as *accepted* despite exceeding the printed threshold by many orders of magnitude (or, in the disparity-only case, despite the threshold itself printing as `0.000` and the residual as `inf`).

## What was ruled out (bisection, not guesswork)
Isolated by rebuilding the config from the package's own known-good `config/euroc_mav/estimator_config.yaml` and re-adding our platform's tuned values one field at a time, retesting after each addition:
- Camera/IMU calibration values (intrinsics, extrinsics, noise densities, measured local gravity 9.790 vs the default 9.81): harmless individually and in combination.
- `max_slam`/`max_slam_in_update` set to 0 (SLAM disabled, MSCKF-only): harmless.
- `calib_cam_*` extrinsic/intrinsic/timeoffset locks (all `false`): harmless.
- Feature-tracking tuning (`num_pts`, `grid_x/y`, `min_px_dist`, `knn_ratio`, `track_frequency`): harmless.
- `init_window_time`/`init_imu_thresh`/`init_max_disparity`/`init_max_features` tuning: harmless.

`try_zupt: true` (IMU-based path, `zupt_chi2_multipler` > 0) is the only field change that reproduces the divergence, confirmed by toggling it alone against an otherwise-identical, independently-verified-stable config (that same config, unmodified except this one field, ran cleanly at 30–70 Hz processing real camera/IMU frames for the full length of every other test conducted that evening).

## Hypothesis
The one architectural fact that distinguishes this build from a typical desktop/x86 openVINS deployment is single precision throughout (`USE_FLOAT=1`). A χ² blow-up of this exact shape — plausible values suddenly present as astronomically large, growing with each subsequent (now-corrupted) update — is consistent with either an overflow/precision-loss bug specific to the ZUPT update's Mahalanobis-distance computation in float32, or a covariance term becoming ill-conditioned (near-singular) in a way double precision would tolerate and float32 does not. Not confirmed by reading the ZUPT update source directly — reported as an observed, reproducible symptom with a strong architectural correlate, not a diagnosed root cause in the library's own code.

## What downstream this affects
Because neither ZUPT sub-mode is safely usable in this configuration, we deployed with `try_zupt: false` on this platform. That configuration ran stably at rest and during a short jerk-triggered initialization, but diverged severely during an actual driven lap of a real environment (loop-closure error effectively unbounded — tens of kilometers of accumulated reported distance for a ~20 m real path), considerably worse than a classic openVINS (MSCKF, double precision) instance run on the identical motion from recorded sensor data. We had hoped the disparity-only path would be a usable middle ground between "no velocity anchoring at all" and "the IMU-based setting that diverges" — it is not; it fails faster and more completely (`inf` rather than a large finite number) than the path it was meant to replace.

## Ask
Happy to provide the full config files, the complete console log from either repro, or run additional diagnostics if useful. Flagging primarily because a χ² gate silently accepting a value 1e15–1e17 past its own threshold seems like something worth knowing about regardless of platform, and we wanted to report it precisely rather than just noting "ZUPT doesn't work for us."
