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

## A candidate root cause, found by reading `UpdaterZeroVelocity.cpp` directly

We went looking in the source rather than stopping at "reported as an observed symptom," and found what looks like a real, concrete bug (not just a float32-precision artifact) in `ov_srvins/src/update/UpdaterZeroVelocity.cpp`:

1. **The χ² consistency check is computed via `S.llt().solve(res)` without ever checking `.info()`.** If the `LLT` decomposition of the innovation covariance `S` fails (not positive-definite — plausible under single precision as terms accumulate/round), Eigen's documented behavior is that `.solve()` silently returns garbage rather than throwing or returning a failure indicator. Nothing downstream inspects `.info()`, so a numerically invalid decomposition is indistinguishable from a valid one at the call site.
2. **In the disparity-only path, the chi2/velocity check is unreachable when it matters most.** The reject condition is written as (paraphrased): `if (!disparity_passed && (chi2 > thresh || vel > max_vel)) reject;`. When `disparity_passed == true` — which is the common case at rest, since disparity is small and stationary — the entire chi2/velocity gate is short-circuited out and the update is accepted unconditionally, regardless of what `chi2` actually evaluated to. This exactly matches what we observed: the console prints "(chi2 inf < 0.000)" as if it were being checked, but that printed comparison is cosmetic in this branch — it never gates anything once `disparity_passed` is true.

Both defects compound: an `LLT` failure produces a garbage (possibly `inf`/`nan`) `chi2`, and the disparity-only path's logic never actually looks at that value before accepting.

## A minimal patch we applied and tested (not upstream, offered as a candidate)

We added an unconditional numerical-validity guard, evaluated *before* the existing disparity-gated reject block, that does not change any tuning/threshold semantics — it only refuses to accept an update whose χ² was not validly computed:

```cpp
Eigen::LLT<MatX> llt_of_S(S);
DataType chi2 = res.dot(llt_of_S.solve(res));
bool chi2_numerically_valid = (llt_of_S.info() == Eigen::Success) &&
                               std::isfinite(static_cast<double>(chi2)) &&
                               (chi2 >= 0);
...
if (!chi2_numerically_valid) {
  last_zupt_state_timestamp_ = 0.0;
  last_zupt_count_ = 0;
  PRINT_WARNING(RED "[ZUPT]: rejected -- chi2 numerically invalid (llt_success=%d, "
                    "chi2=%.6e, disparity_passed=%d)\n" RESET, ...);
  return false;
}
```

Full diff attached: [`sqrtvins_zupt_numerical_guard.patch`](sqrtvins_zupt_numerical_guard.patch) (produced via `git diff` against the package as cloned, 64 lines, one file).

**Tested, and it does exactly what it's supposed to do.** A 60-second live run against real stereo+IMU data with the patch applied, robot stationary, `try_zupt: true`, disparity-only path: 588 updates correctly rejected as numerically invalid, 241 updates correctly accepted with sane χ² values, zero acceptances of an invalid/`inf` χ². The specific bug this report opened with — silent acceptance of astronomically-out-of-threshold residuals — no longer reproduces.

**Reported honestly, it is not sufficient on its own.** With the patch applied and every garbage ZUPT update now correctly rejected, the filter's position estimate (`p_IinG`) still diverges over the same 60-second stationary window — smoothly, not via the ZUPT chatter, from roughly 1e15 m to roughly 3.4e17 m. This means there is a second, independent numerical-stability issue elsewhere in the estimation pipeline (propagation and/or the MSCKF update are the natural suspects, given ZUPT is now provably not the vector) that we have not yet diagnosed. So: this patch appears to be a real fix for a real logic/numerics bug in the ZUPT updater specifically, but it is not, by itself, a fix for "the filter diverges on this platform." We're including it because it seems worth having regardless of the larger issue, and because the `LLT().solve()`-without-`.info()`-check pattern in particular seems like something worth checking for elsewhere in the codebase too.

## Ask
Happy to provide the full config files, the complete console log from either repro, the 60-second patched-run log, or run additional diagnostics if useful. Flagging primarily because a χ² gate silently accepting a value 1e15–1e17 past its own threshold seems like something worth knowing about regardless of platform, and we wanted to report it precisely rather than just noting "ZUPT doesn't work for us." The attached patch is offered as-is for review/adoption at your discretion — it was tested on our platform only (aarch64, float32), not validated against the upstream double-precision build, and we make no claim it's the cleanest fix, only that it's a minimal, additive one that measurably closes the specific gap this report describes.

---

## ADDENDUM (2026-08-21) — the single-precision hypothesis is REFUTED, and the accel-bias state is the vector

The "Hypothesis" section above proposed `USE_FLOAT=1` (single precision
throughout) as the architectural correlate of the divergence. **We built the
package in double precision and the bug reproduces identically.** The
hypothesis is wrong, and we are retracting it.

### Build

Same commit (`30fafc8`), same source, **no source modification** — `option()`
accepts a command-line value, so only the build flag changed:

```bash
catkin config --extend /opt/ros/noetic \
              --cmake-args -DCMAKE_BUILD_TYPE=Release -DUSE_FLOAT=OFF
catkin build
```

Verified: `USE_FLOAT:BOOL=OFF` in `build/ov_srvins/CMakeCache.txt`, and
`-DUSE_FLOAT=1` absent from every `flags.make` (the aarch64 build has it
present). Platform this time: x86_64 desktop (12 cores, 31 GB), Ubuntu 20.04,
ROS Noetic — not the Pi. Same live stereo+IMU feed, republished so that
openVINS and sqrtVINS receive **bit-identical** images and the same
`/imu/data_clean`.

### Result

`try_zupt: true`, robot **stationary**, double precision, tuning otherwise
aligned to our working openVINS config (`max_clones: 25`,
`up_msckf_chi2_multipler: 5`, `zupt_max_velocity: 50`, `fi_max_baseline: 200`):
the filter diverges to **2.06e15 m** with an estimated speed of **4.06e11 m/s**,
within a ~90 s window.

### The new information: which state is corrupted

Double precision changed the failure from unreadable numerical garbage into
something legible, and the console now names the vector directly:

```
q_GtoI = 0.278,0.192,-0.279,0.899 | p_IinG = -8.73e14, 1.32e15, 1.80e15 | dist = 2.06e15 (meters)
bg = -0.0023,0.0009,-0.0080 | ba = -147026.4576,11084.9667,36904.9269
```

- **`ba` (accelerometer bias) is corrupted to −147 026 m/s²** — seven orders of
  magnitude outside anything physical (a real bias is ~1e-2 m/s²).
- **`bg` (gyroscope bias) stays perfectly sane** at −0.0023, 0.0009, −0.0080.
- **The attitude quaternion stays stable** across updates.

So the ZUPT update is not blowing up the state as a whole: it drives the
**accel-bias block specifically** to a nonphysical value, and the position
runaway is the arithmetic consequence of double-integrating a 1.5e5 m/s²
bias. Orientation and gyro bias — updated by the same measurement — are
untouched.

That asymmetry is the useful clue, and it is not a precision artifact: it
reproduces in float64 on x86.

### Consequence for the earlier patch

Our `chi2_numerically_valid` guard remains correct and worth having (it
stops the silent acceptance of `inf`/out-of-threshold residuals), but the
addendum confirms what we already suspected in the original report: it is
**not** sufficient. The residual defect is in what the ZUPT update *does to
the accel-bias state*, not only in whether its χ² is validly computed.

### Also confirmed: without ZUPT there is no velocity anchor

`try_zupt: false`, double precision, robot **driven** (96 % of the window in
motion, wheel-confirmed), same aligned tuning: 5 015 m of estimated path for a
~12 m real trajectory, `v_max` 91.8 m/s. Over the same window and the same
inputs, openVINS reported 11.59 m and MINS 12.44 m — within 7 % of each other.
So neither ZUPT sub-mode is usable, and going without it is not sufficient
either, on double precision exactly as on single.

---

## ADDENDUM 2 (2026-08-21) — root cause found: the disparity check overrides the χ² gate entirely

Following the double-precision addendum above, we instrumented further and
found the defect. It is a **boolean-logic bug**, not a numerical one, which is
consistent with the divergence reproducing identically in float32 and float64.

### The line

`ov_srvins/src/update/UpdaterZeroVelocity.cpp`:

```cpp
// Check if we are currently zero velocity
// We need to pass the chi2 and not be above our velocity threshold
if (!disparity_passed && (chi2 > options_.chi2_multipler * chi2_check ||
                          state->imu->vel().norm() > zupt_max_velocity_)) {
  ... reject ...
}
```

The leading `!disparity_passed &&` gates the **entire** rejection. Once the
disparity check passes, **neither the χ² threshold nor the velocity threshold
is ever evaluated.** The comment directly above the line states the intended
semantics — *"We need to pass the chi2"* — and the code does the opposite.

### Measured, with the χ² gate fully enabled

`zupt_chi2_multipler: 1` (so the threshold is live and correctly computed at
65.171), double precision, robot stationary:

```
[ZUPT]: accepted |v_IinG| = 80973924029883048001536.000 (chi2 8.49e56 < 58.124)
[ZUPT]: accepted |v_IinG| = 101524452023746466676736.000 (chi2 1.34e57 < 65.171)
```

**290 accepted, 0 rejected.** A χ² of 1.34e57 against a threshold of 65.171,
and a velocity of 1.0e23 m/s against `zupt_max_velocity`, both reported as
accepted. This is not a threshold-tuning issue: the thresholds are correct and
simply never consulted.

This also explains why our earlier `chi2_numerically_valid` patch helped but
did not fix the divergence: it added a guard *before* this block, but this
block's own χ² comparison remained unreachable whenever disparity passed.

### The fix

The two guards do not measure the same thing, and only one of them should be
overridable:

- The **velocity** threshold reads the filter's *own* estimate, so it is
  circular: once the filter diverges, that estimate is wrong and would block
  the only mechanism able to recover it. Breaking that circularity with an
  independent witness (image disparity) is exactly what the disparity check is
  for. **This override is legitimate and we kept it.**
- The **χ²** measures whether the measurement is consistent with the state and
  its covariance. A χ² of 1e57 says the update is nonsense; applying it injects
  garbage into the state. No external witness makes an inconsistent update
  acceptable. **This override should not exist.**

```cpp
const bool chi2_non_finite = !std::isfinite(static_cast<double>(chi2)) || chi2 < 0;
const bool chi2_over       = (options_.chi2_multipler > 0) &&
                             (chi2 > options_.chi2_multipler * chi2_check);
const bool vel_over        = state->imu->vel().norm() > zupt_max_velocity_;

if (chi2_non_finite || chi2_over || (!disparity_passed && vel_over)) {
  ... reject ...
}
```

The documented `chi2_multipler: 0` ("disparity-only") semantics are preserved —
with 0, the χ² *threshold* is not applied — but a non-finite χ² is rejected in
every mode, since `nan` does not mean "very good measurement", it means "χ²
not computable".

### Result

Same build, same config, same stationary robot, only the guard changed:

| | before | after |
|---|---|---|
| `\|v_IinG\|` | 1.0e23 m/s | **0.030 m/s** |
| χ² (threshold 65.171) | 1.34e57 | **1.1 – 1.5** |
| final position | 2.06e15 m | **0.019 m** from origin |

Seventeen orders of magnitude, from one boolean.

Measured alongside the two other estimators on the same robot, same inputs,
robot stationary, 90 s: MINS final position (0.025, 0.017, 0.120), openVINS
(0.000, −0.000, 0.001), **sqrtVINS (0.010, 0.006, 0.019)**.

### A second, independent defect found on the way (reported separately below)

`ov_srvins/src/state/StateHelper.cpp`, `propagate_zero_motion()`:

```cpp
U_new.block(0, bg_id, 3, 3) = Mat3::Identity() * sqrt(dt_summed) * sigma_wb;
U_new.block(0, ba_id, 3, 3) = Mat3::Identity() * sqrt(dt_summed) * sigma_ab;
```

Both bias random-walk noise blocks are written to the **same rows 0–2**, while
the function allocates **six** noise rows (`6 + state->U_.rows()`) and leaves
rows 3–5 zero. Computing `P = UᵀU` (invariant under the QR that follows):

- the 6×6 bias noise block has **rank 3 instead of 6** — singular;
- a spurious cross-covariance appears between `b_g` and `b_a`
  (`sqrt(dt)² · σ_wb · σ_ab`, r ≈ 0.044 per update) although the two random
  walks are physically independent;
- the diagonal variances are unaffected, which is what makes it invisible.

The same augmentation is performed **correctly** ~100 lines away in
`UpdaterZeroVelocity.cpp`, where `Q_bias_sqrt` is a full-rank 6×6 block-diagonal
matrix (`block(0,0,3,3)` gyro, `block(3,3,3,3)` accel). The two implementations
of the same computation disagreed. Fix: `block(0, ba_id, ...)` →
`block(3, ba_id, ...)`.

**Honesty note:** we applied this fix *before* the guard fix, and it alone did
**not** stop the divergence. We have not isolated whether it is necessary in
addition to the guard fix, only that it is independently wrong and that the
correct form is already present elsewhere in the package. Both fixes are in
place in the run reported above.
