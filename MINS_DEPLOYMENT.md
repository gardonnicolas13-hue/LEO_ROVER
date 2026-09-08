# MINS deployment on the LEO Rover

Working reference for building, deploying and configuring MINS on this
platform, and the sensor/estimator data requested by Mike.

Written in English on purpose: this file is a handoff document. An earlier
handoff archive went out with French README files, which forced the recipient
to translate the very material meant to save them time.

- **Last verified:** 2026-09-07, robot stationary, full chain reporting GO.
  §5 frequencies re-measured live that day; unchanged from 2026-09-06 within
  normal jitter. The ROS1 fix in §4 is now built and deployed, not just
  described — see §4.2 for the verification.
- **Upstream:** [rpng/MINS](https://github.com/rpng/MINS), package version 1.0.0.
- **Upstream commit we forked from:** [À COMPLÉTER PAR NICOLAS] — the tree
  ships as an unpacked `MINS-master` archive with no `.git`, so the exact
  commit cannot be recovered from the working copy.

---

## 1. Where MINS actually runs

This trips people up on first contact, so it comes first.

**MINS runs on the ground-station PC, not on the robot.** The Raspberry Pi 4B
publishes sensors; the PC subscribes and estimates.

```
Raspberry Pi 4B (robot)                    Ground station PC
  CORE2 board  → /firmware/imu               imu_sanitizer → /imu/data_clean
  D455         → /camera/infra{1,2}/…        republish     → /pc/camera/infra{1,2}/…
  encoders     → /joint_states               wheel_remap   → /joint_states_mins
                                             MINS (subscribe) → /mins/…
                                             pose_selector    → /robot_pose_fused
  roscore lives HERE ─────────────────────►  ROS_MASTER_URI points at the robot
```

Two consequences worth internalising:

- The ROS master is on the **robot**. `tools/robot_env.sh` centralises
  `ROBOT_HOST` / `ROS_MASTER_URI` / `ROS_IP`; source it before any `rostopic`
  command or you will query a master that is not there.
- MINS consumes `/pc/camera/...`, a **local raw republish** of the Pi's
  JPEG-compressed stream. Never point it at the robot's raw `/camera/...`
  topics across the network: two 848×480 raw streams at 30 Hz saturate the
  link.

---

## 2. Installation and deployment

### 2.1 Prerequisites

- Ubuntu 20.04, ROS Noetic (**ROS 1** — the ROS2 path is not used here)
- `catkin_tools` (`catkin build`, not `catkin_make`)
- Eigen 3, OpenCV 4, Boost, Ceres
- Bundled under `MINS-master/thirdparty/`: `open_vins`, `libnabo`,
  `libpointmatcher` — no separate install needed

### 2.2 Source

```bash
git clone https://github.com/rpng/MINS
```

On this machine the tree lives at
`LEO_Rover_Navigation_System/MINS-master/`, and `mins_ws/src/MINS` is a
**symlink** to it. Editing either path edits the same files — `find` under
`mins_ws/` will not follow the symlink, which is why the source can appear to
be missing.

### 2.3 Apply our patch (required on ROS 1)

Without this, MINS starts and subscribes to nothing. See §4.

```bash
cd LEO_Rover_Navigation_System/MINS-master
git apply --check ../../patches_mins/01_ros1_entry_point_20260906.patch
git apply         ../../patches_mins/01_ros1_entry_point_20260906.patch
```

The patch is already applied in this working copy.

### 2.4 Build

```bash
cd mins_ws
catkin build mins
source devel/setup.bash
```

**Done, 2026-09-07:** `All 4 packages succeeded`, 0 warnings, 0 failed,
22.4 s. The deployed binary at
`mins_ws/devel/.private/mins/lib/mins/subscribe` moved from its previous
build date (2026-07-22 — it had sat unbuilt since the fix in §4 landed) to
today. Building alone does not touch the estimator already running in
memory; nothing was disturbed by this step, only by the later relaunch
through `tools/restart_stack.sh` that picks the new binary up.

A syntax-only check of the entry point, without producing a binary or
disturbing a running estimator:

```bash
M=LEO_Rover_Navigation_System/MINS-master
g++ -fsyntax-only -std=c++17 -DROS_AVAILABLE=1 \
    -I"$M/mins/src" -I"$M/thirdparty/open_vins/ov_core/src" \
    -I"$M/thirdparty/open_vins/ov_init/src" \
    -I/opt/ros/noetic/include -I/usr/include/eigen3 -I/usr/include/opencv4 \
    "$M/mins/src/run_subscribe.cpp"
```

### 2.5 Run

Do not launch MINS by hand for routine work. Use the project script, which
holds the maintenance flag for the watchdog, waits for initialisation, and
switches the pose source over:

```bash
./tools/restart_stack.sh
```

It relaunches with the known-good arguments explicitly
(`gyro_recal_period:=0 zupt_clamp_enable:=false accel_recal_period:=0`) so the
watchdog cannot silently restore different defaults.

Direct invocation, for reference:

```bash
rosrun mins subscribe $(rospack find mins)/config/leo/config.yaml
```

### 2.6 Preflight

```bash
python3 tools/preflight_mins.py
```

Returns GO/NO-GO over the whole chain: network, battery (≥ 10.3 V), IMU
liveness and |a| at rest, both cameras, wheels and stationarity, MINS
publishing, `pose_selector` offset, and a 30 s static-drift measurement
(criterion < 30 mm).

**Restart order matters:** `imu_sanitizer` → MINS → `pose_selector`, then
allow ~25 s to converge. Restarting the sanitizer alone leaves MINS on a stale
initialisation.

---

## 3. MINS tuning and parameters

This chapter documents *how* the inertial front-end was characterised and is
maintained. The systematic dump of every configuration value lives in §6; what
follows is the reasoning behind the ones that were not left at their defaults,
and the procedure for keeping them valid over time.

### 3.1 The inertial source: the CORE2 body IMU

MINS propagates from exactly one IMU, and on this platform that is the unit on
the **CORE2 control board** — a sensor physically separate from the camera,
mounted on the chassis. It is not, and has never been, the RealSense D455's
internal motion module.

The signal reaches MINS through three stages:

```
/firmware/imu        raw, from the CORE2 firmware over the serial link
      ↓              firmware_message_converter
/imu/data_raw        standard sensor_msgs/Imu, still unfiltered
      ↓              leo_navigation/imu_sanitizer.py
/imu/data_clean      spike-free and rescaled — what MINS subscribes to
```

**MINS must be pointed at `/imu/data_clean`, never at `/imu/data_raw`.** The
CORE2 unit emits rare single-sample glitches — magnitudes up to 5 × 10¹⁶ m/s²
have been observed — and a single such sample is enough to diverge any
inertial estimator irrecoverably. The sanitizer exists to remove them.

A second consumer exists in parallel: `/imu/data_vins` carries the same signal
on a separate branch for the openVINS and sqrtVINS estimators, so that their
scale factor can be recalibrated without disturbing the stream MINS depends
on. The two branches are deliberately independent.

### 3.2 Noise characterisation by Allan variance

The four noise densities in `config_imu.yaml` were **measured on this specific
unit** using `imu_utils` (Allan-variance characterisation, Y. Turki), not
taken from a datasheet and not left at MINS's defaults:

| Parameter | Meaning | Value | Units |
|---|---|---|---|
| `accel_noise` | accelerometer white noise (σ_a) | 2.3314541632569977e-02 | m/s²/√Hz |
| `accel_bias` | accelerometer bias diffusion | 8.9292060085650762e-04 | m/s³/√Hz |
| `gyro_noise` | gyroscope white noise (σ_g) | 1.2636973145788728e-03 | rad/s/√Hz |
| `gyro_bias` | gyroscope bias diffusion | 3.3010020064503652e-05 | rad/s²/√Hz |

The YAML file is the authoritative copy; the table above and the one in §6.2
are both derived views of it. If these values are ever re-measured, update the
YAML and treat both tables as stale until corrected.

Allan variance was chosen over datasheet figures deliberately: datasheet noise
figures describe a part number under laboratory conditions, whereas the filter
needs the noise of *this* unit as installed, including whatever the mounting
and the vehicle's electrical environment contribute.

### 3.3 The `accel_scale` correction, and why it is re-measured by hand

The CORE2 accelerometer carries a **multiplicative scale error** that drifts
over a timescale of weeks. Left uncorrected it is the dominant inertial error
on this platform — far larger than the noise the densities above describe.

The correction is a single scalar applied by `imu_sanitizer.py`:

```
accel_scale = g_local / |a|_measured_at_rest
```

with `g_local = 9.790 m/s²` (the value also set as `gravity_mag` in
`config_estimator.yaml` — the two must agree). Two independent scale factors
are maintained, both currently **1.1297**: `~accel_scale` for
`/imu/data_clean` (MINS) and `~accel_scale_vins` for `/imu/data_vins` (the
VINS branches), declared in `navigation_supervision.launch`.

**Procedure for re-measurement**, to be run with the robot stationary:

1. Confirm the robot is genuinely at rest — a moving vehicle invalidates the
   measurement silently, since the estimate assumes the only acceleration is
   gravity.
2. Measure the mean accelerometer magnitude at rest on the **raw** branch. On
   the corrected branch `|a|` should already read ≈ 9.790; reading the
   corrected stream to derive a new correction would compound the old one.
3. Compute the new factor as `9.790 / |a|_raw`.
4. Update `accel_scale` (and `accel_scale_vins`) in
   `navigation_supervision.launch`, then relaunch through
   `tools/restart_stack.sh`.
5. Verify with `tools/preflight_mins.py`: the `|a| at rest` check should read
   within ±0.02 m/s² of 9.790.

For reference, the factor was 1.1186 (from a raw reading of 8.7525 m/s²) and
is now 1.1297 (from 8.6642 m/s²) — about a 1 % sensor drift over several
weeks, which is the cadence at which this should be revisited.

**Automatic recalibration exists in the sanitizer but is deliberately
disabled.** `imu_sanitizer.py` implements a periodic self-recalibration
(`~accel_recal_period`), and `tools/restart_stack.sh` explicitly passes
`accel_recal_period:=0` on every launch to keep it off. It was enabled once,
for openVINS's benefit, and degraded MINS — which is already anchored by its
wheel odometry and did not need it. The deployed configuration is the
known-good point; the explicit argument in the relaunch script exists so that
a watchdog restart cannot silently restore a different default.

### 3.4 Verifying a tuning change

No tuning change is considered done until `tools/preflight_mins.py` reports
GO. It checks the whole chain — network, battery, IMU liveness and `|a|` at
rest, both cameras, wheel data and stationarity, MINS publishing, the
`pose_selector` offset — and finishes with a 30-second static-drift
measurement against a 30 mm criterion.

Two rules that this project learned the hard way:

- **Measure over a 30–60 s window, never a single sample.** A one-shot
  position reading cannot distinguish a stable offset from an estimator that
  is actively running away; only the drift over a window can.
- **Re-check immediately before recording data**, not once per session. A GO
  at the start of a session does not guarantee a GO an hour later.

### 3.5 Online extrinsic calibration, and why it is frozen

`do_calib_ext` was set to **`false`** for both the camera and the wheel on
2026-09-07. This is the fix for a divergence that had recurred four times over
2026-09-06/07: MINS drifting to tens of metres with the robot demonstrably
stationary, every input measurably clean.

**The mechanism.** With the robot at rest there is no excitation, and a
camera–IMU or wheel–IMU translation extrinsic is **unobservable** in that
condition. `do_calib_ext: true` nonetheless keeps estimating it, so it
random-walks in exactly the directions the data cannot constrain. Given a
wrong lever arm, the wheel measurement transforms incorrectly into the IMU
frame, disagrees with the inertial prediction, and is rejected by the χ² gate
— removing the only thing anchoring X/Y. The filter then drifts freely while
the wheels report exactly zero velocity, which is the signature that made this
confusing: every input looks perfect, and the estimator still runs away.

**Measured, on a healthy (non-diverged) instance, robot stationary, 75 s:**

| | t = 0 | t = 75 s | drift |
|---|---|---|---|
| cam0 x | 0.2113 | 0.2096 | −1.7 mm |
| cam0 y | 0.0373 | 0.0359 | −1.4 mm |
| cam0 z | 0.0935 | 0.0946 | +1.1 mm |

Monotonic in every axis, roughly **1.4 mm/min**, with no sign of settling —
about 8 cm per hour of standing still.

**An interpretation trap worth recording, because it nearly derailed the
diagnosis.** Two *diverged* instances had converged to almost identical
extrinsics (cam0 y = −0.098 and −0.0981), which looks exactly like a genuine
calibration converging on a true value — and would argue for leaving online
refinement enabled. It was not. The *healthy* instance sat at y = +0.037,
13.5 cm away. The matching values were the far end of the same random walk
after divergence, not convergence. Reading only diverged instances would have
produced the opposite, wrong conclusion.

**Why this also justified freezing the camera, not just the wheel.** The wheel
extrinsic is what breaks the X/Y anchor, so it was the obvious candidate. But
the camera drifts on the same mechanism, and the consequence is quantifiable:
with f ≈ 337 px and features at ~2 m, a 10 cm lateral extrinsic error produces
337 × 0.10 / 2.0 ≈ **17 px** of reprojection error, far outside the χ² gate
(`chi2_mult: 1`). Even 3 cm gives ~5 px. Leaving the camera free would have
traded position divergence for visual-feature rejection.

**Verified after the change:** extrinsics pinned exactly at their seeds,
**0.00 mm of movement over 100 s** (against 1.4 mm/min before), chain GO.

**The accepted cost.** Freezing makes the tape-measure seed permanent. That
raises the priority of the outstanding `kalibr_calibrate_imu_camera` run: seed
quality now sets steady-state accuracy instead of merely seeding an online
estimate that would correct it.

**When to revisit.** If the platform is ever operated continuously in motion,
the extrinsics become observable again and online refinement regains its
value. The freeze is the right call for a robot that spends long periods
stationary, which is how this one is actually used.

**Still to confirm.** Freezing provably stops the extrinsic drift. It does not
*yet* prove the divergence is gone — that needs several hours of uninterrupted
running to demonstrate, since the failures appeared only after extended
uptime.

---

## 4. The ROS 1 entry-point fix

### 4.1 The defect

`mins/src/run_subscribe.cpp` implements the ROS2 path completely —
`rclcpp::init`, node creation, `get_parameter("config_path")`,
`ROS2Publisher`/`ROS2Subscriber`, multi-threaded executor — but the
`#elif ROS_AVAILABLE == 1` branch **did not exist**, even though the file
included `core/ROSPublisher.h` and `core/ROSSubscriber.h`.

On ROS 1, `rosrun mins subscribe` therefore loaded the config, printed
`op->load_print()`'s summary, and **exited immediately without subscribing to
anything**. No error, no crash, exit code 0.

The gap dates from the ROS2 port; the ROS1 branch was never written back. It
was found and fixed independently here (2026-08-31) and by Mike (2026-09-04) —
**one upstream bug, not two diverging versions of MINS**.

### 4.2 The fix

Two parts. First, instantiate the ROS1 path mirroring the ROS2 one:
`ros::init`, `ros::NodeHandle`, `ROSPublisher`/`ROSSubscriber`, `ros::spin()`.

Second, resolve the config path the way openVINS does — MINS derives from
openVINS, and `run_subscribe_msckf.cpp` carries the canonical form:

```cpp
auto nh = std::make_shared<ros::NodeHandle>("~");
nh->param<std::string>("config_path", config_path, config_path);
```

ROS parameter first, `argv[1]` as fallback. Two details are easy to get wrong
and fail silently:

**The node handle must be private (`"~"`).** roslaunch places a `<param>`
declared inside a `<node>` tag in that node's **private** namespace. A global
handle would look up `/config_path`, never find it, and fall back to `argv[1]`
forever — a fix that looks right and does nothing.

This is safe for every topic here, and it was **checked rather than assumed**:
`ROSPublisher` advertises absolute names (`/mins/imu/odom`, …) and every topic
in `config/leo/*.yaml` is absolute as well. A leading `/` makes a name immune
to the handle's namespace. *If a relative topic name is ever added to that
config, it will be remapped under `/mins_subscribe/`.*

**The default passed to `param()` is `config_path`, not `argv[1]`.** Both
express the same intent, because `config_path` already holds `argv[1]` at that
point. But passing `argv[1]` literally is undefined behaviour when
`argc == 1`: the C standard guarantees `argv[argc] == NULL`, so `argv[1]` is a
null pointer there, not a string.

**Verified empirically, 2026-09-07**, not just read off the code. The exact
precedence block was isolated into a standalone test binary (same two lines,
nothing from MINS pulled in, so it cannot touch any topic the live estimator
uses) and run against this robot's real `roscore` under throwaway node names,
never the production `mins_subscribe` name — reusing that name would have
evicted the running node, since ROS enforces one registration per name:

| Case | Argument | Private ROS param | Result | Confirms |
|---|---|---|---|---|
| 1 | none | none | default value, no crash | robustness |
| 2 | `argv[1]` only | none | `argv[1]` | fallback works |
| 3 | `argv[1]` + param set | set | **the param wins** | correct precedence |
| 4 | same, but a **global** handle | set | `argv[1]` (param not found) | **why `"~"` is required**, not merely a style choice |

Case 4 is the one worth dwelling on: it reproduces the exact original bug on
demand, on this robot, confirming the private-namespace explanation above
isn't a theory — a global handle really does miss a parameter declared inside
a `<node>` tag, every time.

### 4.3 Consequence for our launch files

`catkin_ws/src/leo_navigation/launch/mins.launch` passes the config through
`args="…"` precisely because `<param name="config_path">` used to fail
silently — with `<param>`, `argv[1]` became roslaunch's own auto-appended
`__name:=mins_subscribe` and MINS reported "unable to open the configuration
file!" for that literal string. Both routes now work. `args=` is left in place;
there is no urgency to switch.

### 4.4 Known gap, deliberately not fixed

The ROS1 path does **not** call `parser->set_node_handler(nh)`, while the ROS2
path calls `parser->set_node(node)` and openVINS calls the ROS1 equivalent. So
under ROS 1, YAML values cannot be overridden by ROS parameters. Fixing it
would change deployed behaviour — ROS params would suddenly take precedence
over the YAML — which is beyond the scope of an entry-point fix.

---

## 5. Data requested by Mike

All frequencies below were **measured live**, robot stationary, full stack
running — not copied from nominal values, which have diverged from the
measured ones before on this platform. Re-measured 2026-09-07 (originally
2026-09-06); figures are stable within a few percent of normal jitter, which
is the update shown below.

### 5.1 Sensor frequencies and message structures

**Inputs consumed by MINS:**

| Topic | ROS message type | Configured | Measured (2026-09-07) |
|---|---|---|---|
| `/imu/data_clean` | `sensor_msgs/Imu` | — | **82.99 Hz** |
| `/pc/camera/infra1/image_rect_raw` | `sensor_msgs/Image` | 640×480 @ 15 | **15.00 Hz** |
| `/pc/camera/infra2/image_rect_raw` | `sensor_msgs/Image` | 640×480 @ 15 | **15.00 Hz** |
| `/joint_states_mins` | `sensor_msgs/JointState` | — | **19.50 Hz** |
| `/mins/external_ref/carolus` | `geometry_msgs/PoseStamped` | `enabled: false` | not published (no beacon in view, confirmed via `/tag_detections: []`) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | `enabled: false` | not published |
| `/lidar/points` | `sensor_msgs/PointCloud2` | `enabled: false` | not published |

**Outputs published by MINS:**

| Topic | ROS message type | Measured (2026-09-07) |
|---|---|---|
| `/mins/imu/odom` | `nav_msgs/Odometry` | **88.78 Hz** |
| `/mins/imu/pose` | `geometry_msgs/PoseWithCovarianceStamped` | **8.34 Hz** |
| `/mins/imu/path` | `nav_msgs/Path` | — |
| `/mins/cam/msckf` | `sensor_msgs/PointCloud2` | — |

Three things about the inputs that are not visible from the topic names:

- **`/imu/data_clean`, never `/imu/data_raw`.** The CORE2 IMU emits rare
  single-sample glitches (up to 5 × 10¹⁶ m/s²) that diverge any inertial
  estimator. `leo_navigation/imu_sanitizer.py` republishes a spike-free
  stream.
- **`/joint_states_mins`, never `/joint_states`.** MINS reads
  `velocity[0]`/`velocity[1]` **by index**, and this rover's `/joint_states`
  orders wheels `[FL, RL, FR, RR]` — so `velocity[1]` is the *rear* left wheel.
  `wheel_remap.py` looks the two front wheels up by name and republishes them
  in the order MINS assumes.
- **`sub_topics` is mandatory.** `ROSSubscriber::callback_wheel` discards every
  message whose `name.at(0)` is not in that list, and the default
  (`front_left_wheel_joint`) does not match this rover's joint names. Omit it
  and messages arrive — visible on `rostopic hz` — while MINS's internal
  wheel stack silently stays empty forever. It looks exactly like "no wheel
  data", and is really a name mismatch.

### 5.2 The frequency at which fusion happens

There is no single number, and quoting one would be misleading. Four distinct
rates are in play:

| Mechanism | Rate | Set by |
|---|---|---|
| IMU propagation | ~87 Hz | IMU message rate |
| Camera (MSCKF) update | 15 Hz | image rate |
| Wheel update | ~19.7 Hz | `/joint_states_mins` rate |
| State cloning | **4–30 Hz, motion-dependent** | `dynamic_cloning` |

**Cloning is dynamic and this matters.** `config_estimator.yaml` sets
`clone_freq: 20`, but with `dynamic_cloning: true` that value is only a seed:
`SystemManager.cpp:283` reads it, then `SystemManager.cpp:285` **overwrites**
it. `dynamic_cloning()` walks the available clone rates
`{4, 5, 6, 7, 9, 10, 15, 20, 25, 30}` from low to high and returns the *first*
whose interpolation error falls under `intr_error_ori_thr` (0.007) and
`intr_error_pos_thr` (0.003); if none qualifies it falls back to the highest.
The error tables live in `config_estimator.yaml` under `intr_ori` / `intr_pos`.

So the effective clone rate is low when the platform moves smoothly and rises
with angular/linear acceleration. It is an **accuracy** mechanism whose side
effect is efficiency, not the reverse. `window_size: 1.0` s and
`intr_order: 3`.

### 5.3 How the covariances were determined

Provenance differs per sensor, and the weakest link is stated explicitly.

| Quantity | How it was obtained |
|---|---|
| **IMU noise densities** | **Allan variance**, measured on *this* rover's IMU with `imu_utils` (Y. Turki's calibration). Not datasheet, not defaults. |
| **Camera intrinsics** | `kalibr_calibrate_cameras`. Reprojection error 0.20–0.23 px at a 90.2 mm baseline. Frozen (`do_calib_int: false`). |
| **Camera–IMU extrinsic** | ⚠️ **Tape-measure seed, now FROZEN** (`do_calib_ext: false` since 2026-09-07) — *not* a completed `kalibr_calibrate_imu_camera` solution, and no longer refined online. See §3.5. Completing that Kalibr run is now higher priority: the seed sets steady-state accuracy rather than just initialisation. |
| **Wheel noise** | **Empirically tuned from field measurement**, not datasheet (see below). |
| **Wheel intrinsics** | Firmware `rosparam /firmware/diff_drive` (radius 0.0625 m, separation 0.358 m), radius re-measured 0.0622 m over 20 windows, IQR ± 0.0005. |
| **Wheel–IMU extrinsic** | Hand-measured seed with an `Rz(π)` correction, refined online. |
| **Initial covariances** (`init_cov_*`) | MINS defaults, not independently derived. |

Measured IMU values, `config_imu.yaml`
(accel in m/s²/√Hz and m/s³/√Hz, gyro in rad/s/√Hz and rad/s²/√Hz):

```yaml
accel_noise: 2.3314541632569977e-02     # acc_n, white noise
accel_bias:  8.9292060085650762e-04     # acc_w, bias diffusion
gyro_noise:  1.2636973145788728e-03     # gyr_n, white noise
gyro_bias:   3.3010020064503652e-05     # gyr_w, bias diffusion
```

Calibration report reference/date: [À COMPLÉTER PAR NICOLAS]

**The wheel noise values are the interesting case**, because they encode a
measured physical fact rather than a sensor spec:

- `noise_w: 0.8` (raised from 0.2, 2026-07-10). On rigid wheels the rover
  **skids longitudinally** in turns instead of yawing: measured **3.8 rad/s of
  wheel differential for 0.001 rad/s of real yaw** (heading taken from the
  camera). Wheel-derived yaw is close to fiction during turns, so confidence in
  it is deliberately loosened.
- `noise_v: 0.2` (tightened from 0.5, 2026-07-13). The linear encoders are
  excellent. During camera dropouts the filter coasts on inertia and the χ²
  gate would eventually reject the very wheel measurements (v = 0) able to
  anchor it; a tighter velocity confidence kills the drift before runaway.
- `noise_p: 0.1`.
- `do_calib_int: false` — online intrinsic refinement **diverged live**
  (2026-07-06, radii drifted to negative values after motion bursts). Frozen.

χ² gates: wheel `chi2_mult: 5` (15 let slips through, 1 would reject normal
skid-steer turns), camera `chi2_mult: 1`.

### 5.4 Coordinate transformation from the global reference frame

MINS publishes `global` → `imu`, plus `imu` → `cam0`/`cam1` for the online
extrinsics.

**`global` is not a surveyed ground frame.** Its origin is the pose at which
the filter initialised. With `imu_gravity_aligned: true`, roll and pitch are
observable from the accelerometer, so the frame is gravity-aligned — but **yaw
and position are arbitrary**, fixed by wherever the rover happened to be when
initialisation completed.

**No external global anchor is currently active.** All three candidates are
disabled: `config_vicon.yaml` (`enabled: false`, intended to carry the Carolus/
AprilTag beacon fix on `/mins/external_ref/carolus`), `config_gps.yaml`, and
`config_lidar.yaml`.

> ⚠️ **The vicon route does not work in live mode as MINS stands, and this is
> not a configuration problem.** `ROSSubscriber` registers exactly five
> subscriptions — IMU, camera, wheel, GPS, lidar (`ROSSubscriber.cpp`
> lines 52, 69/79, 87, 94, 102). **There is no vicon subscription**, in the
> ROS1 *or* the ROS2 subscriber. `SystemManager::feed_measurement_vicon()` is
> called only from `run_simulation.cpp:135` and `run_bag.cpp:188`, and the
> converter `ROSHelper::PoseStamped2Data` is likewise only reached from bag
> playback.
>
> So setting `enabled: true` would allocate vicon extrinsic state and start
> publishing `/mins/vicon0/pose`, while **no measurement would ever reach the
> filter** — the failure would look like "the anchor has no effect", with no
> error anywhere. Verified 2026-09-06 by reading the source, not inferred from
> behaviour.
>
> This is the same shape of defect as the ROS1 entry point in §4: a path that
> exists for bags and simulation but was never wired into `run_subscribe`.
> Anchoring to a ground frame live therefore requires **implementing a vicon
> callback** in `ROSSubscriber` (the conversion helper already exists and can
> be reused as-is), not merely flipping a flag and putting a beacon in view.

Downstream, `pose_selector.py` applies an SE(3) correction so the fused output
stays continuous when the pose source is switched between estimators. At the
moment of a switch:

```
correction[new] = T_last_fused · inverse(T_new_raw)
```

and thereafter every message is published as:

```
T_fused = correction[source] · T_raw
```

This only ever re-anchors the parent/world frame. It makes the output
continuous across a source change; it does **not** register the estimate to any
externally surveyed frame, and it does not absorb the jump when MINS itself
respawns and re-initialises at a fresh origin.

TF note: MINS's transforms are remapped onto a dedicated `/tf_mins` channel.
Published on `/tf` they would steal the `imu` frame from the URDF (a TF child
has exactly one parent) and fight openVINS's own `global → imu`.

---

## 6. Full parameter reference

Configuration root: `mins/config/leo/config.yaml`, which includes the eight
files below. Values as deployed on 2026-09-06.

### 6.1 `config_estimator.yaml`

| Parameter | Value | Note |
|---|---|---|
| `gravity_mag` | 9.790 | local gravity |
| `clone_freq` | 20 | **seed only** — overwritten while `dynamic_cloning` is true |
| `window_size` | 1.0 | s |
| `intr_order` | 3 | interpolation order |
| `intr_error_mlt` | 3 | |
| `intr_error_ori_thr` | 0.007 | dynamic-cloning orientation threshold |
| `intr_error_pos_thr` | 0.003 | dynamic-cloning position threshold |
| `intr_error_thr_mlt` | 0.5 | |
| `dt_extrapolation` | 0.01 | |
| `use_imu_res` | true | |
| `use_imu_cov` | false | |
| `use_pol_cov` | true | |
| `dynamic_cloning` | true | |

Plus the `intr_ori` / `intr_pos` interpolation-error tables, keyed by candidate
clone rate (4, 5, 6, 7, 9, 10, 15, 20, 25, 30 Hz).

### 6.2 `config_imu.yaml`

| Parameter | Value |
|---|---|
| `accel_noise` | 2.3314541632569977e-02 |
| `accel_bias` | 8.9292060085650762e-04 |
| `gyro_noise` | 1.2636973145788728e-03 |
| `gyro_bias` | 3.3010020064503652e-05 |
| `topic` | `/imu/data_clean` |

### 6.3 `config_camera.yaml`

| Parameter | Value | Note |
|---|---|---|
| `enabled` | true | |
| `max_n` | 2 | stereo |
| `n_pts` | 300 | reduced from 1500 (2026-07-07 density audit) |
| `chi2_mult` | 1 | |
| `do_calib_ext` | **false** | frozen 2026-09-07 — drifted when stationary, see §3.5 |
| `do_calib_int` | false | intrinsics frozen, from Kalibr |
| `do_calib_dt` | true | time offset refined online |
| `resolution` | 640 × 480 | both cameras |
| `timeoffset` | 0.0 | both cameras |
| `cam0.topic` | `/pc/camera/infra1/image_rect_raw` | |
| `cam1.topic` | `/pc/camera/infra2/image_rect_raw` | |

`cam0` `T_imu_cam` as deployed — note this is the tape-measure seed, and that
the rotation matters far more than the translation (a 90° error is outside the
filter's recoverable region; centimetres of translation are not):

```
[ 0.0,  0.0, 1.0, 0.15]
[-1.0,  0.0, 0.0, 0.00]
[ 0.0, -1.0, 0.0, 0.10]
[ 0.0,  0.0, 0.0, 1.00]
```

Distortion `radtan`; intrinsics and coefficients per camera in the file.

### 6.4 `config_wheel.yaml`

| Parameter | Value | Note |
|---|---|---|
| `enabled` | true | |
| `type` | `Wheel2DAng` | differential-drive from two angular velocities |
| `noise_w` | 0.8 | raised — measured skid, see §5.3 |
| `noise_v` | 0.2 | tightened — encoders are excellent |
| `noise_p` | 0.1 | |
| `chi2_mult` | 5 | slip rejection gate |
| `intrinsics` | [0.0622, 0.0622, 0.358] | left r, right r, base (m) |
| `do_calib_int` | false | online refinement diverged, frozen |
| `do_calib_ext` | **false** | frozen 2026-09-07 — see §3.5 |
| `do_calib_dt` | false | |
| `timeoffset` | 0.0 | |
| `topic` | `/joint_states_mins` | **not** `/joint_states` |
| `sub_topics` | `wheel_FL_joint, wheel_FR_joint` | mandatory, see §5.1 |
| `init_cov_dt` / `ex_o` / `ex_p` / `in_b` / `in_r` | 1e-4 / 1e-4 / 1e-3 / 1e-3 / 1e-3 | |

`T_imu_wheel` carries the `Rz(π)` correction found on 2026-07-08 — the
transmission is mounted 180° round, so the firmware wheel frame points away
from the camera/IMU. Without it every wheel measurement contradicted the
inertial prediction and χ² rejected them silently, losing the X/Y anchor:

```
[-1.0,  0.0, 0.0,  0.00]
[ 0.0, -1.0, 0.0,  0.00]
[ 0.0,  0.0, 1.0, -0.03]
[ 0.0,  0.0, 0.0,  1.00]
```

**A planar constraint with a consequence worth knowing:** `Wheel2DAng` has
∂h/∂z ≡ 0. The wheels anchor X and Y and give *zero* information on Z — only
the camera constrains altitude. Check Z after any change to camera geometry;
it has fallen to −25.9 m while X/Y stayed perfect, with no alarm raised.

### 6.5 `config_init.yaml`

| Parameter | Value |
|---|---|
| `window_time` | 2.0 |
| `imu_thresh` | 0.1 |
| `imu_wheel_thresh` | 0.1 |
| `imu_only_init` | false |
| `imu_gravity_aligned` | true |
| `use_gt` / `use_gt_gnss` / `use_gt_lidar` | false |
| `cov_size` | 1e-2 |

Initialisation is the single most sensitive step on this platform. An
initialisation disturbed by hardware being handled produced **772 m/min of
vertical drift with the robot stationary**; an untouched relaunch, same binary
and same config, gave 0.02 m/min. The trap is that the IMU looked healthy —
|a| within 0.7 % of local gravity. It was the *direction* of the vector that
betrayed the misalignment, not its norm.

### 6.6 `config_system.yaml`

| Parameter | Value |
|---|---|
| `verbosity` | 1 |
| `save_timing` | true → `outputs/leo/timing.txt` |
| `save_state` | false |
| `save_trajectory` | true → `outputs/leo/traj.txt` |
| `save_prints` | false |
| `exp_id` | 0 |

### 6.7 Disabled sensors

`config_vicon.yaml`, `config_gps.yaml`, `config_lidar.yaml` — all
`enabled: false`. The vicon slot is reserved: `/mins/external_ref/carolus` is
the private MINS beacon channel and nothing else may publish on it.

Before planning any work on the vicon/beacon anchor, read the warning in §5.4:
MINS has **no vicon subscription in live mode**, so enabling it changes nothing
until a callback is implemented in `ROSSubscriber`.

---

## 7. Operational traps

Each of these cost real time on this project.

- **`rostopic hz | grep` returns "Terminated" on a perfectly healthy topic.**
  Python buffers when stdout is not a TTY. Run unpiped, redirect to a file,
  then read the file.
- **A subscriber waiting on a topic with no publisher never complains.** This
  is how the cockpit's artificial horizon sat dead for weeks on
  `/camera/imu/data_raw`, a topic that has never had a publisher.
- **`accel_scale` drifts over weeks.** The CORE2 accelerometer reads about
  8.66 m/s² where it should read 9.790. Measure |a| at rest and re-apply the
  factor before suspecting extrinsics or the gyro.
- **Measure drift over 60 s, not 5 s.** A transient can read as 0.5 mm while
  the honest 60 s figure is 1677 mm with 20 m excursions.
- **The watchdog can silently restore defaults.** Always relaunch through
  `tools/restart_stack.sh`, which passes the known-good arguments explicitly.
- **MINS respawns rather than being `required`.** An estimator abort must never
  tear down the stack — driving must not depend on estimator health. On
  respawn MINS re-initialises at a fresh origin and `/mins/imu/odom` jumps;
  the `pose_selector` correction does *not* absorb that, since it only
  corrects on a source switch.
- **MINS can diverge spontaneously after hours of healthy running, with every
  input clean.** Seen three times on 2026-09-06/07 alone, stationary, network
  and IMU and cameras and wheels all reporting healthy — the fault is internal
  to MINS, root cause not identified. Symptom: static drift climbing past the
  30 mm preflight threshold, sometimes tens of metres. **Remedy, confirmed
  each time:** `tools/restart_stack.sh` on a confirmed-stationary robot, then
  re-run `tools/preflight_mins.py`. One respawn has occasionally not held (one
  case re-diverged 107 s later); a second respawn always has. **Practical
  consequence for a measurement campaign:** run preflight immediately before
  recording, not just once at the start of a session — the gap between "GO at
  session start" and "still GO an hour later" is not guaranteed.
- **Never judge MINS on a single position snapshot.** A one-shot reading can
  look plausible while the estimator is mid-divergence — only a 30–60 s window
  (`preflight_mins.py`'s own measurement) distinguishes a stable offset from an
  active runaway.
