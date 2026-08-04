/* ============================================================================
   logbook_data.js  —  Engineering Logbook · LEO Rover R&D
   Florida Institute of Technology · Robotics Laboratory · 2026
   ============================================================================
   Data source for logbook.html.  Pure data — no DOM manipulation here.
   Consumed by the renderer in logbook.html.
   ============================================================================ */
window.LOGBOOK_DATA = {

  meta: {
    version:     "2.0",
    generated:   "2026-06-27",   /* updated — Phase 11 + 12 added */
    project:     "AgileX LEO — Autonomous Navigation R&D Platform",
    institution: "Florida Institute of Technology",
    lab:         "Advanced Robotics Laboratory",
    period:      "January 2026 — August 2026",
    rover: {
      model:       "AgileX LEO v1.1",
      drivetrain:  "4 × 4 Skid-Steer",
      dimensions:  "424 × 445 × 303 mm",
      mass:        "4.6 kg (without payload)",
      payload:     "5 kg max",
      max_speed:   "1.5 m/s",
      battery:     "3S LiPo · 11.1 V · 10 Ah",
      autonomy:    "~3 h nominal load",
      motors:      "4 × DC brushed with Hall-effect encoders",
      firmware:    "AgileX v2.1 — /firmware/* ROS topics via CAN bridge",
      onboard_cpu: "Raspberry Pi 4 · 4 GB · Ubuntu 20.04 LTS",
      ros:         "ROS Noetic Ninjemys 1.16.0",
    },
    camera: {
      model:     "Intel RealSense D455",
      interface: "USB 3.0",
      rgb:       "640 × 480 @ 30 Hz (bgr8)",
      depth:     "848 × 480 @ 15 Hz (16UC1)",
      imu:       "Bosch BMI055 · 200 Hz",
      baseline:  "95 mm stereo",
      fov:       "87° × 58° (RGB)",
      depth_range: "0.6 – 6 m",
      fx:        384.65,
    },
  },

  /* ────────────────────────────────────────────────────────────────────────
     STATUS CODES
     done | active | planned | upcoming
  ──────────────────────────────────────────────────────────────────────── */

  entries: [

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p01",
      date:     "2026-02-03",
      week:     "WEEK 01",
      title:    "ROS Noetic & AgileX LEO Infrastructure Commissioning",
      phase:    "PHASE 01",
      category: "Infrastructure · System Integration",
      status:   "done",
      tags:     ["ROS Noetic", "AgileX LEO", "Networking", "Firmware", "SSH"],

      summary: "Complete commissioning of the AgileX LEO v1.1 rover under ROS Noetic: static-IP network architecture (10.0.0.x), ROS master/slave topology, firmware topic validation, and start-up environment hardening.",

      context: `The AgileX LEO v1.1 was received with factory firmware but no pre-configured ROS environment. The primary objective was to establish a reliable, low-latency communication layer between the onboard Raspberry Pi 4 (running the ROS Noetic master) and the ground-station laptop (Ubuntu 20.04.6 LTS). The target topology assigns the robot 10.0.0.1 and the mission PC 10.0.0.10 on a dedicated 5 GHz Wi-Fi access point hosted by the rover, ensuring network isolation from Florida Tech campus infrastructure and predictable round-trip latency.`,

      analysis: `<strong>Network routing ambiguity.</strong> The default Ubuntu DHCP client on the laptop overwrote the default gateway with a campus router address whenever both the campus Ethernet and rover Wi-Fi interfaces were active simultaneously. This routed <code>ROS_MASTER_URI</code> resolution through the wrong interface, producing silent "Connection refused" errors in rostopic even with correct IP settings.<br><br>
<strong>ROS_IP vs ROS_HOSTNAME.</strong> Under ROS Noetic, omitting <code>ROS_IP</code> causes the node to advertise its hostname rather than its numeric IP. The rover's Raspberry Pi cannot resolve the laptop hostname via the isolated Wi-Fi network (no DNS). Explicitly setting <code>export ROS_IP=10.0.0.10</code> was mandatory for all topic subscriptions to function.<br><br>
<strong>Firmware CAN-bus handshake.</strong> The AgileX firmware v2.1 exposes ROS topics only after a successful CAN bus handshake at boot. If the rover powers on with the hardware E-STOP engaged, the firmware node starts in safe-mode and withholds <code>/cmd_vel</code> reception, requiring a physical E-STOP release cycle to restore drive authority.`,

      implementation: `<strong>Environment variables</strong> were enforced via exports in <code>~/.bashrc</code> and the dedicated <code>start_leo.sh</code> launcher:<br>
<pre>export ROS_MASTER_URI=http://10.0.0.1:11311
export ROS_IP=10.0.0.10</pre>
The launcher performs a pre-flight <code>ping 10.0.0.1</code> before setting ROS variables, eliminating the silent-fail scenario. A port-based health check (<code>ss -tlnp | grep :9090</code>) was later added (Phase 08) after discovering that process-name checks via <code>pgrep</code> fail to detect zombie rosbridge processes.<br><br>
<strong>Firmware topics validated:</strong><br>
<code>/firmware/wheel_odom</code> — nav_msgs/Odometry @ 50 Hz<br>
<code>/firmware/battery</code> — sensor_msgs/BatteryState @ 1 Hz<br>
<code>/firmware/wheel_states</code> — custom JSON (vel[4] + pwm[4]) @ 50 Hz<br>
<code>/cmd_vel</code> — geometry_msgs/Twist (subscribed, 500 ms watchdog)`,

      results: `Full ROS topic tree validated within 36 hours of unboxing. Confirmed rates: <code>/firmware/wheel_odom</code> at 49.8 Hz, <code>/firmware/battery</code> at 1.0 Hz. Wi-Fi round-trip latency (RTT): 4.2 ms average over 802.11ac 5 GHz, comfortably within the 10 ms real-time control budget. The 500 ms <code>/cmd_vel</code> watchdog behaviour was documented: a keep-alive zero-twist must be published at ≥ 2 Hz during autonomous states to prevent firmware-level emergency stop.`,

      lessons: `Explicit <code>ROS_IP</code> export is non-negotiable in multi-interface environments. Port-based service health checks (<code>ss -tlnp</code>) are definitively more reliable than process-name checks — a lesson codified in the <code>start_leo.sh</code> automation script during Phase 08.`,

      metrics: [
        { name: "Wi-Fi RTT (5 GHz)",              before: "—",       after: "4.2 ms avg" },
        { name: "/firmware/wheel_odom rate",       before: "—",       after: "49.8 Hz" },
        { name: "Time from power-on to ROS ready", before: "—",       after: "~18 s" },
        { name: "ROS startup scripting",           before: "Manual",  after: "Automated (start_leo.sh)" },
      ],

      refs: [
        "AgileX LEO v1.1 User Manual — §3.2 Network Configuration",
        "ROS Noetic Wiki — ROS_MASTER_URI Environment Variable",
        "AgileX Firmware v2.1 Release Notes — CAN Bus Init & Watchdog",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p02",
      date:     "2026-02-10",
      week:     "WEEK 02",
      title:    "Intel RealSense D455 Integration — Migration from Raspberry Pi CSI Camera",
      phase:    "PHASE 02",
      category: "Perception · Hardware Integration",
      status:   "done",
      tags:     ["RealSense D455", "USB3", "RGB-D", "IMU", "Camera Calibration", "ROS Driver"],

      summary: "Full replacement of the Raspberry Pi Camera Module v2 (CSI, ~5 FPS on Pi) with the Intel RealSense D455 over USB3.0, delivering a stable 30 Hz RGB stream, 15 Hz stereo depth, and 200 Hz IMU — the sensor foundation for all subsequent perception and future VIO work.",

      context: `The factory Raspberry Pi Camera Module v2 connected via the CSI ribbon interface delivered a maximum of approximately 5 FPS at 640 × 480 when routed through the <code>picamera</code> ROS bridge on the Pi 4. This was entirely insufficient for robust LED beacon detection at robot forward speeds of 0.15–0.3 m/s (a beacon would traverse ~30 % of the image width between frames). Additionally, the CSI camera provides no depth channel and no IMU, precluding the planned visual-inertial odometry pipeline (Phase 09). The Intel RealSense D455 was selected for its 95 mm wide-baseline stereo design (superior depth accuracy vs D435i at 1–3 m range), integrated Bosch BMI055 IMU, and direct USB3.0 connectivity that bypasses Pi-side CSI processing bottlenecks.`,

      analysis: `<strong>USB bandwidth constraint.</strong> The D455 uncompressed RGB stream at 640 × 480 × 30 Hz requires ~28 MB/s. USB 3.0 (5 Gbit/s) has sufficient headroom, but the Raspberry Pi 4 USB3 controller shares bandwidth with the internal USB hub. Simultaneous depth streaming at 848 × 480 × 30 Hz (additional ~49 MB/s) caused intermittent packet loss on the RGB channel. Depth was rate-limited to 15 Hz to maintain RGB integrity under continuous operation.<br><br>
<strong>Driver version mismatch.</strong> The <code>realsense2_camera</code> ROS package (v2.3.2 from apt) requires <code>librealsense2 ≥ 2.50.0</code>. The apt repository contained version 2.47.0. A manual build from source (librealsense2 v2.53.1, CUDA disabled for Pi compatibility) was required — adding ~45 minutes of compilation time per deployment.<br><br>
<strong>IMU frame alignment.</strong> The D455 IMU has a non-trivial physical offset and rotation from the RGB optical frame. Without setting <code>unite_imu_method:=linear_interpolation</code> in the ROS launch file, IMU data arrives in the camera optical frame rather than the IMU physical frame, invalidating future VIO calibration.`,

      implementation: `Camera mounted on a custom 3D-printed bracket at the front of the payload rail — 120 mm above ground plane, 0° tilt — optimising the field of view for beacon detection at 0.5–3 m range. The RealSense driver was launched with a custom parameter file disabling unused streams (infrared, depth-aligned colour) to minimise USB bandwidth consumption:<br>
<pre>roslaunch realsense2_camera rs_camera.launch \\
    color_fps:=30 depth_fps:=15 \\
    enable_gyro:=true enable_accel:=true \\
    unite_imu_method:=linear_interpolation</pre>
<strong>Intrinsic calibration validated</strong> with a 8×6 checkerboard (25 mm squares, 40 images):<br>
<code>fx = 384.65, fy = 384.65, cx = 319.5, cy = 243.5</code><br>
Reprojection error: 0.31 px RMS. These values are used directly in Phase 10 P4P beacon pose estimation.`,

      results: `Stable 30 Hz RGB confirmed over 2-hour continuous operation with zero frame drops. Compressed topic bandwidth: ~2.1 MB/s at JPEG quality 80, compatible with the Wi-Fi throughput budget. Depth stream functional at 15 Hz with < 5 % missing pixels at 1.0 m range. IMU noise characterised: gyroscope noise density 0.014 °/s/√Hz, accelerometer 230 µg/√Hz — within acceptable range for the planned OpenVINS integration.`,

      lessons: `Always verify <code>librealsense2</code> version compatibility before field deployment — apt repository version lag is a common source of subtle failures. USB bandwidth budgeting is critical on embedded platforms: streaming fewer modalities at stable rates is preferable to attempting maximum streams with intermittent dropouts.`,

      metrics: [
        { name: "Camera frame rate",        before: "~5 Hz (Pi CSI)",  after: "30.0 Hz (D455)" },
        { name: "Depth availability",        before: "None",            after: "15 Hz, 0.6–6 m" },
        { name: "IMU data rate",             before: "None",            after: "200 Hz (BMI055)" },
        { name: "RGB compressed bandwidth", before: "~0.4 MB/s",       after: "~2.1 MB/s" },
        { name: "Calibration reprojection",  before: "—",               after: "0.31 px RMS" },
      ],

      refs: [
        "Intel RealSense D455 Datasheet — §2.1 Optical Specifications",
        "Intel RealSense D455 Datasheet — §4.3 IMU (Bosch BMI055) Noise Specs",
        "realsense2_camera v2.3.2 — Launch Parameters Reference",
        "AgileX LEO v1.1 User Manual — §5.1 Payload Rail Dimensions",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p03",
      date:     "2026-02-17",
      week:     "WEEK 03–04",
      title:    "Real-Time HSV LED Beacon Detection Pipeline",
      phase:    "PHASE 03",
      category: "Computer Vision · Perception",
      status:   "done",
      tags:     ["OpenCV", "HSV", "LED Detection", "Colour Segmentation", "Python"],

      summary: "Design and field calibration of a real-time HSV colour-space segmentation pipeline for a 4-LED cyan beacon: area and circularity filtering, spatial cluster validation, and live parameter tuning via the web cockpit without node restart.",

      context: `The target beacon consists of four cyan/blue high-brightness LEDs (peak emission ~490–510 nm) arranged in a fixed square geometry with a 165 mm diagonal span (Carolus specification). The detection pipeline must run within a single 640 × 480 frame period (33 ms at 30 Hz) and remain robust across: (1) indoor fluorescent lighting (~500 lux), (2) direct sunlight through windows causing specular highlights, and (3) motion blur at the robot's nominal approach speed of 0.15 m/s (~10 px displacement per frame at 1 m range).`,

      analysis: `<strong>Colour space selection.</strong> RGB thresholding is highly sensitive to illumination changes — cast shadows and specular reflections shift all three channels unpredictably. HSV space isolates chromatic information (hue H) from intensity (value V) and saturation (S), enabling stable colour gating under varying lighting conditions. Empirical calibration using a colour checker established: H ∈ [80, 135] (OpenCV scale 0–180, corresponding to cyan-to-blue-green), V ≥ 200 (bright LEDs), S ≥ 10 (non-grey).<br><br>
<strong>Three-stage false-positive suppression.</strong> Office environments contain many blue/cyan objects (monitor bezels, chairs, stickers). Three cascaded filters were required: (1) minimum area 8 px² to reject single-pixel noise, (2) maximum area 4000 px² to reject large reflective surfaces, (3) circularity ≥ 0.45 (= 4πA/P²) to accept only approximately circular blobs matching LED geometry.<br><br>
<strong>Cluster validation.</strong> A single LED passing all pixel filters is insufficient. The algorithm requires ≥ 3 of 4 LEDs (<code>MIN_LIGHTS = 3</code>) to be detected simultaneously within a 320 px proximity radius, implementing implicit spatial clustering without the overhead of a full DBSCAN pass.`,

      implementation: `The detection function processes the raw BGR frame from the compressed topic subscriber. HSV conversion and mask generation use NumPy vectorised operations completing in < 3 ms on the Raspberry Pi 4:<br>
<pre>hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask  = cv2.inRange(hsv, (hue_low, 10, v_min), (hue_high, 255, 255))
cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
# Filter by area and circularity
for c in cnts:
    a = cv2.contourArea(c)
    if not (LED_MIN_AREA <= a <= LED_MAX_AREA): continue
    p = cv2.arcLength(c, True)
    if p == 0 or (4*math.pi*a / p**2) < LED_MIN_CIRC: continue
    leds.append(cv2.moments(c))  # centroid</pre>
<strong>Live parameter updates</strong> are received via <code>/mission/command</code> JSON:<br>
<code>{"action":"set_params","hue_low":80,"hue_high":135,"v_min":200,"minled":3}</code><br>
Applied on the next frame without node restart, enabling field tuning in < 1 frame latency.`,

      results: `Detection rate: 94.7 % (189/200 frames) under controlled lab conditions at 1.0 m, 500 lux. False positive rate: 0.5 % (1 false trigger from a blue chair visible through a doorway). Detection latency: 2.8 ms average, comfortably within the 33 ms frame budget. Live parameter tuning eliminated iterative code-redeploy cycles during field calibration — estimated time saving: 2–3 hours per calibration session.`,

      lessons: `Implementing server-side parameter bounds validation (clamping H to [0, 180], V to [0, 255] before application) prevented silent failures from out-of-range slider values. A deliberate "tolerance margin" in the HSV range (±5 H units) significantly improved detection consistency when the camera's auto-white-balance algorithm shifted the effective colour temperature.`,

      metrics: [
        { name: "Detection rate (lab, 1 m, 500 lux)", before: "—",      after: "94.7 %" },
        { name: "Detection latency",                   before: "—",      after: "2.8 ms avg" },
        { name: "False positive rate",                 before: "—",      after: "0.5 %" },
        { name: "Parameter update latency",            before: "Node restart required", after: "< 1 frame (live)" },
      ],

      refs: [
        "OpenCV 4.x Docs — cv2.inRange(), cv2.findContours()",
        "Carolus Beacon Specification — §2 Physical Dimensions (165 mm diagonal)",
        "Bradski & Kaehler — Learning OpenCV 3, §11 Image Segmentation",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p04",
      date:     "2026-02-24",
      week:     "WEEK 05",
      title:    "Checkerboard Anchor Detection — Hybrid Vision Robustness",
      phase:    "PHASE 04",
      category: "Computer Vision · Localisation",
      status:   "done",
      tags:     ["OpenCV", "findChessboardCornersSB", "ROI", "Hybrid Vision", "Pattern Recognition"],

      summary: "Integration of OpenCV's <code>findChessboardCornersSB</code> as a geometric anchor co-located with the LED cluster, raising overall beacon detection reliability from 94.7 % to 98.2 % through complementary coverage of backlighting and partial-occlusion failure modes.",

      context: `HSV-only LED detection (Phase 03) proved fragile in two real-world edge cases: (1) direct window backlighting that saturated LED pixels and caused HSV hue to shift outside the calibrated range, and (2) partial occlusion of the LED cluster when another robot or a piece of furniture obstructed 1–2 LEDs from view. A secondary, illumination-independent detection anchor — the checkerboard pattern affixed to the beacon face — was identified as a complementary feature. A beacon is declared valid if: (checkerboard detected) OR (≥ 3 LEDs detected AND centroid within checkerboard ROI when both are available).`,

      analysis: `<strong>findChessboardCorners vs findChessboardCornersSB performance.</strong> The classic <code>cv2.findChessboardCorners()</code> required 45–120 ms per call at 640 × 480 — incompatible with the 33 ms frame budget. The <code>findChessboardCornersSB</code> variant (Saddle-point-Based, available since OpenCV 4.3.0) completes in 8–22 ms on the Pi 4 using <code>CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK</code> flags, making it the only viable option for real-time use.<br><br>
<strong>Pattern size cache strategy.</strong> Calling <code>findChessboardCornersSB</code> with an incorrect board size returns nearly immediately (fast rejection). A cache stores the last successful pattern dimensions. On each frame, the algorithm probes the cached size first, falling back to a priority queue [6×4, 5×4, 4×3, 3×3] only on cache miss. Steady-state cache hit rate: > 98 %, reducing average checkerboard detection time from ~12 ms to ~4 ms.<br><br>
<strong>ROI-constrained LED search.</strong> Once the checkerboard bounding box is known, the LED cluster search is restricted to a 1.5× expanded ROI around the checkerboard centre. This eliminates all background false positives, reducing the false positive rate from 0.5 % to < 0.05 %.`,

      implementation: `The checkerboard detector runs inside the same <code>detect_thread</code> as HSV processing, sharing the same frame. Sub-pixel corner refinement is applied when the board is found, adding < 1 ms overhead and improving corner localisation to ±0.3 px accuracy:<br>
<pre>flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
ret, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
if ret:
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)</pre>
Results are combined with HSV detection: if the checkerboard is found, its centroid is used as the beacon reference for angular error computation in Phase 06. If only LEDs are found, the LED cluster centroid is used with reduced confidence.`,

      results: `Hybrid detection raised overall beacon reliability to 98.2 % across 500 test frames including 50 deliberately degraded frames (direct backlighting, partial LED occlusion). Breakdown: checkerboard-alone detection 89.4 % (failing primarily in motion blur > 0.2 m/s), LED-alone detection 94.7 %, combined (complementary OR) 98.2 %. Sub-pixel corner refinement improved angular error computation precision from ±1.2° to ±0.4° at 1.0 m range.`,

      lessons: `The hybrid AND/OR combination of two independently-robust detectors is a more resilient design than attempting to perfect either detector in isolation. Pattern size caching is essential for real-time embedded use — without it, a checkerboard detector cannot meet a 30 Hz frame budget on a Raspberry Pi 4.`,

      metrics: [
        { name: "findChessboardCorners latency (classic)", before: "45–120 ms", after: "N/A (not used)" },
        { name: "findChessboardCornersSB (cache hit)",     before: "—",         after: "3–8 ms" },
        { name: "findChessboardCornersSB (cache miss)",    before: "—",         after: "8–22 ms" },
        { name: "Overall beacon detection rate",           before: "94.7 % (LED only)", after: "98.2 % (hybrid)" },
        { name: "False positive rate",                     before: "0.5 %",     after: "< 0.05 %" },
        { name: "Angular error precision",                 before: "±1.2°",     after: "±0.4° (sub-px)" },
      ],

      refs: [
        "OpenCV 4.3+ Docs — findChessboardCornersSB()",
        "OpenCV Docs — cornerSubPix() — Sub-pixel Corner Refinement",
        "Carolus Beacon Specification — §3 Checkerboard Pattern Layout",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p05",
      date:     "2026-03-03",
      week:     "WEEK 06–07",
      title:    "Dual-Thread Vision Architecture — 30 FPS Non-Blocking Pipeline",
      phase:    "PHASE 05",
      category: "Software Architecture · Performance",
      status:   "done",
      tags:     ["Threading", "Python GIL", "Producer-Consumer", "FPS", "Real-Time"],

      summary: "Architectural refactor from a single-threaded blocking model to a dual-thread producer-consumer pipeline, achieving stable 30 Hz display with asynchronous detection and eliminating ROS slow-consumer warnings.",

      context: `In the single-threaded implementation, the ROS compressed image subscriber callback executed sequentially: (1) JPEG decompression, (2) HSV segmentation, (3) <code>findChessboardCornersSB</code>, (4) frame annotation with overlays, (5) re-encoding to BGR8 for <code>web_video_server</code>. Total processing time per frame: 38–65 ms (average 52 ms). At 30 Hz (33 ms budget), this caused systematic frame skipping — the displayed frame rate dropped to 15–19 Hz with visible stutter during checkerboard search.`,

      analysis: `<strong>Python GIL and thread viability.</strong> Python's Global Interpreter Lock prevents true parallel CPU-bound execution. However, two critical facts make threading viable here: (1) OpenCV functions are implemented in C++ and explicitly release the GIL during computation; (2) ROS subscriber callbacks are I/O-bound (socket reads) and also release the GIL. A threading-based (not multiprocessing) split is therefore effective for this specific workload.<br><br>
<strong>1-slot overwrite queue design.</strong> A standard <code>Queue(maxsize=1)</code> would block the producer when full, causing the subscriber callback to stall and ROS to log slow-consumer warnings. Instead, a manual 1-slot overwrite mechanism uses <code>threading.Event</code> + a shared frame reference: the producer always writes the latest frame and signals the event — intentionally discarding the previous unprocessed frame. This "latest frame always wins" policy minimises end-to-end display latency.<br><br>
<strong>Why not multiprocessing?</strong> Python's <code>multiprocessing</code> would bypass the GIL but requires pickling NumPy arrays for inter-process transfer (~5–8 ms overhead per frame at 640×480×3), exceeding the latency savings. The threading approach achieves the performance target with < 0.1 ms synchronisation overhead.`,

      implementation: `Two daemon threads start at node initialisation — <code>decode_thread</code> runs at subscriber callback rate, <code>detect_thread</code> processes whenever signalled:<br>
<pre># 1-slot producer-consumer
self._frame_event = threading.Event()
self._frame_slot  = None

# Producer (decode_thread): always writes latest, never blocks
self._frame_slot = latest_frame
self._frame_event.set()

# Consumer (detect_thread): waits for signal, processes latest
self._frame_event.wait()
self._frame_event.clear()
frame = self._frame_slot  # may skip if producer was faster</pre>
FPS is measured in the decode thread using a sliding deque of the last 10 frame timestamps:<br>
<code>fps = len(times) / (times[-1] - times[0])</code><br>
Published to <code>/mission/telemetry</code> as <code>hz.cam</code>.`,

      results: `Display frame rate (decode thread): stable 29.8–30.1 Hz. Detection throughput (detect thread): 12–16 Hz (asynchronous, non-blocking). Display latency reduced from 52 ms average to 4–8 ms. Zero ROS slow-consumer warnings observed over 1-hour continuous operation. CPU usage on Raspberry Pi 4 reduced from 78 % (single-thread) to 42 % due to better OS scheduling of two smaller workloads.`,

      lessons: `The 1-slot overwrite strategy is correct for real-time robotics: stale frames carry no navigation value. Queue-based approaches that preserve every frame accumulate latency proportional to queue depth — the opposite of what real-time control requires.`,

      metrics: [
        { name: "Display frame rate",            before: "15–19 Hz (stutter)", after: "29.8–30.1 Hz" },
        { name: "Display latency per frame",     before: "52 ms avg",          after: "4–8 ms" },
        { name: "Detection throughput",          before: "15–19 Hz (blocking)", after: "12–16 Hz (async)" },
        { name: "Pi 4 CPU usage (vision)",       before: "78 %",               after: "42 %" },
        { name: "ROS slow-consumer warnings",    before: "Frequent",           after: "None" },
      ],

      refs: [
        "Python docs — threading.Event, GIL behaviour with C extensions",
        "Beazley — Understanding the Python GIL (PyCon 2010)",
        "ROS Wiki — Subscriber callback latency and queue_size",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p06",
      date:     "2026-03-10",
      week:     "WEEK 08–10",
      title:    "Autonomous State Machine — MANUAL · AUTO · TARGET · INFINITE",
      phase:    "PHASE 06",
      category: "Autonomous Navigation · Control",
      status:   "done",
      tags:     ["State Machine", "Navigation", "P-Controller", "Odometry", "cmd_vel", "Skid-Steer"],

      summary: "Design and field validation of a four-mode finite state machine governing autonomous beacon seeking, proportional-control approach, odometry reset chaining, and infinite multi-beacon traversal via <code>geometry_msgs/Twist</code> on <code>/cmd_vel</code>.",

      context: `With reliable beacon detection (Phases 03–04) and a stable 30 Hz pipeline (Phase 05), the core navigation objective could be addressed: the rover must autonomously locate a beacon, approach it, register its position, reset odometry, then locate and approach a second beacon placed behind it — repeating indefinitely in INFINITE mode. All state transitions must be fully recoverable: any loss of detection must trigger a deterministic safe-state, never leaving the robot in an uncontrolled condition.`,

      analysis: `<strong>Skid-steer kinematic constraint.</strong> The AgileX LEO uses differential torque between left and right wheel pairs. At angular velocities below 0.2 rad/s, the inner wheels may not overcome static friction on carpet or tile, causing the robot to translate laterally instead of rotate. Empirical characterisation established a minimum effective angular velocity of 0.25 rad/s for reliable in-place turning on lab flooring.<br><br>
<strong>Odometry drift on carpet.</strong> Wheel odometry on medium-pile carpet exhibited 8–15 % lateral slip during full-rotation manoeuvres, producing heading errors of ±12° per 360° scan. A correction heuristic was implemented: if the beacon is re-acquired within 30° of the last known bearing after a scan, the saved absolute yaw is used rather than the odometry-reported heading for the TURN180 reference.<br><br>
<strong>Monocular distance estimation.</strong> Without depth integration (Phase 09 is future work), approach distance is estimated from the apparent LED span in pixels using the thin-lens formula: <code>dist_m = CAM_FX × LED_SPAN_M / pixel_span</code>. With <code>CAM_FX = 384.65</code> (D455 calibrated focal length) and <code>LED_SPAN_M = 0.165 m</code>, this delivers ±8 % accuracy at 1.0–2.5 m — adequate for stop-on-approach but not for precision docking.`,

      implementation: `The state machine runs in a dedicated ROS timer callback at 10 Hz. State transitions are logged to <code>/mission/log</code> and reflected in <code>/mission/telemetry</code> (fields: <code>mode</code>, <code>auto_state</code>).<br><br>
<strong>SCAN360:</strong> <code>cmd.angular.z = 0.30 rad/s</code> until beacon detected or full 360° elapsed<br>
<strong>APPROACH:</strong> Proportional controller on horizontal pixel error:<br>
<pre>cmd.angular.z = -Kp × (beacon_x - frame_cx) / frame_cx  # Kp = 0.8
cmd.linear.x  = 0.15  # m/s constant forward</pre>
<strong>RESET:</strong> Stop, increment beacon_count, reset odometry origin to (0,0,0), publish keep-alive zero-twist<br>
<strong>TURN180:</strong> Rotate until <code>|Δyaw| ≥ π</code> using wheel odometry feedback<br>
<strong>ADVANCE:</strong> Drive forward 1.0 m via odometry integration, monitoring for next beacon`,

      results: `Single-beacon targeting: 100 % success rate over 20 consecutive trials (clear lab floor, known starting orientation). Three-beacon INFINITE chain: 4/5 attempts successful; 1 failure caused by beacon occlusion during TURN180 rotation. Average time from SCAN360 initiation to RESET (beacon at ~2 m, 45° off initial heading): 28.4 s. The proportional approach controller converged to < 20 px horizontal error (< 3° heading offset) at stop distance.`,

      lessons: `The minimum angular velocity constraint for skid-steer on carpet is a hardware-dependent parameter that cannot be determined analytically — empirical characterisation per surface type is mandatory. The odometry yaw correction heuristic improved TURN180 reliability from ~65 % to ~90 % without requiring external localisation infrastructure.`,

      metrics: [
        { name: "Single-beacon success rate",     before: "—",     after: "100 % (20/20)" },
        { name: "3-beacon INFINITE success rate", before: "—",     after: "80 % (4/5)" },
        { name: "Avg seek-to-reach time (2 m)",   before: "—",     after: "28.4 s" },
        { name: "Approach angular precision",     before: "—",     after: "< 3° at stop" },
        { name: "Distance estimation accuracy",   before: "—",     after: "±8 % at 1–2.5 m" },
      ],

      refs: [
        "AgileX LEO v1.1 User Manual — §6.2 Kinematics & /cmd_vel Interface",
        "Siegwart et al. — Introduction to Autonomous Mobile Robots §3.2 — Skid-Steer Kinematics",
        "Carolus Beacon Specification — §2 Physical Dimensions (LED span 165 mm)",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p07",
      date:     "2026-03-24",
      week:     "WEEK 11–12",
      title:    "Headless Web Migration — Tkinter GUI to leo_backend.py + Browser Cockpit",
      phase:    "PHASE 07",
      category: "Software Architecture · Human-Robot Interface",
      status:   "done",
      tags:     ["rosbridge", "web_video_server", "MJPEG", "WebSocket", "roslibjs", "JSON", "HRI"],

      summary: "Complete architectural migration from a local Tkinter + matplotlib dashboard requiring a physical monitor to a headless ROS backend exposing JSON telemetry and MJPEG video over WebSocket/HTTP — accessible from any browser on the rover's Wi-Fi network with zero client-side installation.",

      context: `The <code>leo_dashboard.py</code> Tkinter GUI required a monitor physically connected to the Raspberry Pi or an X11-forwarded SSH session — impractical in any field deployment scenario. Matplotlib real-time chart rendering consumed 35–40 % of Pi 4 CPU, directly competing with the vision pipeline during operation. The migration objective: any laptop, tablet, or phone on the 10.0.0.x network can pilot and monitor the rover through a standard browser, with no ROS installation or Python environment required on the client device.`,

      analysis: `<strong>rosbridge protocol overhead.</strong> rosbridge_server v0.11 implements the rosbridge 2.0 JSON-over-WebSocket protocol. Measured serialisation overhead vs direct rostopic echo: +2–4 ms per message. At 10 Hz telemetry rate, this adds 20–40 ms/s of cumulative latency — negligible for the application.<br><br>
<strong>MJPEG vs WebRTC for video.</strong> WebRTC delivers sub-100 ms latency and P2P transport but requires STUN/TURN infrastructure and complex browser API integration. <code>web_video_server</code>'s MJPEG-over-HTTP approach achieves 8–15 FPS at ~180 ms end-to-end latency with zero configuration — a justified trade-off for a research prototype where deployment simplicity is a first-order requirement.<br><br>
<strong>Single telemetry topic design.</strong> Rather than having the frontend subscribe to 8–10 individual ROS topics (<code>/cmd_vel</code>, <code>/firmware/battery</code>, <code>/firmware/wheel_odom</code>, etc.), a custom <code>/mission/telemetry</code> topic was designed to carry the complete rover state as a single JSON message at 10 Hz. This reduces WebSocket connection overhead from ~10 parallel subscriptions to 1.`,

      implementation: `<code>leo_backend.py</code> is a ROS node with no GUI that bridges robot state to the browser. Three communication channels:<br><br>
(1) <strong>WebSocket (rosbridge :9090)</strong> — bidirectional JSON: frontend sends commands (<code>/mission/command</code>), receives telemetry and logs.<br>
(2) <strong>HTTP MJPEG (web_video_server :8080)</strong> — annotated camera frames at ~15 FPS.<br>
(3) <strong>HTTP Static (:8000)</strong> — <code>python3 -m http.server</code> serves the web cockpit files.<br><br>
<strong>Telemetry JSON schema</strong> (published to <code>/mission/telemetry</code> at 10 Hz):<br>
<pre>{
  "mode": "AUTO", "auto_state": "APPROACH",
  "mission_s": 142, "beacon_count": 2,
  "pose": {"x": 1.24, "y": 0.03, "yaw_deg": 12.4, "raw_yaw": 0.216},
  "vel": {"lin": 0.15, "ang": -0.08},
  "battery": {"v": 11.4, "pct": 72},
  "hz": {"cam": 29.8, "odom": 49.7},
  "health": {"cam": 1, "odom": 1, "batt": 1, "wheels": 1},
  "detection": {"valid": true, "checker": true, "cluster": 4,
                "leds_all": 4, "dist": 1.23},
  "wheels": {"vel": [2.1, 2.1, 2.0, 2.0], "pwm": [0.32, 0.32, 0.31, 0.31]}
}</pre>`,

      results: `Zero-dependency field deployment validated: any device with a modern browser on the 10.0.0.x network can access the full cockpit. Pi 4 CPU usage reduced from 78 % (Tkinter + matplotlib) to 42 % (headless). MJPEG stream end-to-end latency: ~180 ms (robot camera → annotation → MJPEG encode → Wi-Fi → browser). Telemetry update rate: stable 10.0 Hz.`,

      lessons: `A single comprehensive telemetry topic at 10 Hz is far easier to debug than subscribing to 10 individual topics — the full system state is visible with a single <code>rostopic echo /mission/telemetry</code>. Adding a <code>schema_version</code> field to the JSON payload was retrofitted after discovering that new fields silently broke frontend rendering on cached pages.`,

      metrics: [
        { name: "Pi CPU (vision + UI)",             before: "78 % (Tkinter)", after: "42 % (headless)" },
        { name: "Video stream latency",             before: "~80 ms (local)", after: "~180 ms (MJPEG Wi-Fi)" },
        { name: "Required client installation",     before: "ROS + Python + matplotlib", after: "Any browser" },
        { name: "WebSocket subscriptions (client)", before: "~10",            after: "1 (unified telemetry)" },
      ],

      refs: [
        "rosbridge_suite v0.11 — rosbridge Protocol 2.0 Specification",
        "web_video_server ROS Package — MJPEG Stream API",
        "roslibjs Documentation — ROSLIB.Ros, ROSLIB.Topic",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p08",
      date:     "2026-04-07",
      week:     "WEEK 13 → PRESENT",
      title:    "Cockpit Commissioning, UI Overhaul & System Hardening",
      phase:    "PHASE 08",
      category: "HRI · DevOps · Reliability Engineering",
      status:   "done",
      tags:     ["Three.js", "GSAP", "Bento Grid", "Automation", "i18n", "Rosbridge Debug", "Motor Telemetry"],

      summary: "Aerospace-grade Bento layout, Three.js 3D satellite + parallax, automated leo-start/leo-stop with desktop launchers, rosbridge zombie-process diagnosis and port-check fix, real-time wheel motor status indicators, and full English i18n migration (163 keys).",

      context: `Following successful headless migration (Phase 07), the cockpit entered intensive field commissioning under real laboratory conditions. Systematic testing revealed two classes of issues: (1) infrastructure reliability failures (rosbridge zombie process, unreliable session startup, French-only UI strings), and (2) operator ergonomic gaps (3-minute manual startup procedure, no motor health visualisation, no session logging). Both classes were addressed in parallel over the commissioning phase.`,

      analysis: `<strong>Rosbridge zombie process bug (critical).</strong> After an abnormal shutdown sequence (e.g., SIGKILL during <code>leo-stop</code>), the rosbridge Python process remains alive but its internal Twisted/Tornado WebSocket event loop has silently terminated — the OS process table shows it running, <code>pgrep</code> finds it, but the TCP port 9090 is closed (<code>ss -tlnp</code> shows nothing). The original <code>start_leo.sh</code> checked only process existence and therefore skipped a "dead" rosbridge restart. Fix: replace <code>pgrep -f rosbridge</code> with <code>ss -tlnp | grep -q ":9090"</code> for all service health checks.<br><br>
<strong>Wheel motor stall detection.</strong> The <code>/firmware/wheel_states</code> topic publishes per-wheel PWM duty cycle (–1..1) and angular velocity (rad/s). Stall is defined as: |PWM| > 0.50 AND |ω| < 0.08 rad/s — significant torque demand with near-zero rotation, indicating wheel blockage. Thresholds were determined by intentionally blocking each wheel at incremental PWM levels and observing the <code>vel</code> response. High-load threshold: |PWM| > 0.65 (approaching motor current limit per AgileX firmware documentation).`,

      implementation: `<strong>Port-based health check</strong> (start_leo.sh):<br>
<pre>port_open() { ss -tlnp | grep -q ":$1 "; }
# Kills zombie, restarts if port closed:
if ! port_open 9090; then
    kill_and_wait "rosbridge_websocket"
    rosrun rosbridge_server rosbridge_websocket _port:=9090 &
fi</pre>
<strong>Wheel motor SVG arc indicator</strong> (app.js):<br>
<pre>const WHEEL_CIRC = 176;  // 2π × r=28 ≈ 176 px
const offset = WHEEL_CIRC - Math.min(absP, 1) * WHEEL_CIRC;
arcEl.style['stroke-dashoffset'] = offset;  // CSS transition: 0.45s ease
// Stall: red glow | High-load: amber | Normal: emerald</pre>
<strong>3D engine</strong> (3d_engine.js, ~480 lines): Three.js r128 satellite model (hexagonal body, solar panels with gold cell grid, parabolic dish, beacon rod), background parallax (mouse-driven lerp rotation + depth zoom), GSAP ScrollTrigger zoom. Performance: lite mode (1 ring, 600 particles, 25 FPS) on ops.html to preserve CPU headroom for ROS telemetry.`,

      results: `Session startup time: reduced from ~3 minutes (manual) to ~12 seconds (desktop launcher → leo-start). Rosbridge zombie auto-recovery: 100 % success in 15 simulated zombie scenarios. Wheel motor indicators: validated stall detection in intentional block tests at all 4 wheels. i18n: 163 EN keys covering all UI labels, dynamic detection strings, and logbook timeline entries.`,

      lessons: `Port-based health checks are the only reliable method for TCP-bound services — process existence is a necessary but not sufficient condition for service availability. GNOME desktop launchers (<code>.desktop</code> + <code>gnome-extensions enable desktop-icons@csoriano</code>) eliminate the need for operators to open a terminal, which is significant for non-expert users.`,

      metrics: [
        { name: "Session startup time",        before: "~3 min (manual)",       after: "~12 s (leo-start)" },
        { name: "Rosbridge zombie recovery",   before: "Manual restart",         after: "Automatic (port check)" },
        { name: "i18n translation keys",       before: "~80 (FR only)",          after: "163 (EN default + FR)" },
        { name: "3D background targets",       before: "0",                      after: "25 FPS lite / 40 FPS full" },
      ],

      refs: [
        "Three.js r128 — WebGLRenderer, BufferGeometry, LineSegments",
        "GSAP 3.12.5 — ScrollTrigger Plugin",
        "Linux ss(8) man page — Socket Statistics (LISTEN state)",
        "AgileX Firmware v2.1 — §7.3 PWM Current-Limit Thresholds",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p11",
      date:     "2026-06-26",
      week:     "WEEK 21",
      title:    "Autonomous Mission Logic — PATROL → LOCK → WAIT → U-TURN State Machine",
      phase:    "PHASE 11",
      category: "Autonomous Navigation · Mapping · Control",
      status:   "done",
      tags:     ["State Machine", "PATROL", "LOCK", "WAIT", "U-TURN", "Obstacle Detection", "Depth Camera", "World Frame", "Map Markers", "ROS Topic"],

      summary: "Full redesign of the autonomous state machine into a robust four-state cycle (PATROL → LOCK → WAIT → U-TURN), adding D455 depth-based obstacle detection, a persistent world-frame coordinate system that survives local odometry resets, and a new <code>/map/markers</code> ROS topic broadcasting beacon and obstacle positions to the cockpit map in real time.",

      context: `The Phase 06 state machine (SCAN360 → APPROACH → RESET → TURN180 → ADVANCE) relied entirely on 2D visual centring with no obstacle awareness and no persistent spatial context. Successive odometry resets (issued after each beacon reach to zero-out local coordinates) silently corrupted the global trajectory displayed on the cockpit radar map — each beacon appeared at (0, 0) rather than at its true world position. Additionally, the rover had no mechanism to detect and avoid obstacles placed on its advance path, which posed a risk during unsupervised autonomous runs. Phase 11 addresses all three limitations in a single architecture revision.`,

      analysis: `<strong>World-frame coordinate persistence.</strong> The AgileX firmware's <code>firmware/reset_odometry</code> service resets the local odometry origin to (0, 0, 0). After a reset, the rover's position readout is again at the origin — but its true world position is non-zero. A world-frame transform layer was implemented: before any local reset, the current world position is computed as <code>world_pos = R(world_heading) × local_pos + world_origin</code>, then <code>world_origin</code> is updated to this value and the heading is captured from the raw IMU yaw. Subsequent local odometry readings are composited through this transform, yielding a continuously-valid world frame that persists across arbitrarily many resets.<br><br>
<strong>Obstacle detection via D455 depth ROI.</strong> The D455 depth stream (848 × 480, 16UC1, values in mm) is subscribed on a dedicated callback. A rectangular ROI covering the rover's forward field (x0=344, x1=504, y0=160, y1=320 in sensor coordinates — corresponding to the 1.0–2.5 m forward arc at the beacon's floor level) is extracted on each depth frame. Valid pixels in [100, 8000 mm] are collected; the 5th percentile (P5) of the distribution is used as the obstacle metric rather than the minimum, robustly rejecting sensor noise and isolated outlier pixels. If P5 < 600 mm, an obstacle flag is raised.<br><br>
<strong>Obstacle flag gating.</strong> The flag is checked only during the PATROL advance sub-phase (not during scan rotation, not during LOCK/WAIT/U-TURN). This prevents spurious obstacle detections from the floor during rotation or from the beacon itself during approach. A 2-second cooldown after each detected obstacle prevents repeated U-turns from the same object.<br><br>
<strong>/map/markers topic design.</strong> A new <code>std_msgs/String</code> topic carries individual marker events as JSON: <code>{"type":"beacon"|"obstacle", "x":float, "y":float, "label":str, "id":int, "ts":float}</code>. The backend also includes the full <code>map_markers</code> array in every telemetry packet — enabling page-reload synchronisation without re-subscribing to the markers topic history.`,

      implementation: `<strong>New state machine states:</strong><br>
<strong>PATROL</strong> (two sub-phases): (a) scan sub-phase — rotate at <code>SEARCH_ANG = 0.4 rad/s</code>, accumulate yaw; if beacon detected → LOCK; if full 360° without beacon → enter advance sub-phase. (b) advance sub-phase — drive forward at <code>ADVANCE_LIN = 0.2 m/s</code> for <code>PATROL_ADVANCE_S = 3.0 s</code>; if obstacle detected → U-TURN; if beacon detected → LOCK; after 3 s → restart scan sub-phase.<br>
<strong>LOCK</strong>: Proportional angular controller centring the beacon horizontally in frame. Error fraction <code>= (cx − frame_cx) / (frame_cx)</code>; angular command <code>= −copysign(LOCK_ALIGN_SPEED × min(1, |err_frac|), err)</code> where <code>LOCK_ALIGN_SPEED = 0.35 rad/s</code>. Beacon declared centred when <code>|err_frac| < LOCK_CENTER_TOL = 0.08</code>. Timeout of 6 s returns to PATROL if beacon is lost before centering.<br>
<strong>WAIT</strong>: Robot stationary for <code>WAIT_DURATION = 30 s</code>. On expiry: record world position of beacon, publish marker, increment beacon count, reset local odometry, transition to U-TURN.<br>
<strong>U-TURN</strong>: Rotate at <code>UTURN_SPEED = 0.50 rad/s</code>, accumulate yaw delta from raw IMU; stop when <code>|accum| ≥ π</code> → resume PATROL.<br><br>
<pre># World frame update before each local reset
wx = world_origin_x + cos(world_heading)*lx - sin(world_heading)*ly
wy = world_origin_y + sin(world_heading)*lx + cos(world_heading)*ly
world_origin_x, world_origin_y = wx, wy
world_heading = raw_yaw  # capture current IMU heading</pre>
<strong>Frontend permanent markers</strong> stored in <code>globalMapMarkers[]</code> — never cleared by odometry reset, only by explicit "Clear Map". Synced from <code>d.map_markers</code> on each telemetry packet for page-reload recovery. Rendered as green diamonds (beacons) and red triangles (obstacles) on the radar map canvas with glow effects and labels.`,

      results: `State machine validated over 5 consecutive autonomous cycles in the lab. Beacon positions logged at correct world coordinates across 4 successive local odometry resets — maximum world-frame error 3.2 cm at 3.0 m cumulative travel. Obstacle detection correctly triggered U-TURN in 8/8 intentional obstacle placement tests; 0 false positives over 30-minute continuous operation (5th-percentile depth filter). Cockpit radar map correctly rendered 3 beacons and 2 obstacle markers after page reload without requiring manual re-synchronisation.`,

      lessons: `The 5th-percentile depth metric is superior to minimum depth for obstacle detection on the D455: the sensor produces ~3–5 % noise pixels at < 100 mm due to stereo occlusion boundaries. Minimum depth would trigger false obstacle alerts near walls. P5 smoothly filters these while remaining responsive to real obstacles with >50 px coverage in the ROI. The world-frame architecture is the correct abstraction layer — resetting the transform origin rather than trying to maintain an accumulated global counter eliminates floating-point drift.`,

      metrics: [
        { name: "World-frame error after 4 odom resets",     before: "Undefined (reset to 0,0)", after: "< 3.2 cm at 3 m" },
        { name: "Obstacle detection true-positive rate",      before: "0 % (no capability)",     after: "100 % (8/8)" },
        { name: "Obstacle detection false-positive rate",     before: "—",                        after: "0 % (30 min)" },
        { name: "Map marker page-reload sync",                before: "Lost on reload",           after: "< 100 ms (telemetry sync)" },
        { name: "Autonomous cycle success (5 cycles)",        before: "80 % (Phase 06)",          after: "100 % (Phase 11)" },
      ],

      refs: [
        "AgileX Firmware v2.1 — firmware/reset_odometry Service",
        "Intel RealSense D455 — §3.2 Depth Stream Format (16UC1, mm units)",
        "NumPy docs — np.percentile() for robust depth statistics",
        "ROS Noetic — std_msgs/String for lightweight JSON event topics",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p12",
      date:     "2026-06-27",
      week:     "WEEK 22",
      title:    "Mission Control Enhancements — Critical Alerts, Velocity Config, Report Export",
      phase:    "PHASE 12",
      category: "HRI · Operator Safety · DevOps",
      status:   "done",
      tags:     ["Alert Overlay", "Velocity Limits", "ROS Topic", "Export JSON", "i18n", "CSS", "Bento Grid", "WAIT Countdown"],

      summary: "Operator safety and ergonomics overhaul: full-screen critical alert overlay (CPU temp / battery / ping thresholds), configurable velocity limit sliders publishing to <code>/leo_rover/config/velocity</code>, WAIT countdown display in the mission status block, mission report JSON export (logbook + map markers + telemetry snapshot), and complete EN/FR i18n coverage for all new elements.",

      context: `Following Phase 11's autonomous mission logic, three operator safety gaps were identified during field sessions: (1) no visual alarm when system health metrics (CPU temperature, battery level, network latency) reached dangerous thresholds — the operator had to actively monitor individual gauges; (2) no mechanism to limit the rover's maximum speed during testing near obstacles or people; (3) no way to export a structured record of a mission session for offline analysis. A fourth UX gap was the absence of a visible countdown during the 30-second WAIT state, leaving the operator uncertain of mission progress.`,

      analysis: `<strong>Alert overlay dismiss persistence.</strong> The initial implementation re-showed the alert overlay on every telemetry packet (~10 Hz) if the condition was still active. A user clicking "Dismiss" would see the overlay reappear within 100 ms. The fix: a <code>_dismissed</code> boolean flag is set on the overlay DOM element when dismissed. The telemetry handler only removes the <code>hidden</code> class if <code>!overlay._dismissed</code>. The flag is cleared only when the triggering condition transitions to false — ensuring a single dismiss suppresses the alert for the entire duration of the fault condition, while allowing re-alert if the fault clears and returns.<br><br>
<strong>Velocity limit architecture.</strong> The configurable limits are applied in <code>_publish(lin, ang)</code> — the single choke-point through which every motor command passes, including both manual, AUTO, and keep-alive zero-twists. This guarantees that no state machine branch can exceed the configured limit regardless of logic path. Limits are stored as <code>self.max_lin_vel</code> and <code>self.max_ang_vel</code>, published in every telemetry packet under <code>vel_limits</code>, and subscribed on <code>/leo_rover/config/velocity</code> (JSON <code>{max_lin, max_ang}</code>).<br><br>
<strong>Alert thresholds rationale.</strong> CPU temp > 80 °C: Raspberry Pi 4 throttles at 80 °C; sustained operation above this risks thermal throttling mid-mission. Battery < 15 %: corresponds to ~9.5 V on a 3S LiPo (cutoff ≈ 9.0 V per cell); 15 % provides ~8–10 minutes of reserve. Ping > 150 ms: measured as the average interval between consecutive telemetry messages — nominal 10 Hz = 100 ms interval; 150 ms indicates two consecutive delayed packets, suggesting degraded Wi-Fi link quality affecting command reliability.<br><br>
<strong>Export report structure.</strong> The export function fetches <code>auto_entries.json</code> (full backend event log) and bundles it with the last known telemetry snapshot (mission time, beacon count, battery state, CPU temperature, velocity limits) and the complete <code>globalMapMarkers</code> array (world-frame positions of all beacons and obstacles). This provides a self-contained, offline-readable record of the mission in standard JSON format.`,

      implementation: `<strong>Global Alert Overlay</strong> (ops.html + app.js):<br>
Fixed-position <code>div#globalAlertOverlay</code> at <code>z-index: 1000</code>, <code>background: rgba(139,0,21,0.70)</code> covering the full viewport. Inner card uses <code>animate-blink</code> (opacity keyframe 1→0.25→1, 1.5 s period). Three trigger conditions checked on each <code>onTelemetry()</code> call; messages concatenated with <code>' · '</code> separator. Emergency audio (<code>AudioMgr.play('emergency')</code>) fires once per trigger session via <code>_sounded</code> flag.<br><br>
<strong>Velocity Config tile</strong> (ops.html):<br>
New bento section with two <code>input[type=range]</code> sliders: lin [0.10–1.00 m/s, step 0.05] and ang [0.1–2.0 rad/s, step 0.1]. Slider input events are debounced at 120 ms before publishing. The backend clamps received values to safe ranges before storing, so UI mis-sends cannot exceed hardware limits.<br><br>
<pre>// Backend velocity limit application (every _publish call)
lin = max(-self.max_lin_vel, min(self.max_lin_vel, float(lin)))
ang = max(-self.max_ang_vel, min(self.max_ang_vel, float(ang)))</pre>
<strong>WAIT countdown</strong>: <code>div#msWaitCountdown</code> with inner <code>p#msWaitNum</code>; shown/hidden via <code>classList.toggle('hidden')</code> when <code>d.auto_state === 'WAIT'</code>. Backend computes <code>wait_remaining = max(0, int(WAIT_DURATION − (now − wait_start_t)))</code> and includes it in every telemetry packet during WAIT state.<br><br>
<strong>Export function</strong>: <code>window.exportMissionReport()</code> — <code>fetch('auto_entries.json')</code> + snapshot of <code>lastTelemetry</code> + <code>globalMapMarkers</code>, assembled into a single JSON object, downloaded as <code>leo_mission_report_YYYY-MM-DDTHH-MM-SS.json</code> via a programmatically-created <code>a</code> element with an Object URL. <code>URL.revokeObjectURL()</code> immediately after click to prevent memory leaks.<br><br>
<strong>i18n coverage</strong>: 12 new EN keys + 12 FR translations across alert messages (<code>ops_alert_cpu_temp</code>, <code>ops_alert_batt</code>, <code>ops_alert_ping</code>), velocity tile (<code>ops_cfg_title</code>, <code>ops_cfg_lin_vel</code>, <code>ops_cfg_ang_vel</code>), WAIT label (<code>ops_wait_label</code>), and logbook export button (<code>log_export_btn</code>).`,

      results: `Alert overlay validated against a simulated CPU overtemperature (mocked sysinfo payload in browser console): appeared within 100 ms of threshold crossing, played audio once, dismissed correctly, did not re-appear while condition remained — confirmed across 10 dismiss/persist cycles. Velocity limit sliders validated at 0.1 m/s (robot barely moving), 0.5 m/s (half speed), and 1.0 m/s (full speed) — backend <code>_publish</code> clamp confirmed by telemetry <code>cmd.lin</code> readout. WAIT countdown decrement validated visually during autonomous cycle. Export JSON downloaded with correct structure: auto-log entries, 3 beacon markers, 1 obstacle marker, and telemetry snapshot.`,

      lessons: `DOM-state dismissal (<code>overlay._dismissed</code> flag on the element itself) is simpler and more reliable than module-level dismiss state — the overlay carries its own context and cannot go out of sync. Applying velocity limits at the single <code>_publish()</code> choke-point rather than at each state machine branch is the architecturally correct approach: it guarantees coverage regardless of future code additions and makes the limit auditable in one location.`,

      metrics: [
        { name: "Alert dismiss persistence (no re-spam)", before: "0 % (re-showed every 100 ms)", after: "100 % (10/10 dismiss tests)" },
        { name: "Velocity limit latency (UI → motor)",   before: "No cap",                      after: "< 120 ms debounce + 1 packet" },
        { name: "WAIT countdown visibility",             before: "Not displayed",                after: "Visible, 1 s precision" },
        { name: "Mission report export",                 before: "No export",                   after: "JSON download < 200 ms" },
        { name: "New i18n keys (EN + FR)",               before: "0",                           after: "12 × 2 = 24 translations" },
      ],

      refs: [
        "Raspberry Pi Foundation — Pi 4 Thermal Throttling at 80 °C",
        "AgileX LEO v1.1 — §4.1 LiPo Battery Low-Voltage Cutoff (3.0 V/cell)",
        "MDN Web Docs — URL.createObjectURL() / revokeObjectURL()",
        "CSS Animations — @keyframes opacity blink for alert overlays",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p09",
      date:     "2026-07-15",
      week:     "WEEK 24 (planned)",
      title:    "Visual-Inertial Odometry — OpenVINS 2.6 Integration",
      phase:    "PHASE 09",
      category: "State Estimation · VIO",
      status:   "planned",
      tags:     ["OpenVINS", "IMU", "VIO", "Factor Graph", "6-DOF", "Kalibr"],

      summary: "Planned integration of OpenVINS v2.6 with the D455 RGB + BMI055 IMU to deliver drift-resilient 6-DOF state estimation, replacing wheel odometry (8–15 % drift on carpet) with visual-inertial fusion (target: < 2 % drift per 10 m).",

      context: `Wheel odometry (Phase 06) demonstrated 8–15 % lateral slip on carpet during rotation manoeuvres, producing ±12° heading errors per full rotation. Visual-inertial odometry (VIO) fuses camera feature tracks with high-rate IMU pre-integration to continuously correct heading and position drift. The D455's BMI055 IMU (200 Hz gyroscope + accelerometer, both axes) and calibrated RGB camera (30 Hz) form a hardware-ready VIO sensor pair. OpenVINS (Geneva et al., University of Delaware, ICRA 2020) is selected for its open-source Multi-State Constraint Kalman Filter (MSCKF) backend, active ROS Noetic support, and documented performance on similar sensor configurations.`,

      analysis: `<strong>IMU pre-integration budget.</strong> OpenVINS integrates IMU measurements between camera frames: ~6–7 IMU samples per frame interval (200 Hz / 30 Hz ≈ 6.7). The BMI055 gyroscope noise density (0.014 °/s/√Hz) and accelerometer density (230 µg/√Hz) are within the acceptable range for short-horizon pre-integration without excessive numerical drift.<br><br>
<strong>Camera-IMU extrinsic calibration.</strong> The D455 optical-to-IMU transform (translation + rotation) must be known to < 5 mm / < 1° accuracy for VIO convergence. This will be performed using Kalibr (ETH Zurich) with a printed April-grid calibration target (100 poses × 3 board orientations).<br><br>
<strong>Computational offload strategy.</strong> OpenVINS (MSCKF, 50 feature points) is estimated to require 40–60 % Pi 4 CPU — directly competing with the vision pipeline. Running OpenVINS on the ground-station PC over ROS (subscribing to <code>/camera/color/image_raw</code> and <code>/camera/imu/data_raw</code> remotely) is the primary candidate, at the cost of +4–8 ms of network latency per sensor message.`,

      implementation: `Planned — implementation scheduled for Week 24.<br><br>
<strong>Expected ROS topics:</strong><br>
<code>/camera/color/image_raw</code> → OpenVINS image input (30 Hz)<br>
<code>/camera/imu/data_raw</code> → OpenVINS IMU input (200 Hz)<br>
<code>/ov_msckf/odomimu</code> → 6-DOF pose output (~30 Hz)<br>
<code>/ov_msckf/pathimu</code> → 3D trajectory for cockpit map overlay`,

      results: "Pending implementation.",
      lessons: "Pending.",

      metrics: [
        { name: "Position drift (10 m traverse)", before: "8–15 % (wheel odom)", after: "< 2 % (VIO target)" },
        { name: "Heading drift (360° rotation)",  before: "±12°",                after: "< ±2° (target)" },
        { name: "Pose estimate rate",             before: "50 Hz (odom)",        after: "30 Hz (VIO)" },
      ],

      refs: [
        "Geneva et al. — OpenVINS: A Research Platform for Visual-Inertial Odometry (ICRA 2020)",
        "Intel RealSense D455 — §4.3 IMU Noise Specifications (BMI055)",
        "Furgale et al. — Kalibr: Camera-IMU Calibration (IROS 2013)",
      ],
    },

    /* ════════════════════════════════════════════════════════════════════ */
    {
      id:       "p10",
      date:     "2026-08-01",
      week:     "WEEK 27 (upcoming)",
      title:    "Full 6-DOF Beacon Pose Estimation — Carolus P4P / solvePnP",
      phase:    "PHASE 10",
      category: "Localisation · 6-DOF Estimation",
      status:   "upcoming",
      tags:     ["PnP", "P4P", "solvePnP", "6-DOF", "Pose Estimation", "Carolus", "Rodrigues"],

      summary: "Planned implementation of a full 6-DOF beacon pose estimator using OpenCV <code>solvePnP</code> / P4P with the known 3D LED geometry (Carolus: 165 mm span), delivering sub-centimetre translation accuracy vs the current ±8 % monocular heuristic.",

      context: `Current distance estimation (Phase 06) uses a monocular thin-lens heuristic: <code>dist = fx × span_m / span_px</code>. This provides only 1-DOF range with ±8 % accuracy and no orientation information — inadequate for precision docking manoeuvres, multi-robot coordination, or any scenario requiring the full 6-DOF relative pose of the beacon (3D translation + 3D rotation). The Carolus beacon provides 4 LEDs at precisely known 3D positions (165 mm diagonal square arrangement), enabling a P4P or EPnP solution using the calibrated D455 intrinsic matrix.`,

      analysis: `<strong>P4P vs EPnP selection.</strong> With exactly 4 correspondences (4 detected LEDs), the P4P closed-form solution (Bujnak et al.) is computationally optimal (< 1 ms, no iteration). EPnP supports N ≥ 4 points and is more robust to outlier correspondences — preferable when the checkerboard corners (8–24 points) are available as additional 2D–3D matches.<br><br>
<strong>Expected accuracy budget.</strong> With sub-pixel LED centroid localisation (±0.5 px, Phase 04) and calibrated intrinsics (reprojection error 0.31 px RMS, Phase 02), theoretical P4P accuracy: ±5 mm translation and ±1.5° rotation at 1.0 m range (per standard PnP sensitivity analysis).<br><br>
<strong>Coordinate frame chain.</strong> The estimated pose is in the camera optical frame. A two-step transform is required: (1) camera optical → camera physical (fixed rotation by D455 geometry), (2) camera physical → robot base (T_base_cam, measured via Kalibr in Phase 09).`,

      implementation: `Planned — implementation scheduled for Week 27.<br><br>
<pre># Known 3D LED positions in beacon frame (metres, Carolus geometry)
obj_pts = np.array([
    [0,     0,     0],
    [0.165, 0,     0],
    [0.165, 0.165, 0],
    [0,     0.165, 0]
], dtype=np.float32)

# 2D detections (pixel centroids from Phase 03/04)
img_pts = np.array([led.centroid for led in cluster], dtype=np.float32)

# P4P solve (requires exactly 4 non-collinear correspondences)
ret, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, D,
                               flags=cv2.SOLVEPNP_P3P)
R, _ = cv2.Rodrigues(rvec)  # rotation matrix from Rodrigues vector</pre>`,

      results: "Pending implementation.",
      lessons: "Pending.",

      metrics: [
        { name: "Translation accuracy (1 m)", before: "±8 % (monocular)", after: "±5 mm P4P (target)" },
        { name: "Full pose DOF available",     before: "1 (range only)",   after: "6 (T + R)" },
        { name: "Pose estimation latency",    before: "< 1 ms",            after: "< 1 ms (P4P)" },
      ],

      refs: [
        "Lepetit et al. — EPnP: Efficient Perspective-n-Point Pose Estimation (IJCV 2009)",
        "OpenCV Docs — solvePnP(), Rodrigues()",
        "Carolus Beacon Specification — §2 LED Physical Layout (165 mm square)",
        "Intel RealSense D455 — §2.1 fx = 384.65 (calibrated, Phase 02)",
      ],
    },

  ], /* end entries */

}; /* end LOGBOOK_DATA */
