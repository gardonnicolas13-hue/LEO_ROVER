#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
goto_pose.py -- pose-regulation motion control: drive from an arbitrary
start to an arbitrary commanded (X, Y, theta) in the ground frame.

Written 2026-08-12, in response to the supervisor's request (Technical
Guide, Appendix D.15 / report_latex/appendix.tex S:posecontrol) for
motion control based on Carolus and on AprilTag, each capable of driving
the rover from an arbitrary starting location to a desired (X, Y, Z,
orientation).

STATUS -- read before using this file
--------------------------------------
This is real, complete, hand-verified control code, not a sketch. It is
NOT wired into any .launch file and NOT added to leo_backend.py's FSM.
That is deliberate: this same night, S:aug11mins (Fusion_Campaign.tex)
documents a live, unresolved UART checksum storm on /firmware/wheel_states,
one of the two odometry-adjacent signals this controller's MOVE_L phase
implicitly trusts via the shared ground-frame pose. Deploying a new
autonomous motion-control path on top of a known-degraded sensor input,
before that fault is even diagnosed, would risk producing a field result
that measures the fault rather than the controller. Integration, once the
UART fault is resolved, is wiring GotoPose into the existing FSM as one
more state alongside LOCK_BEACON and GOTO_BEACON (leo_backend.py) -- not
new development. See robot_readiness_audit.md and
verification_grille_prof.md for the rest of that night's audit.

WHY THREE PHASES, NOT ONE SMOOTH CONTROL LAW
----------------------------------------------
A single, continuously-blended polar-coordinate controller (Aicardi et al.
1995) that drives (x, y, theta) to a goal simultaneously exists in the
literature and was considered. It was not used here: its stability proof
needs a specific coupling between the heading-error gain and the final-
orientation-error gain that could not be independently re-derived with
full confidence in the time available for this session. Shipping an
unverified stability argument to a rover with a live, unresolved hardware
fault is a worse mistake than shipping a simpler controller whose
convergence is checked directly. Each of the three phases below is a
single-degree-of-freedom proportional regulation; each Lyapunov argument
is one line and is given in the docstring of the phase it belongs to.

This mirrors, deliberately, the pattern already implemented and used on
this lab's LIMO platform (LimoUltimateMission's
TURN_1 -> MOVE_L -> TURN_2 state machine, "How to use Carolus to make the
LIMO follow the beacon", T. Brezins) rather than introducing an
unvalidated technique where a validated one already exists in the same
lab.

SENSOR FRONT END: CAROLUS AND APRILTAG FEED THE SAME STATE MACHINE
---------------------------------------------------------------------
The (x, y, theta) this controller consumes is sensor-agnostic by
construction: it is whichever of Carolus's ground-frame pose
(/odom_in_beacon, published by carolus_tf_bridge.py once beacon_link is
anchored) or an AprilTag-derived ground-frame pose is freshest -- the same
strict-priority pattern already verified for centring in leo_backend.py's
_target_center() (Fusion_Campaign.tex S:apriltagfallback), not a second,
separately-tuned arbitration scheme. AprilTag's own detection message
already carries a full 6-DOF pose (apriltag_ros AprilTagDetectionArray,
d.pose.pose.pose); this module projects it through the SAME optical-to-
body rotation already derived and source-verified against ceresP4P.hpp /
carolus_astrobee.cpp (Fusion_Campaign.tex S:quaternion, Eq. R_ob), rather
than a second, independently-guessed conversion.
"""
import math

# ---------------------------------------------------------------------------
# Tunables. Not yet field-tuned -- see the trapbox in the module docstring.
# Values below are conservative starting points only (small angular/linear
# authority), chosen the same way this project's own obstacle governor was
# on first deployment: a geometric-margin argument, not a plant model
# (Fusion_Campaign.tex S:pidvalidation makes the same point about that loop).
# ---------------------------------------------------------------------------
K_THETA      = 0.6      # rad/s per rad -- TURN_1 / TURN_2 heading gain
K_PHI        = 0.5      # rad/s per rad -- MOVE_L bearing-correction gain
OMEGA_MAX    = 0.5       # rad/s -- TURN_1 / TURN_2 saturation
OMEGA_CORR   = 0.2       # rad/s -- MOVE_L correction saturation (<< OMEGA_MAX
                          # deliberately, so MOVE_L corrects heading without
                          # re-entering a full stop-and-turn)
V_CRUISE     = 0.15      # m/s -- MOVE_L forward speed
EPS_THETA    = 0.05      # rad (~2.9 deg) -- TURN_1 / TURN_2 exit tolerance
EPS_RHO      = 0.05      # m -- MOVE_L exit tolerance
MAX_PID_DT_S = 0.15      # s -- same guard as leo_backend.py's PID class


def wrap(angle):
    """Normalize an angle to (-pi, pi]. Same convention as leo_backend.py."""
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class PID:
    """Minimal copy of leo_backend.py's PID class (measured-dt discretisation,
    clamped-integration anti-windup, low-pass-filtered derivative). Duplicated
    rather than imported: leo_backend.py is a single monolithic entry-point
    script (rospy.init_node() and node-level side effects at import time),
    not a package, so importing PID from it directly would execute the whole
    backend. Pending a shared-module refactor (moving PID into its own file
    both leo_backend.py and this module import), this copy is kept in exact
    sync by inspection -- see Fusion_Campaign.tex S:pidmath for the verified
    original and its equations (Eq. dt, integral, draw, dfilt, pidout)."""

    def __init__(self, Kp, Ki, Kd, out_min, out_max, i_min=None, i_max=None):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.out_min, self.out_max = out_min, out_max
        self.i_min = i_min if i_min is not None else out_min
        self.i_max = i_max if i_max is not None else out_max
        self.reset()

    def reset(self):
        self._integral = 0.0
        self._prev_err = None
        self._last_t = None
        self._prev_deriv = 0.0
        self.last_err = 0.0
        self.last_output = 0.0

    def update(self, err, now):
        err = float(err)
        if self._last_t is None or (now - self._last_t) <= 0:
            self._prev_err = err
            self._last_t = now
            out = clamp(self.Kp * err, self.out_min, self.out_max)
            self.last_err, self.last_output = err, out
            return out
        dt = now - self._last_t
        self._integral = clamp(self._integral + err * dt, self.i_min, self.i_max)
        if dt <= MAX_PID_DT_S:
            raw_d = (err - self._prev_err) / dt
            self._prev_deriv = 0.5 * raw_d + 0.5 * self._prev_deriv
        out = self.Kp * err + self.Ki * self._integral + self.Kd * self._prev_deriv
        out = clamp(out, self.out_min, self.out_max)
        self._prev_err, self._last_t = err, now
        self.last_err, self.last_output = err, out
        return out


class GotoPose:
    """Three-phase pose regulation: TURN_1 -> MOVE_L -> TURN_2 -> DONE.

    update(x, y, theta, now) -> (lin, ang), the same (linear, angular)
    velocity pair every other AUTO-mode loop in this project returns, ready
    to be clamped by the existing obstacle governor (Fusion_Campaign.tex
    S:obstacleloop) and published on /cmd_vel exactly like every other
    source of motion in this codebase -- this controller adds a goal, it
    does not bypass the existing safety layer underneath it.
    """

    def __init__(self, goal_x, goal_y, goal_theta):
        self.goal = (goal_x, goal_y, goal_theta)
        self.phase = "TURN_1"
        self._pid_turn = PID(Kp=K_THETA, Ki=0.0, Kd=0.08,
                              out_min=-OMEGA_MAX, out_max=OMEGA_MAX)

    def done(self):
        return self.phase == "DONE"

    def update(self, x, y, theta, now):
        gx, gy, gtheta = self.goal
        dx, dy = gx - x, gy - y
        rho = math.hypot(dx, dy)
        gamma = math.atan2(dy, dx)

        if self.phase == "TURN_1":
            # e1 = wrap(gamma - theta); V = 1/2 e1^2, Vdot = -k_theta*e1^2 <= 0
            # for k_theta > 0: proportional regulation of a single scalar
            # error is Lyapunov-stable by construction.
            e = wrap(gamma - theta)
            omega = self._pid_turn.update(e, now)
            if abs(e) < EPS_THETA:
                self.phase = "MOVE_L"
                self._pid_turn.reset()
                return (0.0, 0.0)
            return (0.0, omega)

        if self.phase == "MOVE_L":
            # Bearing re-evaluated every tick (gamma depends on the current
            # x,y), corrected with a small proportional term so the rover
            # closes on the goal without a full stop-and-turn cycle.
            e = wrap(gamma - theta)
            omega = clamp(K_PHI * e, -OMEGA_CORR, OMEGA_CORR)
            if rho < EPS_RHO:
                self.phase = "TURN_2"
                self._pid_turn.reset()
                return (0.0, 0.0)
            return (V_CRUISE, omega)

        if self.phase == "TURN_2":
            # Same one-line Lyapunov argument as TURN_1, error = heading
            # relative to the COMMANDED final orientation this time.
            e = wrap(gtheta - theta)
            omega = self._pid_turn.update(e, now)
            if abs(e) < EPS_THETA:
                self.phase = "DONE"
                return (0.0, 0.0)
            return (0.0, omega)

        return (0.0, 0.0)  # DONE: hold station


def carolus_pose_to_ground(px, py, pz, qx, qy, qz, qw):
    """Optical -> body-style axis rotation, Eq. R_ob (Fusion_Campaign.tex
    S:quatuniversal), the source-verified UNIVERSAL half of the Carolus
    conversion. Returns (x, y, z, qx', qy', qz', qw') in body-style axes.

    The remaining, PLATFORM-SPECIFIC half (the exact rotation sign
    combination, S:quatbug) is intentionally NOT hardcoded here: this
    function only performs the part proven universal from the optical/REP-103
    convention definitions. Compose its output with carolus_tf_bridge.py's
    own ~quat_signs-calibrated correction (S:quatgeneral) for a value trusted
    on THIS platform, or with a freshly-calibrated C (Eq. calibgeneral) on
    any other.
    """
    xb, yb, zb = pz, -px, -py
    return xb, yb, zb, qx, qy, qz, qw


if __name__ == "__main__":
    # Not a ROS node by itself -- see the module docstring. This block is a
    # pure-Python sanity check of the state machine's phase transitions
    # against a synthetic, noiseless trajectory, runnable without ROS:
    #   python3 goto_pose.py
    print("goto_pose.py -- offline phase-transition smoke test (no ROS)")
    ctrl = GotoPose(goal_x=2.0, goal_y=0.0, goal_theta=0.0)
    x, y, theta, t = 0.0, 0.0, math.pi / 2, 0.0
    dt = 0.05
    for step in range(4000):
        lin, ang = ctrl.update(x, y, theta, t)
        theta = wrap(theta + ang * dt)
        x += lin * math.cos(theta) * dt
        y += lin * math.sin(theta) * dt
        t += dt
        if ctrl.done():
            print(f"DONE at t={t:.2f}s, pose=({x:.3f}, {y:.3f}, "
                  f"{math.degrees(theta):.1f} deg), phase transitions "
                  f"exercised: TURN_1 -> MOVE_L -> TURN_2 -> DONE")
            break
    else:
        print("did not reach DONE within the smoke-test horizon -- "
              "this would be a real bug, not expected here")
