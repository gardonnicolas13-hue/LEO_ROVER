# Robot Readiness Audit — 2026-08-12

Scope: Section 1 (TF2 / MINS / VICON relocalization) and Section 2 (motion
control, PID, AprilTag fallback) of the Technical Guide. Every claim below
was checked directly against the running code or by executing the tool in
question — nothing here is asserted from memory of an earlier design
discussion.

**Reading this table:** "Code-complete" means the implementation is correct
and free of the bugs it was checked for. It does **not** mean "validated on
the robot" — those are tracked as separate columns on purpose, because
conflating them is exactly the mistake this audit exists to avoid (see the
Technical Guide's own §1.3 discussion of the same distinction).

## Section 1 — TF2, MINS Estimation, VICON Relocalization

| Component | Code-complete? | Field-validated? | Notes |
|---|---|---|---|
| `carolus_vicon_bridge.py` | Yes | No | TF lookup failure (`LookupException`/`ConnectivityException`/`ExtrapolationException`) is caught explicitly; the node logs once, publishes nothing, and loops — confirmed by reading the `except` block, no crash path exists for "beacon not visible." |
| `config_vicon.yaml` | Yes | No | Every field already carries an explanatory comment (`noise_o`/`noise_p`, `T_imu_vicon` marked as placeholder identity). `vicon.enabled: false` with an explicit header explaining why (bridge unverified live + beacon coordinates unsurveyed). No change made — it was already at the standard this audit checks for. |
| `tools/bag_to_csv.py` | Yes | Yes (used routinely) | `py_compile` clean. Output path handling verified: defaults to the bag's own directory via `os.path.abspath`, creates it if missing. |
| `tools/plot_relocalization.m` | **Created this audit** | No | Did not exist in `tools/` before today — it had only been drafted inline in a chat response, never saved. Written now with explicit error handling for every missing-file / empty-data case (was the literal gap flagged: "exploitable sans erreur de chemin"). Verified by running it against a nonexistent path: raises a clean, named error (`plot_relocalization:missingFile`), not a raw MATLAB stack trace. **Has never been run against a real recording** — cannot have been, since no VICON-corrected bag exists yet. |

**Blocking dependency, found live tonight, not yet resolved:** `serial_node`
is currently emitting a sustained `wrong checksum for topic id and msg`
burst on the robot (confirmed via `journalctl`, ~2.7 msg/s, ARM clock
steady at 1500 MHz so not the known thermal cause). This can silently drop
`/firmware/wheel_states`, one of MINS's three fused inputs. Any relocalization
test run before this is diagnosed risks measuring against a degraded MINS
estimate without any visible symptom. This is tracked as item 40 in the
report's open-items registry — flagged again here because it directly
threatens Section 1's field test, not just a separate concern.

## Section 2 — Motion Control, PID, AprilTag Fallback

| Component | Code-complete? | Field-validated? | Notes |
|---|---|---|---|
| `PID` class | Yes | Yes (live in AUTO mode) | Measured-`Δt` discretization confirmed (no fixed-rate assumption). Anti-windup by clamped integration confirmed (`_integral` clamped to `[i_min, i_max]` before use). Derivative low-pass confirmed exact: `0.5*raw_d + 0.5*prev_deriv`. |
| `_target_center()` | **One bug fixed this audit** | Partially — Carolus-only path is live; the AprilTag handoff specifically is not | The AprilTag freshness threshold was a bare literal `0.7` duplicated in two places (`_target_center()` and the LOCK approach-distance logic) — a real risk of tuning one and forgetting the other during a field session. Promoted to a single named constant, `TAG_FALLBACK_MAX_AGE_S = 0.7`, used in both. `CENTER_HOLD_S = 0.5` was already a named constant; no change needed there. `py_compile` clean after the edit. **Still open, not a bug, a known limitation:** no hysteresis exists on the Carolus↔AprilTag handoff itself — see the Technical Guide §2.3.3 for why this is a plausible (unconfirmed) jitter source near the range boundary. |
| Four PID instances (`_pid_patrol`, `_pid_lock_align`, `_pid_lock_approach`, `_pid_obstacle`) | Yes | No | All four gain constants (`OPEN_STEER_KP`, `LOCK_ALIGN_SPEED`, `TARGET_KP`, `OBSTACLE_GOV_KP`, `OBSTACLE_MIN_SPEED_SCALE`) confirmed defined — no `NameError` risk at runtime. Output/integral bounds confirmed sane (`_pid_obstacle`'s asymmetric bound `[scale-1, 0]` is intentional: speed can only be de-rated, never boosted, by this loop). Live in the deployed control path. **Not yet exercised over a live AUTO traverse under normal network conditions** — the session that deployed this ran through an unresolved building-WiFi outage that prevented that trial. |

## Overall verdict

**Code-complete and internally consistent for both sections. Not ready to
present as "operational on the robot" without qualification** — three
concrete, distinct things remain unproven by measurement, not by code:
the VICON relocalization jump itself, the AUTO-mode qualitative PID
behaviour, and the Carolus/AprilTag handoff under motion. A fourth item,
the live UART checksum storm, is a precondition for trusting either of the
first two once network access returns.

## Immediate next steps (unchanged from the prior audit, still accurate)

1. Diagnose and resolve the UART checksum storm (item 40) before trusting
   any pose-dependent field test.
2. Survey the beacon's global coordinates, confirm `/mins/external_ref/carolus`
   publishes live, then `vicon.enabled: true` + `restart_stack.sh`.
3. Run the straight-line relocalization test; analyse with
   `tools/plot_relocalization.m` (now it actually exists).
4. Run a live AUTO traverse to qualitatively validate all four PID loops,
   watching specifically for jitter at the Carolus/AprilTag handoff.
