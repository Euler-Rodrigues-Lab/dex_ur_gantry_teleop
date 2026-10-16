# UR gantry EEF teleoperation implementation checklist

Status: Rust retargeting and one-year CMU IAM wheel implemented on 2026-10-14. No commits requested. The original detailed checklist below is retained as planning history; the completion ledger records delivered scope and remaining work.
Prepared 2026-10-13. Repository: `Euler-Rodrigues-Lab/dex_ur_gantry_teleop`.

## Confirmed scope

- [x] Two six-revolute-joint arms, each commanded by a 6D EEF target; no human elbow angle, SEW arm objective, or collision-avoidance filter.
- [x] First version controls both arms with the gantry fixed.
- [x] Sharpa finger retargeting reuses the existing `Alignment` preset and index-MCP-origin palm box. Full finger landmarks remain available for finger articulation.
- [x] Arm input is hand orientation plus index-MCP position; shoulder/body tracking is used for coordinate alignment, not elbow tracking.
- [x] Support selectable first-frame anchoring for fixed bases and shoulder-attached alignment. User confirmed: adjust the EEF alignment offset once, then hold it fixed; no continuous reachability adaptation in this phase.
- [x] Preserve an extension point for Wuji; implementing its retargeter is a later phase.
- [x] Initial plan delivered uncommitted. Follow-up authorizes submodules, generated XML, environment setup, and kinematic visualization only.

## Inspected sources and model findings

- [x] Simulator: https://github.com/kczttm/urgantry_sim, inspected commit `46a784ac2cb59080205e722d91084bb244195092` in a temporary checkout.
- [x] The model is identified by the simulator as UR7e, despite the historical `universal_robots_ur5e/ur5e.xml` path. Use the supplied geometry, not nominal UR5/UR7 DH constants.
- [x] Compiled `build_model(hand="sharpa")` and ran MuJoCo forward kinematics without changing model files. Sharpa scene has 56 position-controlled joints/actuators: left arm 6, left hand 22, right arm 6, right hand 22.
- [x] At the model zero pose, h1/h2 axis-line intersections are world positions `[-0.28403841, -0.3825, 1.37974159]` and `[+0.28403841, -0.3825, 1.37974159]` metres. Numerical intersection residual was zero; midpoint is `[0, -0.3825, 1.37974159]`. Extract these from geometry rather than hard-coding them.
- [x] h2, h3, and h4 are parallel on both arms. The requested three-parallel-axis IK family applies.
- [x] Existing `geo-kin-subproblems` re-exports SP1–SP6. Existing Sharpa code provides the index-MCP palm-box transforms and alignment solver.
- [x] Existing `RetargetFrame` couples wrist rotation to `SEWPose`; the new path needs an explicit EEF input rather than fabricated shoulder/elbow values.
- [x] RBY1's functional offset calls a SEW-based centroid-alignment helper. Reusing it unchanged would violate the EEF-only requirement.

## 1. Repository and package boundaries

- [x] Add pinned `external/urgantry_sim` submodule from the requested repository, plus `external/XRT_devices` and `external/geo_kin_core` following the existing teleop applications.
- [ ] Create the public Python application package, `pyproject.toml`, lockfile, CLI entry points, simulator adapter, tests, and replay assets. Keep simulator assets owned by the simulator submodule.
- [ ] Follow the existing private/public split: implement the Rust robot solver in the private `geo_kin` workspace as a new robot crate (proposed `geo-kin-ur-gantry`); expose it through the licensed Python wheel. Public application code handles input, simulation, and display.
- [ ] Add the robot/session dispatch, feature selection, source-spec signature, and `geo-kin-provision` product mapping (proposed `ur-gantry-sharpa`). Keep existing robot products compatible.
- [ ] Extract signed/provenanced kinematic specs from the exact assembled simulator model. Audit the mounted Sharpa chains against the existing embedded Sharpa geometry, joint axes/signs, zero poses, and names before reusing it.
- [ ] Preserve upstream code/asset license notices, including IK-Geo BSD-3-Clause attribution if adapting source.

## 2. Frames and explicit input contract

- [ ] Define transforms consistently: `T_A_B` maps coordinates in B into A. Name world, human body, robot upper body, arm base, flange, hand base, and palm-box frames explicitly.
- [ ] Compute each robot shoulder from the intersection of its first two joint-axis lines, not the second joint's anchor. Check intersection and parallel-axis residuals during extraction.
- [ ] Set the robot upper-body origin to the midpoint of the shoulders. Adopt the shared +x forward, +y left, +z up convention; for this scene use +x toward world +y, +y toward world -x, and +z world up. Validate handedness and left/right labeling against the assembled scene.
- [ ] Introduce optional left/right EEF targets carrying index-MCP position and hand rotation, with explicit frame conventions and validity. Preserve independent hand landmarks and body-frame pose. Do not require an elbow measurement to populate this input.
- [ ] Extend shared input conversion and replay serialization compatibly. Existing recordings with SEW data may supply wrist orientation, but their elbow values must have no effect on this solver.
- [ ] Audit XRT and MediaPipe world/body/wrist conventions, handedness, units, timestamps, and confidence gates. Require metric position from the existing body/hand fusion, not hand-local MediaPipe coordinates treated as world positions.
- [ ] Preserve Sharpa palm-box orientation calibration: compose device hand rotation with the established human-to-robot hand/palm alignment. Do not replace it with an identity rotation. Landmarks used to estimate a hand frame are distinct from an elbow control objective.

## 3. Base alignment modes

- [ ] `first_frame` (default for the fixed gantry): latch a valid human upper-body pose at engagement, align it to the robot shoulder-midpoint frame, and keep that mapping fixed. Subsequent human body displacement must remain visible in the EEF targets rather than being subtracted every frame.
- [ ] `shoulder_attached`: express targets relative to the live human upper body and map them to the current robot upper-body frame; accept an externally provided robot-base pose for future mobile deployments.
- [ ] Keep the gantry physically fixed in the initial environment. Selecting a coordinate-alignment mode does not create or command a mobile-base degree of freedom.
- [ ] Calibrate one shared EEF-only alignment translation at engagement using valid index-MCP targets and robot home/current FK. Preserve relative hand placement and hold the resulting offset fixed until explicit recenter; do not use human elbow geometry.
- [x] Offset policy resolved: once adjusted, keep it fixed. Continuous reachability-based offset adaptation is deferred.
- [ ] Keep translation gain configurable with a proposed default of 1 metre/metre; apply any offset in a declared frame. Confirm reachability using real targets before tuning defaults.
- [ ] Define engage/disengage, reset, and recenter separately. Missing initial calibration must hold commands, not silently substitute an identity frame. Recenter explicitly clears the anchor and continuity state.

## 4. Six-DOF arm IK using existing subproblems

Reference: pinned [`three_parallel`](https://github.com/rpiRobotics/ik-geo/blob/a3a1675e1f01ad6f8f15f2cc787fa01472082a11/rust/src/inverse_kinematics/mod.rs#L231). The supplied L253 link is inside this function, not a different solver.

- [ ] Adapt the orchestration to our spec/FK/session APIs and call the existing subproblem crate. Do not duplicate the subproblem implementations or substitute a numerical Jacobian solver.
- [ ] Convert the chain's fixed rotations and axes into the common H/P convention; remove the terminal tool rotation/translation consistently before solving.
- [ ] Preserve call order: remove base/tool position offsets; SP6 gives `(q1,q5)` candidates; SP1 solves the combined parallel-joint angle and q6; SP3 gives q3 branches; SP1 gives q2; recover `q4 = wrap(theta14-q2-q3)`. Retain every branch and least-squares diagnostic.
- [ ] Verify each result with independent FK, deduplicate equivalent branches, and distinguish exact, approximate, singular, and invalid results.
- [ ] Choose valid joint-limit lifts nearest current feedback/previous command. Respect the model's multi-turn limits instead of wrapping every actuator unconditionally to [-pi,pi].
- [ ] Preserve branch continuity near singularities; reject non-finite results. On no acceptable candidate, hold that arm and report residual/status. Do not present a least-squares branch as an exact solution.

## 5. EEF-to-flange composition and hand retargeting

- [ ] Define the controlled robot TCP as the Sharpa index-MCP palm-box origin. Extract each fixed flange-to-hand-base transform from the mounted model, including its adapter and side-specific roll.
- [ ] Compose `T_upper_flange_goal = T_upper_palmbox_goal * inverse(T_flange_palmbox)`, then transform to the corresponding arm-base frame for IK. Distinguish zero-pose hand geometry from the moving index proximal link.
- [ ] Use existing `SharpaHandRetargeter` with `RetargetPreset::Alignment`, side-specific transforms, and joint limits. Feed landmarks in its expected wrist/hand-local coordinates.
- [ ] Map the resulting 22 joints by simulator names, not by an assumed shared ordering. Validate left/right flexion signs and partial-landmark behavior; preserve prior finger commands for unsolved groups.
- [ ] Introduce a small hand interface for geometry/TCP transform, landmarks, retargeting, and joint mapping. Keep Sharpa-specific 22-DOF assumptions inside its adapter. Future Wuji support supplies its own geometry and solver; selecting Wuji now must report unsupported retargeting clearly.

## 6. Runtime, simulation, and replay

- [ ] Runtime order: acquire input -> validate/normalize -> establish body mapping -> form palm-box targets -> update optional shared offset -> convert to flange goals -> solve/select each arm IK -> solve Sharpa fingers -> assemble held/updated commands -> apply simulator action -> render diagnostics.
- [ ] Build the scene with `hand="sharpa"` explicitly (upstream default is Wuji). Use named actuator/joint lookup and unnormalized radian commands. Preserve the upstream home poses and physics.
- [ ] Provide XRT, MediaPipe, and deterministic offline replay through one solver path. Keep device loss, clutch, reset, and cleanup behavior consistent with existing applications.
- [ ] Provide kinematic preview as the proposed default, plus explicit dynamic position-servo stepping. A kinematic preview must set joint positions/forward kinematics; dynamic mode follows the simulator's `step(action)` pattern and control timestep.
- [ ] Show goal/achieved palm-box frames, shoulder midpoint, and IK validity. Use the existing human overlay and hidden MuJoCo side panels. No solver collision-avoidance or safety-capsule pipeline; ordinary simulator contacts remain intact.
- [ ] Adapt the existing sample motions where their hand data is sufficient, with documented frame conversion and a short dedicated EEF fixture. Elbow loss alone must not stop arm teleoperation.

## 7. Validation and completion gates

- [ ] Compare extracted FK to MuJoCo for both sides at home, zero, and randomized valid configurations, including the mounted hand and index-MCP TCP.
- [ ] Test FK -> IK -> FK against the pinned upstream solver, including branch sets, limits, singularities, unreachable goals, and continuous trajectories. Record separate position/orientation tolerances.
- [ ] Verify frame composition under arbitrary human/robot rotations; first-frame anchoring, shoulder-attached mode, reset/recenter, and adaptive-offset continuity. Changing only human elbow landmarks must leave outputs unchanged.
- [ ] Run existing Sharpa alignment parity and mounted-model hand-FK checks; validate independent side dropout and partial finger tracking.
- [ ] Headless replay and simulator startup/stepping tests must assert finite, bounded commands, correct named mappings, held commands on invalid input, and deterministic reset.
- [ ] Test licensed wheel construction/provisioning and ensure the public application imports no private source. Existing robots' frame adapters and replay files must remain compatible.
- [ ] Document separate results for analytic correctness, replay, live XRT/MediaPipe, and dynamic simulation. Hardware control is outside this first implementation.

## 8. README and citation deliverables

- [ ] Write `README.md` with recursive-submodule setup, provisioning, simulator model provenance, the two frame modes, EEF-only arm contract, Sharpa alignment, launch/replay examples, and Wuji's deferred status.
- [ ] Include a frame diagram and explain the index-MCP TCP versus flange distinction and failure/hold behavior.
- [ ] Cite the requested paper and link the pinned implementation:

  Alexander J. Elias and John T. Wen, **IK-Geo: Unified robot inverse kinematics using subproblem decomposition**, *Mechanism and Machine Theory*, 209, 105971, 2025. DOI: [10.1016/j.mechmachtheory.2025.105971](https://doi.org/10.1016/j.mechmachtheory.2025.105971). [Preprint](https://arxiv.org/abs/2211.05737).

- [ ] Add third-party attribution and distinguish reused subproblem/Sharpa components from the new EEF session and simulator integration.

## Remaining design decisions before implementation

- [x] User selected fixed-after-adjustment EEF offset; no live adaptive update.
- [ ] Confirm whether first-frame anchoring should use a startup body pose or the first valid engaged frame (proposed: first valid engaged frame, explicit recenter thereafter).
- [ ] Confirm the proposed private Rust solver/public application split and kinematic-preview default if a different deployment is intended.

## Completed simulation milestone

- [x] Added simulator, geo_kin_core, and XRT_devices submodules and local uv environment.
- [x] Generated Sharpa XML with 56 explicit position actuators and a home keyframe.
- [x] Compared reloaded body positions/rotations against the upstream assembled model at home and ten random configurations.
- [x] Added kinematic viewer (mj_forward only), reset key, optional joint-control panels, and headless startup check.
- [x] Started the interactive viewer for inspection. No commits made.

## Retargeting completion ledger — 2026-10-14

- [x] Private `geo-kin-ur-gantry` crate, `urgantry` feature, licensed session dispatch and `urgantry-sharpa` provisioning.
- [x] Exact XML arm/Sharpa Wave specs, model signature guard, index-MCP TCP and shoulder-intersection body frame.
- [x] Analytic three-parallel IK with the required intersecting-wrist specialization; independent MuJoCo and upstream branch parity tests.
- [x] Existing Sharpa Alignment algorithm using mounted Wave geometry, named 22-joint mapping and hand FK checks.
- [x] First valid engaged frame anchoring or live shoulder-attached mapping; one shared fixed calibration offset, explicit recenter, independent holds.
- [x] Explicit EEF types, XRT/MediaPipe adapters without elbow dependence and legacy replay conversion; local submodule edits retained.
- [x] Kinematic-only runtime, both bundled samples, documented unreachable targets, README and paper attribution.
- [x] One-year signed academic CMU IAM license for sarveshp@andrew.cmu.edu, restricted urgantry/sharpa wheel and redistribution bundle.
- [ ] Publish shared submodule changes and advance pins (no commit/push requested).
- [ ] Exercise live XR/MediaPipe acquisition and interactive teleop with the recipient.
- [ ] Dedicated EEF recording/serialization format, human/goal overlays, exposed residual/branch diagnostics and broader singular-trajectory tests.
- [ ] Future optional scope: mobile robot-base pose, configurable position scale, dynamic servo simulation and Wuji. These are not supplied in this fixed-gantry version.

Actual geometry makes general SP6 rank deficient; `three_parallel_two_intersecting`
provides SP4/SP4 initialization and preserves the remaining requested decomposition.
Calibration uses the first valid engaged frame. Continuous adaptive offset remains
explicitly deferred. Existing plan items describing dynamic stepping, full overlays,
mobile-base input, or residual diagnostics are not completion claims.


## Follow-up corrections — 2026-10-14

- [x] No-offset application default; fixed translation remains opt-in via `--offset`.
- [x] Unrestricted arm angles, verified nearest IK branch/whole-turn lift, reset policy and simulator clipping override; hand limits retained.
- [x] Wave right-hand palm forward-axis correction with existing HA4 parity preserved.
- [x] Recover measured DIP and thumb CMC/IP from replay skeletons and live XR.
- [x] Complete both recordings with every arm EEF target accepted; regression tests cover position and orientation.

- [x] Replay `--loop` with deterministic reset per pass, total `--steps` cap, joined renderer shutdown and clean Ctrl+C. Fourteen application tests pass; real-window one-pass, loop and early-stop checks exit without GLX errors.
- [x] OpenArm-style front three-quarter camera, rotated into the gantry frame and shared by preview/replay.


## Wuji implementation — 2026-10-15

- [x] Exact pinned Wuji v1.0.2 mounted scene, 52 explicit position actuators,
  named authored tip-link frames; original geometry matches 11 upstream poses.
- [x] `geo-kin-wuji` 20-joint direction alignment using SP2 seeds, offset-aware
  bounded proximal refinement, and SP1 distal alignment. Shared landmark and
  index-MCP palm conventions, hand-specific geometry and limits.
- [x] Generalized gantry hand adapter, mounted arm specs, license/build feature,
  model hash, hand names and fingertip FK; retained Sharpa defaults.
- [x] `--hand wuji` for preview, replay, live input and video exporter.
- [x] 17 app tests, independent MuJoCo FK, 80 synthetic articulation tests,
  both full replays, offscreen preview and real-window startup/shutdown.
- [x] Rebuilt CMU wheel/license with Wuji, unchanged expiration; clean install
  validation, old-license rejection and release binary hygiene.
- [ ] Live XR/MediaPipe acquisition and physical Wuji hardware validation.
