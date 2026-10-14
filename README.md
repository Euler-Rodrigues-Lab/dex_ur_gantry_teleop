# Dexterous UR gantry teleoperation

Current milestone: kinematic visualization of the exact gantry-mounted pair of
UR7e arms with Sharpa Wave hands from the pinned
[urgantry_sim](https://github.com/kczttm/urgantry_sim) submodule.
EEF arm IK and hand retargeting are planned in [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md).

```bash
git submodule update --init --recursive
uv sync
uv run gantry-generate
uv run gantry-viz
```

The generated model is `dex_ur_gantry_teleop/assets/scene_sharpa.xml`.
It has 56 explicit position actuators (6 arm + 22 hand joints per side), a `home`
keyframe, and relative mesh paths into `external/urgantry_sim`. Keep that
submodule with the XML; the XML alone does not contain its mesh assets.

The viewer applies position targets directly to joint positions and calls
`mj_forward`; it never calls `mj_step`. The gantry stays fixed. Mouse controls
orbit/zoom, **R** resets home, and `--show-ui` exposes MuJoCo's actuator target
sliders. Both side panels are hidden by default. `--headless` checks startup.
The explicit position actuators have zero velocity gain for this kinematic
preview; this is not a tuned dynamics controller.

`geo_kin_core` supplies the source hash utility; `XRT_devices` is installed from
its submodule for later live input. Neither live device acquisition nor a
licensed retargeting session is needed to preview the model.

Model/mesh licenses remain in the simulator submodule. The arm's historical
`universal_robots_ur5e` directory name does not change the supplied UR7e model.

Planned IK reference: Alexander J. Elias and John T. Wen,
**IK-Geo: Unified robot inverse kinematics using subproblem decomposition**,
*Mechanism and Machine Theory*, 209, 105971, 2025.
[DOI](https://doi.org/10.1016/j.mechmachtheory.2025.105971),
[pinned three-parallel solver](https://github.com/rpiRobotics/ik-geo/blob/a3a1675e1f01ad6f8f15f2cc787fa01472082a11/rust/src/inverse_kinematics/mod.rs#L231).
