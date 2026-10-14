"""Position-target visualization with forward kinematics only (no physics steps)."""
import argparse
import time
import mujoco
import numpy as np
from urgantry_sim.build_urgantry import apply_initial_view
from . import model_path


def apply_position_targets(model, data):
    joints = model.actuator_trnid[:, 0]
    data.qpos[model.jnt_qposadr[joints]] = np.clip(
        data.ctrl, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
    data.qvel[:] = 0
    data.qacc_warmstart[:] = 0
    mujoco.mj_forward(model, data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show-ui", action="store_true", help="show joint target controls")
    parser.add_argument("--headless", action="store_true", help="validate one frame without a window")
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(model_path()))
    data = mujoco.MjData(model)
    home = model.key("home").id
    mujoco.mj_resetDataKeyframe(model, data, home)
    apply_position_targets(model, data)
    if args.headless:
        assert np.isfinite(data.xpos).all() and data.time == 0
        print(f"Kinematic scene OK: {model.nu} position targets, simulation time {data.time}")
        return
    from mujoco import viewer as mj_viewer
    reset_requested = False

    def key_callback(key):
        nonlocal reset_requested
        if key in (ord("R"), ord("r")):
            reset_requested = True

    with mj_viewer.launch_passive(model, data, key_callback=key_callback,
            show_left_ui=args.show_ui, show_right_ui=args.show_ui) as viewer:
        apply_initial_view(viewer)
        print("UR7e + Sharpa kinematic preview. R: reset home. Close window to exit.", flush=True)
        while viewer.is_running():
            with viewer.lock():
                if reset_requested:
                    mujoco.mj_resetDataKeyframe(model, data, home)
                    reset_requested = False
                apply_position_targets(model, data)
            viewer.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
