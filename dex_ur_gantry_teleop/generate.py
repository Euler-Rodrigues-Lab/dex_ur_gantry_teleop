"""Export the exact mounted Sharpa geometry with explicit position actuators."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from geo_kin_core.spec import sha256_of_file
from urgantry_sim import build_urgantry as builder
from . import model_path


def main():
    path = model_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    builder.export_mjcf(str(path), hand="sharpa")
    source = mujoco.MjModel.from_xml_path(str(path))
    tree = ET.parse(path)
    actuators = tree.getroot().find("actuator")
    # MuJoCo exports position servos as <general>. Make their position-target
    # contract explicit. This kinematic preview never integrates actuator forces.
    for element in actuators:
        aid = source.actuator(element.attrib["name"]).id
        kp = float(source.actuator_gainprm[aid, 0])
        assert kp > 0
        element.tag = "position"
        for key in ("gainprm", "biasprm", "gaintype", "biastype", "dyntype", "dynprm"):
            element.attrib.pop(key, None)
        element.set("kp", format(kp, ".17g"))
        element.set("kv", "0")
    ET.indent(tree)
    tree.write(path, encoding="unicode")
    model = mujoco.MjModel.from_xml_path(str(path))
    original = builder.build_model(hand="sharpa")
    assert model.nu == model.nq == model.nv == 56
    np.testing.assert_array_equal(model.jnt_axis, original.jnt_axis)
    np.testing.assert_allclose(model.jnt_range, original.jnt_range, atol=1e-5)
    for aid in range(model.nu):
        assert model.actuator(aid).name == original.actuator(aid).name
        assert model.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT
        assert model.actuator_dyntype[aid] == mujoco.mjtDyn.mjDYN_NONE
        np.testing.assert_allclose(model.actuator_biasprm[aid, :3],
                                   [0, -model.actuator_gainprm[aid, 0], 0])
    # Compare assembled and reloaded geometry at home and random arm/hand poses.
    data, reference = mujoco.MjData(model), mujoco.MjData(original)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
    builder.set_initial_pose(original, reference)
    np.testing.assert_allclose(data.qpos, reference.qpos, atol=1e-6)
    rng = np.random.default_rng(7)
    for sample in range(11):
        if sample:
            data.qpos[:] = rng.uniform(model.jnt_range[:, 0], model.jnt_range[:, 1])
        reference.qpos[:] = data.qpos
        mujoco.mj_forward(model, data)
        mujoco.mj_forward(original, reference)
        np.testing.assert_allclose(data.xpos, reference.xpos, atol=1e-6)
        np.testing.assert_allclose(data.xmat, reference.xmat, atol=1e-6)
    repo = Path(builder.__file__).resolve().parents[1]
    revision = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
    metadata = {"source": "https://github.com/kczttm/urgantry_sim", "revision": revision,
                "sha256": sha256_of_file(path), "hand": "sharpa", "robot": "UR7e",
                "actuators": 56, "mode": "kinematic", "actuator_kv": 0,
                "note": "Position targets; force integration is not used. Meshes remain in the simulator submodule."}
    path.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Validated {path}: 56 position actuators; home and 10 random FK comparisons passed.")


if __name__ == "__main__":
    main()
