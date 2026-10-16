"""UR gantry kinematic preview; asset meshes live in the simulator submodule."""
from pathlib import Path


def model_path(hand="sharpa"):
    if hand not in ("sharpa", "wuji"):
        raise ValueError("hand must be sharpa or wuji")
    return Path(__file__).resolve().parent / "assets" / f"scene_{hand}.xml"


def load_model(hand="sharpa"):
    """Load the signed geometry and apply the hand's display palette.

    Wuji v1 reference: https://wuji.tech/en/hand (black palm, silver
    mechanisms, white fingertip pads). Meshes are monolithic, so colors
    approximate the finish per part rather than individual fasteners.
    """
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(model_path(hand)))
    if hand == "wuji":
        for index in range(model.ngeom):
            mesh_id = model.geom_dataid[index]
            if model.geom_type[index] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_group[index] != 1:
                continue
            name = model.mesh(mesh_id).name
            if not name.startswith(("left_hand_", "right_hand_")):
                continue
            if name.endswith("palm_link"):
                color = (0.055, 0.060, 0.065, 1.0)
            elif name.endswith("tip_link"):
                color = (0.92, 0.93, 0.94, 1.0)
            else:
                color = (0.58, 0.61, 0.64, 1.0)
            model.geom_rgba[index] = color
    return model
