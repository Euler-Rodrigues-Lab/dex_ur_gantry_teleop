"""UR gantry kinematic preview; asset meshes live in the simulator submodule."""
from pathlib import Path


def model_path():
    return Path(__file__).resolve().parent / "assets" / "scene_sharpa.xml"
