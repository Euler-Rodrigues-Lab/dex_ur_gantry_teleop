# Replay samples

Unmodified `ipman_roll.npz` and `picking_up_mustard.npz` recordings shared with
OpenArm/G1/RBY1. The legacy SEW wrist position and rotation are body-relative;
the adapter ignores elbow coordinates and combines the wrist pose with the
wrist-local index MCP and Sharpa human palm box. These motions were not captured
specifically for the gantry; targets outside its workspace hold the affected arm.
