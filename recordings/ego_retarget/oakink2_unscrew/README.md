# OakInk2 run `unscrew_bottle_cap`: unscrewing a bottle cap

One 9.2-second egocentric clip with per-frame 3D hand skeletons and the poses of the two objects
(bottle cap and bottle body), for testing retargeting from human hands to robot hands: in OakInk2's annotation the
right hand unscrews the cap while the left hand holds the bottle body. Part of the
[OakInk2 set](../README.md). Same file layout, array keys and conventions as the EgoVerse runs in
[`data/egoverse/trace`](../../egoverse/trace/README.md), plus the object files.

| | |
|---|---|
| source | [OakInk2](https://oakink.net/v2) (Zhan et al., CVPR 2024), sequence `scene_01__A007++seq__90724a6acd7d7f462f5a__2023-04-21-10-17-48`, egocentric camera (head-mounted, serial `104422070969`) |
| objects | cap `O02@0015@00001` (large black and yellow striped bottle cap), body `O02@0015@00002` (large black and yellow striped bottle body) |
| task name | `unscrew_bottle_cap` |
| task description | Unscrew the cap of the large black and yellow striped bottle. (OakInk2 task target: "Unscrew the bottle.") |
| window | OakInk2 mocap frames 678-1787 (278 camera frames; the whole "unscrew" program segment) |
| rate, length | about 30 fps (see timestamps below), 9.24 s |
| image | 848 x 480 |

Both hands are labelled in all 278 frames; in view of the egocentric camera: left 100.0 %, right
82.7 % (the right hand works at the top edge of the image and is often partly outside it; the labels
come from the mocap rig and are complete either way).

## Task name and annotations

OakInk2 describes each recording as a program of primitives (one verb per hand with its objects). This window is the
one segment of the sequence, named by its primitive and object: **`unscrew`** + bottle cap, written `unscrew_bottle_cap`.
`meta.json` holds the same list under `annotations`.

| # | OakInk2 description | per hand | mocap frames (inclusive) | clip frames |
|---|---|---|---|---|
| 1 | Unscrew the cap of the large black and yellow striped bottle. | right hand: unscrew large black and yellow striped bottle cap; left hand: hold large black and yellow striped bottle body | 650-1787 | 0-278 |

**Label caveat.** The hand and object poses are OakInk2's shipped annotation: MANO hands and rigid object poses from
its multi-camera mocap and RGB-D rig, not estimates from this egocentric video.

## Files

| file | what it is |
|---|---|
| `frames/0000.png` ... `frames/0277.png` | the egocentric camera's original OakInk2 PNG frames, byte for byte; frame `i` is row `i` of every array (OakInk2 frame `frame_index[i]`) |
| `video.mp4` | the same frames as H.264 (CRF 18, yuv420p) at a constant 30 fps, for viewing |
| `overlay.mp4` | the video with both 21-joint skeletons drawn on it (left hand blue, right hand green), CRF 18 |
| `unscrew_bottle_cap_world3d_only.mp4` | floating 3D view in the world frame: MANO hand meshes over the skeletons, the cap and bottle body at their recorded poses, the head camera as a pyramid, a fixed virtual camera, 1280 x 960, CRF 18 |
| `data.npz` | per-frame skeleton, fingertip, wrist and camera arrays (below) |
| `mano_fit.npz` | OakInk2's MANO hand parameters in the repo's convention (below) |
| `objects.npz` | per-frame poses of the cap and the bottle body (below) |
| `objects/<obj_id>/model.obj`, `model.mtl`, `textured.png` | OakInk2's repaired object meshes (`object_repair`, metres, textured), byte for byte |
| `meta.json` | clip details, conventions, the world transform and OakInk2's program annotation |
| `LICENSE` | CC BY-SA 4.0 licence text |

## `data.npz`

T = 278. Axis 1 of every `(T, 2, ...)` array is the hand: 0 = left, 1 = right. float32 unless noted. Missing values
would be NaN, never zeros (none are missing in this clip).

| key | shape | units | frame | meaning |
|---|---|---|---|---|
| `timestamps` | (T,) | s | – | time from the first frame: `(frame_index - frame_index[0]) / 120` |
| `frame_index` | (T,) int32 | – | – | OakInk2 mocap frame id of the camera frame (the PNG's file name) |
| `skeleton_world` | (T, 2, 21, 3) | m | world | 21 joints per hand |
| `skeleton_cam` | (T, 2, 21, 3) | m | camera | the same joints in the egocentric camera's frame |
| `skeleton_px` | (T, 2, 21, 2) | px | image | the joints projected with `K` |
| `fingertips_world` | (T, 2, 5, 3) | m | world | thumb, index, middle, ring, pinky tips (joints 4, 8, 12, 16, 20) |
| `fingertips_cam` | (T, 2, 5, 3) | m | camera | the same, camera frame |
| `wrist_pos_world` | (T, 2, 3) | m | world | wrist position (joint 0) |
| `wrist_pos_cam` | (T, 2, 3) | m | camera | the same, camera frame |
| `wrist_quat_world` | (T, 2, 4) | – | world | wrist orientation `[qw, qx, qy, qz]`: the rotation of OakInk2's MANO root (global orient) in the world frame |
| `wrist_vel_world` | (T, 2, 3) | m/s | world | central difference of the wrist position over `timestamps`; NaN at the first and last frame |
| `fingertip_vel_world` | (T, 2, 5, 3) | m/s | world | central difference of each fingertip; NaN at the first and last frame |
| `valid` | (T, 2) bool | – | – | hand labelled |
| `in_view` | (T, 2) bool | – | – | labelled, wrist more than 5 cm in front of the camera and inside the image |
| `head_pose_world` | (T, 4, 4) | m | world | egocentric camera pose, camera-to-world |
| `K` | (3, 3) | px | – | pinhole intrinsics (OakInk2's `cam_intr`, constant over the clip) |
| `image_size` | (2,) int32 | px | – | `[width, height]` |

**Joint order** (21 per hand): 0 wrist; thumb 1-4, index 5-8, middle 9-12, ring 13-16, pinky 17-20, each from base to
tip.

**Timing.** OakInk2's mocap runs at 120 Hz and its cameras at about 30 fps; each camera frame is tagged with the mocap
frame it was taken at. In this window the spacing is 2 to 6 mocap frames (median 4), so
`timestamps` are not exactly uniform; use them, not a fixed 1/30 s, for velocities. The mp4 files play at a constant
30 fps.

**Frames.** World: OakInk2's world frame (metres, +y up) rotated by +90° about x so that +z is up and the table top is at
z = 0 (the lowest vertex of the bottle body, from its recorded pose and mesh, sits at z = 0.6 mm);
`meta.json` `world_from_oakink2_world` is that 4x4. Camera: OpenCV (x
right, y down, z forward). With `head_pose_world = [R t; 0 0 0 1]`:

$$
x_{\text{cam}} = R^\top (x_{\text{world}} - t)
$$

**Wrist orientation.** OakInk2 ships no tracker wrist frame, so `wrist_quat_world` is MANO's root rotation (as for
the H2O runs). Compare orientations across datasets only after aligning the two wrist frames.

**Skeleton.** `skeleton_world` is OakInk2's MANO annotation evaluated the way its own toolkit does (manotorch
`ManoLayer(rot_mode="quat", center_idx=0, flat_hand_mean=True)`): the left model used exactly as shipped, the posed
wrist moved to the stored translation, fingertips at mesh vertices 745, 317, 444 (445 for the left hand), 556, 673.

## `mano_fit.npz`

OakInk2's own MANO parameters (not a new fit), written in the repo's convention (`smplx.MANO`, `flat_hand_mean=True`),
one shape per hand. For `<s>` = `left`, `right`:

| key | shape | units | meaning |
|---|---|---|---|
| `<s>_betas` | (10,) | – | MANO shape (OakInk2's, constant over the sequence) |
| `<s>_global_orient` | (T, 3) | rad | wrist rotation, axis-angle, world frame |
| `<s>_transl` | (T, 3) | m | MANO translation, world frame, chosen so that the posed wrist lands at OakInk2's stored wrist position |
| `<s>_hand_pose` | (T, 45) | rad | 15 finger joints, axis-angle, absolute (`flat_hand_mean=True` in `smplx.MANO`) |
| `<s>_joints21_world` | (T, 21, 3) | m | the MANO hand's 21 joints in the order above (MANO's 16 joints + fingertip vertices 744, 320, 443, 554, 671) |
| `<s>_err` | (T, 21) | mm | distance from each of these joints to `skeleton_world` |

The 16 MANO joints are identical to `skeleton_world` (largest difference 0.0000 mm, numerical only); the fingertips
differ by 2.9 mm (left) and 3.1 mm (right) on average because OakInk2 uses neighbouring tip vertices.
**Left hand:** OakInk2 built its left hands with `MANO_LEFT.pkl` exactly as shipped, without the sign fix of the left
model's shape directions that `smplx` applies; use the left model unchanged to reproduce these joints. The MANO model
files are not included; get them from [mano.is.tue.mpg.de](https://mano.is.tue.mpg.de) (free research licence).

## `objects.npz` and `objects/`

| key | shape | meaning |
|---|---|---|
| `part` | (2,) str | `cap`, `body` |
| `obj_id` | (2,) str | OakInk2 object ids `O02@0015@00001`, `O02@0015@00002` |
| `obj_name` | (2,) str | OakInk2's English names |
| `pose_world` | (T, 2, 4, 4) float32 | mesh frame -> world (the frame above) for each part |

`objects/<obj_id>/model.obj` (with `model.mtl` and `textured.png`) are OakInk2's `object_repair` meshes in metres,
unchanged. A vertex `v` of part `p` at frame `t` sits at `pose_world[t, p] @ [v; 1]` in the world frame.

## Loading

Install the pinned packages from the repo root with `pip install -r requirements.txt`, then:

```python
import numpy as np
d = np.load("data/oakink2/unscrew_bottle_cap/data.npz")
tips = d["fingertips_world"]                                 # (278, 2, 5, 3) metres
wrist = d["wrist_pos_world"][:, 1]                           # right wrist, (278, 3)
speed = np.linalg.norm(d["wrist_vel_world"][:, 1], axis=-1)  # m/s
o = np.load("data/oakink2/unscrew_bottle_cap/objects.npz")
cap_pose = o["pose_world"][:, list(o["part"]).index("cap")]  # (278, 4, 4)
```

## Licence and attribution

The data in this folder is licensed under the
[Creative Commons Attribution-ShareAlike 4.0 International licence (CC BY-SA 4.0)](https://creativecommons.org/licenses/by-sa/4.0/);
the full text is in [`LICENSE`](LICENSE).

**Source.** OakInk2 ([oakink.net/v2](https://oakink.net/v2), [paper](https://arxiv.org/abs/2403.19417),
[code](https://github.com/oakink/OakInk2)), sequence `scene_01__A007++seq__90724a6acd7d7f462f5a__2023-04-21-10-17-48`, released under CC BY-SA 4.0 on its
Hugging Face dataset page [kelvin34501/OakInk-v2](https://huggingface.co/datasets/kelvin34501/OakInk-v2) (checked
2026-11-04, revision `21705616`). Files used: `anno_preview/<sequence>.pkl`, `data/<sequence>.tar` (egocentric
camera only), `object_repair.tar`, `object_raw.tar` (`obj_desc.json`), `program.tar`.

**Changes made from the source.** We cut the window of the "unscrew" program segment (mocap frames
678-1787, 278 egocentric camera frames); copied its PNG frames and the two object meshes unchanged;
encoded `video.mp4` from the frames; evaluated the shipped MANO parameters into 21 joints per hand and drew them onto
the frames (`overlay.mp4`); converted the shipped hand, object and camera data into the arrays of `data.npz`,
`mano_fit.npz` and `objects.npz` (rotation to a z-up world, camera-frame copies, pixel projections, fingertip
subsets, velocities); and rendered the world-frame 3D view (`unscrew_bottle_cap_world3d_only.mp4`). All of these derived files are
shared under the same CC BY-SA 4.0 licence.

Cite OakInk2 in any publication that uses it:

```bibtex
@inproceedings{zhan2024oakink2,
  title     = {{OAKINK2}: A Dataset of Bimanual Hands-Object Manipulation in Complex Task Completion},
  author    = {Zhan, Xinyu and Yang, Lixin and Zhao, Yifei and Mao, Kangrui and Xu, Hanlin and Lin, Zenan and Li, Kailin and Lu, Cewu},
  booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
  year      = {2024}
}
```

The MANO model itself is not part of this folder and has its own licence
([mano.is.tue.mpg.de](https://mano.is.tue.mpg.de)); `mano_fit.npz` holds only OakInk2's fitted parameters.
