# Replay previews

Full `picking_up_mustard` and `ipman_roll` recordings, rendered with the licensed
UR gantry + Sharpa solver and the application's default camera/human overlay.
First-frame body alignment, no positional offset, unrestricted arm IK, limited
finger joints, kinematic `mj_forward` only. The solver runs at 60 Hz; MP4 is
640×480 at 30 fps, and the looping GIF preview is 480×360 at 15 fps. Playback
uses the recordings' original speed. The full skeleton supplies distal finger
landmarks. The blue overlay is the recorded human, not collision geometry. The exporter copies initialized live overlay geometry
directly, preserving capsule endpoints, radius, color and material.

Regenerate from the repository root with FFmpeg installed and your license set:

```bash
export GEO_KIN_LICENSE="$HOME/.config/geo-kin/licenses/cmu-iam-urgantry.toml"
uv run python scripts/render_replays.py
```

The renderer defaults to EGL for offscreen rendering. `--sample ipman_roll` or
`--sample picking_up_mustard` renders only one recording; `--output PATH` chooses
another output folder. No desktop window or additional Python video dependency
is required. Scene/model provenance and recording sources are documented in
the main README and `dex_ur_gantry_teleop/assets/sample_motion/README.md`.
