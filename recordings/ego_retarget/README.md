# ego_retarget replay inputs

Downloaded from [Ingenio17/ego_retarget](https://github.com/Ingenio17/ego_retarget), revision `907dbaae8a14c448b5d8a41e8e03bce28c8d5e6e`.

Each clip directory retains the upstream `data.npz`, `meta.json`, README and CC BY-SA 4.0 LICENSE. `frames.npz` is our conversion, also CC BY-SA 4.0. See each upstream README for the original dataset attribution and citations (OakInk2 / EgoVerse). No MANO models or H2O recordings are included.

Changes: renamed skeleton joints to OpenXR identifiers, retained all 21 joints per hand, reconstructed the palm orientation from landmarks rather than interpreting MANO root orientation as an XR wrist, sampled irregular timestamps causally at 60 Hz, and added a fixed virtual shoulder frame. Invalid hands remain missing. No body or elbow tracking is fabricated; the overlay contains recorded hands only.

The fixed frame uses initial camera XY and horizontal camera heading. Its height is 0.60474159 m above the source tabletop, matching the gantry shoulder-to-table distance. OakInk2 documents table Z=0. EgoVerse has no measured table plane: `conversion.json` records a provisional 1st-percentile fingertip height, with Z-up inferred from upstream documentation. This is a placement assumption, not recovered calibration. `--workspace-offset` allows controlled placement changes.

Reproduce OakInk2 after cloning the pinned source:

```bash
uv run python scripts/import_ego_retarget.py /path/to/ego_retarget/data/oakink2/unscrew_bottle_cap/data.npz recordings/ego_retarget/oakink2_unscrew/frames.npz --table-z 0
```

For EgoVerse use the table Z recorded in each `conversion.json`. To override the virtual frame, pass `--body-origin X Y Z` in source-world metres.

The videos under `previews/` show derived kinematic replay with the recorded hand overlay and Rust safety capsules; they do not simulate object interaction.

For sewing, `frames_mean_heading.npz` adds an alternative fixed placement using
the duration-weighted circular mean horizontal camera heading (`--heading mean`).
Its origin and tabletop estimate are unchanged. This camera-derived proxy is not
measured shoulder orientation. The original `frames.npz` is retained for comparison.
Prefer `--wrist-branch wrist_up --safety-filter`; see the updated branch report.
