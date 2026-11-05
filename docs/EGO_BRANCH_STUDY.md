# Ego hand-motion branch study

Historical measurements below use the two-capsule configuration identified by the JSON hashes. The current configuration adds a third capsule on `wrist_1_link`; rerun the study to compare current safety behavior.

With the current capsules and safety filter, **wrist-up is the better starting choice for bottle-unscrewing and stitching**. Sewing favors wrist-down in the tested placements, but neither branch tracks reliably at the baseline placement. Appliance repair remains poorly tracked in both modes. No global default was changed.

This is a kinematic tracking study, not demonstrated task completion: there is no bottle/fabric/tool contact simulation. Finger IK uses the full recorded hand landmarks; held arm commands also hold the hand. We have not measured fingertip contact success.

## Baseline placement results

Errors are mean index-MCP EEF position errors over both arms after a common one-second startup. Holds are the percentage of valid arm updates rejected. Both modes use the same input and obstacle geometry; startup begins at the corresponding branch home, as in teleop.

| Clip | Hand | Wrist-up mean error / holds | Wrist-down mean error / holds |
|---|---|---:|---:|
| oakink2_unscrew | wuji | 55.8 mm / 0.2% | 169.3 mm / 50.0% |
| oakink2_unscrew | sharpa | 82.2 mm / 52.8% | 171.4 mm / 50.4% |
| stitching | wuji | 30.2 mm / 9.7% | 32.6 mm / 20.2% |
| stitching | sharpa | 33.4 mm / 11.7% | 154.5 mm / 78.6% |
| sewing | wuji | 525.4 mm / 86.3% | 338.6 mm / 84.3% |
| sewing | sharpa | 524.5 mm / 85.6% | 334.6 mm / 86.8% |
| appliance_repair | wuji | 123.3 mm / 64.9% | 104.6 mm / 76.8% |
| appliance_repair | sharpa | 111.0 mm / 49.7% | 127.0 mm / 79.7% |

## Interpretation

- Bottle / Wuji: wrist-up tracks orientation much better (6.2° mean versus 13.2°) and avoids the persistent blocked arm seen with wrist-down. However, mean position error is still 55.8 mm. Neither mode keeps both hands simultaneously within 20 mm and 10° at baseline; low hold rate alone is not sufficient for unscrewing.
- Stitching: wrist-up gives both hands within those tolerances on 20.2% of post-startup frames with Wuji (versus 12.1% down), and 18.4% with Sharpa (versus 0% down). This is an improvement, not reliable completion.
- Sewing: wrist-up frequently reports `selected_branch_unavailable`; both modes also hit `path_blocked`. At a -50 mm workspace Z offset, wrist-down improves to 69.7 mm mean error with Wuji and 74.5 mm with Sharpa, but still holds about 46% of arm updates.
- Appliance repair: wrist-up generally reduces holds, but translation and orientation remain poor. At +50 mm, wrist-down can have lower position error while holding more often. There is no clean winner for task completion.
- `path_blocked` means the sampled joint path to the proposed correction fails the safety guard. A feasible endpoint is insufficient; the local filter is not a global obstacle-avoiding path planner. Branch selection alone does not solve safe entry from home.

## Placement sensitivity and reproducibility

48 runs: four clips × two hands × two fixed branches × workspace Z offsets {-50, 0, +50} mm. The detailed [JSON](ego_branch_study.json) includes mean/p95 position and orientation errors, relative-hand displacement error, joint travel, status counts and bimanual tracking percentage. Its hashes identify the wheel, input streams and safety configuration.

OakInk2 has a documented tabletop. EgoVerse uses a provisional tabletop inferred from the first percentile of fingertip heights; its Z-up convention is also only inferred upstream. All clips lack shoulders, so initial camera heading/XY plus the gantry shoulder-to-table height defines a **virtual fixed body frame**. Results depend on this placement. The sweep does not resolve uncertainty in heading, horizontal placement or table estimation.

Import details and dataset attribution: [recordings README](../recordings/ego_retarget/README.md). Source: [ego_retarget](https://github.com/Ingenio17/ego_retarget/tree/907dbaae8a14c448b5d8a41e8e03bce28c8d5e6e). Data and derived replay streams are CC BY-SA 4.0; original attribution and license files are retained per clip.

```bash
uv run python scripts/study_ego_branches.py recordings/ego_retarget --output docs/ego_branch_study.json
```

## Replay and comparison video

```bash
uv run gantry-teleop --hand wuji --frames recordings/ego_retarget/oakink2_unscrew/frames.npz --wrist-branch wrist_up --safety-filter --loop
```

Change `wrist_up` to `wrist_down` to compare; replace the directory with `stitching`, `sewing`, or `appliance_repair` for the other clips. Keep identical workspace offsets when comparing branches.

[Bottle comparison video: wrist-up left, wrist-down right](../recordings/ego_retarget/previews/oakink2_wuji_branch_comparison.mp4) includes the recorded human hands and the same Rust safety capsules as teleop. Both runs have safety filtering enabled.

## Sewing: average facing direction, wrist-up preferred

The updated sewing stream is `recordings/ego_retarget/sewing/frames_mean_heading.npz`.
It uses the duration-weighted circular mean of normalized horizontal camera-forward
vectors across the whole recording: yaw **53.19°**, compared with **-59.64°** in the
first frame (112.83° change). Camera heading is a proxy: no measured upper-body
orientation is present. Near-vertical/invalid headings are excluded; cancelling
headings are rejected rather than selecting an arbitrary direction. Placement
remains fixed for the entire replay. Initial camera XY and estimated table height
are unchanged, isolating the effect of facing direction.

Wrist-up is now the teleop and renderer default and stays locked. The wheel's
compatibility default is unchanged; direct Python callers should explicitly pass
`wrist_branch="wrist_up"`. The historical tables above remain the original
first-heading experiment, not results for this new stream.

At zero workspace offset, with safety filtering on and the same startup exclusion:

| Hand | First heading: mean error / holds | Mean heading: mean error / holds |
|---|---:|---:|
| Wuji | 525.4 mm / 86.3% | 464.5 mm / 75.1% |
| Sharpa | 524.5 mm / 85.6% | 435.6 mm / 66.5% |

This improves tracking, but both hands still never simultaneously reach the
20 mm / 10° thresholds. Testing ±50 mm vertical workspace offsets does not fix
that. We retain wrist-up as requested; reaching this trajectory needs more than
a heading change. Full results: [mean-heading JSON](sewing_mean_heading_study.json).
[Wuji wrist-up video with safety and human hands](../recordings/ego_retarget/previews/sewing_mean_heading/sewing_wuji.mp4).

```bash
uv run python scripts/import_ego_retarget.py \
  recordings/ego_retarget/sewing/data.npz \
  recordings/ego_retarget/sewing/frames_mean_heading.npz \
  --table-z -0.6171835064888 --heading mean

uv run gantry-teleop --hand wuji \
  --frames recordings/ego_retarget/sewing/frames_mean_heading.npz \
  --wrist-branch wrist_up --safety-filter --loop

uv run python scripts/study_ego_branches.py recordings/ego_retarget \
  --frames recordings/ego_retarget/sewing/frames_mean_heading.npz \
  --branches wrist_up --output docs/sewing_mean_heading_study.json
```

This uses the whole recorded clip and is an offline calibration; it cannot be
computed at the start of a live session without a prior calibration interval.
