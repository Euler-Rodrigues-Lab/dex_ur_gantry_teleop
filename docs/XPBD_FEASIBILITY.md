# Gantry capsule safety feasibility

Status: preliminary numerical feasibility check, not implemented in the wheel.

## Agreed scope

- Start from the original wrist-up branch used in the first preview (higher
  elbow breaks wrist-height ties), not the later elbow-outward variant.
- One combined palm/wrist capsule per hand; fingertips may protrude.
- Obstacles: tabletop surface and a vertical capsule enclosing the torso column.
- Do not switch IK branches automatically during filtering. Retain continuity;
  if a local correction fails or exceeds a configured budget, hold and report.

## Existing code and geometry

`geo-kin-sew` already provides capsule/capsule closest-distance calculations,
rounded-box/capsule distances, and XPBD multiplier/compliance logic. Its SEW
filter moves elbow/wrist/tip points with link-length constraints. That filter
cannot be plugged directly into the six-DOF gantry: arbitrary adjusted points
need not correspond to its full EEF orientation and joint geometry.

The table boards have top surface world Z=0.775 m. The column is centered at
world (0,-0.3825), with 0.045 m half-width and vertical extent approximately
0.766–1.555 m. A capsule along that extent with radius sqrt(2)*0.045 m encloses
the column box conservatively, including rounded overhang at its ends. It does
not cover the gantry head, mounting plates, or the full base footprint.

## Proposed solver

Use six arm joint angles as projection variables. Compute the protected capsule
from robot FK and a calibrated palm/wrist envelope. Use joint-space derivatives
of the capsule/table and capsule/column gaps, unilateral XPBD multipliers and
compliance scaled by timestep squared. Keep a task-space attraction to the
requested EEF pose, with hard bounds on permitted correction and joint changes.
Recheck final FK capsule gaps and joint constraints before accepting a command.

At an ordinary nonsingular six-DOF pose, fixing all six EEF coordinates leaves
no continuous redundant motion. If the selected exact branch collides, avoiding
the obstacle generally requires a positional and/or orientation deviation.
Priority/weights must make that deviation explicit; exact grasp tracking and
collision clearance cannot both be promised for an infeasible target.

A horizontal half-space is simple and conservative for table clearance; use a
finite box if motions below the surface outside the tabletop should be allowed.
Finger articulation and wrist branch changes affect the capsule envelope. Fit
and validate over the allowed configurations rather than assuming one static
fit at a single pose covers them all. Render exactly the capsule used by the
solver for inspection.

## Numerical probe (2026-11-04)

A temporary Python probe used MuJoCo FK and the existing private reference
capsule-distance routine. It enclosed mesh bounding-box corners of all three
wrist links and the palm in one capsule, excluding finger geometry. The capsule
was refitted each iteration, using finite-difference joint-space gradients and
zero compliance (the hard-constraint limit of XPBD). Targets seeded from the
chosen wrist-up preview intentionally penetrated the table or column.

| Probe | Final table gap | Final column gap | Iterations | EEF translation | EEF rotation |
|---|---:|---:|---:|---:|---:|
| Near table | 5.00 mm | 75.04 mm | 8 | 28.9 cm | 24.2 degrees |
| Near column | 104.13 mm | 5.00 mm | 7 | 12.7 cm | 23.4 degrees |

This demonstrates local constraint projection through the arm FK, not acceptable
retargeting quality. The deliberately conservative capsule radius was about
14–15 cm: too bulky for close manipulation. A tighter envelope, task-preserving
weights, bounded corrections and failure handling are necessary before use.
This probe did not test nonzero compliance, temporal streams, branch retention,
full-arm/self-collision, finger contacts, hardware, or runtime performance.
Finite-difference gradients and per-frame PCA refitting are exploratory tools,
not the proposed real-time production implementation.
