# Mazda CX-9 geometry replay — bounded acceptance (November 2026)

**Scope: software replay only. This is not approval to install or drive the port.**

The fork's openpilot PR #22 corrects the shared Mazda CX-9 TC-platform
wheelbase from 3.10 m to a nominal 2.93 m. Mazda's official documentation
lists **2,930 mm** for the 2018 CX-9
(https://www.mazdausa.com/static/manuals/2018/cx9/contents/10020103.html)
and **2,928 mm** for the 2021 CX-9
(https://www.mazdausa.com/static/manuals/2021/cx-9/contents/10020100.html).
The 2021 nominal model rounds 2 mm; it is not an independently measured
center of gravity or tire stiffness.

## Reproduction and findings

- Reference: SunnyPilot `3d8f211d4b2d7c4a981633783232c8ff8dbe402b`.
- Complete replay job: https://github.com/rjskalet/openpilot/actions/runs/42753049024
- Segment: `regenACF84CCF482|2024-08-30--03-21-55--0` (2021 CX-9).
- 64 of 66 process comparisons unchanged; exactly `card` and
  `controlsd` changed; **zero execution errors**. No GM/other-make replay
  changed. The upstream suite explicitly notes poor sensor timing for its
  Mazda route; it is not a dedicated 2018 CX-9 donor-EPS replay.
- `card`: wheelbase 3.10 -> 2.93 m, centerToFront 1.271 -> 1.2013 m
  (wheelbase × 0.41), rotationalInertiaPerMass 2.25417948 -> 2.013726
  (scales with wheelbase squared). All match deterministic model formulas.
- `controlsd`: initial curvature -0.21957369 -> -0.23231345 1/m.
  The curvature ratio matches 3.10/2.93. Exactly three normalized torque
  samples changed by +0.0105461 to +0.0105478. The source segment is
  **not the recognized donor-EPS 2018 configuration**. Stock Mazda
  controller normalization is 800 counts, so these represent about
  8.44 counts on that controller scale, not a 1200-count donor-EPS claim.
- Across the full replay, maximum absolute changes were 0.0234355 m/s²
  in estimated actual lateral acceleration, 0.0186847 m/s² in desired
  lateral acceleration, 0.018675 in friction/feedforward state, 0.027055
  in P state, and 1.86154 in *estimated* lateral jerk (not a commanded
  vehicle jerk). No changed signal reversed sign. The saturation
  indicator rises exactly one 10 ms control frame earlier; there is
  no observed new sustained saturation period.

## Automated review conditions

`accepted_deltas.py` recognizes only the two comparisons from this one
route against this one reference commit. It requires **all** audited
fields, exact changed-field counts, unchanged reference geometry, precise
new geometry and curvature, bounded numerical changes, three specified
torque frames and a single saturation bit transition within the
audited timing window. The full `diff.txt` and `diff_report.txt` remain
available. Any new field, missing reference, other vehicle change,
additional changed frame, larger magnitude, changed reference version,
partial replay, or invalid comparison **fails** the replay job.

This is a documented exception to the upstream *exact byte-for-byte*
replay comparison. It is not a general tolerance increase, a field ignore,
or a regeneration of upstream reference artifacts.

## Remaining hardware gate

Even an accepted 2021 CX-9 software replay **cannot** establish safety
for the 2018 CX-9 with a swapped 2022 donor EPS, or the 2019 Suburban.
Before vehicle installation/driving, verify actual firmware/fingerprint,
Panda safety model and 1200/300 count limits, factory MRCC/ACC, no
openpilot longitudinal actuation, EPS and steering override/rejection
behavior, Comma 4 build, diagnostic/rollback readiness and
vehicle-specific logs. Keep the integration PR in draft and do not merge
or deploy solely because this replay exception passes.
