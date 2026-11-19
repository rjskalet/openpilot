# Mazda CX-9 geometry replay — bounded acceptance (November 2026)

**Scope: software replay only. This is not approval to install or drive the port.**

The fork's openpilot PR #26 retains the previously reviewed correction to the shared Mazda CX-9 TC-platform
wheelbase from 3.10 m to a nominal 2.93 m. Mazda's official documentation
lists **2,930 mm** for the 2018 CX-9
(https://www.mazdausa.com/static/manuals/2018/cx9/contents/10020103.html)
and **2,928 mm** for the 2021 CX-9
(https://www.mazdausa.com/static/manuals/2021/cx-9/contents/10020100.html).
The 2021 nominal model rounds 2 mm; it is not an independently measured
center of gravity or tire stiffness.

## Reproduction and findings

- Reference: SunnyPilot `9489b2f4815c463baa32439a46566f342b4500d3`.
- Complete replay job: https://github.com/rjskalet/openpilot/actions/runs/43772507310
- Previous reviewed reference: PR #22, whose 1,293 changed-value entries in the same `card` and `controlsd` comparisons are byte-for-byte identical in the GitHub job logs. New SunnyPilot reference is pinned above.
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

## PR #26 acceptance restoration

The November 15, 2026 workflow failed only because the strict replay acceptance gate from PR #22 was not transferred to PR #26 when rebasing on the SunnyPilot-first foundation. Raw GitHub Actions logs for the two PRs contain identical ordered 1,293 changed-value entries (1,287 `controlsd`, six `card`) for the same 2021 CX-9 segment. The 64 other process comparisons passed and no execution error occurred. This restoration updates the reference commit to the new upstream SunnyPilot pin without regenerating golden output or changing any vehicle controller code. The reference pin and all count/value checks remain strict, so any future change fails closed. This is still not hardware, donor-EPS, or driving validation.

### November 18 reference artifact reconciliation

SunnyPilot's process-replay artifact baseline advanced from `c6e564cdcd68dc6334381ac0d929133b1d824479` to
`9489b2f4815c463baa32439a46566f342b4500d3` after the earlier review. The two intervening upstream
commits modify only model download management and model-selection UI/tests,
not Mazda carParams, controlsd, torque-control, or process-replay logic.

Compared the complete ordered `diff.txt` changed-value entries from the original
reviewed full replay (GitHub Actions run `43772507310`, old reference) against
PR #28's full replay (run `44430425874`, new reference): all **1,293 of
1,293** entries matched exactly. Both runs report **2 changed, 64 passed,
0 errors**. The 1,293 changes remain subject to the unchanged field/count,
finite-value, magnitude, sign, torque, geometry, saturation, and reference
identity checks in `accepted_deltas.py`. This correction changes **only**
the required artifact commit identity, not the acceptance thresholds or the
underlying golden data. A fresh complete CI run must still pass before this
software-only replay exception is considered accepted; this is not donor-EPS
hardware validation or permission to engage steering.
