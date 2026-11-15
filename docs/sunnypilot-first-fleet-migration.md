# SunnyPilot-first fleet and Blendful integration — draft

## Source provenance
- Latest SunnyPilot `master` verified November 15, 2026: `c6e564cdcd68dc6334381ac0d929133b1d824479`.
- Matched native upstream opendbc: `c0e74738a8068b2fc88c68f1f475887e1dec290f`.
- Our tailored companion: `rjskalet/opendbc@607dcd052dd6b8f79ba04783dc68d0daf4615c7d` (draft PR #15).
- Working fork prior to integration: `rjskalet/openpilot@ae03dbf65a770a257e401bb326b5a61711be56f4`.

## Why SunnyPilot first?
Current SunnyPilot **already includes** Blendful implementation, Cap'n Proto fields, settings, modeld integration and UI. Do not import older ZoomPilot versions over it. Bring selected ZoomPilot steering refinements only after code comparison and deterministic replay.

## Included in this stage
- Exact latest native SunnyPilot modeld/Blendful and associated matching upstream modules.
- Submodule remapped to our fork's forward-ported custom Mazda and Suburban code.
- `BlendfulIntegrationValidated` persistent parameter, false by default. Blendful remains inert even if its old UI toggle was saved true until a consciously validated installation arms the independent gate.
- Minimal regression tests for the two-factor activation.
- Native latest SunnyPilot panda and other submodule pins are preserved.

## Stage 2: custom torque stack reintroduced under a fail-closed gate
- Reintroduced our v2 torque controller, derivative schedule, center-chatter deadzone and steering-limit classifier, speed-binned live torque learning, vehicle-specific caches, and `CustomReserved19` telemetry schema/service.
- `FleetTorqueIntegrationValidated` defaults to **false**. Stored old toggle values alone cannot turn on v2 or speed-dependent torque updates; new controller/learner activation requires explicit validation.
- Ported the torque estimator integration, preserving SunnyPilot's normal global learning when the gate is off. Vehicle-specific caches and legacy-cache migration activate only after the gate has been explicitly validated.
- Ported offroad Mazda/Suburban tuning initialization behind the gate, and donor EPS tune normalization after the latest SunnyPilot torque tune is configured.
- Keep the original measured Suburban `SuburbanFactorFloorScale` knob but at its neutral 1.0 setting; do not ship PR #21's experimental U-floor tune.
- The controller stack is staged for CI; it is **not** yet certified in newest SunnyPilot.

## Per-vehicle and per-commit rollout protection
- The two original booleans `BlendfulIntegrationValidated` and `FleetTorqueIntegrationValidated` are **not sufficient** on their own.
- Each feature now requires an exact full SHA in its matching `BlendfulValidatedCommitMazda`, `BlendfulValidatedCommitSuburban`, `FleetTorqueValidatedCommitMazda`, or `FleetTorqueValidatedCommitSuburban` key, matching the **running build metadata**, not a stale saved `GitCommit`.
- Mazda approvals apply only to CX-9 firmware-confirmed steer-to-zero EPS. Suburban approvals apply only to the exact camera-harness fingerprint. No automatic cross-car or cross-version carryover.
- Both global enable booleans remain false by default. No validation token is preloaded; this PR cannot enable the experimental logic by itself.
- Do not hand-set these keys until both cars have passed independent replay, Panda and on-device validation.

## Mazda fault-handling decision
- Keep SunnyPilot's default temporary steering fault as a disengaging event; the old donor-EPS temporary-fault-to-silent-warning downgrade is deliberately **not** restored.
- The companion opendbc differentiates permanent EPS faults from camera-only faults; true EPS faults remain permanent and camera-only errors remain temporary.

## Must port/validate before any installation
1. Our existing Mazda donor-EPS logic: firmware-verified, 1200-count envelope and 12/12 limits, proportional torque scaling, speed-dependent EPS rail, non-delivery protections, factory MRCC.
2. Suburban camera-ACC, ALT_ACCS, factory ACC, steering ceiling 300, 10/15 slew, no OP longitudinal.
3. Existing custom openpilot speed-dependent live torque bins, `latcontrol_torque_v2`, torque cache migration, `SuburbanFactorFloorScale`, model-specific tuning; changes must be opt-in until deterministic replay passes.
4. Independently evaluate ZoomPilot's Smooth Big Model, additional steering fault handling, derivative control and model version selection—do not overwrite native SunnyPilot components.
5. Do not import PRs #21–#25 into master; they are references. PR #24 bridge and PR #25 native ZoomPilot base are superseded as migration foundations.
6. Full release and regression validation (Panda safety, CIF, process replay Mazda+GM routes, fingerprint and engagement, Comma 4 build, then supervised road checks).

**Not installation ready.** Do not merge to master or install on either car.

## CI validation follow-up (2026-11-15)
- 2018 Mazda CX-9 factory wheelbase is 2,930 mm; confirmed in the Mazda 2018 CX-9 owner's manual (https://www.mazdausa.com/static/manuals/2018/cx9/contents/10020103.html). The 3.10 m replay reference is not correct for this platform and must not be restored solely to hide replay diffs.
- Mazda `controlsd` and `card` outputs still diverge from the upstream recorded replay baseline. This is an explicitly **open** validation item: analyze the corrected geometry's dynamics and real car logs before accepting a new reference.
- A missing `Conversions as CV` import in the donor EPS initialization was fixed in companion opendbc. New unittest-discoverable fleet carParams, CAN parser/packer and EPS fault checks were added.
- Earlier test functions were not collected by SunnyPilot's unittest runner; these gate/steering-policy tests are now wrapped in unittest classes. Openpilot CI now includes a separate pinned opendbc and native Panda safety job.
- The correct Mazda hardware firmware and real donor-EPS feedback CAN, plus the Suburban's steering safety, need validation before installation; CI success is not sufficient.

- Native Panda safety discovery exposed a donor EPS test-class MRO error. Companion opendbc commit `607dcd052dd6b8f79ba04783dc68d0daf4615c7d` fixes the test class; safety C/controller limits are unchanged. Review the rerun results before clearing the gate.
