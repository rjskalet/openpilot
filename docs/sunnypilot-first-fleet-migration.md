# SunnyPilot-first fleet and Blendful integration — draft

## Source provenance
- Latest SunnyPilot `master` verified November 15, 2026: `c6e564cdcd68dc6334381ac0d929133b1d824479`.
- Matched native upstream opendbc: `c0e74738a8068b2fc88c68f1f475887e1dec290f`.
- Our tailored companion: `rjskalet/opendbc@60fe56fc545fd697472d92322913dd23967febde` (draft PR #15).
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

## Must port/validate before any installation
1. Our existing Mazda donor-EPS logic: firmware-verified, 1200-count envelope and 12/12 limits, proportional torque scaling, speed-dependent EPS rail, non-delivery protections, factory MRCC.
2. Suburban camera-ACC, ALT_ACCS, factory ACC, steering ceiling 300, 10/15 slew, no OP longitudinal.
3. Existing custom openpilot speed-dependent live torque bins, `latcontrol_torque_v2`, torque cache migration, `SuburbanFactorFloorScale`, model-specific tuning; changes must be opt-in until deterministic replay passes.
4. Independently evaluate ZoomPilot's Smooth Big Model, additional steering fault handling, derivative control and model version selection—do not overwrite native SunnyPilot components.
5. Do not import PRs #21–#25 into master; they are references. PR #24 bridge and PR #25 native ZoomPilot base are superseded as migration foundations.
6. Full release and regression validation (Panda safety, CIF, process replay Mazda+GM routes, fingerprint and engagement, Comma 4 build, then supervised road checks).

**Not installation ready.** Do not merge to master or install on either car.
