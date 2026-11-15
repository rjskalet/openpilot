# Blendful-first ZoomPilot migration (draft only)

This branch is the **first isolated migration change**, not the final ZoomPilot-based release. It ports upstream Blendful curvature math and matching model data fields onto both existing modeld entry points **only as a review/test scaffold**. The next migration steps replace the wider model and lateral-control architecture with the pinned ZoomPilot implementation, avoiding a permanent hybrid fork.

## Provenance
- ZoomPilot develop pinned at `04e8f79413efb9f16ede11014c546324c48377ce` (Blendful source and tests copied verbatim).
- Desired next upstream base: ZoomPilot develop `04e8f79413efb9f16ede11014c546324c48377ce`, companion opendbc `936309777989f41ce17b5658113164db879a3d7e`; verify actual dependency pin before moving.
- Branch starting point: `rjskalet/openpilot` master `ae03dbf65a770a257e401bb326b5a61711be56f4`.

## Implementation in this draft
- Import the native Blendful curvature estimator and upstream unit tests.
- Register the `Blendful` parameter disabled by default.
- Publish seven matching Cap'n Proto fields in `ModelDataV2SP` (ordinals 3–9).
- Apply the optional Blendful curvature delta before action smoothing in both stock modeld and sunnypilot/modeld_v2, with independent runtime state and lane-change gating. When disabled the algorithm returns zero.
- Publish Blendful diagnostics from both modeld paths.

## Stage 2: shared model behavior (pending validation)
- Native ZoomPilot `ModelSettings` drives both modeld paths, including big-model 0.1-second curvature smoothing and inactive-controller curvature synchronization.
- Added fail-closed `BlendfulIntegrationValidated` (off by default); stale persisted Blendful=true alone cannot activate corrections.
- Malformed/non-finite lane or plan geometry disables Blendful for that frame rather than letting the estimator throw.
- Dedicated tests added for the gate, shape/finite checks, smoothing, lane changes and low-speed wrapper.

## Deliberately deferred
- UI renderer and settings menu integration, ZoomPilot ModelSettings/BigModelSmoothing, model-size-specific v0/v1/v2 torque selection, donor-EPS and GM-compatible opendbc rebase, seed/cache migration, comma 4 validation. None of those are represented as done in this PR.
- No vehicle controller, panda, steering limits, longitudinal behavior, MRCC/ACC or device configurations changed by this draft.
- No merge or vehicle installation before all safety, model, and replay gates pass.

## Safety constraints for later integration
- 2018 Mazda CX-9: firmware-verified 2022 donor EPS and appropriate 1200-count normalization; reconcile 12 vs 10 count slew and CX-9 vs CX-5 seeds against hardware evidence. Factory MRCC retained, openpilot longitudinal disabled.
- 2019 Chevy Suburban: preserve `CHEVROLET_SUBURBAN_CAMERA_11TH_GEN`, camera harness, `ALT_ACCS`, GM factory ACC, 300-count steering ceiling, 10/15 count slew, no openpilot longitudinal.
- Preserve original PRs #21–#23 separately as references; do not merge them into this branch.

## Required gates
1. Python Blendful tests and Cap'n Proto schema validation.
2. Both modeld processes publish expected diagnostics without attribute errors; disabled mode must produce zero delta.
3. Model and device compatibility, UI/renderer integration, deterministic replay, Mazda and GM safety tests, Comma 4 build, then supervised controlled validation.
