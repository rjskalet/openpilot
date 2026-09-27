# Repository Safety Rules

## Scope

These instructions apply to this repository and all of its subdirectories. They protect the known-good vehicle branches and the safety-critical behavior used by the owner's Comma 4 installations.

## Known-good branches

- `cx9` is the known-good branch for the 2018 Mazda CX-9 with a newer CX-5 donor EPS.
- `suburban-camera-acc` is the known-good branch for the 2019 Chevrolet Suburban using the camera harness and factory GM adaptive cruise control.
- Never modify a known-good branch directly.
- Perform every upstream synchronization, experiment, conflict resolution, or update on a newly created test branch based on the intended known-good branch.
- Never automatically merge a test branch into `cx9` or another known-good branch.
- Never force-push.
- Do not commit, push, merge, rebase, or rewrite history unless the user explicitly requests that exact operation.

## CX-9 behavior that must be preserved

The CX-9 configuration is a 2018 Mazda CX-9 with a newer CX-5 donor EPS. Preserve all of the following unless the user explicitly approves a reviewed change:

- `MAZDA_CX9` vehicle identification.
- Firmware-gated `MazdaFlags.STEER_TO_ZERO_EPS`.
- Firmware-gated `MazdaSafetyFlags.STEER_TO_ZERO_EPS`.
- Stock Mazda MRCC and factory longitudinal control.
- Openpilot longitudinal control must remain disabled.
- Donor-EPS zero-speed steering behavior.
- The existing speed-dependent steering maximum.
- The existing EPS-derived steering ceiling.
- Existing driver-torque limiting.
- Torque controller v2.
- Speed-dependent torque learning.
- Torque-cache persistence and restoration.
- Mazda-specific lateral tuning.
- The shared `CameraOffset` default of `-0.08`.
- Suburban torque-cache parameters intentionally shared across branches.

## Steering and CAN safety constraints

- Panda steer-to-zero mode currently permits no more than 1200 CAN counts.
- Controller code must continue applying the tighter speed-dependent steering schedule and EPS-derived ceiling.
- Do not increase steering authority or broaden Panda safety limits.
- Do not remove, bypass, or weaken driver-torque limits.
- Do not remove, bypass, or weaken CAN safety enforcement.
- Do not alter steer-to-zero delivery protections without explicit review and approval.

## Required review and reporting

Explicitly identify and report every proposed change that affects any of these areas:

- Steering behavior or steering authority.
- Panda or CAN safety logic, limits, flags, or tests.
- Vehicle identification, fingerprints, firmware matching, or platform selection.
- EPS firmware detection or donor-EPS gating.
- Factory ACC/MRCC behavior or longitudinal-control selection.
- Torque learning, torque tuning, or torque-cache persistence/restoration.
- Branch-switch parameter persistence, including shared Suburban parameters.

Do not recommend merging an upstream synchronization until the relevant openpilot, opendbc, Panda safety, Mazda, torque-control, vehicle-identification, process-replay, and branch-persistence tests pass. Report test commands, results, failures, skips, and any coverage gaps before recommending a merge.

## Suburban isolation

Mazda work must not alter the known-good Suburban behavior. Preserve the camera-harness implementation, factory GM adaptive cruise control, disabled openpilot longitudinal control, and shared Suburban torque-cache parameters. Any unavoidable cross-vehicle change requires explicit reporting and review.
