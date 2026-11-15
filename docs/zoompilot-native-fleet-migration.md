# Native ZoomPilot/Blendful fleet migration

**Draft only; not an installation package.** The native base is pinned to ZoomPilot `04e8f79413efb9f16ede11014c546324c48377ce`. Companion `rjskalet/opendbc` is pinned to `403b6e048d19b6e892fb47a25a0ca7297b79c1f7` based on upstream `936309777989f41ce17b5658113164db879a3d7e`.

## Architecture
- Keep ZoomPilot's native modeld, Blendful, `ModelSettings`, big model smoothing, model handoffs, torque tuning and app/UI; no legacy controller cherry-picks.
- `Blendful` remains off until both `Blendful` and new `BlendfulIntegrationValidated` are explicitly true; no UI switch for validation gate.
- Malformed or nonfinite model output disables correction for that frame.

## Vehicles
- 2018 Mazda CX-9: donor EPS identified by firmware, 1200-count normalization, **native 10/10** torque rate safety limits; original CX-9 speed-bin seeds via `MAZDA_CX9_2021` alias, **not** substituted CX-5 seeds. Factory MRCC, no openpilot longitudinal control.
- 2019 Chevrolet Suburban: camera-harness fingerprint, stock GM ACC, `ALT_ACCS` behavior, `pcmCruise=True`, no openpilot longitudinal control, 300-count ceiling, 10/15 slew limits, original measured speed-bin seeds. No experimental U-shaped floor.
- Safety retesting must include an explicit request for Alpha longitudinal, which must still leave the Suburban on factory ACC and the CX-9 on MRCC.

## Remaining blockers
- Automated Suburban CAN fingerprinting is not established by this change; vehicle selection must be verified against real logs, without borrowing a Yukon fingerprint.
- Native ZoomPilot static analysis currently reports 105 diagnostics under this fork's lint configuration, and process replay plus hardware-specific checks remain unresolved. A passing schema check does not imply the branch is installation-ready.
- Complete CI and process replay; reconcile all nonzero steering and carParams differences against known-good reference logs.
- Panda safety tests and car-controller failover behavior on both vehicles.
- Verify native opendbc gitlink, firmware fingerprinting and car compatibility.
- Model selection/retained tuning cache migration and clean initial params.
- Comma 4 build and on-device visual diagnostics; then supervised road validation.

No merge or install before every critical safety gate passes.
