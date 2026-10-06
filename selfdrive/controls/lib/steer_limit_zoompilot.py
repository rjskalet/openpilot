"""ZoomPilot steering-limit classifier for the donor-EPS Mazda lateral path.

StarPilot's generic torque mismatch flag treats normal Mazda 12-count/frame slew as a safety
limit. ZoomPilot separates expected rate limiting, the measured EPS rail, and driver limiting
so the v2 integrator freezes only when continuing in the same error direction would wind it up.
"""
from typing import NamedTuple

MISMATCH_THRESHOLD = 1e-2
RAIL_EPS = 1e-3
RATE_STEP_FRACTION = 0.9


class SteerLimit(NamedTuple):
  limited: bool
  driver_limited: bool
  rate_limited: bool
  at_rail: bool


CLEAN = SteerLimit(False, False, False, False)


def classify(cmd: float, applied: float, applied_prev: float | None, slew_up: float, slew_down: float,
             rail_scale: float, upstream_flag: bool, error_prev: float, integrator: float) -> SteerLimit:
  mag = abs(applied)
  at_rail = mag >= rail_scale - RAIL_EPS or mag >= 1.0 - RAIL_EPS
  if abs(cmd - applied) <= MISMATCH_THRESHOLD:
    return SteerLimit(False, False, False, at_rail)
  if applied_prev is None:
    return SteerLimit(bool(upstream_flag), bool(upstream_flag), False, at_rail)

  move = applied - applied_prev
  step = slew_up if mag > abs(applied_prev) else slew_down
  toward = move * (cmd - applied_prev) > 0.0
  rate_limited = toward and (
    abs(move) >= RATE_STEP_FRACTION * step or
    abs(cmd - applied) <= abs(move) + MISMATCH_THRESHOLD
  )

  # Keep upstream's actuator-rate windup protection, but allow a stale integrator to decay
  # toward zero and don't call the physical EPS rail a safety clamp.
  deepening = error_prev * integrator >= 0.0
  return SteerLimit(
    not at_rail and deepening,
    not rate_limited and not at_rail,
    rate_limited,
    at_rail,
  )
