"""
Per-car deceleration planning limits for Smart Cruise Control.

Derived from ZoomPilot's measured Mazda stock-MRCC response. The planner uses what the
consumer can actually deliver to decide when a curve target must begin affecting the stock
set speed; it does not grant openpilot direct longitudinal actuation.
"""
from dataclasses import dataclass

import numpy as np

from opendbc.sunnypilot.car.icbm_actuation_profile import get_actuation_profile
from openpilot.common.realtime import DT_MDL

_OP_LONG_A_BUDGET = 1.2
_OP_LONG_J_BP = [0., 10., 25., 40.]
_OP_LONG_J_VALS = [1.6, 1.2, 0.8, 0.6]

_STOCK_A_BUDGET = {'mazda': 0.75}
_STOCK_A_BUDGET_DEFAULT = 0.5
_STOCK_RESPONSE_T = 1.0
_MPH_PER_MS = 2.23694
_SERVO_WALK_RATE = {'mazda': 4.0}

COMMIT_FRAC = 0.7
A_PUB_MIN = -2.0
PUB_JERK = 2.0


@dataclass(frozen=True)
class PlanningLimits:
  a_budget: float
  t_lead: float
  op_long: bool
  walk_rate: float = 5.

  def jerk(self, v_ego: float) -> float:
    if not self.op_long:
      return 0.
    return float(np.interp(v_ego, _OP_LONG_J_BP, _OP_LONG_J_VALS))

  @property
  def a_pub_min(self) -> float:
    return -self.a_budget if self.op_long else A_PUB_MIN

  def pub_jerk(self, v_ego: float) -> float:
    return self.jerk(v_ego) or PUB_JERK

  def dash_traversal_time(self, delta_v_ms: float) -> float:
    if self.op_long or delta_v_ms <= 0.:
      return 0.
    return delta_v_ms * _MPH_PER_MS / max(self.walk_rate, 1.)


def get_planning_limits(CP=None) -> PlanningLimits:
  # Unit tests and generic helpers historically constructed SCC controllers without CarParams.
  # Keep that path conservative while production receives the real platform-specific limits.
  if CP is None:
    return PlanningLimits(a_budget=_STOCK_A_BUDGET_DEFAULT, t_lead=_STOCK_RESPONSE_T, op_long=False)

  if CP.openpilotLongitudinalControl:
    return PlanningLimits(a_budget=_OP_LONG_A_BUDGET, t_lead=float(CP.longitudinalActuatorDelay), op_long=True)

  profile = get_actuation_profile(CP.brand)
  return PlanningLimits(a_budget=_STOCK_A_BUDGET.get(CP.brand, _STOCK_A_BUDGET_DEFAULT),
                        t_lead=_STOCK_RESPONSE_T, op_long=False,
                        walk_rate=_SERVO_WALK_RATE.get(CP.brand, profile.tap_rate_hz))


def publish_ramp(a_des: float, a_prev: float, lim: PlanningLimits, v_ego: float, dt: float = DT_MDL) -> float:
  a_des = max(a_des, lim.a_pub_min)
  step = lim.pub_jerk(v_ego) * dt
  return float(min(max(a_des, a_prev - step), a_prev + step))
