"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.sunnypilot.car.interfaces import get_steer_rail_schedule
from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride


class LatControlTorqueExt(NeuralNetworkLateralControl, LatControlTorqueExtOverride):
  def __init__(self, lac_torque, CP, CP_SP, CI):
    NeuralNetworkLateralControl.__init__(self, lac_torque, CP, CP_SP, CI)
    LatControlTorqueExtOverride.__init__(self, CP)
    self._output_overrides_disabled = False
    self.steer_rail_schedule = get_steer_rail_schedule(CP)
    self._commanded = False
    self._applied_torque = 0.0
    self._at_rail = False

  def rail_scale_at(self, v_ego: float) -> float:
    if self.steer_rail_schedule is None:
      return 1.0
    return float(np.interp(v_ego, self.steer_rail_schedule[0], self.steer_rail_schedule[1]))

  @property
  def commanded_torque(self) -> float:
    return -self._output_torque if self._commanded else 0.0

  @property
  def last_error(self) -> float:
    return float(self._pid_log.error) if self._pid_log is not None else 0.0

  @property
  def integrator(self) -> float:
    return float(self._pid.i)

  def set_actuator_state(self, applied_torque: float, at_rail: bool) -> None:
    self._applied_torque = applied_torque
    self._at_rail = at_rail

  def update_override_torque_params(self, torque_params) -> bool:
    self._commanded = False
    changed = LatControlTorqueExtOverride.update_override_torque_params(self, torque_params)
    if self.steer_rail_schedule is not None:
      rail = self.rail_scale_at(self._last_vego)
      if rail != self.lac_torque.steer_max:
        self.lac_torque.steer_max = rail
        changed = True
    return changed

  def disable_output_overrides(self):
    self._output_overrides_disabled = True

  @property
  def overrides_output(self) -> bool:
    return not self._output_overrides_disabled and super().overrides_output

  def update_limits(self):
    if self._output_overrides_disabled:
      return
    super().update_limits()

  def update(self, CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
             desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
             desired_curvature, actual_curvature, steer_limited_by_safety, output_torque):
    self._last_vego = CS.vEgo
    self._commanded = True
    self._ff = ff
    self._pid = pid
    self._pid_log = pid_log
    self._setpoint = setpoint
    self._measurement = measurement
    self._roll_compensation = roll_compensation
    self._lateral_accel_deadzone = lateral_accel_deadzone
    self._desired_lateral_accel = desired_lateral_accel
    self._actual_lateral_accel = actual_lateral_accel
    self._desired_curvature = desired_curvature
    self._actual_curvature = actual_curvature
    self._gravity_adjusted_lateral_accel = gravity_adjusted_lateral_accel
    self._steer_limited_by_safety = steer_limited_by_safety
    self._output_torque = output_torque

    if self._output_overrides_disabled:
      return self._pid_log, self._output_torque

    self.update_calculations(CS, VM, desired_lateral_accel)
    self.update_jerk_aware_torque_control(CS, roll_compensation, gravity_adjusted_lateral_accel)
    self.update_neural_network_feedforward(CS, params, calibrated_pose)
    return self._pid_log, self._output_torque

  def disable_speed_dep_torque(self):
    if not self._speed_dep_active:
      return
    self._speed_dep_active = False
    tune = self.CP.lateralTuning.torque
    self.lac_torque.torque_params.latAccelFactor = tune.latAccelFactor
    self.lac_torque.torque_params.latAccelOffset = tune.latAccelOffset
    self.lac_torque.torque_params.friction = tune.friction
    self.lac_torque.update_limits()

  def update_speed_dep_torque(self, tp, tp_sp):
    if not tp.useParams or tp_sp is None or not tp_sp.speedBinCenters:
      self.disable_speed_dep_torque()
      return

    speed_bp = list(tp_sp.speedBinCenters)
    factors = list(tp_sp.speedBinLatAccelFactors)
    frictions = list(tp_sp.speedBinFrictions)
    valid_bp = list(tp_sp.speedBinValid)

    if self._speed_dep_car_cfg is None:
      from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car
      self._speed_dep_car_cfg = get_speed_dep_config_for_car(self.CP)
    cfg = self._speed_dep_car_cfg
    seed_factors = cfg.get('laf_bp')
    seed_frictions = cfg.get('friction_bp')
    if (seed_factors and seed_frictions and len(seed_factors) == len(speed_bp) and len(seed_frictions) == len(speed_bp)):
      fallback_factors = seed_factors
      fallback_frictions = seed_frictions
    else:
      fallback_factors = [tp.latAccelFactorFiltered] * len(speed_bp)
      fallback_frictions = [tp.frictionCoefficientFiltered] * len(speed_bp)

    self._speed_dep_active = True
    self._speed_dep_speed_bp = speed_bp
    self._speed_dep_lat_accel_factor_bp = [factors[i] if valid_bp[i] else fallback_factors[i] for i in range(len(speed_bp))]
    self._speed_dep_friction_bp = [frictions[i] if valid_bp[i] else fallback_frictions[i] for i in range(len(speed_bp))]

    schedule = cfg.get('steer_max_schedule')
    self._speed_dep_steer_max_schedule = schedule
    if schedule:
      sm_bp, sm_v = schedule
      steer_max_at_bins = [float(np.interp(c, sm_bp, sm_v)) for c in speed_bp]
      self._speed_dep_laf_per_count_bp = [factor / sm for factor, sm in zip(self._speed_dep_lat_accel_factor_bp, steer_max_at_bins, strict=True)]
      self._speed_dep_friction_per_count_bp = [fric * sm for fric, sm in zip(self._speed_dep_friction_bp, steer_max_at_bins, strict=True)]
    else:
      self._speed_dep_laf_per_count_bp = []
      self._speed_dep_friction_per_count_bp = []

    self.lac_torque.torque_params.latAccelFactor = tp.latAccelFactorFiltered
    self.lac_torque.torque_params.latAccelOffset = tp.latAccelOffsetFiltered
    self.lac_torque.torque_params.friction = tp.frictionCoefficientFiltered
    self.lac_torque.update_limits()
