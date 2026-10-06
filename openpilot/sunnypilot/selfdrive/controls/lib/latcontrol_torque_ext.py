"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.sunnypilot.selfdrive.controls.lib.nnlc.nnlc import NeuralNetworkLateralControl
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride


class LatControlTorqueExt(NeuralNetworkLateralControl, LatControlTorqueExtOverride):
  def __init__(self, lac_torque, CP, CP_SP, CI):
    NeuralNetworkLateralControl.__init__(self, lac_torque, CP, CP_SP, CI)
    LatControlTorqueExtOverride.__init__(self, CP)

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
    valid = list(tp_sp.speedBinValid)
    if self._speed_dep_car_cfg is None:
      from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car
      self._speed_dep_car_cfg = get_speed_dep_config_for_car(self.CP)
    cfg = self._speed_dep_car_cfg
    seed_factors = cfg.get('laf_bp', [tp.latAccelFactorFiltered] * len(speed_bp))
    seed_frictions = cfg.get('friction_bp', [tp.frictionCoefficientFiltered] * len(speed_bp))

    self._speed_dep_active = True
    self._speed_dep_speed_bp = speed_bp
    self._speed_dep_lat_accel_factor_bp = [factors[i] if valid[i] else seed_factors[i] for i in range(len(speed_bp))]
    self._speed_dep_friction_bp = [frictions[i] if valid[i] else seed_frictions[i] for i in range(len(speed_bp))]
    self.lac_torque.torque_params.latAccelOffset = tp.latAccelOffsetFiltered

  def update(self, CS, VM, pid, params, ff, pid_log, setpoint, measurement, calibrated_pose, roll_compensation,
             desired_lateral_accel, actual_lateral_accel, lateral_accel_deadzone, gravity_adjusted_lateral_accel,
             desired_curvature, actual_curvature, steer_limited_by_safety, output_torque):
    self._last_vego = CS.vEgo
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

    self.update_calculations(CS, VM, desired_lateral_accel)
    self.update_jerk_aware_torque_control(CS, roll_compensation, gravity_adjusted_lateral_accel)
    self.update_neural_network_feedforward(CS, params, calibrated_pose)

    return self._pid_log, self._output_torque
