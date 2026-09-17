"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car
from openpilot.common.params import Params


class LatControlTorqueExtOverride:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.enforce_torque_control_toggle = self.params.get_bool("EnforceTorqueControl")
    self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
    self.frame = -1
    self._override_lat_accel_factor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
    self._override_friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))

    self._speed_dep_active = False
    self._speed_dep_speed_bp = []
    self._speed_dep_lat_accel_factor_bp = []
    self._speed_dep_friction_bp = []
    self._speed_dep_steer_max_schedule = None
    self._speed_dep_laf_per_count_bp = []
    self._speed_dep_friction_per_count_bp = []
    self._speed_dep_car_cfg = None
    self._last_vego = 0.0

    # Mazda lateral validation branch: preload ZoomPilot's measured speed-dependent
    # CX-9/CX-5-EPS seed table immediately. Live learning can be layered on after the
    # first road validation; these seeds are the values ZoomPilot starts from.
    cfg = get_speed_dep_config_for_car(CP)
    speed_bp = list(cfg.get('speed_bp', []))
    lafs = list(cfg.get('laf_bp', []))
    frictions = list(cfg.get('friction_bp', []))
    if speed_bp and len(lafs) == len(speed_bp) and len(frictions) == len(speed_bp):
      self._speed_dep_car_cfg = cfg
      self._speed_dep_active = True
      self._speed_dep_speed_bp = speed_bp
      self._speed_dep_lat_accel_factor_bp = lafs
      self._speed_dep_friction_bp = frictions
      self._set_per_count_tables(cfg)

  def _set_per_count_tables(self, cfg):
    schedule = cfg.get('steer_max_schedule')
    self._speed_dep_steer_max_schedule = schedule
    if schedule:
      sm_bp, sm_v = schedule
      steer_max_at_bins = [float(np.interp(c, sm_bp, sm_v)) for c in self._speed_dep_speed_bp]
      self._speed_dep_laf_per_count_bp = [laf / sm for laf, sm in zip(self._speed_dep_lat_accel_factor_bp, steer_max_at_bins, strict=True)]
      self._speed_dep_friction_per_count_bp = [fric * sm for fric, sm in zip(self._speed_dep_friction_bp, steer_max_at_bins, strict=True)]
    else:
      self._speed_dep_laf_per_count_bp = []
      self._speed_dep_friction_per_count_bp = []

  def update_override_torque_params(self, torque_params) -> bool:
    changed = False

    # Manual override retains priority when explicitly enabled.
    if self.enforce_torque_control_toggle:
      self.frame += 1
      if self.frame % 300 == 0:
        self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
        if self.torque_override_enabled:
          self._override_lat_accel_factor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
          self._override_friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))

      if self.torque_override_enabled:
        if torque_params.latAccelFactor != self._override_lat_accel_factor or torque_params.friction != self._override_friction:
          torque_params.latAccelFactor = self._override_lat_accel_factor
          torque_params.friction = self._override_friction
          changed = True
        return changed

    # ZoomPilot speed-dependent LAF/friction, interpolated every frame. For the donor
    # Mazda EPS the values are converted through CAN-count space so the 1200->800
    # STEER_MAX transition does not smear the learned response across speed.
    if self._speed_dep_active and self._speed_dep_speed_bp:
      if self._speed_dep_steer_max_schedule and self._speed_dep_laf_per_count_bp:
        sm_bp, sm_v = self._speed_dep_steer_max_schedule
        steer_max = float(np.interp(self._last_vego, sm_bp, sm_v))
        new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_laf_per_count_bp)) * steer_max
        new_fric = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_per_count_bp)) / steer_max
      else:
        new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_lat_accel_factor_bp))
        new_fric = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_bp))

      new_lat_accel_factor = float(np.float32(new_lat_accel_factor))
      new_fric = float(np.float32(new_fric))
      if new_lat_accel_factor != torque_params.latAccelFactor or new_fric != torque_params.friction:
        torque_params.latAccelFactor = new_lat_accel_factor
        torque_params.friction = new_fric
        changed = True

    return changed
