"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from openpilot.common.params import Params


class LatControlTorqueExtOverride:
  def __init__(self, CP):
    self.CP = CP
    self.params = Params()
    self.enforce_torque_control_toggle = self.params.get_bool("EnforceTorqueControl")  # only during init
    self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
    self.frame = -1

    # ZoomPilot speed-dependent torque state. It is inert until controlsd explicitly supplies
    # live bins to a v2 Mazda controller, so the original SunnyPilot override behavior below
    # remains unchanged for every other controller and car.
    self._speed_dep_active = False
    self._speed_dep_speed_bp = []
    self._speed_dep_lat_accel_factor_bp = []
    self._speed_dep_friction_bp = []
    self._speed_dep_steer_max_schedule = None
    self._speed_dep_laf_per_count_bp = []
    self._speed_dep_friction_per_count_bp = []
    self._speed_dep_car_cfg = None
    self._last_vego = 0.0

  def update_override_torque_params(self, torque_params) -> bool:
    # Preserve SunnyPilot v0/manual-override semantics exactly: poll every 300 frames, apply
    # only on that poll frame, and leave those values untouched between polls. Speed-dependent
    # interpolation must never write over a manual override.
    if self.enforce_torque_control_toggle:
      self.frame += 1
      if self.frame % 300 == 0:
        self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")

        if self.torque_override_enabled:
          torque_params.latAccelFactor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
          torque_params.friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True))
          return True

      if self.torque_override_enabled:
        return False

    if self._speed_dep_active and self._speed_dep_speed_bp:
      if self._speed_dep_steer_max_schedule and self._speed_dep_laf_per_count_bp:
        sm_bp, sm_v = self._speed_dep_steer_max_schedule
        steer_max = float(np.interp(self._last_vego, sm_bp, sm_v))
        new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_laf_per_count_bp)) * steer_max
        new_friction = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_per_count_bp)) / steer_max
      else:
        new_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_lat_accel_factor_bp))
        new_friction = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_bp))

      # Cap'n Proto stores Float32. Compare in the same precision so we don't report a change
      # and rebuild PID limits on every frame for a numerically identical value.
      new_lat_accel_factor = float(np.float32(new_lat_accel_factor))
      new_friction = float(np.float32(new_friction))
      if new_lat_accel_factor != torque_params.latAccelFactor or new_friction != torque_params.friction:
        torque_params.latAccelFactor = new_lat_accel_factor
        torque_params.friction = new_friction
        return True

    return False
