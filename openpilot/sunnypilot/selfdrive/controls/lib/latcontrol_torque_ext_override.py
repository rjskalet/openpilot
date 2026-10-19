"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

import numpy as np

from opendbc.sunnypilot.car.interfaces import get_tune_scale
from openpilot.common.params import Params


SUBURBAN_FINGERPRINT = "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"
SUBURBAN_FACTOR_FLOOR_SPEED_BP = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
SUBURBAN_FACTOR_FLOOR_BP = [0.68, 0.68, 0.65, 0.60, 0.70, 0.90, 1.10]
SUBURBAN_FACTOR_FLOOR_SCALE_MIN = 0.90
SUBURBAN_FACTOR_FLOOR_SCALE_MAX = 1.15
SUBURBAN_FACTOR_FLOOR_POLL_FRAMES = 300


def clamp_suburban_factor_floor_scale(scale: float) -> float:
  return float(np.clip(scale, SUBURBAN_FACTOR_FLOOR_SCALE_MIN, SUBURBAN_FACTOR_FLOOR_SCALE_MAX))


def get_suburban_factor_floor(v_ego: float, scale: float) -> float:
  base = float(np.interp(v_ego, SUBURBAN_FACTOR_FLOOR_SPEED_BP, SUBURBAN_FACTOR_FLOOR_BP))
  return base * clamp_suburban_factor_floor_scale(scale)


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
    self._speed_dep_car_cfg = None
    self._last_vego = 0.0

    # Suburban-only applied-factor guardrail. The live learner remains untouched; this
    # floor limits how aggressive the controller can become when a learned factor is low.
    self._suburban_factor_floor_scale = 1.0
    self._live_lat_accel_factor = float(CP.lateralTuning.torque.latAccelFactor)
    self._factor_floor = 0.0
    self._applied_lat_accel_factor = self._live_lat_accel_factor

    # Manual overrides are typed on upstream Mazda's 800-count tune scale. Convert once at
    # the controller boundary so the donor EPS can stay on a flat 1200-count normalization.
    self._tune_scale = get_tune_scale(CP)

  @property
  def live_lat_accel_factor(self) -> float:
    return self._live_lat_accel_factor

  @property
  def factor_floor(self) -> float:
    return self._factor_floor

  @property
  def applied_lat_accel_factor(self) -> float:
    return self._applied_lat_accel_factor

  def _refresh_suburban_factor_floor_scale(self) -> None:
    try:
      scale = float(self.params.get("SuburbanFactorFloorScale", return_default=True))
    except (TypeError, ValueError):
      scale = 1.0
    self._suburban_factor_floor_scale = clamp_suburban_factor_floor_scale(scale)

  def update_override_torque_params(self, torque_params) -> bool:
    # Preserve SunnyPilot v0/manual-override semantics exactly: poll every 300 frames, apply
    # only on that poll frame, and leave those values untouched between polls. Speed-dependent
    # interpolation must never write over a manual override.
    changed = False
    is_suburban = self.CP.carFingerprint == SUBURBAN_FINGERPRINT

    if self.enforce_torque_control_toggle:
      self.frame += 1
      if self.frame % SUBURBAN_FACTOR_FLOOR_POLL_FRAMES == 0:
        self.torque_override_enabled = self.params.get_bool("TorqueParamsOverrideEnabled")
        if is_suburban:
          self._refresh_suburban_factor_floor_scale()

        if self.torque_override_enabled:
          torque_params.latAccelFactor = float(self.params.get("TorqueParamsOverrideLatAccelFactor", return_default=True)) * self._tune_scale
          torque_params.friction = float(self.params.get("TorqueParamsOverrideFriction", return_default=True)) / self._tune_scale
          changed = True

      if self.torque_override_enabled:
        # The Suburban offset shadow must be reasserted after controlsd copies the global
        # learned torque params each frame. Otherwise the live offset silently leaks back
        # into actuation before update_speed_dep_torque runs later in the loop.
        if is_suburban and self._speed_dep_active and torque_params.latAccelOffset != 0.0:
          torque_params.latAccelOffset = 0.0
          changed = True
        self._factor_floor = 0.0
        self._live_lat_accel_factor = float(torque_params.latAccelFactor)
        self._applied_lat_accel_factor = float(torque_params.latAccelFactor)
        return changed

    if self._speed_dep_active and self._speed_dep_speed_bp:
      live_lat_accel_factor = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_lat_accel_factor_bp))
      new_lat_accel_factor = live_lat_accel_factor
      new_friction = float(np.interp(self._last_vego, self._speed_dep_speed_bp, self._speed_dep_friction_bp))

      factor_floor = 0.0
      if is_suburban:
        # Clamp the floor scale even if the persisted value was written outside the UI.
        factor_floor = get_suburban_factor_floor(self._last_vego, self._suburban_factor_floor_scale)
        new_lat_accel_factor = max(live_lat_accel_factor, factor_floor)

      # Cap'n Proto stores Float32. Compare in the same precision so we don't report a change
      # and rebuild PID limits on every frame for a numerically identical value.
      live_lat_accel_factor = float(np.float32(live_lat_accel_factor))
      factor_floor = float(np.float32(factor_floor))
      new_lat_accel_factor = float(np.float32(new_lat_accel_factor))
      new_friction = float(np.float32(new_friction))

      self._live_lat_accel_factor = live_lat_accel_factor
      self._factor_floor = factor_floor

      if new_lat_accel_factor != torque_params.latAccelFactor or new_friction != torque_params.friction:
        torque_params.latAccelFactor = new_lat_accel_factor
        torque_params.friction = new_friction
        changed = True

      # This is deliberately after the live/global parameter copy and on every control frame.
      if is_suburban and torque_params.latAccelOffset != 0.0:
        torque_params.latAccelOffset = 0.0
        changed = True

    else:
      self._live_lat_accel_factor = float(torque_params.latAccelFactor)
      self._factor_floor = 0.0

    self._applied_lat_accel_factor = float(torque_params.latAccelFactor)
    return changed
