"""ZoomPilot v2 torque controller isolated to verified donor-EPS Mazda platforms.

This is intentionally separate from StarPilot's general LatControlTorque. The Mazda donor
architecture was validated and tuned as one system: fixed torque units, the physical EPS rail,
filtered-jerk friction shaping, driver hand-back, low-speed damping, and curvature buffering.
Keeping it Mazda-local prevents those assumptions from leaking into StarPilot's other cars.
"""
import math
from collections import deque

import numpy as np

from cereal import log
from opendbc.car.lateral import get_friction
from opendbc.car.mazda.values import CarControllerParams
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.pid import PIDController
from openpilot.selfdrive.controls.lib.latcontrol import LatControl

VERSION = 2

# ZoomPilot v0/v2 base gains from the known-good CX-9 architecture.
KP = 1.0
KI = 0.3
INTERP_SPEEDS = [1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0, 30.0]
KP_INTERP = [250.0, 120.0, 65.0, 30.0, 11.5, 5.5, 3.5, 2.0, KP]
KD_INTERP_SPEEDS = [7.5, 10.0, 12.0, 14.5]
KD_INTERP = [1.65, 1.05, 0.85, 0.0]

LP_FILTER_CUTOFF_HZ = 1.2
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
FRICTION_THRESHOLD = 0.3
MAX_FRICTION_JERK = 2.5

CENTER_CHATTER_JERK_DEADZONE_SPEED_BP = [0.0, 5.0, 12.0, 25.0]
CENTER_CHATTER_JERK_DEADZONE_SPEED_V = [0.08, 0.12, 0.18, 0.18]
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP = [0.0, 0.18, 0.35]
CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V = [1.0, 1.0, 0.0]

STEER_RELEASE_I_DECAY = 0.8
RELEASE_ERROR_RAMP_T = 0.3


def get_center_chatter_jerk_deadzone(v_ego: float, setpoint: float) -> float:
  center_weight = np.interp(abs(setpoint), CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_BP,
                            CENTER_CHATTER_JERK_DEADZONE_LAT_ACCEL_V)
  if center_weight == 0.0:
    return 0.0
  speed_deadzone = np.interp(max(v_ego, 0.0), CENTER_CHATTER_JERK_DEADZONE_SPEED_BP,
                             CENTER_CHATTER_JERK_DEADZONE_SPEED_V)
  return float(speed_deadzone * center_weight)


class LatControlTorqueZoomPilot(LatControl):
  """ZoomPilot v2 lateral controller for Mazda's verified steer-to-zero donor EPS."""

  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.CP = CP
    self.controller_params = CarControllerParams(CP)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI,
                             [KD_INTERP_SPEEDS, KD_INTERP], rate=1 / self.dt)

    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.curvature_request_buffer = deque([0.0] * self.request_buffer_len,
                                          maxlen=self.request_buffer_len)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(
      0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt,
    )
    self.jerk_filter = FirstOrderFilter(
      0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt,
    )
    self.prev_steering_pressed = False
    self._release_error_ramp = 1.0
    self.last_error = 0.0
    self.last_command = 0.0
    self._speed_bin_active = False
    self._speed_bin_bp: list[float] = []
    self._speed_bin_factor: list[float] = []
    self._speed_bin_friction: list[float] = []
    self._global_lat_accel_factor = self.torque_params.latAccelFactor
    self._global_lat_accel_offset = self.torque_params.latAccelOffset
    self._global_friction = self.torque_params.friction

    self.update_rail(0.0)

  def rail_scale_at(self, v_ego: float) -> float:
    ceiling = float(np.interp(v_ego, self.controller_params.EPS_CEILING_LOOKUP[0],
                              self.controller_params.EPS_CEILING_LOOKUP[1]))
    return ceiling / self.controller_params.STEER_MAX

  def update_rail(self, v_ego: float) -> None:
    rail = self.rail_scale_at(v_ego)
    if abs(rail - self.steer_max) > 1e-9:
      self.steer_max = rail
      self.update_limits()

  def update_limits(self) -> None:
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    """StarPilot's global live-torque hook, in the donor's already-converted 1200-count units."""
    self._global_lat_accel_factor = latAccelFactor
    self._global_lat_accel_offset = latAccelOffset
    self._global_friction = friction
    if not self._speed_bin_active:
      self.torque_params.latAccelFactor = latAccelFactor
      self.torque_params.friction = friction
    self.torque_params.latAccelOffset = latAccelOffset
    self.update_limits()

  def update_speed_bin_torque_params(self, speed_bp, factors, frictions) -> None:
    """Install validated per-speed-bin values published by the Mazda torque learner."""
    if not speed_bp or len(speed_bp) != len(factors) or len(speed_bp) != len(frictions):
      self.clear_speed_bin_torque_params()
      return
    self._speed_bin_bp = list(speed_bp)
    self._speed_bin_factor = list(factors)
    self._speed_bin_friction = list(frictions)
    self._speed_bin_active = True

  def clear_speed_bin_torque_params(self) -> None:
    was_active = self._speed_bin_active
    self._speed_bin_active = False
    self._speed_bin_bp = []
    self._speed_bin_factor = []
    self._speed_bin_friction = []
    if was_active:
      # Never leave the last interpolated bin value latched after live torque/bin data goes
      # unavailable or a user takes manual torque-param control.
      self.torque_params.latAccelFactor = self._global_lat_accel_factor
      self.torque_params.latAccelOffset = self._global_lat_accel_offset
      self.torque_params.friction = self._global_friction
      self.update_limits()

  def _apply_speed_bin_torque_params(self, v_ego: float) -> None:
    if not self._speed_bin_active:
      return
    factor = float(np.interp(v_ego, self._speed_bin_bp, self._speed_bin_factor))
    friction = float(np.interp(v_ego, self._speed_bin_bp, self._speed_bin_friction))
    if factor != self.torque_params.latAccelFactor or friction != self.torque_params.friction:
      self.torque_params.latAccelFactor = factor
      self.torque_params.friction = friction
      self.torque_params.latAccelOffset = self._global_lat_accel_offset
      self.update_limits()

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature,
             curvature_limited, lat_delay, calibrated_pose, model_data, starpilot_toggles):
    del calibrated_pose, model_data, starpilot_toggles

    self._apply_speed_bin_torque_params(CS.vEgo)
    self.update_rail(CS.vEgo)

    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    measured_curvature = -VM.calc_curvature(
      math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll,
    )
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2

    if not active:
      output_torque = 0.0
      pid_log.active = False
      # Keep the delayed-request and derivative state warm while inactive. LatControl.reset()
      # only resets saturation state, so the v2 integrator survives brief StarPilot AOL/MADS
      # transitions exactly as it did in the archived ZoomPilot architecture.
      self.curvature_request_buffer.append(desired_curvature)
      self.previous_measurement = measurement
      self.measurement_rate_filter.x = 0.0
      self.jerk_filter.x = 0.0
      self._release_error_ramp = 1.0
      self.last_error = 0.0
      self.last_command = 0.0
    else:
      if self.prev_steering_pressed and not CS.steeringPressed:
        self.pid.i *= STEER_RELEASE_I_DECAY
        self._release_error_ramp = 0.0
      self._release_error_ramp = min(1.0, self._release_error_ramp + self.dt / RELEASE_ERROR_RAMP_T)

      roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
      curvature_deadzone = abs(VM.calc_curvature(
        math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0,
      ))
      lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

      delay_frames = int(np.clip(lat_delay / self.dt, 1, self.request_buffer_len))
      expected_lateral_accel = self.curvature_request_buffer[-delay_frames] * CS.vEgo ** 2
      self.curvature_request_buffer.append(desired_curvature)
      gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
      desired_lateral_jerk = (
        future_desired_lateral_accel - expected_lateral_accel
      ) / max(lat_delay, self.dt)

      measurement_rate = self.measurement_rate_filter.update(
        (measurement - self.previous_measurement) / self.dt,
      )
      self.previous_measurement = measurement

      setpoint = lat_delay * desired_lateral_jerk + expected_lateral_accel
      error = (setpoint - measurement) * self._release_error_ramp
      pid_log.error = float(error)
      self.last_error = float(error)

      ff = gravity_adjusted_future_lateral_accel - self.torque_params.latAccelOffset
      shaped_jerk = self.jerk_filter.update(float(np.clip(
        desired_lateral_jerk, -MAX_FRICTION_JERK, MAX_FRICTION_JERK,
      )))
      friction_deadzone = get_center_chatter_jerk_deadzone(CS.vEgo, setpoint)
      friction_jerk = math.copysign(max(abs(shaped_jerk) - friction_deadzone, 0.0), shaped_jerk)
      friction_error = expected_lateral_accel + friction_jerk * lat_delay - measurement
      ff += get_friction(friction_error, lateral_accel_deadzone, FRICTION_THRESHOLD,
                         self.torque_params)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5.0
      error_rate = 0.0 if CS.steeringPressed else -measurement_rate
      output_lataccel = self.pid.update(
        pid_log.error,
        error_rate,
        feedforward=ff,
        speed=CS.vEgo,
        freeze_integrator=freeze_integrator,
      )
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque)
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(
        self.steer_max - abs(output_torque) < 1e-3,
        CS,
        steer_limited_by_safety,
        curvature_limited,
      ))
      self.last_command = float(-output_torque)

    self.prev_steering_pressed = CS.steeringPressed
    return -output_torque, 0.0, pid_log
