"""
Intelligent Cruise Button Management: a closed-loop servo that walks stock ACC's dash set
speed onto the Smart Cruise target with synthesized button presses.

This keeps SunnyPilot's stock-longitudinal architecture while adopting the measured Mazda
response behavior developed in ZoomPilot.
"""
from dataclasses import dataclass

import numpy as np

from openpilot.cereal import custom
from opendbc.car.structs import car
from opendbc.car import structs
from opendbc.sunnypilot.car.icbm_actuation_profile import get_actuation_profile
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.helpers import get_minimum_set_speed
from openpilot.sunnypilot.selfdrive.car.cruise_ext import CRUISE_BUTTON_TIMER, update_manual_button_timers

ButtonType = car.CarState.ButtonEvent.Type
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource
State = custom.IntelligentCruiseButtonManagement.IntelligentCruiseButtonManagementState
SendButtonState = custom.IntelligentCruiseButtonManagement.SendButtonState
SessionState = custom.LongitudinalPlanSP.SpeedLimit.AssistState

INACTIVE_TIMER = 0.4
DRIVER_PRESS_GRACE_T = 3.0
DRIVER_PRESS_GRACE_FRAMES = int(DRIVER_PRESS_GRACE_T / DT_CTRL)
REACT_DEADBAND = 2
REACT_TIMER = 0.3
RESTORE_QUIET_TIME = 1.0
RESTORE_QUIET_FRAMES = int(RESTORE_QUIET_TIME / DT_CTRL)


@dataclass(frozen=True)
class DecelOvershootParams:
  decel_bp: tuple[float, ...]
  gap_v: tuple[float, ...]
  max_gap: float
  min_decel: float


# Measured Mazda MRCC response: deeper temporary dash gaps request more deceleration from the
# factory ACC. This never commands throttle/brake directly; it only changes the stock set speed.
DECEL_OVERSHOOT_PARAMS: dict[str, DecelOvershootParams] = {
  'mazda': DecelOvershootParams(
    decel_bp=(0.02, 0.09, 0.26, 0.44, 0.73),
    gap_v=(2.0, 4.0, 6.0, 8.5, 10.0),
    max_gap=10.,
    min_decel=0.15,
  ),
}
DECEL_OVERSHOOT_RISE = 10.
DECEL_OVERSHOOT_RELEASE = 3.
DECEL_OVERSHOOT_SOURCES = (LongitudinalPlanSource.sccVision, LongitudinalPlanSource.sccMap,
                           LongitudinalPlanSource.speedLimitAssist)

SEND_BUTTONS = {
  State.increasing: SendButtonState.increase,
  State.decreasing: SendButtonState.decrease,
}


class IntelligentCruiseButtonManagement:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    self.profile = get_actuation_profile(CP.brand)

    self.v_target = 0
    self.v_target_raw = 0
    self.v_target_raw_prev = 0
    self.v_cruise_cluster = 0
    self.v_cruise_min = 0
    self.cruise_button = SendButtonState.none
    self.state = State.inactive
    self.pre_active_timer = 0
    self.restore_quiet_timer = 0
    self.react_deadband = REACT_DEADBAND
    self.down_grace_timer = 0
    self.up_grace_timer = 0

    self.is_ready = False
    self.is_ready_prev = False
    self.is_metric = False
    self.prompt_frozen = False
    self.overshoot_mph: float = 0.0
    self.overshoot_params: DecelOvershootParams | None = DECEL_OVERSHOOT_PARAMS.get(CP.brand)
    self.limiter_active = False

    self.cruise_button_timers = dict(CRUISE_BUTTON_TIMER)

  def update_decel_overshoot(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> float:
    p = self.overshoot_params
    if p is None:
      return 0.0

    want = 0.0
    if (self.is_ready and not self.prompt_frozen and self.down_grace_timer <= 0
        and LP_SP.longitudinalPlanSource in DECEL_OVERSHOOT_SOURCES
        and LP_SP.aTarget < -p.min_decel and CS.vEgo > LP_SP.vTarget):
      want = min(float(np.interp(-LP_SP.aTarget, p.decel_bp, p.gap_v)), p.max_gap)

    if want > self.overshoot_mph:
      self.overshoot_mph = min(want, self.overshoot_mph + DECEL_OVERSHOOT_RISE * DT_CTRL)
    else:
      release = DECEL_OVERSHOOT_RELEASE if self.limiter_active else DECEL_OVERSHOOT_RISE
      self.overshoot_mph = max(want, self.overshoot_mph - release * DT_CTRL)

    return self.overshoot_mph

  def update_calculations(self, CS: car.CarState, LP_SP: custom.LongitudinalPlanSP) -> None:
    speed_conv = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH
    self.limiter_active = LP_SP.longitudinalPlanSource != LongitudinalPlanSource.cruise

    v_target_ms = LP_SP.vTarget
    overshoot_ms = self.update_decel_overshoot(CS, LP_SP) * CV.MPH_TO_MS
    if overshoot_ms > 0:
      v_target_ms = min(v_target_ms, max(CS.vEgo, LP_SP.vTarget) - overshoot_ms)

    self.v_target = round(v_target_ms * speed_conv)
    self.v_target_raw_prev = self.v_target_raw
    self.v_target_raw = round(LP_SP.vTarget * speed_conv)
    self.v_cruise_min = get_minimum_set_speed(self.is_metric)
    self.v_cruise_cluster = round(CS.cruiseState.speedCluster * speed_conv)
    self.react_deadband = REACT_DEADBAND if self.limiter_active or self.overshoot_mph > 0 else 1

  def update_restore_quiet_timer(self) -> None:
    up_error = self.v_target_raw - self.v_cruise_cluster
    if self.prompt_frozen:
      self.restore_quiet_timer = 0
    elif up_error >= self.react_deadband and self.v_target_raw == self.v_target_raw_prev:
      self.restore_quiet_timer += 1
    else:
      self.restore_quiet_timer = 0

  def update_state_machine(self) -> custom.IntelligentCruiseButtonManagement.SendButtonState:
    self.pre_active_timer = max(0, self.pre_active_timer - 1)
    self.update_restore_quiet_timer()

    if self.prompt_frozen and self.state in (State.preActive, State.increasing, State.decreasing):
      self.state = State.holding

    if self.state != State.inactive:
      if not self.is_ready:
        self.state = State.inactive
      else:
        up_allowed = ((self.overshoot_mph > 0 and self.limiter_active)
                      or not self.profile.decel_needs_stable_setpoint
                      or self.restore_quiet_timer >= RESTORE_QUIET_FRAMES)
        up_allowed = up_allowed and self.up_grace_timer <= 0
        down_allowed = (self.limiter_active or self.overshoot_mph <= 0) and self.down_grace_timer <= 0

        if self.state == State.preActive:
          if self.pre_active_timer <= 0:
            if self.v_target - self.v_cruise_cluster >= self.react_deadband and up_allowed:
              self.state = State.increasing
            elif self.v_cruise_cluster - self.v_target >= self.react_deadband \
                 and self.v_cruise_cluster > self.v_cruise_min and down_allowed:
              self.state = State.decreasing
            else:
              self.state = State.holding

        elif self.state == State.holding and not self.prompt_frozen:
          down_pending = self.v_cruise_cluster - self.v_target >= self.react_deadband and down_allowed
          up_pending = self.v_target - self.v_cruise_cluster >= self.react_deadband and up_allowed
          if down_pending or up_pending:
            self.pre_active_timer = int(REACT_TIMER / DT_CTRL)
            self.state = State.preActive

        elif self.state == State.increasing:
          if self.v_target <= self.v_cruise_cluster:
            self.state = State.holding

        elif self.state == State.decreasing:
          if self.v_target >= self.v_cruise_cluster or self.v_cruise_cluster <= self.v_cruise_min:
            self.state = State.holding

    elif self.state == State.inactive:
      if self.is_ready and not self.is_ready_prev:
        self.pre_active_timer = int(INACTIVE_TIMER / DT_CTRL)
        self.state = State.preActive

    return SEND_BUTTONS.get(self.state, SendButtonState.none)

  def update_readiness(self, CS: car.CarState, CC: car.CarControl) -> None:
    update_manual_button_timers(CS, self.cruise_button_timers)

    ready = CC.enabled and not CC.cruiseControl.override and not CC.cruiseControl.cancel and not CC.cruiseControl.resume
    button_pressed = any(self.cruise_button_timers[k] > 0 for k in self.cruise_button_timers)

    # Give the driver's set-speed intent priority for three seconds in the opposite direction.
    if self.cruise_button_timers[ButtonType.accelCruise] > 0:
      self.down_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.up_grace_timer = 0
    elif self.cruise_button_timers[ButtonType.decelCruise] > 0:
      self.up_grace_timer = DRIVER_PRESS_GRACE_FRAMES
      self.down_grace_timer = 0
    else:
      self.down_grace_timer = max(0, self.down_grace_timer - 1)
      self.up_grace_timer = max(0, self.up_grace_timer - 1)

    self.is_ready = ready and not button_pressed

  def run(self, CS: car.CarState, CC: car.CarControl, LP_SP: custom.LongitudinalPlanSP, is_metric: bool) -> None:
    if self.CP_SP.pcmCruiseSpeed:
      return

    self.is_metric = is_metric
    self.prompt_frozen = LP_SP.speedLimit.assist.state == SessionState.preActive

    self.update_calculations(CS, LP_SP)
    self.update_readiness(CS, CC)
    self.cruise_button = self.update_state_machine()
    self.is_ready_prev = self.is_ready
