from collections import deque

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, Buttons, MazdaFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(CP)
    self.steer_to_zero = bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS)
    self.apply_torque_last = 0
    self.driver_torque_samples: deque[float] = deque(maxlen=self.params.STEER_DRIVER_SAMPLES)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.brake_counter = 0

  def update(self, CC, CS, now_nanos, starpilot_toggles):
    can_sends = []
    apply_torque = 0
    steer_max = self.params.STEER_MAX

    self.driver_torque_samples.append(CS.out.steeringTorque)

    if CS.lkas_rejected:
      # Panda reports refused transmitted 0x243 frames on loopback bus 192. A rejection resets
      # Panda's steering rate-limit reference, so match that state before the normal limiter runs.
      # This is the current ZoomPilot recovery path and prevents a stale controller ramp from
      # repeatedly producing commands Panda will refuse.
      self.apply_torque_last = 0

    if CC.latActive:
      # ZoomPilot donor architecture: normalized torque always maps onto one fixed 1200-count
      # scale. Vehicle speed changes only the physical EPS clamp below, never the tune scale.
      new_torque = int(round(CC.actuators.torque * steer_max))

      if self.steer_to_zero:
        # Clamp to the measured applied EPS authority so controlsd sees real saturation while
        # preserving a stable torque-controller normalization across speed.
        eps_ceiling = round(float(np.interp(CS.out.vEgoRaw, self.params.EPS_CEILING_LOOKUP[0],
                                            self.params.EPS_CEILING_LOOKUP[1])))
        new_torque = int(np.clip(new_torque, -eps_ceiling, eps_ceiling))

      if new_torque >= 0:
        driver_torque = min(self.driver_torque_samples) - self.params.STEER_DRIVER_MARGIN
      else:
        driver_torque = max(self.driver_torque_samples) + self.params.STEER_DRIVER_MARGIN

      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      driver_torque, self.params, steer_max)

    # ZoomPilot protection for a steer-to-zero EPS reporting sustained zero delivery, including
    # its known first-engagement standby behavior at a crawl. Recovery always starts from zero.
    if self.steer_to_zero and (CS.steer_undelivered or CS.steer_first_engage_hold or CS.lkas_arming):
      apply_torque = 0

    if CC.cruiseControl.cancel:
      # If brake is pressed, wait >70 ms before trying to disable cruise to avoid a race with stock.
      self.brake_counter = self.brake_counter + 1
      if self.frame % 10 == 0 and not (CS.out.brakePressed and self.brake_counter < 7):
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.CANCEL))
    else:
      self.brake_counter = 0
      if CC.cruiseControl.resume and self.frame % 5 == 0:
        can_sends.append(mazdacan.create_button_cmd(self.packer, self.CP, CS.crz_btns_counter, Buttons.RESUME))

    self.apply_torque_last = apply_torque

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
