from cereal import custom
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, CarControllerParams, MazdaFlags

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False

    self.params = CarControllerParams(CP)
    self.lkas_blocked = False
    self.lkas_effective = 0
    self.lkas_track_state = False
    self.steer_undelivered_frames = 0
    self.steer_undelivered = False
    self.steer_undelivered_alert = False
    self.lkas_block_origin_speed: float | None = None
    self.lkas_delivered = False
    self.steer_first_engage_hold = False

    # Panda returns refused bus-0 transmit frames on source 192. Only nonzero steering refusals
    # matter to the controller's rate-limit synchronization.
    self.lkas_rejected = 0

    # The Mazda lane-keep setting is independent of openpilot. If the optional CAM_SETTINGS frame
    # is present, remember its setting; cars that never send it retain the permissive default.
    self.lkas_setting_on = True
    self.lkas_setting_invalid = False

    self.distance_button = 0

  def update_steer_undelivered(self, v_ego_raw: float, lkas_request: float) -> None:
    self.lkas_delivered |= self.lkas_effective != 0
    self.steer_first_engage_hold = (not self.lkas_delivered and self.lkas_blocked and self.lkas_track_state and
                                    v_ego_raw < self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED)

    # When the driver's own LKAS setting is off, zero delivery is expected and must not become a
    # donor-EPS delivery fault. lkas_setting_invalid is intentionally last frame's state here,
    # matching ZoomPilot's ordering.
    if not self.lkas_blocked or self.lkas_setting_invalid:
      self.steer_undelivered_frames = 0
      self.steer_undelivered = False
      self.steer_undelivered_alert = False
      self.lkas_block_origin_speed = None
    elif self.lkas_block_origin_speed is None:
      self.lkas_block_origin_speed = v_ego_raw

    if self.lkas_blocked and not self.lkas_setting_invalid and not self.steer_undelivered:
      if self.lkas_effective == 0 and abs(lkas_request) > self.params.STEER_UNDELIVERED_MIN:
        self.steer_undelivered_frames += 1
        self.steer_undelivered = self.steer_undelivered_frames >= self.params.STEER_UNDELIVERED_FRAMES
      else:
        self.steer_undelivered_frames = 0

    if self.steer_undelivered:
      # The first 200 ms latch stops commands before the camera faults. Escalate to a standard
      # temporary steering fault only when the same zero-delivery block persists at road speed,
      # did not originate from the near-zero standby region, and is no longer TRACK_STATE standby.
      self.steer_undelivered_frames += 1
      if (not self.steer_undelivered_alert and not self.lkas_track_state and
          self.steer_undelivered_frames >= self.params.STEER_UNDELIVERED_FRAMES + self.params.STEER_UNDELIVERED_ALERT_FRAMES and
          v_ego_raw >= self.params.STEER_UNDELIVERED_ALERT_MIN_SPEED and
          self.lkas_block_origin_speed is not None and
          self.lkas_block_origin_speed >= self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED):
        self.steer_undelivered_alert = True

  def update(self, can_parsers, starpilot_toggles) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    # TODO: this should be from 0 - 1.
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off on legacy firmware.
    lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1
    self.lkas_blocked = lkas_blocked
    self.lkas_effective = cp.vl["STEER_RATE"]["LKAS_EFFECTIVE"]
    self.lkas_track_state = cp.vl["STEER_RATE"]["LKAS_TRACK_STATE"] == 1

    # ZoomPilot recovery contract: Panda reports refused transmitted CAM_LKAS frames on
    # bus 0 + 0xC0. Ignore refused zero commands while disengaged.
    self.lkas_rejected = sum(1 for request in can_parsers[Bus.loopback].vl_all["CAM_LKAS"]["LKAS_REQUEST"] if request != 0)

    if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      self.update_steer_undelivered(ret.vEgoRaw, cp.vl["STEER_RATE"]["LKAS_REQUEST"])
      self.lkas_allowed_speed = True
    else:
      # LKAS is enabled at 52 kph going up and disabled at 45 kph going down.
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # Stock LKAS must remain enabled for the EPS to apply torque. CAM_SETTINGS is optional; if
    # the car sends it, either intervention bit means enabled. Cars that do not send it keep the
    # default True state. Preserve StarPilot's existing LANE_LINES check as the other invalid gate.
    if len(cp_cam.vl_all["CAM_SETTINGS"]["LKAS_INERVENTION_ON1"]) > 0:
      self.lkas_setting_on = any(cp_cam.vl["CAM_SETTINGS"][s]
                                 for s in ("LKAS_INERVENTION_ON1", "ILKAS_NTERVENTION_ON2"))
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0 or not self.lkas_setting_on
    self.lkas_setting_invalid = ret.invalidLkasSetting

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      # A normal LKAS_BLOCK is not itself a fault on this EPS. Only ZoomPilot's sustained rolling
      # zero-delivery alert becomes a temporary fault after commands have already been suppressed.
      ret.steerFaultTemporary = self.steer_undelivered_alert
    else:
      ret.steerFaultTemporary = self.lkas_allowed_speed and lkas_blocked

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1

    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    fp_ret = custom.StarPilotCarState.new_message()
    fp_ret.dashboardStopSign = 1 if cp_cam.vl["CAM_TRAFFIC_SIGNS"]["STOP_SIGN"] == 9 else 0

    return ret, fp_ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      # Optional camera messages must never affect canValid/canTimeout.
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [
        ("CAM_SETTINGS", float("nan")),
        ("CAM_TRAFFIC_SIGNS", float("nan")),
      ], 2),
      # Rejected outgoing bus-0 transmissions are returned by Panda on source 192. They are
      # sporadic by definition, so this parser is also non-validity-affecting.
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], [("CAM_LKAS", float("nan"))], 192),
    }
