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
    self.lkas_block_origin_speed: float | None = None
    self.lkas_delivered = False
    self.steer_first_engage_hold = False
    # Panda reports refused bus-0 transmissions on source 192 (0 + 0xC0). Count only
    # refused nonzero steering commands so the controller can resynchronize its limiter.
    self.lkas_rejected = 0

    self.distance_button = 0

  def update_steer_undelivered(self, v_ego_raw: float, lkas_request: float) -> None:
    self.lkas_delivered |= self.lkas_effective != 0
    self.steer_first_engage_hold = (not self.lkas_delivered and self.lkas_blocked and self.lkas_track_state and
                                    v_ego_raw < self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED)

    if not self.lkas_blocked:
      self.steer_undelivered_frames = 0
      self.steer_undelivered = False
      self.lkas_block_origin_speed = None
    elif self.lkas_block_origin_speed is None:
      self.lkas_block_origin_speed = v_ego_raw

    if self.lkas_blocked and not self.steer_undelivered:
      if self.lkas_effective == 0 and abs(lkas_request) > self.params.STEER_UNDELIVERED_MIN:
        self.steer_undelivered_frames += 1
        self.steer_undelivered = self.steer_undelivered_frames >= self.params.STEER_UNDELIVERED_FRAMES
      else:
        self.steer_undelivered_frames = 0

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

    # stock lkas should be on
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      # LKAS_BLOCK is not itself a fault on this EPS; controller-side zero-delivery protection
      # handles sustained non-delivery without turning normal low-speed behavior into a disable.
      ret.steerFaultTemporary = False
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
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [("CAM_TRAFFIC_SIGNS", 0)], 2),
      # Rejected transmit traffic is sporadic and must not affect canValid or timeouts.
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], [("CAM_LKAS", float("nan"))], 192),
    }
