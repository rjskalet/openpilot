from itertools import pairwise
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import CAR, STEER_TO_ZERO_EPS_FW, CarControllerParams, MazdaFlags, MazdaSafetyFlags


@pytest.fixture
def empty_fingerprint():
  return {i: {} for i in range(8)}


def get_params(fw_version: bytes | None, fingerprint, candidate=CAR.MAZDA_CX9):
  car_fw = [] if fw_version is None else [structs.CarParams.CarFw(ecu="eps", fwVersion=fw_version)]
  return interfaces[candidate].get_params(
    candidate,
    fingerprint,
    car_fw,
    alpha_long=False,
    is_release=False,
    docs=False,
    starpilot_toggles=None,
  )


@pytest.mark.parametrize("fw_version", sorted(STEER_TO_ZERO_EPS_FW))
def test_recognized_eps_firmware_enables_only_donor_capability(fw_version, empty_fingerprint):
  CP = get_params(fw_version, empty_fingerprint)

  assert CP.carFingerprint == CAR.MAZDA_CX9
  assert CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
  assert CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.STEER_TO_ZERO_EPS
  assert CP.minSteerSpeed == 0.0
  assert CP.steerActuatorDelay == pytest.approx(0.14)
  assert CP.dashcamOnly is False
  assert CP.wheelbase == pytest.approx(2.93)
  assert CP.steerRatio == pytest.approx(17.6)


@pytest.mark.parametrize("fw_version", [
  b'UNKNOWN-3210X-A-00\x00\x00\x00\x00\x00\x00\x00',
  b'TK80-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  None,
])
def test_stock_unknown_or_missing_eps_firmware_keeps_legacy_capability(fw_version, empty_fingerprint):
  CP = get_params(fw_version, empty_fingerprint)

  assert CP.carFingerprint == CAR.MAZDA_CX9
  assert not CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
  assert not CP.safetyConfigs[0].safetyParam & MazdaSafetyFlags.STEER_TO_ZERO_EPS
  assert CP.minSteerSpeed == pytest.approx(45 * CV.KPH_TO_MS)
  assert CP.steerActuatorDelay == pytest.approx(0.1)
  assert CP.dashcamOnly is True


@pytest.mark.parametrize("candidate, expected_dashcam_only", [
  (CAR.MAZDA_CX5, True),
  (CAR.MAZDA_3, True),
  (CAR.MAZDA_6, True),
  (CAR.MAZDA_CX9, True),
  (CAR.MAZDA_CX9_2021, False),
  (CAR.MAZDA_CX5_2022, False),
])
def test_non_donor_mazda_behavior_is_unchanged(candidate, expected_dashcam_only, empty_fingerprint):
  CP = get_params(None, empty_fingerprint, candidate)
  assert not CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
  assert CP.dashcamOnly is expected_dashcam_only


def test_donor_controller_limits_use_fixed_zoompilot_scale(empty_fingerprint):
  params = CarControllerParams(get_params(next(iter(STEER_TO_ZERO_EPS_FW)), empty_fingerprint))

  assert params.STEER_MAX == 1200
  assert params.TUNE_STEER_MAX == 800
  assert params.TUNE_SCALE == pytest.approx(1.5)
  assert params.STEER_DELTA_UP == 12
  assert params.STEER_DELTA_DOWN == 12
  assert params.STEER_DRIVER_MULTIPLIER == 15
  assert params.STEER_DRIVER_ALLOWANCE == 15
  assert not hasattr(params, "STEER_MAX_LOOKUP")
  assert params.EPS_CEILING_LOOKUP == (
    [8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5],
    [1148, 1132, 1092, 1048, 1012, 920, 808, 676, 620],
  )


def test_donor_torque_tune_is_converted_once_to_1200_count_units(empty_fingerprint):
  legacy = get_params(None, empty_fingerprint)
  donor = get_params(next(iter(STEER_TO_ZERO_EPS_FW)), empty_fingerprint)

  assert donor.lateralTuning.torque.latAccelFactor == pytest.approx(
    legacy.lateralTuning.torque.latAccelFactor * CarControllerParams.TUNE_SCALE)
  assert donor.lateralTuning.torque.friction == pytest.approx(
    legacy.lateralTuning.torque.friction / CarControllerParams.TUNE_SCALE)


class TestMazdaDonorSteering:
  @pytest.fixture(autouse=True)
  def setup(self, empty_fingerprint):
    self.CP = get_params(next(iter(STEER_TO_ZERO_EPS_FW)), empty_fingerprint)
    self.CS = CarState(self.CP, None)
    self.controller = CarController({Bus.pt: "mazda_2017"}, self.CP)
    self.controller.frame = 1
    self.packer = CANPacker("mazda_2017")
    self.can_parser = CANParser("mazda_2017", [("CAM_LKAS", 100)], 0)
    self.controller_state = SimpleNamespace(
      out=SimpleNamespace(vEgoRaw=10.0, steeringTorque=0.0, brakePressed=False),
      steer_undelivered=False, steer_first_engage_hold=False, lkas_arming=False, lkas_rejected=0,
      crz_btns_counter=0,
      cam_lkas={"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
      cam_laneinfo={
        "LINE_VISIBLE": 0, "LINE_NOT_VISIBLE": 0, "LANE_LINES": 0,
        "BIT1": 0, "BIT2": 0, "BIT3": 0, "NO_ERR_BIT": 0, "S1": 0, "S1_HBEAM": 0,
      },
      lkas_allowed_speed=True,
    )

  def update_delivery(self, *, blocked, effective, request, track=False, speed=10.0):
    self.CS.lkas_blocked = blocked
    self.CS.lkas_effective = effective
    self.CS.lkas_track_state = track
    self.CS.update_steer_undelivered(speed, request)

  def controller_update(self, *, lat_active, torque):
    CC = structs.CarControl(latActive=lat_active)
    CC.actuators.torque = torque
    actuators, sends = self.controller.update(CC.as_reader(), self.controller_state, 0, None)
    steer = next(msg for msg in sends if msg[0] == 0x243)
    self.can_parser.update([self.controller.frame, [steer]])
    return actuators, int(self.can_parser.vl["CAM_LKAS"]["LKAS_REQUEST"]), sends

  def rejected_steer(self, torque, bus=192, frame=0):
    msg = mazdacan.create_steering_control(
      self.packer, self.CP, frame, torque, {"BIT_1": 0, "ERR_BIT_1": 0, "ERR_BIT_2": 0},
    )
    return msg[0], msg[1], bus

  def test_fixed_normalization_is_independent_of_speed(self):
    # 0.5 normalized torque always means 600 counts on the 1200-count donor scale. At 14.5 m/s
    # the physical ceiling is 620, so this request remains 600; the old speed-varying scale made 400.
    self.controller_state.out.vEgoRaw = 14.5
    for _ in range(60):
      actuators, request, _ = self.controller_update(lat_active=True, torque=0.5)
    assert request == 600
    assert actuators.torqueOutputCan == 600
    assert actuators.torque == pytest.approx(0.5)

  def test_physical_eps_ceiling_is_separate_from_normalization(self):
    self.controller_state.out.vEgoRaw = 14.5
    for _ in range(60):
      actuators, request, _ = self.controller_update(lat_active=True, torque=1.0)
    assert request == 620
    assert actuators.torqueOutputCan == 620
    assert actuators.torque == pytest.approx(620 / 1200)

  def test_lkas_block_alone_is_not_rejection_or_undelivered(self):
    for _ in range(self.CS.params.STEER_UNDELIVERED_FRAMES * 2):
      self.update_delivery(blocked=True, effective=1, request=600)
    assert not self.CS.steer_undelivered
    assert self.CS.steer_undelivered_frames == 0
    assert self.CS.lkas_rejected == 0

  @pytest.mark.parametrize("request_value", [-200, -199, 0, 199, 200])
  def test_undelivered_threshold_is_strictly_above_200(self, request_value):
    for _ in range(self.CS.params.STEER_UNDELIVERED_FRAMES + 1):
      self.update_delivery(blocked=True, effective=0, request=request_value)
    assert not self.CS.steer_undelivered
    assert self.CS.steer_undelivered_frames == 0

  @pytest.mark.parametrize("request_value", [-201, 201])
  def test_undelivered_20_frame_latch_and_block_clear(self, request_value):
    for _ in range(self.CS.params.STEER_UNDELIVERED_FRAMES - 1):
      self.update_delivery(blocked=True, effective=0, request=request_value)
    assert not self.CS.steer_undelivered
    self.update_delivery(blocked=True, effective=0, request=request_value)
    assert self.CS.steer_undelivered
    self.update_delivery(blocked=True, effective=0, request=0)
    assert self.CS.steer_undelivered
    self.update_delivery(blocked=False, effective=0, request=0)
    assert not self.CS.steer_undelivered

  def test_invalid_stock_lkas_setting_suppresses_undelivered_latch(self):
    self.CS.lkas_setting_invalid = True
    for _ in range(self.CS.params.STEER_UNDELIVERED_FRAMES + 5):
      self.update_delivery(blocked=True, effective=0, request=600)
    assert not self.CS.steer_undelivered
    assert self.CS.steer_undelivered_frames == 0

  def test_nonzero_effective_resets_counter_regardless_of_sign(self):
    for _ in range(self.CS.params.STEER_UNDELIVERED_FRAMES - 1):
      self.update_delivery(blocked=True, effective=0, request=600)
    self.update_delivery(blocked=True, effective=-1, request=600)
    assert not self.CS.steer_undelivered
    assert self.CS.steer_undelivered_frames == 0

  def test_first_engage_hold_entry_and_release_paths(self):
    self.update_delivery(blocked=True, effective=0, request=0, track=True, speed=0.0)
    assert self.CS.steer_first_engage_hold
    self.update_delivery(blocked=True, effective=0, request=0, track=False, speed=0.3)
    assert not self.CS.steer_first_engage_hold
    self.CS.lkas_delivered = False
    self.update_delivery(blocked=True, effective=0, request=0, track=True, speed=1.0)
    assert not self.CS.steer_first_engage_hold
    self.CS.lkas_delivered = False
    self.update_delivery(blocked=True, effective=1, request=1, track=True, speed=0.3)
    assert not self.CS.steer_first_engage_hold

  def test_carstate_counts_only_nonzero_panda_rejections(self):
    car_state = CarState(self.CP, None)
    parsers = car_state.get_can_parsers(self.CP)
    loopback = parsers[Bus.loopback]
    loopback.update([1, [self.rejected_steer(12), self.rejected_steer(-12, frame=1), self.rejected_steer(0, frame=2)]])
    car_state.update(parsers, None)
    assert car_state.lkas_rejected == 2
    for bus in (0, 2, 128):
      loopback.update([2, [self.rejected_steer(12, bus=bus)]])
      car_state.update(parsers, None)
      assert car_state.lkas_rejected == 0

  def test_loopback_parser_is_optional(self):
    loopback = CarState.get_can_parsers(self.CP)[Bus.loopback]
    assert loopback.can_valid
    loopback.update([10_000_000_000, []])
    assert not loopback.bus_timeout
    assert loopback.can_valid

  def test_lkas_setting_return_holds_until_eps_rearms(self):
    # Last frame invalid -> current setting valid starts the ZoomPilot re-arm hold.
    self.CS.lkas_setting_invalid = True
    self.CS.lkas_blocked = False
    self.CS.lkas_effective = 0
    self.CS.update_lkas_arming(False)
    assert self.CS.lkas_arming

    # The short clear gap after the switch returns must not release the hold early.
    for _ in range(10):
      self.CS.update_lkas_arming(False)
      assert self.CS.lkas_arming

    # Actual delivery is authoritative and ends the hold immediately.
    self.CS.lkas_effective = 1
    self.CS.update_lkas_arming(False)
    assert not self.CS.lkas_arming

    # The controller itself must send zero while the re-arm hold is active.
    self.controller_state.lkas_arming = True
    _, request, _ = self.controller_update(lat_active=True, torque=1.0)
    assert request == 0
    assert self.controller.apply_torque_last == 0

  def test_rejected_nonzero_command_restarts_ramp_from_zero(self):
    for _ in range(10):
      actuators, _, _ = self.controller_update(lat_active=True, torque=1.0)
    assert actuators.torqueOutputCan > self.controller.params.STEER_DELTA_UP
    self.controller_state.lkas_rejected = 1
    actuators, request, _ = self.controller_update(lat_active=True, torque=1.0)
    assert request == self.controller.params.STEER_DELTA_UP
    assert actuators.torqueOutputCan == self.controller.params.STEER_DELTA_UP

  def test_disabled_zero_reenable_sign_crossing_and_rails(self):
    params = self.controller.params
    self.controller_state.out.vEgoRaw = 0.0
    positive = []
    for _ in range(params.EPS_CEILING_LOOKUP[1][0] // params.STEER_DELTA_UP + 5):
      _, request, _ = self.controller_update(lat_active=True, torque=1.0)
      positive.append(request)
    assert max(positive) == params.EPS_CEILING_LOOKUP[1][0]
    assert all(0 <= b - a <= params.STEER_DELTA_UP for a, b in pairwise(positive))
    previous = positive[-1]
    while previous > 0:
      _, request, _ = self.controller_update(lat_active=True, torque=-1.0)
      assert previous - request <= params.STEER_DELTA_DOWN
      previous = request
    assert previous >= -params.STEER_DELTA_UP
    actuators, request, sends = self.controller_update(lat_active=False, torque=-1.0)
    assert actuators.torqueOutputCan == 0
    assert request == 0
    assert sum(msg[0] == 0x243 for msg in sends) == 1
    assert self.controller.apply_torque_last == 0
    _, request, _ = self.controller_update(lat_active=True, torque=-1.0)
    assert request == -params.STEER_DELTA_UP
