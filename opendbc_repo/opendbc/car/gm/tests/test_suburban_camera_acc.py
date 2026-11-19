"""Regression checks for the camera-harness 11th-gen Suburban ZoomPilot port.

These tests preserve its factory GM ACC and steering safety envelopes without
changing ZoomPilot's Mazda, Yukon, or other GM vehicle implementations.
"""
from types import SimpleNamespace
import pytest

from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.gm.carstate import CarState, REJECTED_LOOPBACK
from opendbc.car.gm.interface import CarInterface
from opendbc.car.gm.values import ALT_ACCS, CAMERA_ACC_CAR, CAR, DBC, CanBus, CarControllerParams, GMSafetyFlags
from opendbc.sunnypilot.car.lateral_tune import get_speed_dep_config_for_car


SUBURBAN = CAR.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN


def test_suburban_camera_acc_platform_keeps_original_steering_envelope():
  assert SUBURBAN in CAMERA_ACC_CAR
  assert SUBURBAN in ALT_ACCS
  assert SUBURBAN != CAR.CHEVROLET_SUBURBAN_NON_ACC_11TH_GEN
  cp = SimpleNamespace(carFingerprint=SUBURBAN)
  limits = CarControllerParams(cp)
  assert limits.STEER_MAX == 300
  assert limits.STEER_DELTA_UP == 10
  assert limits.STEER_DELTA_DOWN == 15
  assert Bus.pt in DBC[SUBURBAN] and Bus.radar in DBC[SUBURBAN]


@pytest.mark.parametrize("alpha_long", [False, True])
def test_suburban_retains_factory_acc_even_when_alpha_long_is_requested(alpha_long):
  ret = structs.CarParams.new_message()
  ret.carFingerprint = str(SUBURBAN)
  ret.lateralTuning.init("pid")
  got = CarInterface._get_params(
    ret, SUBURBAN, {CanBus.POWERTRAIN: {}, CanBus.OBSTACLE: {}},
    [], alpha_long, False, False,
  )
  assert got.networkLocation == structs.CarParams.NetworkLocation.fwdCamera
  assert got.pcmCruise
  assert not got.alphaLongitudinalAvailable
  assert not got.openpilotLongitudinalControl
  assert not (got.safetyConfigs[0].safetyParam & GMSafetyFlags.HW_CAM_LONG.value)
  assert got.safetyConfigs[0].safetyParam & GMSafetyFlags.HW_CAM.value
  assert got.minSteerSpeed == pytest.approx(10 * CV.KPH_TO_MS)
  assert got.minEnableSpeed == -1.
  assert got.steerActuatorDelay == pytest.approx(0.2)
  assert got.lateralTuning.torque.latAccelOffset == 0.0
  assert not got.dashcamOnly


def test_suburban_keeps_independent_seven_bin_torque_seed():
  cp = SimpleNamespace(carFingerprint=str(SUBURBAN), brand="gm", minSteerSpeed=10 * CV.KPH_TO_MS)
  table = get_speed_dep_config_for_car(cp)
  assert table["speed_bp"] == [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
  assert table["laf_bp"] == [0.680, 0.680, 0.680, 0.435, 0.469, 0.378, 0.413]
  assert table["friction_bp"] == [0.205, 0.205, 0.205, 0.195, 0.208, 0.181, 0.135]
  assert table["seed_version"] == 1


def test_rejected_command_parser_only_exists_on_suburban(monkeypatch):
  import opendbc.car.gm.carstate as cs_module
  monkeypatch.setattr(cs_module, "CANParser", lambda dbc, messages, bus: (dbc, messages, bus))
  cp = SimpleNamespace(carFingerprint=str(SUBURBAN), networkLocation=structs.CarParams.NetworkLocation.fwdCamera)
  suburban_parsers = CarState.get_can_parsers(cp, None)
  assert suburban_parsers[REJECTED_LOOPBACK][-1] == CanBus.DROPPED

  cp.carFingerprint = str(CAR.GMC_YUKON)
  yukon_parsers = CarState.get_can_parsers(cp, None)
  assert REJECTED_LOOPBACK not in yukon_parsers
