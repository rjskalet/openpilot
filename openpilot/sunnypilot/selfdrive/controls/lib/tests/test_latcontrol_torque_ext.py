"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np
from types import SimpleNamespace
from unittest.mock import MagicMock

from openpilot.cereal import log, messaging
from opendbc.car.structs import car
from opendbc.car.car_helpers import interfaces
from opendbc.car.honda.values import CAR as HONDA
from opendbc.car.mazda.values import CAR as MAZDA, CarControllerParams as MazdaControllerParams, MazdaFlags
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.interfaces import get_tune_scale
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.car.helpers import convert_to_capnp
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.locationd.helpers import Pose
from openpilot.common.mock.generators import generate_deviceMotion
from openpilot.sunnypilot.selfdrive.car import interfaces as sunnypilot_interfaces
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_v0 import (
  INTERP_SPEEDS,
  KP_INTERP,
  SUBURBAN_KP_INTERP,
  SUBURBAN_KP_INTERP_SPEEDS,
  get_kp_schedule,
)
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import (
  LatControlTorqueExtOverride,
  SUBURBAN_FACTOR_FLOOR_BP,
  SUBURBAN_FACTOR_FLOOR_SCALE_MAX,
  SUBURBAN_FACTOR_FLOOR_SPEED_BP,
  SUBURBAN_FINGERPRINT,
  get_suburban_factor_floor,
)
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.common.test import OpenpilotTestCase


def _make_controller(enhanced=False, nnlc=False):
  params = Params()
  params.put_bool("EnforceTorqueControl", True, block=True)
  params.put_bool("LateralJerkTorqueController", enhanced, block=True)
  params.put_bool("NeuralNetworkLateralControl", nnlc, block=True)

  car_name = HONDA.HONDA_CIVIC
  CarInterface = interfaces[car_name]
  CP = CarInterface.get_non_essential_params(car_name)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, car_name)
  CI = CarInterface(CP, CP_SP)
  sunnypilot_interfaces.setup_interfaces(CI, params)
  CP_SP = convert_to_capnp(CP_SP)
  VM = VehicleModel(CP)
  controller = LatControlTorque(CP.as_reader(), CP_SP.as_reader(), CI, DT_CTRL)
  return controller, VM, CP


def _make_model_v2():
  model = messaging.new_message('modelV2')
  position = log.XYZTData.new_message()
  position.x = [float(x) for x in 30.0 * np.array(ModelConstants.T_IDXS)]
  model.modelV2.position = position
  orientation = log.XYZTData.new_message()
  orientation.x = [0.0 for _ in ModelConstants.T_IDXS]
  orientation.y = [0.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.orientation = orientation
  velocity = log.XYZTData.new_message()
  velocity.x = [30.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.velocity = velocity
  acceleration = log.XYZTData.new_message()
  acceleration.x = [0.0 for _ in ModelConstants.T_IDXS]
  acceleration.y = [0.0 for _ in ModelConstants.T_IDXS]
  model.modelV2.acceleration = acceleration
  return model


def _run_update(controller, VM):
  CS = car.CarState.new_message()
  CS.vEgo = 30
  CS.steeringPressed = False
  lp = generate_deviceMotion()
  pose = Pose.from_device_motion(lp.deviceMotion)
  params = log.VehicleParameters.new_message()
  model_v2 = _make_model_v2().modelV2
  controller.extension.update_model_v2(model_v2)
  controller.extension.update_lateral_lag(0.2)
  return controller.update(True, CS, VM, params, False, 0.5, pose, False, 0.2)


class TestLatControlTorqueExt(OpenpilotTestCase):
  def test_init_enhanced_only(self):
    controller, VM, _ = _make_controller(enhanced=True, nnlc=False)
    assert controller.extension._jerk_aware_enabled
    assert not controller.extension.enabled  # NNLC disabled

  def test_init_nnlc_only(self):
    controller, VM, _ = _make_controller(enhanced=False, nnlc=True)
    assert not controller.extension._jerk_aware_enabled
    assert controller.extension.enabled

  def test_init_neither(self):
    controller, VM, _ = _make_controller(enhanced=False, nnlc=False)
    assert not controller.extension._jerk_aware_enabled
    assert not controller.extension.enabled

  def test_init_both_no_crash(self):
    controller, VM, _ = _make_controller(enhanced=True, nnlc=True)
    assert not controller.extension._jerk_aware_enabled
    assert not controller.extension.enabled

  def test_update_enhanced_only(self):
    controller, VM, _ = _make_controller(enhanced=True, nnlc=False)
    output_torque, _, pid_log = _run_update(controller, VM)
    assert pid_log.active

  def test_update_neither(self):
    controller, VM, _ = _make_controller(enhanced=False, nnlc=False)
    output_torque, _, pid_log = _run_update(controller, VM)
    assert pid_log.active

  def test_update_both_no_crash(self):
    controller, VM, _ = _make_controller(enhanced=True, nnlc=True)
    output_torque, _, pid_log = _run_update(controller, VM)
    assert pid_log.active


class TestSuburbanSpeedDependentTorque(OpenpilotTestCase):
  @staticmethod
  def make_extension(live_factor=0.45, speed=35.0, floor_scale=1.0):
    cp = car.CarParams(brand="gm", carFingerprint=SUBURBAN_FINGERPRINT)
    cp.lateralTuning.init('torque')
    ext = LatControlTorqueExt.__new__(LatControlTorqueExt)
    ext.CP = cp
    ext.params = MagicMock()
    ext.params.get_bool.return_value = False
    ext.params.get.return_value = floor_scale
    ext.enforce_torque_control_toggle = True
    ext.torque_override_enabled = False
    ext.frame = -1
    ext._speed_dep_active = True
    ext._speed_dep_speed_bp = list(SUBURBAN_FACTOR_FLOOR_SPEED_BP)
    ext._speed_dep_lat_accel_factor_bp = [live_factor] * len(SUBURBAN_FACTOR_FLOOR_SPEED_BP)
    ext._speed_dep_friction_bp = [0.18] * len(SUBURBAN_FACTOR_FLOOR_SPEED_BP)
    ext._speed_dep_car_cfg = None
    ext._last_vego = speed
    ext._suburban_factor_floor_scale = 1.0
    ext._tune_scale = 1.0
    ext.lac_torque = SimpleNamespace(
      torque_params=SimpleNamespace(latAccelFactor=live_factor, latAccelOffset=-0.25, friction=float(np.float32(0.18))),
      update_limits=MagicMock(),
    )
    ext.steer_rail_schedule = None
    return ext

  def test_u_floor_shape(self):
    for speed, expected in zip(SUBURBAN_FACTOR_FLOOR_SPEED_BP, SUBURBAN_FACTOR_FLOOR_BP, strict=True):
      assert np.isclose(get_suburban_factor_floor(speed, 1.0), expected)
    assert np.isclose(get_suburban_factor_floor(31.5, 1.0), 1.0)

  def test_low_live_factor_is_raised_to_u_floor(self):
    ext = self.make_extension(live_factor=0.45, speed=35.0)

    assert ext.update_override_torque_params(ext.lac_torque.torque_params) is True
    assert np.isclose(ext.lac_torque.torque_params.latAccelFactor, np.float32(1.10))
    assert ext.lac_torque.torque_params.latAccelOffset == -0.25

  def test_live_factor_can_exceed_u_floor(self):
    ext = self.make_extension(live_factor=1.25, speed=35.0)

    assert ext.update_override_torque_params(ext.lac_torque.torque_params) is False
    assert np.isclose(ext.lac_torque.torque_params.latAccelFactor, 1.25)

  def test_u_floor_scale_clamps(self):
    ext = self.make_extension(live_factor=0.45, speed=35.0, floor_scale=9.0)

    assert ext.update_override_torque_params(ext.lac_torque.torque_params) is True
    expected = 1.10 * SUBURBAN_FACTOR_FLOOR_SCALE_MAX
    assert np.isclose(ext.lac_torque.torque_params.latAccelFactor, np.float32(expected))

  def test_invalid_highway_bin_still_uses_applied_u_floor(self):
    ext = self.make_extension(live_factor=0.45, speed=35.0)
    ext._speed_dep_active = False
    ext._speed_dep_car_cfg = {
      'laf_bp': [0.680, 0.680, 0.680, 0.435, 0.469, 0.378, 0.413],
      'friction_bp': [0.205, 0.205, 0.205, 0.195, 0.208, 0.181, 0.135],
    }
    tp = SimpleNamespace(useParams=True, latAccelFactorFiltered=0.68, latAccelOffsetFiltered=-0.25,
                         frictionCoefficientFiltered=0.205)
    tp_sp = SimpleNamespace(
      speedBinCenters=list(SUBURBAN_FACTOR_FLOOR_SPEED_BP),
      speedBinLatAccelFactors=[0.45] * len(SUBURBAN_FACTOR_FLOOR_SPEED_BP),
      speedBinFrictions=[0.18] * len(SUBURBAN_FACTOR_FLOOR_SPEED_BP),
      speedBinValid=[False] * len(SUBURBAN_FACTOR_FLOOR_SPEED_BP),
    )

    ext.update_speed_dep_torque(tp, tp_sp)

    assert np.isclose(ext._speed_dep_lat_accel_factor_bp[-1], 0.413)
    assert ext.update_override_torque_params(ext.lac_torque.torque_params) is True
    assert np.isclose(ext.lac_torque.torque_params.latAccelFactor, np.float32(1.10))
    assert ext.lac_torque.torque_params.latAccelOffset == -0.25


class TestSuburbanTorqueKpSchedule:
  def test_suburban_uses_progressive_low_mid_speed_kp(self):
    cp = car.CarParams(carFingerprint=SUBURBAN_FINGERPRINT)

    speeds, gains = get_kp_schedule(cp)

    assert speeds == SUBURBAN_KP_INTERP_SPEEDS
    assert gains == SUBURBAN_KP_INTERP
    assert np.isclose(np.interp(5.0, speeds, gains), 10.0)
    assert np.isclose(np.interp(10.0, speeds, gains), 2.8)
    assert np.isclose(np.interp(15.0, speeds, gains), 1.7)
    assert np.isclose(np.interp(20.0, speeds, gains), np.interp(20.0, INTERP_SPEEDS, KP_INTERP))

  def test_suburban_schedule_only_reduces_kp_and_rejoins_stock_by_20_ms(self):
    cp = car.CarParams(carFingerprint=SUBURBAN_FINGERPRINT)
    speeds, gains = get_kp_schedule(cp)

    sample_speeds = np.linspace(1.0, 30.0, 117)
    suburban_gains = np.interp(sample_speeds, speeds, gains)
    stock_gains = np.interp(sample_speeds, INTERP_SPEEDS, KP_INTERP)

    assert np.all(suburban_gains <= stock_gains + 1e-12)
    assert np.allclose(suburban_gains[sample_speeds >= 20.0], stock_gains[sample_speeds >= 20.0])

  def test_mazda_and_other_cars_keep_stock_kp_schedule(self):
    for fingerprint in (str(MAZDA.MAZDA_CX9), "HONDA CIVIC 2016"):
      speeds, gains = get_kp_schedule(car.CarParams(carFingerprint=fingerprint))

      assert speeds is INTERP_SPEEDS
      assert gains is KP_INTERP


class TestMazdaSpeedDependentTorque(OpenpilotTestCase):
  @staticmethod
  def make_extension():
    cp = car.CarParams(brand="mazda", carFingerprint=str(MAZDA.MAZDA_CX9),
                       flags=int(MazdaFlags.STEER_TO_ZERO_EPS), minSteerSpeed=0.0)
    cp.lateralTuning.init('torque')
    cp.lateralTuning.torque.latAccelFactor = 9.9
    cp.lateralTuning.torque.latAccelOffset = 0.03
    cp.lateralTuning.torque.friction = 0.44

    ext = LatControlTorqueExt.__new__(LatControlTorqueExt)
    ext.CP = cp
    ext.lac_torque = SimpleNamespace(
      torque_params=SimpleNamespace(latAccelFactor=0.0, latAccelOffset=0.0, friction=0.0),
      update_limits=MagicMock(),
    )
    ext.enforce_torque_control_toggle = False
    ext.torque_override_enabled = False
    ext.frame = -1
    ext._speed_dep_active = False
    ext._speed_dep_speed_bp = []
    ext._speed_dep_lat_accel_factor_bp = []
    ext._speed_dep_friction_bp = []
    ext._speed_dep_car_cfg = None
    ext._last_vego = 0.0
    ext.steer_rail_schedule = None
    return ext

  def test_invalid_live_bin_uses_current_mazda_seed_and_interpolates(self):
    ext = self.make_extension()
    tp = SimpleNamespace(useParams=True, latAccelFactorFiltered=3.1, latAccelOffsetFiltered=0.02,
                         frictionCoefficientFiltered=0.2)
    tp_sp = SimpleNamespace(
      speedBinCenters=[6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0],
      speedBinLatAccelFactors=[3.0] * 7,
      speedBinFrictions=[0.2] * 7,
      speedBinValid=[True, False, True, True, True, True, True],
    )

    ext.update_speed_dep_torque(tp, tp_sp)

    assert ext._speed_dep_active is True
    assert ext._speed_dep_lat_accel_factor_bp[1] == 2.70
    assert ext._speed_dep_friction_bp[1] == 0.154
    assert ext.lac_torque.torque_params.latAccelFactor == 3.1
    assert ext.lac_torque.torque_params.latAccelOffset == 0.02
    assert ext.lac_torque.torque_params.friction == 0.2

    ext._last_vego = 9.5
    assert ext.update_override_torque_params(ext.lac_torque.torque_params) is True
    assert np.isclose(ext.lac_torque.torque_params.latAccelFactor, np.float32(2.70))
    assert np.isclose(ext.lac_torque.torque_params.friction, np.float32(0.154))

  def test_unavailable_live_data_restores_offline_tune(self):
    ext = self.make_extension()
    ext._speed_dep_active = True
    tp = SimpleNamespace(useParams=False)

    ext.update_speed_dep_torque(tp, None)

    assert ext._speed_dep_active is False
    assert ext.lac_torque.torque_params.latAccelFactor == ext.CP.lateralTuning.torque.latAccelFactor
    assert ext.lac_torque.torque_params.latAccelOffset == ext.CP.lateralTuning.torque.latAccelOffset
    assert ext.lac_torque.torque_params.friction == ext.CP.lateralTuning.torque.friction
    ext.lac_torque.update_limits.assert_called_once()

  def test_donor_nnlc_scale_preserves_wire_counts(self):
    cp = car.CarParams(brand="mazda", carFingerprint=str(MAZDA.MAZDA_CX9),
                       flags=int(MazdaFlags.STEER_TO_ZERO_EPS))
    tune_scale = get_tune_scale(cp)
    assert tune_scale == 1.5

    for nn_torque in (-1.0, -0.4, 0.0, 0.4, 1.0):
      old_counts = nn_torque * MazdaControllerParams.TUNE_STEER_MAX
      new_counts = (nn_torque / tune_scale) * MazdaControllerParams.EPS_STEER_MAX
      assert np.isclose(old_counts, new_counts)

  def test_donor_manual_override_converts_from_upstream_scale(self):
    params = Params()
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.put_bool("TorqueParamsOverrideEnabled", True, block=True)
    params.put("TorqueParamsOverrideLatAccelFactor", 4.2, block=True)
    params.put("TorqueParamsOverrideFriction", 0.3, block=True)

    cp = car.CarParams(brand="mazda", carFingerprint=str(MAZDA.MAZDA_CX9),
                       flags=int(MazdaFlags.STEER_TO_ZERO_EPS))
    ext = LatControlTorqueExtOverride(cp)
    torque_params = SimpleNamespace(latAccelFactor=0.0, friction=0.0)

    assert ext.update_override_torque_params(torque_params) is True
    assert np.isclose(torque_params.latAccelFactor, 6.3)
    assert np.isclose(torque_params.friction, 0.2)

  def test_manual_override_takes_precedence_over_speed_bins(self):
    params = Params()
    params.put_bool("EnforceTorqueControl", True, block=True)
    params.put_bool("TorqueParamsOverrideEnabled", True, block=True)
    params.put("TorqueParamsOverrideLatAccelFactor", 4.2, block=True)
    params.put("TorqueParamsOverrideFriction", 0.3, block=True)

    ext = LatControlTorqueExtOverride(car.CarParams())
    ext._speed_dep_active = True
    ext._speed_dep_speed_bp = [0.0, 30.0]
    ext._speed_dep_lat_accel_factor_bp = [1.0, 2.0]
    ext._speed_dep_friction_bp = [0.1, 0.2]
    ext._last_vego = 15.0
    torque_params = SimpleNamespace(latAccelFactor=0.0, friction=0.0)

    assert ext.update_override_torque_params(torque_params) is True
    assert torque_params.latAccelFactor == 4.2
    assert torque_params.friction == 0.3

    assert ext.update_override_torque_params(torque_params) is False
    assert torque_params.latAccelFactor == 4.2
    assert torque_params.friction == 0.3
