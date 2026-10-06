from types import SimpleNamespace

import numpy as np

from opendbc.car.car_helpers import interfaces
from opendbc.car.gm.values import CAR as GM
from opendbc.car.mazda.values import CAR as MAZDA
from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.cereal.services import SERVICE_LIST
from openpilot.selfdrive.locationd.torqued import TorqueEstimator, VERSION
from openpilot.sunnypilot.selfdrive.car.interfaces import _seed_suburban_live_table_defaults
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext import LatControlTorqueExt
from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import LatControlTorqueExtOverride


SUBURBAN = GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN
SPEED_BP = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
LAF_BP = [0.680, 0.680, 0.680, 0.435, 0.469, 0.378, 0.413]
FRICTION_BP = [0.205, 0.205, 0.205, 0.195, 0.208, 0.181, 0.135]


def car_params(candidate):
  cp = interfaces[candidate].get_non_essential_params(candidate)
  interfaces[candidate].configure_torque_tune(candidate, cp.lateralTuning)
  return cp


def configured_estimator():
  params = Params()
  params.put_bool("EnforceTorqueControl", True, block=True)
  params.put_bool("LiveTorqueParamsToggle", True, block=True)
  params.put_bool("LiveTorqueParamsRelaxedToggle", True, block=True)
  params.put_bool("SpeedDependentTorqueToggle", True, block=True)
  return TorqueEstimator(car_params(SUBURBAN))


class TestSuburbanLiveTable(OpenpilotTestCase):
  def test_defaults_enable_only_suburban(self):
    params = Params()
    suburban = car_params(SUBURBAN)
    _seed_suburban_live_table_defaults(suburban, params)
    assert params.get_bool("EnforceTorqueControl")
    assert params.get_bool("LiveTorqueParamsToggle")
    assert params.get_bool("LiveTorqueParamsRelaxedToggle")
    assert params.get_bool("SpeedDependentTorqueToggle")

    params.remove("SuburbanLiveTableDefaultsApplied")
    for key in ("EnforceTorqueControl", "LiveTorqueParamsToggle", "LiveTorqueParamsRelaxedToggle", "SpeedDependentTorqueToggle"):
      params.put_bool(key, False, block=True)
    _seed_suburban_live_table_defaults(car_params(MAZDA.MAZDA_CX9), params)
    assert not params.get_bool("SpeedDependentTorqueToggle")

  def test_speed_config_and_seed_fallback(self):
    cfg = get_speed_dep_config_for_car(car_params(SUBURBAN))
    assert cfg["speed_bp"] == SPEED_BP
    assert cfg["laf_bp"] == LAF_BP
    assert cfg["friction_bp"] == FRICTION_BP
    assert cfg["seed_version"] == 1

    extension = LatControlTorqueExt.__new__(LatControlTorqueExt)
    extension.CP = car_params(SUBURBAN)
    extension._speed_dep_active = False
    extension._speed_dep_car_cfg = None
    extension._speed_dep_speed_bp = []
    extension.lac_torque = SimpleNamespace(
      torque_params=SimpleNamespace(latAccelFactor=1.0, latAccelOffset=0.0, friction=0.1),
    )
    extension.lac_torque.update_limits = lambda: None
    tp = SimpleNamespace(useParams=True, latAccelFactorFiltered=1.0, latAccelOffsetFiltered=0.02, frictionCoefficientFiltered=0.1)
    tp_sp = SimpleNamespace(speedBinCenters=SPEED_BP, speedBinLatAccelFactors=[9.0] * 7,
                            speedBinFrictions=[8.0] * 7, speedBinValid=[False] * 7)
    extension.update_speed_dep_torque(tp, tp_sp)
    assert extension._speed_dep_lat_accel_factor_bp == LAF_BP
    assert extension._speed_dep_friction_bp == FRICTION_BP

  def test_learning_cutoff_and_expected_bin(self):
    estimator = configured_estimator()
    estimator._on_torque_point(0.25, 0.1, 38.0)
    assert len(estimator.speed_bin_points[-1]) == 1
    estimator._on_torque_point(0.25, 0.1, 38.01)
    assert len(estimator.speed_bin_points[-1]) == 1
    assert sum(len(bucket) for bucket in estimator.speed_bin_points) == 1

  def test_interpolation_and_hold_last(self):
    override = LatControlTorqueExtOverride.__new__(LatControlTorqueExtOverride)
    override.enforce_torque_control_toggle = False
    override.torque_override_enabled = False
    override._speed_dep_active = True
    override._speed_dep_speed_bp = SPEED_BP
    override._speed_dep_lat_accel_factor_bp = LAF_BP
    override._speed_dep_friction_bp = FRICTION_BP
    torque = SimpleNamespace(latAccelFactor=0.0, friction=0.0)

    override._last_vego = (16.4 + 21.0) / 2
    assert override.update_override_torque_params(torque)
    assert np.isclose(torque.latAccelFactor, np.float32((0.435 + 0.469) / 2))
    assert np.isclose(torque.friction, np.float32((0.195 + 0.208) / 2))

    override._last_vego = 45.0
    assert override.update_override_torque_params(torque)
    assert torque.latAccelFactor == float(np.float32(LAF_BP[-1]))
    assert torque.friction == float(np.float32(FRICTION_BP[-1]))

  def test_restore_key_is_fingerprint_specific(self):
    suburban = car_params(SUBURBAN)
    mazda = car_params(MAZDA.MAZDA_CX9)
    suburban_key = TorqueEstimator.get_restore_key(suburban, VERSION)
    mazda_key = TorqueEstimator.get_restore_key(mazda, VERSION)
    assert suburban_key == TorqueEstimator.get_restore_key(car_params(SUBURBAN), VERSION)
    assert suburban_key != mazda_key
    assert mazda_key != suburban_key

  def test_schema_service_and_persistent_params(self):
    estimator = configured_estimator()
    msg = estimator._sp_msg(True, (LAF_BP, FRICTION_BP, [False] * 7), with_points=True)
    assert msg.customReserved19.seedVersion == 1
    assert np.allclose(msg.customReserved19.speedBinCenters, SPEED_BP)
    assert SERVICE_LIST["customReserved19"].frequency == 4.0
    params = Params()
    for key in ("LiveTorqueParametersSP", "LiveTorqueParametersSuburban", "LiveTorqueCarParamsSuburban"):
      assert params.check_key(key) == key.encode()

  def test_seed_version_mismatch_keeps_seeds(self):
    estimator = configured_estimator()
    ltp = SimpleNamespace(version=VERSION, decay=1.0, valid=True)
    sp = SimpleNamespace(version=VERSION, seedVersion=0, speedBinCenters=SPEED_BP,
                         speedBinLatAccelFactors=[0.5] * 7, speedBinFrictions=[0.1] * 7,
                         speedBinPoints=[[] for _ in SPEED_BP])
    estimator._restore_ext_cache(ltp, car_params(SUBURBAN), sp)
    assert [f['latAccelFactor'].x for f in estimator.speed_bin_filtered] == LAF_BP
    assert [f['frictionCoefficient'].x for f in estimator.speed_bin_filtered] == FRICTION_BP

  def test_same_fingerprint_cache_restores(self):
    estimator = configured_estimator()
    factors = [value * 1.05 for value in LAF_BP]
    frictions = [value * 1.05 for value in FRICTION_BP]
    ltp = SimpleNamespace(version=VERSION, decay=1.0, valid=True)
    sp = SimpleNamespace(version=VERSION, seedVersion=1, speedBinCenters=SPEED_BP,
                         speedBinLatAccelFactors=factors, speedBinFrictions=frictions,
                         speedBinPoints=[[] for _ in SPEED_BP])
    estimator._restore_ext_cache(ltp, car_params(SUBURBAN), sp)
    assert [f['latAccelFactor'].x for f in estimator.speed_bin_filtered] == factors
    assert [f['frictionCoefficient'].x for f in estimator.speed_bin_filtered] == frictions

  def test_cross_vehicle_cache_does_not_restore(self):
    estimator = configured_estimator()
    ltp = SimpleNamespace(version=VERSION, decay=1.0, valid=True)
    sp = SimpleNamespace(version=VERSION, seedVersion=1, speedBinCenters=SPEED_BP,
                         speedBinLatAccelFactors=[0.5] * 7, speedBinFrictions=[0.1] * 7,
                         speedBinPoints=[[] for _ in SPEED_BP])
    estimator._restore_ext_cache(ltp, car_params(MAZDA.MAZDA_CX9), sp)
    assert [f['latAccelFactor'].x for f in estimator.speed_bin_filtered] == LAF_BP
