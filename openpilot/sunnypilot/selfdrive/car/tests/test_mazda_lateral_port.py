from typing import cast
from tempfile import TemporaryDirectory
from unittest.mock import patch

import numpy as np

from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import (get_speed_dep_config_for_car, get_steer_max_schedule,
                                               get_steer_rail_schedule, get_steer_slew_schedule, get_tune_scale)

from openpilot.cereal import custom, messaging
from openpilot.cereal.services import SERVICE_LIST
from openpilot.common.params import ParamKeyFlag, ParamKeyType, Params
from openpilot.sunnypilot.selfdrive.car.interfaces import (MAZDA_STEER_TO_ZERO_TORQUE_TUNE,
                                                           _initialize_intelligent_cruise_button_management,
                                                           _seed_mazda_torque_defaults,
                                                           seed_car_defaults_offroad)
from openpilot.sunnypilot.selfdrive.car.intelligent_cruise_button_management.controller import IntelligentCruiseButtonManagement
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import (LIVE_TORQUE_PARAMETERS_SP_KEY,
                                                                 LIVE_TORQUE_PARAMETERS_SP_SERVICE)

CarParams = car.CarParams
SEEDED_KEYS = ("EnforceTorqueControl", "LiveTorqueParamsToggle", "SpeedDependentTorqueToggle")


class FakeParams:
  def __init__(self, initial=None):
    self._store = dict(initial or {})

  def get_bool(self, key):
    return bool(self._store.get(key, False))

  def put_bool(self, key, val, block=False):
    self._store[key] = bool(val)

  def get(self, key, return_default=False):
    return self._store.get(key)

  def put(self, key, val, block=False):
    assert not isinstance(val, str), f"{key}: string written into a numeric parameter"
    self._store[key] = val


def as_params(params: FakeParams) -> Params:
  return cast(Params, params)


def donor_eps_cp(fingerprint=CAR.MAZDA_CX9):
  return CarParams(
    brand="mazda",
    carFingerprint=str(fingerprint),
    flags=int(MazdaFlags.STEER_TO_ZERO_EPS),
    minSteerSpeed=0.0,
  )


def stock_eps_cp():
  return CarParams(brand="mazda", carFingerprint=str(CAR.MAZDA_CX9), minSteerSpeed=12.5)


def non_mazda_cp():
  return CarParams(brand="toyota", flags=int(MazdaFlags.STEER_TO_ZERO_EPS))


def torque_cp(brand="mazda", flags=0):
  cp = CarParams(brand=brand, flags=flags, carFingerprint=str(CAR.MAZDA_CX9))
  cp.lateralTuning.init('torque')
  cp.lateralTuning.torque.latAccelFactor = 2.67
  cp.lateralTuning.torque.friction = 0.161
  return cp


class TestMazdaTorqueDefaults:
  def test_donor_eps_gets_zoom_defaults(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(donor_eps_cp(), as_params(params))
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is True
    assert params.get("TorqueControlTune") == MAZDA_STEER_TO_ZERO_TORQUE_TUNE
    assert params.get_bool("MazdaTorqueDefaultsApplied") is True

  def test_stock_eps_not_seeded(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(stock_eps_cp(), as_params(params))
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False
    assert params.get("TorqueControlTune") is None

  def test_other_brand_not_seeded(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(non_mazda_cp(), as_params(params))
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False
    assert params.get("TorqueControlTune") is None

  def test_user_choice_after_seed_is_kept(self):
    params = FakeParams({
      "MazdaTorqueDefaultsApplied": True,
      "MazdaTorqueTuneSeeded": MAZDA_STEER_TO_ZERO_TORQUE_TUNE,
      "TorqueControlTune": 0.0,
    })
    _seed_mazda_torque_defaults(donor_eps_cp(), as_params(params))
    assert params.get("TorqueControlTune") == 0.0
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False

  def test_offroad_seed_from_persistent_carparams(self):
    params = FakeParams({"CarParamsPersistent": donor_eps_cp().to_bytes(), "TorqueControlTune": 0.0})
    seed_car_defaults_offroad(as_params(params))
    assert params.get("TorqueControlTune") == 2.0
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is True


class TestMazdaSpeedDependentConfig:
  def test_2018_cx9_alias_loads_flat_scale_donor_eps_table(self):
    cp = donor_eps_cp(CAR.MAZDA_CX9)
    cfg = get_speed_dep_config_for_car(cp)
    assert cfg["speed_bp"] == [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
    assert cfg["laf_bp"] == [2.67, 2.70, 2.56, 2.295, 1.92, 2.58, 2.955]
    assert cfg["friction_bp"] == [0.161, 0.154, 0.116, 0.1086666667, 0.0906666667, 0.0853333333, 0.072]
    assert cfg["seed_version"] == 1

  def test_flat_scale_preserves_old_highway_wire_torque(self):
    cfg = get_speed_dep_config_for_car(donor_eps_cp(CAR.MAZDA_CX9))
    old_laf = [1.53, 1.28, 1.72, 1.97]
    old_friction = [0.163, 0.136, 0.128, 0.108]
    for old, new in zip(old_laf, cfg["laf_bp"][3:], strict=True):
      assert abs((800.0 / old) - (1200.0 / new)) < 1e-9
    for old, new in zip(old_friction, cfg["friction_bp"][3:], strict=True):
      assert abs((old * 800.0) - (new * 1200.0)) < 1e-6

  def test_flat_scale_preserves_wire_torque_through_old_scale_transition(self):
    cfg = get_speed_dep_config_for_car(donor_eps_cp(CAR.MAZDA_CX9))

    # Legacy controller behavior: seeds were interpolated in per-count space while
    # STEER_MAX fell from 1200 to 800 between 14.2 and 14.5 m/s.
    old_speed_bp = [12.0, 16.4]
    old_laf_bp = [2.56, 1.53]
    old_friction_bp = [0.116, 0.163]
    old_steer_at_bins = [1200.0, 800.0]
    old_laf_per_count = [v / s for v, s in zip(old_laf_bp, old_steer_at_bins, strict=True)]
    old_friction_counts = [v * s for v, s in zip(old_friction_bp, old_steer_at_bins, strict=True)]

    for speed in (12.0, 14.0, 14.2, 14.35, 14.5, 15.0, 16.4):
      old_steer_max = float(np.interp(speed, [0.0, 14.2, 14.5], [1200.0, 1200.0, 800.0]))
      old_laf = float(np.interp(speed, old_speed_bp, old_laf_per_count)) * old_steer_max
      old_friction = float(np.interp(speed, old_speed_bp, old_friction_counts)) / old_steer_max

      new_laf = float(np.interp(speed, cfg["speed_bp"], cfg["laf_bp"]))
      new_friction = float(np.interp(speed, cfg["speed_bp"], cfg["friction_bp"]))

      # These are the physically meaningful invariants at the controller boundary.
      assert np.isclose(old_steer_max / old_laf, 1200.0 / new_laf)
      assert np.isclose(old_steer_max * old_friction, 1200.0 * new_friction)

  def test_stock_eps_does_not_receive_donor_scale_bins(self):
    assert get_speed_dep_config_for_car(stock_eps_cp()) == {}

  def test_donor_eps_tune_scale_and_flat_steer_max(self):
    cp = donor_eps_cp()
    assert get_tune_scale(cp) == 1.5
    assert get_steer_max_schedule(cp) is None
    assert get_tune_scale(stock_eps_cp()) == 1.0

  def test_donor_eps_slew_schedule_matches_12_counts(self):
    cp = donor_eps_cp()
    bp, up, down = get_steer_slew_schedule(cp)
    assert bp == [0.0]
    assert up == [12.0 / 1200.0]
    assert down == up

  def test_measured_eps_rail_schedule_is_bounded(self):
    cp = donor_eps_cp()
    bp, rail = get_steer_rail_schedule(cp)
    assert len(bp) == len(rail)
    assert min(rail) > 0.0
    assert max(rail) <= 1.0
    assert 14.5 in bp


class TestMazdaControllerSelection:
  @staticmethod
  def controller_ext(cp, params):
    ext = ControlsExt.__new__(ControlsExt)
    ext.CP = cp
    ext.CP_SP = custom.CarParamsSP()
    ext.params = as_params(params)
    ext._zoompilot_mazda = cp.brand == "mazda" and bool(cp.flags & MazdaFlags.STEER_TO_ZERO_EPS)
    return ext

  def test_donor_eps_selects_torque_v2(self):
    cp = torque_cp(flags=int(MazdaFlags.STEER_TO_ZERO_EPS))
    ext = self.controller_ext(cp, FakeParams({"EnforceTorqueControl": True, "TorqueControlTune": 2.0}))
    expected = object()
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV2", return_value=expected) as torque_v2:
      assert ext.initialize_lateral_control(object(), object(), 0.01) is expected
    torque_v2.assert_called_once()

  def test_stock_eps_stays_on_legacy_torque_path(self):
    cp = torque_cp()
    ext = self.controller_ext(cp, FakeParams({"EnforceTorqueControl": False}))
    expected = object()
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV0", return_value=expected) as torque_v0:
      assert ext.initialize_lateral_control(object(), object(), 0.01) is expected
    torque_v0.assert_called_once()

  def test_non_mazda_cannot_enter_donor_eps_path(self):
    cp = torque_cp(brand="toyota", flags=int(MazdaFlags.STEER_TO_ZERO_EPS))
    ext = self.controller_ext(cp, FakeParams({"EnforceTorqueControl": False, "TorqueControlTune": 2.0}))
    legacy = object()
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV2") as torque_v2, \
         patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV0", return_value=legacy):
      assert ext.initialize_lateral_control(object(), object(), 0.01) is legacy
    torque_v2.assert_not_called()


class TestCX9PersistenceAndPlumbing:
  def test_branch_switch_parameters_keep_current_flags(self):
    with TemporaryDirectory() as params_dir:
      params = Params(params_dir)
      params.put_bool("MazdaTorqueDefaultsApplied", True, block=True)
      params.put("MazdaTorqueTuneSeeded", 2.0, block=True)
      params.clear_all(ParamKeyFlag.BACKUP)
      assert params.get("MazdaTorqueDefaultsApplied") is None
      assert params.get("MazdaTorqueTuneSeeded") is None

      for key in ("LiveTorqueParametersSP", "LiveTorqueParametersSuburban", "LiveTorqueCarParamsSuburban"):
        assert params.get_type(key) == ParamKeyType.BYTES
        params.put(key, b"cached", block=True)
      params.clear_all(ParamKeyFlag.PERSISTENT)
      for key in ("LiveTorqueParametersSP", "LiveTorqueParametersSuburban", "LiveTorqueCarParamsSuburban"):
        assert params.get(key) is None

  def test_shared_camera_offset_default(self):
    assert Params().get_default_value("CameraOffset") == -0.08

  def test_speed_bin_service_and_schema_are_registered(self):
    assert LIVE_TORQUE_PARAMETERS_SP_KEY == "LiveTorqueParametersSP"
    assert LIVE_TORQUE_PARAMETERS_SP_SERVICE == "customReserved19"
    service = SERVICE_LIST[LIVE_TORQUE_PARAMETERS_SP_SERVICE]
    assert service.should_log is True
    assert service.frequency == 4.0
    assert service.decimation == 1

    msg = messaging.new_message(LIVE_TORQUE_PARAMETERS_SP_SERVICE)
    speed_bins = getattr(msg, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
    speed_bins.version = 1
    speed_bins.seedVersion = 2
    speed_bins.speedBinCenters = [6.5]
    speed_bins.speedBinLatAccelFactors = [2.67]
    speed_bins.speedBinFrictions = [0.161]
    speed_bins.speedBinValid = [True]
    speed_bins.speedBinPoints = [[[0.1, 0.2]]]
    assert speed_bins.to_dict() == {
      "version": 1,
      "speedBinCenters": [6.5],
      "speedBinLatAccelFactors": [2.6700000762939453],
      "speedBinFrictions": [0.16099999845027924],
      "speedBinValid": [True],
      "speedBinPoints": [[[0.10000000149011612, 0.20000000298023224]]],
      "seedVersion": 2,
    }


class TestCX9FactoryLongitudinal:
  def test_donor_eps_keeps_factory_mrcc_and_icbm_button_control(self):
    CarInterface = interfaces[CAR.MAZDA_CX9]
    donor_fw = car.CarParams.CarFw(ecu="eps", fwVersion=b"K0A1-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
    cp = CarInterface.get_params(CAR.MAZDA_CX9, {0: {}, 1: {}, 2: {}}, [donor_fw], False, False, False)
    cp_sp = CarInterface.get_params_sp(cp, CAR.MAZDA_CX9, {0: {}, 1: {}, 2: {}}, [donor_fw], False, False, False)

    assert cp.flags & MazdaFlags.STEER_TO_ZERO_EPS
    assert cp.openpilotLongitudinalControl is False
    assert cp.pcmCruise is True
    assert cp_sp.intelligentCruiseButtonManagementAvailable is True

    _initialize_intelligent_cruise_button_management(cp, cp_sp, as_params(FakeParams({"IntelligentCruiseButtonManagement": True})))
    assert cp_sp.pcmCruiseSpeed is False

    icbm = IntelligentCruiseButtonManagement(cp, cp_sp)
    cs = car.CarState(vEgo=20.0, cruiseState={"speedCluster": 20.0})
    cc = car.CarControl(enabled=True)
    cc.actuators.accel = 0.25
    plan = custom.LongitudinalPlanSP(vTarget=18.0, aTarget=-0.4)
    plan.longitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource.sccVision
    icbm.run(cs, cc, plan, False)

    assert cc.actuators.accel == 0.25
    assert icbm.cruise_button in (
      custom.IntelligentCruiseButtonManagement.SendButtonState.none,
      custom.IntelligentCruiseButtonManagement.SendButtonState.increase,
      custom.IntelligentCruiseButtonManagement.SendButtonState.decrease,
    )
