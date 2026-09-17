#!/usr/bin/env python3

from opendbc.car.mazda.values import CAR, MazdaFlags
from opendbc.car.structs import car
from opendbc.sunnypilot.car.interfaces import (get_speed_dep_config_for_car, get_steer_max_schedule,
                                               get_steer_rail_schedule, get_steer_slew_schedule)

from openpilot.sunnypilot.selfdrive.car.interfaces import (MAZDA_STEER_TO_ZERO_TORQUE_TUNE,
                                                           _seed_mazda_torque_defaults,
                                                           seed_car_defaults_offroad)

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


class TestMazdaTorqueDefaults:
  def test_donor_eps_gets_zoom_defaults(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(donor_eps_cp(), params)
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is True
    assert params.get("TorqueControlTune") == MAZDA_STEER_TO_ZERO_TORQUE_TUNE
    assert params.get_bool("MazdaTorqueDefaultsApplied") is True

  def test_stock_eps_not_seeded(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(stock_eps_cp(), params)
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False
    assert params.get("TorqueControlTune") is None

  def test_other_brand_not_seeded(self):
    params = FakeParams()
    _seed_mazda_torque_defaults(non_mazda_cp(), params)
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False
    assert params.get("TorqueControlTune") is None

  def test_user_choice_after_seed_is_kept(self):
    params = FakeParams({
      "MazdaTorqueDefaultsApplied": True,
      "MazdaTorqueTuneSeeded": MAZDA_STEER_TO_ZERO_TORQUE_TUNE,
      "TorqueControlTune": 0.0,
    })
    _seed_mazda_torque_defaults(donor_eps_cp(), params)
    assert params.get("TorqueControlTune") == 0.0
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is False

  def test_offroad_seed_from_persistent_carparams(self):
    params = FakeParams({"CarParamsPersistent": donor_eps_cp().to_bytes(), "TorqueControlTune": 0.0})
    seed_car_defaults_offroad(params)
    assert params.get("TorqueControlTune") == 2.0
    for key in SEEDED_KEYS:
      assert params.get_bool(key) is True


class TestMazdaSpeedDependentConfig:
  def test_2018_cx9_alias_loads_donor_eps_table(self):
    cp = donor_eps_cp(CAR.MAZDA_CX9)
    cfg = get_speed_dep_config_for_car(cp)
    assert cfg["speed_bp"] == [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
    assert cfg["laf_bp"] == [2.67, 2.70, 2.56, 1.53, 1.28, 1.72, 1.97]
    assert cfg["friction_bp"] == [0.161, 0.154, 0.116, 0.163, 0.136, 0.128, 0.108]

  def test_donor_eps_steer_max_schedule(self):
    cp = donor_eps_cp()
    bp, values = get_steer_max_schedule(cp)
    assert bp == [0.0, 14.2, 14.5]
    assert values == [1200.0, 1200.0, 800.0]

  def test_donor_eps_slew_schedule_matches_12_counts(self):
    cp = donor_eps_cp()
    bp, up, down = get_steer_slew_schedule(cp)
    assert bp == [0.0, 14.2, 14.5]
    assert up == [12.0 / 1200.0, 12.0 / 1200.0, 12.0 / 800.0]
    assert down == up

  def test_measured_eps_rail_schedule_is_bounded(self):
    cp = donor_eps_cp()
    bp, rail = get_steer_rail_schedule(cp)
    assert len(bp) == len(rail)
    assert min(rail) > 0.0
    assert max(rail) <= 1.0
    assert 14.5 in bp
