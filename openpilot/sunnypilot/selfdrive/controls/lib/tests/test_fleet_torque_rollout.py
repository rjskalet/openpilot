"""SunnyPilot-first integration: fleet-specific behavior is opt-in until validated."""
from typing import cast

from opendbc.car import structs
from opendbc.car.gm.values import CAR as GM
from opendbc.car.mazda.values import CAR as MAZDA, MazdaFlags

from openpilot.common.params import Params
from openpilot.selfdrive.locationd.torqued import get_torque_cache_keys
from openpilot.sunnypilot.selfdrive.car.interfaces import _seed_mazda_torque_defaults, _seed_suburban_live_table_defaults


class FakeParams:
  def __init__(self, **values):
    self.values = values
  def get_bool(self, key):
    return bool(self.values.get(key, False))
  def get(self, key, return_default=False):
    return self.values.get(key)
  def put_bool(self, key, value, block=False):
    self.values[key] = bool(value)
  def put(self, key, value, block=False):
    self.values[key] = value


def cp(candidate, brand, flags=0):
  return structs.CarParams(brand=brand, carFingerprint=candidate, flags=flags)


def test_fleet_cache_keys_are_disabled_until_validation():
  suburban = cp(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
  mazda = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.STEER_TO_ZERO_EPS))
  for car in (suburban, mazda):
    assert get_torque_cache_keys(car) == ("LiveTorqueParameters", "CarParamsPrevRoute")
    assert get_torque_cache_keys(car, False) == ("LiveTorqueParameters", "CarParamsPrevRoute")


def test_validated_cache_keys_are_separate_per_vehicle():
  suburban = cp(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
  mazda = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.STEER_TO_ZERO_EPS))
  assert get_torque_cache_keys(suburban, True) == ("LiveTorqueParametersSuburban", "LiveTorqueCarParamsSuburban")
  assert get_torque_cache_keys(mazda, True) == ("LiveTorqueParametersMazda", "LiveTorqueCarParamsMazda")


def test_old_saved_toggles_cannot_auto_seed_unvalidated_cars():
  settings = FakeParams(EnforceTorqueControl=True, LiveTorqueParamsToggle=True, SpeedDependentTorqueToggle=True)
  suburban = cp(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
  mazda = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.STEER_TO_ZERO_EPS))
  _seed_suburban_live_table_defaults(suburban, cast(Params, settings))
  _seed_mazda_torque_defaults(mazda, cast(Params, settings))
  assert not settings.get_bool("SuburbanLiveTableDefaultsApplied")
  assert not settings.get_bool("MazdaTorqueDefaultsApplied")
  assert not settings.get_bool("FleetTorqueIntegrationValidated")


def test_explicit_validation_allows_existing_defaults_to_seed():
  settings = FakeParams(FleetTorqueIntegrationValidated=True)
  suburban = cp(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
  _seed_suburban_live_table_defaults(suburban, cast(Params, settings))
  assert settings.get_bool("SuburbanLiveTableDefaultsApplied")
  assert settings.get_bool("EnforceTorqueControl")
  assert settings.get_bool("SpeedDependentTorqueToggle")


def test_stock_car_never_receives_fleet_defaults():
  settings = FakeParams(FleetTorqueIntegrationValidated=True)
  car = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.GEN1))
  _seed_mazda_torque_defaults(car, cast(Params, settings))
  assert not settings.get_bool("MazdaTorqueDefaultsApplied")
