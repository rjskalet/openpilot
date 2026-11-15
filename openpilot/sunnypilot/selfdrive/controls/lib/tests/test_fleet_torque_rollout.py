"""Fleet torque rollout must be exact-commit and vehicle-scoped, with no implicit seeding."""
from types import SimpleNamespace
from typing import cast
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.gm.values import CAR as GM
from opendbc.car.mazda.values import CAR as MAZDA, MazdaFlags

from openpilot.common.params import Params
from openpilot.selfdrive.locationd.torqued import get_torque_cache_keys
from openpilot.sunnypilot.selfdrive.car.interfaces import _seed_mazda_torque_defaults, _seed_suburban_live_table_defaults
from openpilot.sunnypilot.fleet_validation import feature_validated


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


COMMIT = "a" * 40
def cp(candidate, brand, flags=0):
  return structs.CarParams(brand=brand, carFingerprint=candidate, flags=flags)


def test_old_global_gate_alone_will_not_seed_both_cars():
  settings = FakeParams(FleetTorqueIntegrationValidated=True)
  suburban = cp(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
  mazda = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.STEER_TO_ZERO_EPS))
  with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
    _seed_suburban_live_table_defaults(suburban, cast(Params, settings))
    _seed_mazda_torque_defaults(mazda, cast(Params, settings))
  assert not settings.get_bool("SuburbanLiveTableDefaultsApplied")
  assert not settings.get_bool("MazdaTorqueDefaultsApplied")


def test_suburban_approval_does_not_activate_mazda():
  settings = FakeParams(FleetTorqueIntegrationValidated=True, FleetTorqueValidatedCommitSuburban=COMMIT)
  suburban = cp(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
  mazda = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.STEER_TO_ZERO_EPS))
  with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
    assert feature_validated(settings, suburban, "FleetTorque")
    assert not feature_validated(settings, mazda, "FleetTorque")
    _seed_suburban_live_table_defaults(suburban, cast(Params, settings))
    _seed_mazda_torque_defaults(mazda, cast(Params, settings))
  assert settings.get_bool("SuburbanLiveTableDefaultsApplied")
  assert not settings.get_bool("MazdaTorqueDefaultsApplied")


def test_donor_eps_torque_is_locked_to_validated_revision():
  settings = FakeParams(FleetTorqueIntegrationValidated=True, FleetTorqueValidatedCommitMazda=COMMIT)
  mazda = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.STEER_TO_ZERO_EPS))
  with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value="b" * 40):
    _seed_mazda_torque_defaults(mazda, cast(Params, settings))
  assert not settings.get_bool("MazdaTorqueDefaultsApplied")


def test_cache_keys_preserve_separate_vehicle_names_when_authorized():
  suburban = cp(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
  mazda = cp(MAZDA.MAZDA_CX9, "mazda", int(MazdaFlags.STEER_TO_ZERO_EPS))
  assert get_torque_cache_keys(suburban, False) == ("LiveTorqueParameters", "CarParamsPrevRoute")
  assert get_torque_cache_keys(suburban, True) == ("LiveTorqueParametersSuburban", "LiveTorqueCarParamsSuburban")
  assert get_torque_cache_keys(mazda, True) == ("LiveTorqueParametersMazda", "LiveTorqueCarParamsMazda")
