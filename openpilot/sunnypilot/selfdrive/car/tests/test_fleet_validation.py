"""Exact-commit, per-vehicle feature validation and dirty/unknown build behavior."""
from types import SimpleNamespace
from unittest.mock import patch

import pytest

from opendbc.car.mazda.values import MazdaFlags
from openpilot.sunnypilot.fleet_validation import feature_validated, fleet_vehicle, running_revision

COMMIT = "d" * 40
SUBURBAN = "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"


class Params:
  def __init__(self, **settings):
    self.settings = settings
  def get_bool(self, key):
    return bool(self.settings.get(key, False))
  def get(self, key):
    return self.settings.get(key)


def cp(brand, fingerprint, flags=0):
  return SimpleNamespace(brand=brand, carFingerprint=fingerprint, flags=flags)


def test_donor_only_and_no_unsupported_platform():
  assert fleet_vehicle(cp("mazda", "MAZDA_CX9", int(MazdaFlags.STEER_TO_ZERO_EPS))) == "Mazda"
  assert fleet_vehicle(cp("mazda", "MAZDA_CX9")) is None
  assert fleet_vehicle(cp("gm", SUBURBAN)) == "Suburban"
  assert fleet_vehicle(cp("gm", "GMC_YUKON")) is None


def test_revision_mismatch_disables_fleet_torque():
  params = Params(FleetTorqueIntegrationValidated=True, FleetTorqueValidatedCommitSuburban="a" * 40)
  with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
    assert not feature_validated(params, cp("gm", SUBURBAN), "FleetTorque")


def test_revision_and_vehicle_pair_must_match():
  params = Params(FleetTorqueIntegrationValidated=True, FleetTorqueValidatedCommitMazda=COMMIT)
  with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
    assert feature_validated(params, cp("mazda", "MAZDA_CX9", int(MazdaFlags.STEER_TO_ZERO_EPS)), "FleetTorque")
    assert not feature_validated(params, cp("gm", SUBURBAN), "FleetTorque")


@pytest.mark.parametrize("commit,dirty", [("unknown", False), (COMMIT, True), ("a" * 12, False)])
def test_untrusted_build_identity_cannot_validate(commit, dirty):
  metadata = SimpleNamespace(openpilot=SimpleNamespace(git_commit=commit, is_dirty=dirty))
  running_revision.cache_clear()
  with patch("openpilot.sunnypilot.fleet_validation.get_build_metadata", return_value=metadata):
    assert running_revision() == ""
  running_revision.cache_clear()


def test_clean_full_git_sha_identifies_current_build():
  metadata = SimpleNamespace(openpilot=SimpleNamespace(git_commit=COMMIT, is_dirty=False))
  running_revision.cache_clear()
  with patch("openpilot.sunnypilot.fleet_validation.get_build_metadata", return_value=metadata):
    assert running_revision() == COMMIT
  running_revision.cache_clear()
