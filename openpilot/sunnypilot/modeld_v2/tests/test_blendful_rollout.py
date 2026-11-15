"""Blendful gating is explicit per car and exact software revision."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.car.mazda.values import MazdaFlags
from openpilot.sunnypilot.modeld_v2.blendful_rollout import blendful_integration_enabled

COMMIT = "a" * 40
SUBURBAN = "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"


class FakeParams:
  def __init__(self, **settings):
    self.settings = settings
  def get_bool(self, key):
    return bool(self.settings.get(key, False))
  def get(self, key):
    return self.settings.get(key)


def cp(brand, fingerprint, flags=0):
  return SimpleNamespace(brand=brand, carFingerprint=fingerprint, flags=flags)



class TestBlendfulRollout(unittest.TestCase):
  def test_stale_blendful_toggle_and_global_validation_are_insufficient(self):
    params = FakeParams(Blendful=True, BlendfulIntegrationValidated=True)
    with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
      assert not blendful_integration_enabled(params, cp("gm", SUBURBAN))


  def test_suburban_validation_is_not_transferable_to_mazda(self):
    params = FakeParams(Blendful=True, BlendfulIntegrationValidated=True,
                        BlendfulValidatedCommitSuburban=COMMIT)
    with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
      assert blendful_integration_enabled(params, cp("gm", SUBURBAN))
      assert not blendful_integration_enabled(params, cp("mazda", "MAZDA_CX9", int(MazdaFlags.STEER_TO_ZERO_EPS)))


  def test_mazda_requires_real_donor_eps_and_matching_revision(self):
    params = FakeParams(Blendful=True, BlendfulIntegrationValidated=True,
                        BlendfulValidatedCommitMazda=COMMIT)
    with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
      assert blendful_integration_enabled(params, cp("mazda", "MAZDA_CX9", int(MazdaFlags.STEER_TO_ZERO_EPS)))
      assert not blendful_integration_enabled(params, cp("mazda", "MAZDA_CX9"))
    with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value="b" * 40):
      assert not blendful_integration_enabled(params, cp("mazda", "MAZDA_CX9", int(MazdaFlags.STEER_TO_ZERO_EPS)))


  def test_unsupported_vehicle_and_absent_build_metadata_fail_closed(self):
    params = FakeParams(Blendful=True, BlendfulIntegrationValidated=True,
                        BlendfulValidatedCommitMazda=COMMIT, BlendfulValidatedCommitSuburban=COMMIT)
    with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=""):
      assert not blendful_integration_enabled(params, cp("gm", SUBURBAN))
    with patch("openpilot.sunnypilot.fleet_validation.running_revision", return_value=COMMIT):
      assert not blendful_integration_enabled(params, cp("honda", "HONDA_CIVIC"))
