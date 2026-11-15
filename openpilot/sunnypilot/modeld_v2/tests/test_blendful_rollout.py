"""Blendful must not silently enable from old saved parameters."""
from openpilot.sunnypilot.modeld_v2.blendful_rollout import blendful_integration_enabled

class FakeParams:
  def __init__(self, **settings):
    self.settings = settings
  def get_bool(self, key):
    return bool(self.settings.get(key, False))


def test_old_blendful_toggle_alone_cannot_activate():
  assert not blendful_integration_enabled(FakeParams(Blendful=True))


def test_explicitly_validated_install_can_enable_blendful():
  assert blendful_integration_enabled(FakeParams(Blendful=True, BlendfulIntegrationValidated=True))


def test_explicit_validation_gate_alone_is_not_sufficient():
  assert not blendful_integration_enabled(FakeParams(Blendful=False, BlendfulIntegrationValidated=True))
