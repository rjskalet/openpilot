"""Regression for ZoomPilot ModelSettings and guarded Blendful rollout."""
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from openpilot.cereal import log
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants
from openpilot.sunnypilot.modeld_v2.model_settings import ModelSettings, with_blendful


class FakeParams:
  def __init__(self, **values):
    self.values = values
  def get_bool(self, key):
    return bool(self.values.get(key, False))
  def get(self, key, return_default=False):
    return self.values.get(key, 0.0)

class FakeSm:
  frame = 0
  def __getitem__(self, key):
    assert key == "lateralDelay"
    return SimpleNamespace(lateralDelay=0.2)

def make_settings(**values):
  settings = ModelSettings()
  model = SimpleNamespace(lat_delay=0., chestnut=True, LAT_SMOOTH_SECONDS=0.)
  with patch("openpilot.sunnypilot.modeld_v2.model_settings.get_lat_delay", return_value=0.2):
    assert settings.refresh(FakeSm(), FakeParams(**values), model)
  return settings, model

def frame():
  x = np.asarray(ModelConstants.X_IDXS, dtype=float)
  lines = np.zeros((1, 4, 33, 2))
  stds = np.ones_like(lines) * 0.1
  for i, y in enumerate([-5.25, -1.75, 1.75, 5.25]):
    lines[0, i, :, 0] = y
  probs = np.zeros((1, 8))
  probs[0, 3] = probs[0, 5] = 0.9
  plan = np.zeros((1, 33, 3))
  plan[0, :, 0] = x
  return {"lane_lines": lines, "lane_lines_stds": stds, "lane_lines_prob": probs, "plan": plan}

def test_blendful_cannot_activate_from_stale_setting():
  settings, _ = make_settings(Blendful=True, BlendfulIntegrationValidated=False)
  assert not settings.blendful_enabled
  assert settings.blendful_callback(frame(), 20., log.Desire.none, log.LaneChangeState.off)(0.) == 0.

def test_explicit_validation_gate_allows_activation():
  settings, _ = make_settings(Blendful=True, BlendfulIntegrationValidated=True)
  assert settings.blendful_enabled
  assert np.isfinite(settings.blendful_callback(frame(), 20., log.Desire.none, log.LaneChangeState.off)(0.))

def test_missing_or_nonfinite_model_geometry_does_not_crash():
  settings, _ = make_settings(Blendful=True, BlendfulIntegrationValidated=True)
  assert settings.blendful_callback({}, 20., log.Desire.none, log.LaneChangeState.off)(0.01) == 0.
  bad = frame()
  bad["lane_lines"][0, 1, 0, 0] = np.nan
  assert settings.blendful_callback(bad, 20., log.Desire.none, log.LaneChangeState.off)(0.01) == 0.

def test_invalid_curvature_does_not_enter_lane_fit():
  settings, _ = make_settings(Blendful=True, BlendfulIntegrationValidated=True)
  assert settings.blendful_callback(frame(), 20., log.Desire.none, log.LaneChangeState.off)(float("nan")) == 0.
  assert np.isfinite(settings.blendful.model_curvature)

def test_lane_change_fades_correction():
  settings, _ = make_settings(Blendful=True, BlendfulIntegrationValidated=True)
  assert settings.blendful_callback(frame(), 20., log.Desire.laneChangeLeft, log.LaneChangeState.off)(0.) == 0.

def test_big_model_smoothing_respects_existing_filter():
  settings, model = make_settings(BigModelSmoothing=True)
  assert settings.big_lat_smooth(model) == 0.1
  model.LAT_SMOOTH_SECONDS = 0.2
  assert settings.big_lat_smooth(model) == 0.
  model.LAT_SMOOTH_SECONDS = 0.
  model.chestnut = False
  assert settings.big_lat_smooth(model) == 0.

def test_stock_wrapper_preserves_low_speed_curvature():
  def original(_out, _prev, _lat, _long, _v):
    return SimpleNamespace(desiredCurvature=0.)
  wrapped = with_blendful(original, 0.3)
  prev = SimpleNamespace(desiredCurvature=0.)
  assert wrapped({}, prev, 0., 0., 0.2, lambda _k: 0.1).desiredCurvature == 0.
  assert wrapped({}, prev, 0., 0., 10., lambda _k: 0.1).desiredCurvature == 0.1
