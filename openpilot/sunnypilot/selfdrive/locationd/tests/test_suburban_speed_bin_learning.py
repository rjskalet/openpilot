"""Suburban-only learning bounds and high-speed exclusion regression tests."""
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import (
  SUBURBAN_CAMERA_ACC, SUBURBAN_MAX_LEARNING_SPEED, SpeedBinLearner,
)


class FakeParams:
  def __init__(self, floor=1.0):
    self.floor = floor

  def get(self, name, return_default=False):
    assert name == "SuburbanFactorFloorScale" and return_default
    return self.floor


def fake_learner(fingerprint, floor=1.0):
  return SimpleNamespace(
    CP=SimpleNamespace(carFingerprint=fingerprint),
    _params=FakeParams(floor),
    factor_sanity=0.3,
  )


@pytest.mark.parametrize("factor,floor", [(0.68, 1.0), (0.435, 0.9)])
def test_suburban_factor_floor_is_retained(factor, floor):
  obj = fake_learner(SUBURBAN_CAMERA_ACC, floor)
  bounds = SpeedBinLearner._get_speed_bin_lat_accel_factor_bounds(obj, [factor])
  assert bounds[0][0] == pytest.approx(factor * floor)
  assert bounds[0][1] == pytest.approx(factor * 1.3)


def test_other_cars_keep_unmodified_zoompilot_sanity_bounds():
  obj = fake_learner("MAZDA_CX9")
  bounds = SpeedBinLearner._get_speed_bin_lat_accel_factor_bounds(obj, [2.0])
  assert bounds == pytest.approx([(1.4, 2.6)])


@pytest.mark.parametrize("fingerprint,speed,expected_points", [
  (SUBURBAN_CAMERA_ACC, 38.1, 0),
  (SUBURBAN_CAMERA_ACC, 38.0, 1),
  ("MAZDA_CX9", 38.1, 1),
])
def test_high_speed_learning_gate_only_applies_to_suburban(fingerprint, speed, expected_points):
  bucket = MagicMock()
  obj = SimpleNamespace(
    CP=SimpleNamespace(carFingerprint=fingerprint),
    speed_binned=True,
    speed_bin_bounds=[(0.0, 45.0)],
    speed_bin_points=[bucket],
    _speed_bin_dirty=[False],
  )
  SpeedBinLearner._on_torque_point(obj, 0.2, 0.5, speed)
  assert bucket.add_point.call_count == expected_points
  assert obj._speed_bin_dirty[0] == bool(expected_points)
  assert SUBURBAN_MAX_LEARNING_SPEED == 38.0
