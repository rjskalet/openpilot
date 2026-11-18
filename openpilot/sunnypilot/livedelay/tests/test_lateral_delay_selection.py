"""Regression contract for the selected lateral-delay source.

These tests are deliberately independent of a vehicle and do not establish
that a controller tune is safe. They protect the live/fixed delay-selection
contract shared by modeld and controlsd.
"""
import unittest
from pathlib import Path

from openpilot.sunnypilot.livedelay.helpers import get_lat_delay


OPENPILOT_DIR = Path(__file__).resolve().parents[3]
CONTROLSD = OPENPILOT_DIR / "selfdrive/controls/controlsd.py"
CONTROLSD_EXT = OPENPILOT_DIR / "sunnypilot/selfdrive/controls/controlsd_ext.py"
MODELD_V2 = OPENPILOT_DIR / "sunnypilot/modeld_v2/modeld.py"


class FakeParams:
  def __init__(self, live: bool, cached: float):
    self.live = live
    self.cached = cached

  def get_bool(self, key):
    assert key == "LagdToggle"
    return self.live

  def get(self, key, return_default=False):
    assert key == "LagdValueCache"
    return self.cached


class TestLateralDelaySelection(unittest.TestCase):
  def test_live_delay_uses_published_measurement(self):
    self.assertAlmostEqual(get_lat_delay(FakeParams(True, 0.55), 0.31), 0.31)

  def test_fixed_delay_uses_cached_actuator_plus_software_delay(self):
    self.assertAlmostEqual(get_lat_delay(FakeParams(False, 0.55), 0.31), 0.55)

  def test_switching_selector_changes_base_delay(self):
    params = FakeParams(False, 0.55)
    self.assertAlmostEqual(get_lat_delay(params, 0.31), 0.55)
    params.live = True
    self.assertAlmostEqual(get_lat_delay(params, 0.31), 0.31)

  def test_controller_consumes_selected_delay_not_raw_published_delay(self):
    core = CONTROLSD.read_text()
    extension = CONTROLSD_EXT.read_text()
    modeld = MODELD_V2.read_text()

    self.assertIn('self.CP.brand == "mazda" and bool(self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS)', core)
    self.assertIn("lat_delay = self.lat_delay + LAT_SMOOTH_SECONDS", core)
    # Non-donor Mazda, Suburban and all other cars keep the baseline selection.
    self.assertIn('lat_delay = self.sm["lateralDelay"].lateralDelay + LAT_SMOOTH_SECONDS', core)
    self.assertIn("self.LaC.extension.update_lateral_lag(self.lat_delay)", core)
    self.assertIn('self.lat_delay = get_lat_delay(self.params, sm["lateralDelay"].lateralDelay)', extension)
    self.assertIn('model.lat_delay = get_lat_delay(params, sm["lateralDelay"].lateralDelay)', modeld)
    self.assertIn("lat_delay = model.lat_delay + model.LAT_SMOOTH_SECONDS", modeld)


if __name__ == "__main__":
  unittest.main()
