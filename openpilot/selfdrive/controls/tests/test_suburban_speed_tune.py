import unittest

from openpilot.selfdrive.controls.lib.suburban_speed_tune import suburban_v32_lat_accel_factor


class TestSuburbanSpeedTune(unittest.TestCase):
  def test_route25_anchor_points(self):
    self.assertAlmostEqual(suburban_v32_lat_accel_factor(0.0), 0.68)
    self.assertAlmostEqual(suburban_v32_lat_accel_factor(13.4112), 0.68)
    self.assertAlmostEqual(suburban_v32_lat_accel_factor(16.5), 0.4437182678)
    self.assertAlmostEqual(suburban_v32_lat_accel_factor(20.0), 0.4759647790)
    self.assertAlmostEqual(suburban_v32_lat_accel_factor(24.0), 0.6728052303)
    self.assertAlmostEqual(suburban_v32_lat_accel_factor(26.0), 0.68)
    self.assertAlmostEqual(suburban_v32_lat_accel_factor(40.2336), 0.68)

  def test_interpolation_is_continuous_and_bounded(self):
    values = [suburban_v32_lat_accel_factor(v / 10.0) for v in range(0, 403)]
    self.assertGreaterEqual(min(values), 0.4437182678)
    self.assertLessEqual(max(values), 0.68)


if __name__ == "__main__":
  unittest.main()
