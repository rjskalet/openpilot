from types import SimpleNamespace
import unittest

from openpilot.sunnypilot.selfdrive.controls.lib.latcontrol_torque_ext_override import (
  LatControlTorqueExtOverride,
  SUBURBAN_FACTOR_FLOOR_BP,
  SUBURBAN_FACTOR_FLOOR_SCALE_MAX,
  SUBURBAN_FACTOR_FLOOR_SCALE_MIN,
  SUBURBAN_FACTOR_FLOOR_SPEED_BP,
  SUBURBAN_FINGERPRINT,
  clamp_suburban_factor_floor_scale,
  get_suburban_factor_floor,
)
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import (
  SUBURBAN_FACTOR_MAX_BP,
  get_speed_bin_factor_bounds,
)


class ParamsStub:
  def __init__(self, floor_scale=1.0):
    self.floor_scale = floor_scale

  def get_bool(self, key):
    return False

  def get(self, key, return_default=False):
    if key == "SuburbanFactorFloorScale":
      return self.floor_scale
    if key == "TorqueParamsOverrideLatAccelFactor":
      return 2.5
    if key == "TorqueParamsOverrideFriction":
      return 0.1
    raise KeyError(key)


def make_override(fingerprint, live_factor=0.45, speed=35.0, floor_scale=1.0):
  ext = LatControlTorqueExtOverride.__new__(LatControlTorqueExtOverride)
  ext.CP = SimpleNamespace(carFingerprint=fingerprint)
  ext.params = ParamsStub(floor_scale)
  ext.enforce_torque_control_toggle = True
  ext.torque_override_enabled = False
  ext.frame = -1
  ext._speed_dep_active = True
  ext._speed_dep_speed_bp = list(SUBURBAN_FACTOR_FLOOR_SPEED_BP)
  ext._speed_dep_lat_accel_factor_bp = [live_factor] * len(SUBURBAN_FACTOR_FLOOR_SPEED_BP)
  ext._speed_dep_friction_bp = [0.18] * len(SUBURBAN_FACTOR_FLOOR_SPEED_BP)
  ext._speed_dep_car_cfg = None
  ext._last_vego = speed
  ext._suburban_factor_floor_scale = 1.0
  ext._live_lat_accel_factor = live_factor
  ext._factor_floor = 0.0
  ext._applied_lat_accel_factor = live_factor
  ext._tune_scale = 1.0
  return ext


class TestSuburbanTorqueGuardrail(unittest.TestCase):
  def test_floor_shape(self):
    for speed, expected in zip(SUBURBAN_FACTOR_FLOOR_SPEED_BP, SUBURBAN_FACTOR_FLOOR_BP, strict=True):
      self.assertAlmostEqual(get_suburban_factor_floor(speed, 1.0), expected)
    self.assertAlmostEqual(get_suburban_factor_floor(31.5, 1.0), 1.0)

  def test_floor_scale_clamps(self):
    self.assertEqual(clamp_suburban_factor_floor_scale(0.1), SUBURBAN_FACTOR_FLOOR_SCALE_MIN)
    self.assertEqual(clamp_suburban_factor_floor_scale(2.0), SUBURBAN_FACTOR_FLOOR_SCALE_MAX)
    self.assertEqual(clamp_suburban_factor_floor_scale(1.05), 1.05)

  def test_suburban_offset_shadow_reasserted_after_live_copy(self):
    ext = make_override(SUBURBAN_FINGERPRINT, live_factor=0.45, speed=35.0)
    torque_params = SimpleNamespace(latAccelFactor=0.45, friction=0.18, latAccelOffset=-0.25)

    changed = ext.update_override_torque_params(torque_params)

    self.assertTrue(changed)
    self.assertEqual(torque_params.latAccelOffset, 0.0)
    self.assertAlmostEqual(torque_params.latAccelFactor, 1.10, places=5)
    self.assertAlmostEqual(ext.live_lat_accel_factor, 0.45, places=5)
    self.assertAlmostEqual(ext.factor_floor, 1.10, places=5)
    self.assertAlmostEqual(ext.applied_lat_accel_factor, 1.10, places=5)

  def test_live_factor_can_exceed_floor(self):
    ext = make_override(SUBURBAN_FINGERPRINT, live_factor=1.25, speed=35.0)
    torque_params = SimpleNamespace(latAccelFactor=1.25, friction=0.18, latAccelOffset=-0.20)

    ext.update_override_torque_params(torque_params)

    self.assertAlmostEqual(torque_params.latAccelFactor, 1.25, places=5)
    self.assertAlmostEqual(ext.factor_floor, 1.10, places=5)
    self.assertAlmostEqual(ext.applied_lat_accel_factor, 1.25, places=5)
    self.assertEqual(torque_params.latAccelOffset, 0.0)

  def test_floor_scale_applies_and_clamps_at_runtime(self):
    ext = make_override(SUBURBAN_FINGERPRINT, live_factor=0.45, speed=35.0, floor_scale=2.0)
    torque_params = SimpleNamespace(latAccelFactor=0.45, friction=0.18, latAccelOffset=-0.20)

    ext.update_override_torque_params(torque_params)

    self.assertAlmostEqual(ext.factor_floor, 1.10 * SUBURBAN_FACTOR_FLOOR_SCALE_MAX, places=5)
    self.assertAlmostEqual(torque_params.latAccelFactor, 1.10 * SUBURBAN_FACTOR_FLOOR_SCALE_MAX, places=5)

  def test_mazda_offset_is_not_shadowed(self):
    ext = make_override("MAZDA_CX9_2021", live_factor=2.70, speed=35.0)
    torque_params = SimpleNamespace(latAccelFactor=2.70, friction=0.08, latAccelOffset=-0.25)

    ext.update_override_torque_params(torque_params)

    self.assertAlmostEqual(torque_params.latAccelOffset, -0.25)
    self.assertEqual(ext.factor_floor, 0.0)
    self.assertAlmostEqual(torque_params.latAccelFactor, 2.70, places=5)

  def test_suburban_learner_upper_bounds_are_not_seed_limited(self):
    refs = [0.680, 0.680, 0.680, 0.435, 0.469, 0.378, 0.413]
    suburban = SimpleNamespace(carFingerprint=SUBURBAN_FINGERPRINT)
    bounds = get_speed_bin_factor_bounds(suburban, refs, 1.0)

    self.assertEqual([hi for _, hi in bounds], SUBURBAN_FACTOR_MAX_BP)
    self.assertEqual([lo for lo, _ in bounds], [0.0] * len(refs))

  def test_other_car_learner_bounds_unchanged(self):
    refs = [2.67, 2.70, 2.56, 2.295, 1.92, 2.58, 2.955]
    mazda = SimpleNamespace(carFingerprint="MAZDA_CX9_2021")
    bounds = get_speed_bin_factor_bounds(mazda, refs, 1.0)

    self.assertEqual(bounds, [(0.0, 2.0 * factor) for factor in refs])


if __name__ == "__main__":
  unittest.main()
