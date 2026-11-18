"""Regression coverage for validated, vehicle-scoped speed-bin cache restoration."""
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.gm.values import CAR as GM
from opendbc.car.mazda.values import CAR as MAZDA, MazdaFlags

from openpilot.selfdrive.locationd.torqued import MIN_FILTER_DECAY, VERSION, get_torque_cache_keys
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import TorqueEstimatorExt, get_speed_dep_cache_key


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})
    self.gets = []

  def get(self, key, return_default=False, block=False):
    self.gets.append(key)
    return self.values.get(key)


class Filter:
  def __init__(self, value):
    self.x = value

  def update_alpha(self, _value):
    pass


class Bucket:
  def __init__(self):
    self.loaded = None

  def load_points(self, points):
    self.loaded = points


def car_params(candidate, brand, donor=False):
  CP = structs.CarParams(brand=brand, carFingerprint=candidate)
  CP.flags = int(MazdaFlags.STEER_TO_ZERO_EPS) if donor else 0
  CP.lateralTuning.init("torque")
  return CP


def estimator(CP=None, enabled=True):
  ext = TorqueEstimatorExt.__new__(TorqueEstimatorExt)
  ext.speed_binned = enabled
  ext.CP = CP or car_params(MAZDA.MAZDA_CX9, "mazda", True)
  ext._params = FakeParams()
  ext.speed_dep_seed_version = 1
  ext.speed_bin_centers = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]
  ext.speed_bin_bounds = [(0.0, 1.0)] * 7
  ext.speed_bin_lat_accel_factor_bounds = [(2.0, 6.0)] * 7
  ext.speed_bin_friction_bounds = [(0.05, 0.2)] * 7
  ext.speed_bin_filtered = [
    {"latAccelFactor": Filter(4.005), "frictionCoefficient": Filter(0.1073333333)} for _ in range(7)
  ]
  ext.speed_bin_points = [Bucket() for _ in range(7)]
  ext.speed_bin_decays = [MIN_FILTER_DECAY] * 7
  return ext


def cache_ltp(valid=True, decay=MIN_FILTER_DECAY):
  return SimpleNamespace(version=VERSION, valid=valid, decay=decay)


def cache_sp(factors=None, points=None):
  return SimpleNamespace(
    version=VERSION,
    seedVersion=1,
    speedBinCenters=[6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0],
    speedBinLatAccelFactors=factors if factors is not None else [4.1] * 7,
    speedBinFrictions=[0.11] * 7,
    speedBinPoints=points if points is not None else [[] for _ in range(7)],
  )


class TestFleetTorqueCacheRestore(unittest.TestCase):
  def test_valid_mazda_cache_restores_all_seven_bins(self):
    ext = estimator()
    points = [[[0.1, 0.2]]] * 7
    with patch("openpilot.selfdrive.locationd.torqued.TorqueEstimator.get_restore_key", return_value=("mazda", VERSION)):
      ext._restore_ext_cache(cache_ltp(), ext.CP, cache_sp(points=points))

    self.assertEqual([f["latAccelFactor"].x for f in ext.speed_bin_filtered], [4.1] * 7)
    self.assertEqual([f["frictionCoefficient"].x for f in ext.speed_bin_filtered], [0.11] * 7)
    self.assertEqual([bucket.loaded for bucket in ext.speed_bin_points], points)

  def test_absent_cache_uses_validated_mazda_namespace_and_keeps_seeds(self):
    ext = estimator()
    calls = []

    def key_probe(CP, enabled=False):
      calls.append(enabled)
      return get_torque_cache_keys(CP, enabled)

    with patch("openpilot.selfdrive.locationd.torqued.get_torque_cache_keys", side_effect=key_probe):
      ext._restore_ext_cache()

    self.assertEqual(calls, [True])
    self.assertEqual(ext._params.gets, ["LiveTorqueParametersMazda", "LiveTorqueParameters"])
    self.assertEqual([f["latAccelFactor"].x for f in ext.speed_bin_filtered], [4.005] * 7)

  def test_corrupt_cache_keeps_all_seed_values(self):
    ext = estimator()
    corrupt = cache_sp(factors=[math.nan] + [4.1] * 6)
    with patch("openpilot.selfdrive.locationd.torqued.TorqueEstimator.get_restore_key", return_value=("mazda", VERSION)):
      ext._restore_ext_cache(cache_ltp(), ext.CP, corrupt)

    self.assertEqual([f["latAccelFactor"].x for f in ext.speed_bin_filtered], [4.005] * 7)
    self.assertTrue(all(bucket.loaded is None for bucket in ext.speed_bin_points))

  def test_disabled_fleet_torque_never_reads_or_restores_cache(self):
    ext = estimator(enabled=False)
    ext._restore_ext_cache(cache_ltp(), ext.CP, cache_sp())
    self.assertEqual(ext._params.gets, [])
    self.assertEqual([f["latAccelFactor"].x for f in ext.speed_bin_filtered], [4.005] * 7)

  def test_mazda_and_suburban_restore_namespaces_are_isolated(self):
    mazda = estimator()
    suburban = estimator(car_params(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm"))

    mazda._restore_ext_cache()
    suburban._restore_ext_cache()

    self.assertEqual(mazda._params.gets[0], "LiveTorqueParametersMazda")
    self.assertEqual(suburban._params.gets[0], "LiveTorqueParametersSuburban")
    self.assertNotEqual(get_speed_dep_cache_key(mazda.CP), get_speed_dep_cache_key(suburban.CP))
    self.assertEqual(get_speed_dep_cache_key(mazda.CP), "LiveTorqueParametersSPMazda")
    self.assertEqual(get_speed_dep_cache_key(suburban.CP), "LiveTorqueParametersSPSuburban")

  def test_rejects_wrong_vehicle_cache_even_when_supplied_directly(self):
    ext = estimator()

    def restore_key(CP, version):
      return CP.carFingerprint, version

    suburban_CP = car_params(GM.CHEVROLET_SUBURBAN_CAMERA_11TH_GEN, "gm")
    with patch("openpilot.selfdrive.locationd.torqued.TorqueEstimator.get_restore_key", side_effect=restore_key):
      ext._restore_ext_cache(cache_ltp(), suburban_CP, cache_sp())

    self.assertEqual([f["latAccelFactor"].x for f in ext.speed_bin_filtered], [4.005] * 7)


if __name__ == "__main__":
  unittest.main()
