import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.values import CAR, MazdaFlags, STEER_TO_ZERO_EPS_FW
from openpilot.selfdrive.locationd import torqued_zoompilot
from openpilot.selfdrive.locationd.torqued_zoompilot import MazdaTorqueBins, SpeedTorqueBuckets, get_speed_dep_config


class FakeParams:
  def get(self, key):
    return None

  def put_nonblocking(self, key, value):
    pass


@pytest.fixture
def donor_cp():
  fw = next(iter(STEER_TO_ZERO_EPS_FW))
  return interfaces[CAR.MAZDA_CX9].get_params(
    CAR.MAZDA_CX9,
    {i: {} for i in range(8)},
    [structs.CarParams.CarFw(ecu="eps", fwVersion=fw)],
    alpha_long=False,
    is_release=False,
    docs=False,
    starpilot_toggles=None,
  )


@pytest.fixture
def no_cache(monkeypatch):
  monkeypatch.setattr(torqued_zoompilot, "Params", lambda: FakeParams())


def make_bins(CP):
  return MazdaTorqueBins(
    CP,
    min_bucket_points=np.array([100, 300, 500, 500, 500, 500, 300, 100]),
    factor_sanity=0.3,
    friction_sanity=0.5,
    fit_points=2000,
    version=1,
  )


def test_cx9_donor_uses_cx5_2022_speed_table(donor_cp):
  cfg = get_speed_dep_config(donor_cp)
  assert cfg["seed_version"] == 2
  assert cfg["speed_bp"] == [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 34.5, 37.0]
  assert cfg["laf_bp"] == [2.37, 2.63, 2.13, 1.74, 1.85, 2.27, 2.54, 2.56]


def test_non_donor_cx9_cannot_use_donor_speed_table():
  CP = interfaces[CAR.MAZDA_CX9].get_params(
    CAR.MAZDA_CX9,
    {i: {} for i in range(8)},
    [],
    alpha_long=False,
    is_release=False,
    docs=False,
    starpilot_toggles=None,
  )
  assert not CP.flags & MazdaFlags.STEER_TO_ZERO_EPS
  assert get_speed_dep_config(CP) == {}


def test_live_message_uses_seed_until_bin_is_valid(donor_cp, no_cache):
  bins = make_bins(donor_cp)
  assert bins.enabled

  # A partially learned filter must not influence steering until its bucket passes validity.
  bins.speed_bin_filtered[0]["latAccelFactor"].x = bins.seed_factors[0] * 1.1
  bins.speed_bin_filtered[0]["frictionCoefficient"].x = bins.seed_frictions[0] * 1.1

  live = bins._message(True, with_points=False).customReserved16
  assert live.speedBinLatAccelFactors[0] == pytest.approx(bins.seed_factors[0])
  assert live.speedBinFrictions[0] == pytest.approx(bins.seed_frictions[0])

  bins.speed_bin_valid[0] = True
  live = bins._message(True, with_points=False).customReserved16
  assert live.speedBinLatAccelFactors[0] == pytest.approx(bins.seed_factors[0] * 1.1)
  assert live.speedBinFrictions[0] == pytest.approx(bins.seed_frictions[0] * 1.1)


def test_speed_bin_routes_low_speed_point(donor_cp, no_cache):
  bins = make_bins(donor_cp)
  assert len(bins.speed_bin_points[0]) == 0
  bins.on_torque_point(0.2, 0.3, bins.speed_bin_centers[0])
  assert len(bins.speed_bin_points[0]) == 1


def test_cached_point_shape_round_trips_through_bucket():
  bucket = SpeedTorqueBuckets(
    x_bounds=[(-0.5, 0.5)],
    min_points=[1],
    min_points_total=1,
    points_per_bucket=10,
    rowsize=3,
  )
  bucket.load_points([[0.2, 0.3]])
  points = bucket.get_points()
  assert np.allclose(points, [[0.2, 1.0, 0.3]])
  cache_shape = points[:, [0, 2]].tolist()
  restored = SpeedTorqueBuckets(
    x_bounds=[(-0.5, 0.5)],
    min_points=[1],
    min_points_total=1,
    points_per_bucket=10,
    rowsize=3,
  )
  restored.load_points(cache_shape)
  assert np.allclose(restored.get_points(), points)
