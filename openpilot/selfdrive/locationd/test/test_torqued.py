from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.locationd.torqued import TorqueEstimator, VERSION
from openpilot.sunnypilot.selfdrive.locationd.torqued_ext import (LIVE_TORQUE_PARAMETERS_SP_KEY,
                                                                 LIVE_TORQUE_PARAMETERS_SP_SERVICE,
                                                                 TorqueEstimatorExt)


class TestTorqued(OpenpilotTestCase):
  def test_cal_percent(self):
    est = TorqueEstimator(car.CarParams())
    msg = est.get_msg()
    assert msg.lateralTorqueParameters.calPerc == 0

    for (low, high), min_pts in zip(est.filtered_points.buckets.keys(),
                                    est.filtered_points.buckets_min_points.values(), strict=True):
      for _ in range(int(min_pts)):
        est.filtered_points.add_point((low + high) / 2.0, 0.0)

    # enough bucket points, but not enough total points
    msg = est.get_msg()
    assert msg.lateralTorqueParameters.calPerc == (len(est.filtered_points) / est.min_points_total * 100 + 100) / 2

    # add enough points to bucket with most capacity
    key = list(est.filtered_points.buckets)[0]
    for _ in range(est.min_points_total - len(est.filtered_points)):
      est.filtered_points.add_point((key[0] + key[1]) / 2.0, 0.0)

    msg = est.get_msg()
    assert msg.lateralTorqueParameters.calPerc == 100


class TestSpeedDependentTorqued:
  def test_speed_point_enters_only_matching_bin(self):
    ext = TorqueEstimatorExt.__new__(TorqueEstimatorExt)
    ext.speed_binned = True
    ext.speed_bin_bounds = [(5.0, 8.0), (8.0, 12.0), (12.0, 18.0)]
    ext.speed_bin_points = [MagicMock(), MagicMock(), MagicMock()]

    ext._on_torque_point(0.4, 0.8, 10.0)

    ext.speed_bin_points[0].add_point.assert_not_called()
    ext.speed_bin_points[1].add_point.assert_called_once_with(0.4, 0.8)
    ext.speed_bin_points[2].add_point.assert_not_called()

  def test_speed_bin_message_uses_registered_schema(self):
    ext = TorqueEstimatorExt.__new__(TorqueEstimatorExt)
    ext.speed_dep_seed_version = 3
    ext.speed_bin_centers = [6.5, 10.0]

    msg = ext._sp_msg(True, ([2.67, 2.70], [0.161, 0.154], [True, False]), with_points=False)
    speed_bins = getattr(msg, LIVE_TORQUE_PARAMETERS_SP_SERVICE)

    assert msg.which() == LIVE_TORQUE_PARAMETERS_SP_SERVICE
    assert msg.valid is True
    assert speed_bins.version == VERSION
    assert speed_bins.seedVersion == 3
    assert list(speed_bins.speedBinCenters) == [6.5, 10.0]
    assert list(speed_bins.speedBinValid) == [True, False]
    assert LIVE_TORQUE_PARAMETERS_SP_KEY == "LiveTorqueParametersSP"

  def test_valid_donor_eps_cache_restores_learned_bins_and_points(self):
    ext = TorqueEstimatorExt.__new__(TorqueEstimatorExt)
    ext.speed_binned = True
    ext.CP = car.CarParams(brand="mazda", carFingerprint="MAZDA CX-9 2016-20")
    ext.speed_dep_seed_version = 4
    ext.speed_bin_bounds = [(5.0, 8.0), (8.0, 12.0)]
    ext.speed_bin_centers = [6.5, 10.0]
    ext.speed_bin_lat_accel_factor_bounds = [(2.0, 3.0), (2.0, 3.0)]
    ext.speed_bin_friction_bounds = [(0.1, 0.2), (0.1, 0.2)]
    ext.speed_bin_filtered = [
      {"latAccelFactor": SimpleNamespace(x=2.1, update_alpha=MagicMock()),
       "frictionCoefficient": SimpleNamespace(x=0.11, update_alpha=MagicMock())},
      {"latAccelFactor": SimpleNamespace(x=2.2, update_alpha=MagicMock()),
       "frictionCoefficient": SimpleNamespace(x=0.12, update_alpha=MagicMock())},
    ]
    ext.speed_bin_points = [MagicMock(), MagicMock()]

    cache_ltp = SimpleNamespace(version=VERSION, valid=True, decay=5.0)
    cache_sp = SimpleNamespace(
      version=VERSION,
      seedVersion=4,
      speedBinCenters=[6.5, 10.0],
      speedBinLatAccelFactors=[2.6, 2.7],
      speedBinFrictions=[0.16, 0.15],
      speedBinPoints=[[[0.1, 0.2]], [[0.3, 0.4]]],
    )

    with patch.object(TorqueEstimator, "get_restore_key", return_value=("mazda", "cx9", VERSION)):
      ext._restore_ext_cache(cache_ltp=cache_ltp, cache_CP=ext.CP, cache_sp=cache_sp)

    assert [filters["latAccelFactor"].x for filters in ext.speed_bin_filtered] == [2.6, 2.7]
    assert [filters["frictionCoefficient"].x for filters in ext.speed_bin_filtered] == [0.16, 0.15]
    ext.speed_bin_points[0].load_points.assert_called_once_with([[0.1, 0.2]])
    ext.speed_bin_points[1].load_points.assert_called_once_with([[0.3, 0.4]])

  def test_invalid_cache_keeps_seed_values(self):
    ext = TorqueEstimatorExt.__new__(TorqueEstimatorExt)
    ext.speed_binned = True
    ext.CP = car.CarParams(brand="mazda", carFingerprint="MAZDA CX-9 2016-20")
    ext.speed_dep_seed_version = 4
    ext.speed_bin_bounds = [(5.0, 8.0)]
    ext.speed_bin_centers = [6.5]
    ext.speed_bin_lat_accel_factor_bounds = [(2.0, 3.0)]
    ext.speed_bin_friction_bounds = [(0.1, 0.2)]
    ext.speed_bin_filtered = [{
      "latAccelFactor": SimpleNamespace(x=2.67, update_alpha=MagicMock()),
      "frictionCoefficient": SimpleNamespace(x=0.161, update_alpha=MagicMock()),
    }]
    ext.speed_bin_points = [MagicMock()]
    cache_ltp = SimpleNamespace(version=VERSION, valid=False, decay=5.0)
    cache_sp = SimpleNamespace(
      version=VERSION,
      seedVersion=4,
      speedBinCenters=[6.5],
      speedBinLatAccelFactors=[2.9],
      speedBinFrictions=[0.19],
      speedBinPoints=[[]],
    )

    with patch.object(TorqueEstimator, "get_restore_key", return_value=("mazda", "cx9", VERSION)):
      ext._restore_ext_cache(cache_ltp=cache_ltp, cache_CP=ext.CP, cache_sp=cache_sp)

    assert ext.speed_bin_filtered[0]["latAccelFactor"].x == 2.67
    assert ext.speed_bin_filtered[0]["frictionCoefficient"].x == 0.161
    ext.speed_bin_points[0].load_points.assert_called_once_with([])
