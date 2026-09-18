from opendbc.car.structs import car
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.locationd.torqued import TorqueEstimator


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


  def test_initial_offset_from_static_tune(self):
    cp = car.CarParams()
    cp.lateralTuning.init('torque')
    cp.lateralTuning.torque.latAccelFactor = 0.68
    cp.lateralTuning.torque.latAccelOffset = -0.26
    cp.lateralTuning.torque.friction = 0.205

    est = TorqueEstimator(cp)
    msg = est.get_msg()
    assert msg.lateralTorqueParameters.latAccelFactorFiltered == cp.lateralTuning.torque.latAccelFactor
    assert msg.lateralTorqueParameters.latAccelOffsetFiltered == cp.lateralTuning.torque.latAccelOffset
    assert msg.lateralTorqueParameters.frictionCoefficientFiltered == cp.lateralTuning.torque.friction
