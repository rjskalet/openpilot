from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.suburban_lane_center import SuburbanLaneCentering


def line(y):
  return SimpleNamespace(x=[0.0, 10.0, 20.0, 30.0], y=[y, y, y, y])


def model(path_y=0.0, left_y=-1.75, right_y=1.75, left_prob=0.95, right_prob=0.95,
          lane_change_state=log.LaneChangeState.off):
  return SimpleNamespace(
    position=line(path_y),
    laneLines=[line(0.0), line(left_y), line(right_y), line(0.0)],
    laneLineProbs=[0.0, left_prob, right_prob, 0.0],
    meta=SimpleNamespace(laneChangeState=lane_change_state),
  )


def test_v2_straight_centering_strength_is_preserved():
  helper = SuburbanLaneCentering()
  assert helper.CENTERING_GAIN == 0.30
  assert helper.MAX_CORRECTION_LAT_ACCEL == 0.15
  assert helper.MAX_CORRECTION_CURVATURE == 0.0010
  assert helper.MAX_CORRECTION_STEP == 2.0e-5


def test_centered_lane_has_no_bias():
  helper = SuburbanLaneCentering()
  corrected = helper.update(model(), 20.0, 0.0)
  assert corrected == 0.0


def test_left_of_lane_center_generates_rightward_curvature_correction():
  helper = SuburbanLaneCentering()
  # Model path is at y=0 while lane midpoint is +0.20 m (right).
  m = model(path_y=0.0, left_y=-1.55, right_y=1.95)
  corrected = 0.0
  for _ in range(100):
    corrected = helper.update(m, 20.0, 0.0)
  assert corrected > 0.0
  assert corrected <= helper.MAX_CORRECTION_LAT_ACCEL / (20.0 ** 2)


def test_low_confidence_and_lane_change_release_correction():
  helper = SuburbanLaneCentering()
  biased = model(path_y=0.0, left_y=-1.55, right_y=1.95)
  for _ in range(50):
    helper.update(biased, 20.0, 0.0)
  assert helper.correction_curvature > 0.0

  low_confidence = model(path_y=0.0, left_y=-1.55, right_y=1.95, left_prob=0.2)
  previous = helper.correction_curvature
  helper.update(low_confidence, 20.0, 0.0)
  assert 0.0 <= helper.correction_curvature < previous

  lane_change = model(path_y=0.0, left_y=-1.55, right_y=1.95,
                      lane_change_state=log.LaneChangeState.preLaneChange)
  helper.update(lane_change, 20.0, 0.0)
  assert helper.correction_curvature == 0.0


def test_curve_weight_keeps_straights_and_fades_gentle_bends():
  helper = SuburbanLaneCentering()

  # At 20 m/s these correspond to 0.04, 0.14, and 0.24 m/s^2.
  full = helper._curve_weight(0.00010, 20.0)
  partial = helper._curve_weight(0.00035, 20.0)
  off = helper._curve_weight(0.00060, 20.0)

  assert full == 1.0
  assert 0.0 < partial < 1.0
  assert off == 0.0


def test_meaningful_curve_releases_centering_without_reversing_it():
  helper = SuburbanLaneCentering()
  biased = model(path_y=0.0, left_y=-1.55, right_y=1.95)
  for _ in range(100):
    helper.update(biased, 20.0, 0.0)
  initial = helper.correction_curvature
  assert initial > 0.0

  # 0.001 1/m at 20 m/s is 0.4 m/s^2, so the lane-centering target is zero.
  corrected = helper.update(biased, 20.0, 0.001)
  assert 0.0 <= helper.correction_curvature < initial
  assert corrected >= 0.001

  # The straight-road correction should decay to zero and never flip sign.
  for _ in range(100):
    helper.update(biased, 20.0, 0.001)
    assert helper.correction_curvature >= 0.0
  assert helper.correction_curvature == 0.0
