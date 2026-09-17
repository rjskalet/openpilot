from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.sunnypilot.selfdrive.controls.lib.suburban_lane_center import SuburbanLaneCentering


def line(y):
  return SimpleNamespace(x=[0.0, 10.0, 20.0, 30.0], y=[y, y, y, y])


def model(path_y=0.0, left_y=1.75, right_y=-1.75, left_prob=0.95, right_prob=0.95,
          lane_change_state=log.LaneChangeState.off):
  return SimpleNamespace(
    position=line(path_y),
    laneLines=[line(0.0), line(left_y), line(right_y), line(0.0)],
    laneLineProbs=[0.0, left_prob, right_prob, 0.0],
    meta=SimpleNamespace(laneChangeState=lane_change_state),
  )


def test_centered_lane_has_no_bias():
  helper = SuburbanLaneCentering()
  corrected = helper.update(model(), 20.0, 0.001)
  assert corrected == 0.001


def test_left_of_lane_center_generates_rightward_curvature_correction():
  helper = SuburbanLaneCentering()
  # Lane midpoint is 0.20 m to the right of the model path.
  m = model(path_y=0.0, left_y=1.55, right_y=-1.95)
  corrected = 0.0
  for _ in range(100):
    corrected = helper.update(m, 20.0, 0.0)
  assert corrected > 0.0
  assert corrected <= helper.MAX_CORRECTION_LAT_ACCEL / (20.0 ** 2)


def test_low_confidence_and_lane_change_fade_correction():
  helper = SuburbanLaneCentering()
  biased = model(path_y=0.0, left_y=1.55, right_y=-1.95)
  for _ in range(50):
    helper.update(biased, 20.0, 0.0)
  assert helper.correction_curvature > 0.0

  low_confidence = model(path_y=0.0, left_y=1.55, right_y=-1.95, left_prob=0.2)
  previous = helper.correction_curvature
  helper.update(low_confidence, 20.0, 0.0)
  assert 0.0 <= helper.correction_curvature < previous

  lane_change = model(path_y=0.0, left_y=1.55, right_y=-1.95,
                      lane_change_state=log.LaneChangeState.preLaneChange)
  previous = helper.correction_curvature
  helper.update(lane_change, 20.0, 0.0)
  assert 0.0 <= helper.correction_curvature < previous
