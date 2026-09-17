import numpy as np

from openpilot.cereal import log


class SuburbanLaneCentering:
  """Small, confidence-gated curvature correction for the 11th-gen Suburban.

  The driving model can place its path slightly inside/left of the detected lane.
  This helper compares the model path with the midpoint of the two lane lines at
  10 m and 20 m, then gently biases desired curvature back toward lane center.
  """

  MIN_LANE_PROB = 0.75
  MIN_LANE_WIDTH_M = 2.5
  MAX_LANE_WIDTH_M = 4.8

  # Only apply a fraction of the geometric correction on the first iteration.
  CENTERING_GAIN = 0.35
  MAX_CORRECTION_LAT_ACCEL = 0.25  # m/s^2
  MAX_CORRECTION_CURVATURE = 0.0012  # 1/m, additional hard guard
  MAX_CORRECTION_STEP = 2.0e-5  # 1/m per 100 Hz control step

  FULL_CORRECTION_SPEED = 10.0  # m/s; fade in from 5 m/s to avoid low-speed jitter
  MIN_CORRECTION_SPEED = 5.0

  def __init__(self):
    self.correction_curvature = 0.0

  @staticmethod
  def _line_y_at(line, distance_m: float) -> float:
    if len(line.x) < 2 or len(line.y) < 2:
      raise ValueError("lane/path line has insufficient points")
    return float(np.interp(distance_m, np.asarray(line.x), np.asarray(line.y)))

  def _target_correction(self, model_v2, v_ego: float) -> float:
    if model_v2.meta.laneChangeState != log.LaneChangeState.off:
      return 0.0

    if len(model_v2.laneLines) < 3 or len(model_v2.laneLineProbs) < 3:
      return 0.0

    left_prob = float(model_v2.laneLineProbs[1])
    right_prob = float(model_v2.laneLineProbs[2])
    if min(left_prob, right_prob) < self.MIN_LANE_PROB:
      return 0.0

    try:
      path_10 = self._line_y_at(model_v2.position, 10.0)
      path_20 = self._line_y_at(model_v2.position, 20.0)
      left_10 = self._line_y_at(model_v2.laneLines[1], 10.0)
      left_20 = self._line_y_at(model_v2.laneLines[1], 20.0)
      right_10 = self._line_y_at(model_v2.laneLines[2], 10.0)
      right_20 = self._line_y_at(model_v2.laneLines[2], 20.0)
    except (ValueError, TypeError):
      return 0.0

    width_10 = abs(left_10 - right_10)
    width_20 = abs(left_20 - right_20)
    if not (self.MIN_LANE_WIDTH_M <= width_10 <= self.MAX_LANE_WIDTH_M and
            self.MIN_LANE_WIDTH_M <= width_20 <= self.MAX_LANE_WIDTH_M):
      return 0.0

    center_10 = 0.5 * (left_10 + right_10)
    center_20 = 0.5 * (left_20 + right_20)

    # Model coordinates are +Y left while openpilot desired curvature is +right.
    # For a small arc y ~= -0.5 * curvature * x^2.
    correction_10 = -2.0 * (center_10 - path_10) / (10.0 ** 2)
    correction_20 = -2.0 * (center_20 - path_20) / (20.0 ** 2)

    # The 20 m estimate is less sensitive to lane-line noise, but 10 m responds
    # faster to the straight-road offset visible in the Suburban route data.
    target = self.CENTERING_GAIN * (0.35 * correction_10 + 0.65 * correction_20)

    speed_weight = float(np.clip((v_ego - self.MIN_CORRECTION_SPEED) /
                                 (self.FULL_CORRECTION_SPEED - self.MIN_CORRECTION_SPEED), 0.0, 1.0))
    target *= speed_weight

    accel_curvature_cap = self.MAX_CORRECTION_LAT_ACCEL / max(v_ego * v_ego, 25.0)
    curvature_cap = min(self.MAX_CORRECTION_CURVATURE, accel_curvature_cap)
    return float(np.clip(target, -curvature_cap, curvature_cap))

  def update(self, model_v2, v_ego: float, desired_curvature: float) -> float:
    target = self._target_correction(model_v2, v_ego)
    step = float(np.clip(target - self.correction_curvature,
                         -self.MAX_CORRECTION_STEP, self.MAX_CORRECTION_STEP))
    self.correction_curvature += step
    return float(desired_curvature + self.correction_curvature)
