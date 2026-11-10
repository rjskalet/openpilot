"""Adversarial tests: unexpected replay changes must never receive automatic acceptance."""
import unittest
from copy import deepcopy

from openpilot.selfdrive.test.process_replay.accepted_deltas import (
  REFERENCE_COMMIT, MAZDA_ROUTE, EXPECTED_COUNTS, GEOMETRY, CURVATURE,
  CONTROLLER, TORQUE_FIELDS, CURVATURE_FIELDS, acceptance_summary,
  accepts_reviewed_mazda_delta,
)


def sample(proc):
  raw, frames = [], []
  for field, count in EXPECTED_COUNTS[proc].items():
    for i in range(count):
      frame = i
      if field in GEOMETRY:
        old, new = GEOMETRY[field]
      elif field in CURVATURE_FIELDS:
        old, new = CURVATURE
      elif field == CONTROLLER + "saturated":
        old, new, frame = False, True, 1294
      elif field in TORQUE_FIELDS:
        old, new, frame = 0.4, 0.405, 105 + i
      else:
        old, new, frame = 0.5, 0.505, 104 + i
      raw.append(("change", field, (old, new)))
      frames.append((field, frame, (old, new), 0))
  return raw, (frames, {}, {})


class TestMazdaReplayAcceptance(unittest.TestCase):
  def setUp(self):
    self.card, self.card_details = sample("card")
    self.control, self.control_details = sample("controlsd")

  def pair(self, proc="controlsd", route=MAZDA_ROUTE, ref=REFERENCE_COMMIT, raw=None, details=None):
    return accepts_reviewed_mazda_delta(
      route, proc, ref,
      self.control if raw is None else raw,
      self.control_details if details is None else details)

  def test_exact_reviewed_card_and_control_pairs(self):
    self.assertTrue(self.pair())
    self.assertTrue(accepts_reviewed_mazda_delta(
      MAZDA_ROUTE, "card", REFERENCE_COMMIT, self.card, self.card_details))

  def test_complete_pass_requires_exactly_two_known_pairs(self):
    results = {MAZDA_ROUTE: {"card": self.card, "controlsd": self.control}, "GM": {"controlsd": []}}
    data = [(MAZDA_ROUTE, "card", self.card_details),
            (MAZDA_ROUTE, "controlsd", self.control_details)]
    self.assertTrue(acceptance_summary(results, data, REFERENCE_COMMIT, True))
    self.assertFalse(acceptance_summary(results, data, REFERENCE_COMMIT, False))
    self.assertFalse(acceptance_summary(results, data, "new-baseline", True))
    results["GM"]["controlsd"] = "missing reference"
    self.assertFalse(acceptance_summary(results, data, REFERENCE_COMMIT, True))

  def test_reject_other_route_or_reference_commit(self):
    self.assertFalse(self.pair(route="other-segment"))
    self.assertFalse(self.pair(ref="different-reference"))

  def test_reject_extra_field_and_count_change(self):
    raw, data = deepcopy(self.control), deepcopy(self.control_details)
    raw.append(("change", "carControl.enabled", (False, True)))
    data[0].append(("carControl.enabled", 0, (False, True), 0))
    self.assertFalse(self.pair(raw=raw, details=data))
    self.assertFalse(self.pair(raw=self.control[:-1]))

  def test_reject_unbounded_actuator_torque(self):
    raw, data = deepcopy(self.control), deepcopy(self.control_details)
    for i, (kind, field, pair) in enumerate(raw):
      if field == TORQUE_FIELDS[0]:
        raw[i] = (kind, field, (0.4, 0.5))
        break
    for i, (field, frame, pair, ts) in enumerate(data[0]):
      if field == TORQUE_FIELDS[0]:
        data[0][i] = (field, frame, (0.4, 0.5), ts)
        break
    self.assertFalse(self.pair(raw=raw, details=data))

  def test_reject_wrong_curvature_and_geometry(self):
    raw, data = deepcopy(self.card), deepcopy(self.card_details)
    raw[0] = ("change", "carParams.wheelbase", (3.10, 3.10))
    data[0][0] = ("carParams.wheelbase", 0, (3.10, 3.10), 0)
    self.assertFalse(accepts_reviewed_mazda_delta(MAZDA_ROUTE, "card", REFERENCE_COMMIT, raw, data))

  def test_reject_saturation_shift_outside_single_frame_window(self):
    raw, data = deepcopy(self.control), deepcopy(self.control_details)
    for i, (field, frame, pair, ts) in enumerate(data[0]):
      if field == CONTROLLER + "saturated":
        data[0][i] = (field, 1500, pair, ts)
        break
    self.assertFalse(self.pair(raw=raw, details=data))

  def test_reject_report_mismatch_or_non_finite_data(self):
    data = deepcopy(self.control_details)
    data[0][0] = ("carControl.currentCurvature", 0, (float("nan"), CURVATURE[1]), 0)
    self.assertFalse(self.pair(details=data))
    self.assertFalse(self.pair(details="error"))


if __name__ == "__main__":
  unittest.main()
