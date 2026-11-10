"""Strict, reference-pinned review of the Mazda CX-9 2021 geometry replay delta.

DO NOT add broad ignores or regenerate the upstream reference to make a failing replay pass.
This gate accepts exactly the already reviewed two Mazda comparisons and their bounded
changes. All underlying diffs still appear in diff.txt and diff_report.txt.
"""
from collections import Counter, defaultdict
import math

# SunnyPilot reference artifacts against which the November 2026 review was performed.
REFERENCE_COMMIT = "3d8f211d4b2d7c4a981633783232c8ff8dbe402b"
MAZDA_ROUTE = "regenACF84CCF482|2024-08-30--03-21-55--0"
EXPECTED_PAIRS = {(MAZDA_ROUTE, "card"), (MAZDA_ROUTE, "controlsd")}

GEOMETRY = {
  "carParams.wheelbase": (3.10, 2.93),
  "carParams.centerToFront": (3.10 * 0.41, 2.93 * 0.41),
  "carParams.rotationalInertiaPerMass": (2.2541794776916504, 2.013725996017456),
}
CURVATURE = (-0.21957369148731232, -0.23231345415115356)
CONTROLLER = "controlsState.lateralControlState.torqueState."
TORQUE_FIELDS = ("carControl.actuators.torque", CONTROLLER + "output")
CURVATURE_FIELDS = ("carControl.currentCurvature", "controlsState.curvature")

# Field counts are from the *full* 2021 CX-9 replay, not from a truncated report.
# Any additional field or changed count reopens the review and fails CI.
EXPECTED_COUNTS = {
  "card": {
    "carParams.wheelbase": 2,
    "carParams.centerToFront": 2,
    "carParams.rotationalInertiaPerMass": 2,
  },
  "controlsd": {
    "carControl.currentCurvature": 2,
    "controlsState.curvature": 2,
    "carControl.actuators.torque": 3,
    CONTROLLER + "actualLateralAccel": 1219,
    CONTROLLER + "desiredLateralAccel": 27,
    CONTROLLER + "desiredLateralJerk": 3,
    CONTROLLER + "error": 1,
    CONTROLLER + "f": 25,
    CONTROLLER + "output": 3,
    CONTROLLER + "p": 1,
    CONTROLLER + "saturated": 1,
  },
}

# Absolute change caps deliberately sit just above the observed maxima. They are
# limits for this one software replay, NOT vehicle steering or Panda safety limits.
MAX_ABS_DELTAS = {
  "carControl.actuators.torque": 0.011,
  CONTROLLER + "output": 0.011,
  CONTROLLER + "actualLateralAccel": 0.024,   # m/s^2; observed max 0.0234355
  CONTROLLER + "desiredLateralAccel": 0.019,  # m/s^2; observed max 0.0186846
  CONTROLLER + "desiredLateralJerk": 1.9,     # estimated jerk; observed max 1.86154
  CONTROLLER + "error": 0.019,
  CONTROLLER + "f": 0.019,
  CONTROLLER + "p": 0.028,
}


def _finite_number(value):
  return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def accepts_reviewed_mazda_delta(segment, proc, ref_commit, result, diff_data):
  """Return True only for the two individually audited Mazda geometry differences.

  result is the full compare_logs result; diff_data is diff_process's
  (field, frame, (old, new), timestamp) collection with reference/new messages.
  Both must match to prevent a report mismatch or comparison-filter bypass.
  """
  if (segment, proc) not in EXPECTED_PAIRS or ref_commit != REFERENCE_COMMIT:
    return False
  if not isinstance(result, list) or not isinstance(diff_data, tuple) or len(diff_data) != 3:
    return False

  diffs = diff_data[0]
  if not isinstance(diffs, list):
    return False
  expected = EXPECTED_COUNTS[proc]
  if len(result) != sum(expected.values()) or len(diffs) != len(result):
    return False

  try:
    result_fields = Counter(d[1] for d in result if len(d) == 3 and d[0] == "change")
    detail_fields = Counter(d[0] for d in diffs if len(d) == 4)
    if result_fields != Counter(expected) or detail_fields != Counter(expected):
      return False

    # Each raw compare_logs tuple must have the exact same old/new values as its
    # per-frame entry in diff_process; no other kinds of difference are accepted.
    raw_values = Counter((d[1], repr(d[2][0]), repr(d[2][1])) for d in result)
    details = Counter((d[0], repr(d[2][0]), repr(d[2][1])) for d in diffs)
    if raw_values != details:
      return False

    seen = defaultdict(list)
    for field, frame, pair, _ in diffs:
      if not isinstance(frame, int) or not isinstance(pair, tuple) or len(pair) != 2:
        return False
      old, new = pair
      seen[field].append((frame, old, new))

      if field == CONTROLLER + "saturated":
        if (old, new) != (False, True) or not 1289 <= frame <= 1299:
          return False
        continue

      if not _finite_number(old) or not _finite_number(new):
        return False

      if field in GEOMETRY:
        expected_old, expected_new = GEOMETRY[field]
        if not (math.isclose(old, expected_old, abs_tol=1e-4) and
                math.isclose(new, expected_new, abs_tol=1e-4)):
          return False
      elif field in CURVATURE_FIELDS:
        if not (math.isclose(old, CURVATURE[0], abs_tol=1e-5) and
                math.isclose(new, CURVATURE[1], abs_tol=1e-5) and
                math.isclose(new / old, 3.10 / 2.93, rel_tol=1e-5)):
          return False
      elif field in MAX_ABS_DELTAS:
        if abs(new - old) > MAX_ABS_DELTAS[field]:
          return False
        # Control output is normalized by platform-specific STEER_MAX, so guard
        # that any accepted normalized actuator command stays within [-1, 1].
        if field in TORQUE_FIELDS and (abs(old) > 1.0 or abs(new) > 1.0):
          return False
        # The observed geometry change did not reverse any actual/control signal.
        if old * new < 0:
          return False
      else:
        return False

    if proc == "card":
      return all(sorted(frame for frame, _, _ in seen[field]) == [0, 1] for field in GEOMETRY)

    if any(sorted(frame for frame, _, _ in seen[field]) != [0, 1] for field in CURVATURE_FIELDS):
      return False

    # The only changed actuator requests are three consecutive 10ms frames.
    actual = sorted(seen[TORQUE_FIELDS[0]])
    output = sorted(seen[TORQUE_FIELDS[1]])
    if actual != output or [frame for frame, _, _ in actual] != [105, 106, 107]:
      return False
    return True
  except (AttributeError, IndexError, KeyError, TypeError, ValueError, ZeroDivisionError):
    return False


def acceptance_summary(results, diffs, ref_commit, full_test):
  """Accept only when *both* reviewed comparisons are present and no others differ."""
  if not full_test or ref_commit != REFERENCE_COMMIT:
    return False

  diff_lookup = {(segment, proc): data for segment, proc, data in diffs}
  changed = {(segment, proc) for segment, procs in results.items()
             for proc, value in procs.items() if isinstance(value, str) or bool(value)}
  if changed != EXPECTED_PAIRS:
    return False

  return all(accepts_reviewed_mazda_delta(segment, proc, ref_commit,
                                           results[segment][proc], diff_lookup.get((segment, proc)))
             for segment, proc in EXPECTED_PAIRS)
