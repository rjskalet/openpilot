"""Per-vehicle, per-build validation gates for experimental lateral features.

The existing boolean integration keys alone are insufficient because they are
persisted on the comma device across both cars and across software upgrades.
An enabled feature additionally requires an exact, deliberately approved Git
commit for the detected platform. Missing metadata fails closed.
"""
from functools import cache
import re

from opendbc.car.mazda.values import MazdaFlags
from openpilot.common.version import get_build_metadata

SUBURBAN = "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"
MAZDA_CX9 = frozenset(("MAZDA_CX9", "MAZDA_CX9_2021"))


@cache
def running_revision() -> str:
  """Use the running build, never a potentially stale persisted GitCommit key."""
  try:
    metadata = get_build_metadata()
    if metadata.openpilot.is_dirty:
      return ""
    commit = metadata.openpilot.git_commit
    return commit if isinstance(commit, str) and re.fullmatch(r"[0-9a-fA-F]{40}", commit) else ""
  except Exception:
    # get_build_metadata can also raise a generic Exception when no build identity exists.
    # Failure to identify the running source must disable, never enable, the feature.
    return ""


def fleet_vehicle(cp) -> str | None:
  """Only the two validated fleet configurations are eligible."""
  if cp.brand == "gm" and cp.carFingerprint == SUBURBAN:
    return "Suburban"
  if (cp.brand == "mazda" and cp.carFingerprint in MAZDA_CX9
      and bool(cp.flags & MazdaFlags.STEER_TO_ZERO_EPS)):
    return "Mazda"
  return None


def feature_validated(params, cp, feature: str) -> bool:
  if feature not in ("Blendful", "FleetTorque"):
    raise ValueError(f"Unexpected fleet feature {feature!r}")
  vehicle = fleet_vehicle(cp)
  if vehicle is None or not params.get_bool(f"{feature}IntegrationValidated"):
    return False
  commit = running_revision()
  return bool(commit and params.get(f"{feature}ValidatedCommit{vehicle}") == commit)
