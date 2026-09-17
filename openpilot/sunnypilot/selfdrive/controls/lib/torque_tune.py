"""ZoomPilot torque tune version resolver."""
import json
import os

TORQUE_VERSIONS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "latcontrol_torque_versions.json")


def load_versions() -> dict:
  with open(TORQUE_VERSIONS_PATH) as f:
    return json.load(f)


def resolved_tune_version(params, torque_lateral_tuning: bool = True) -> float | None:
  if not params.get_bool("EnforceTorqueControl"):
    return 0.0 if torque_lateral_tuning else None
  return float(params.get("TorqueControlTune", return_default=True))
