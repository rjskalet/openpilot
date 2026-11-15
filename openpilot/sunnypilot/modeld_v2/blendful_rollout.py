"""Fail-closed activation gate for the SunnyPilot-first Blendful integration.

The stock UI toggle is not sufficient during a migration: devices may retain
Blendful=True from an earlier installation. Only intentionally validated
vehicle builds may enable steering curvature correction.
"""

from openpilot.sunnypilot.fleet_validation import feature_validated


def blendful_integration_enabled(params, cp) -> bool:
  return bool(params.get_bool("Blendful") and feature_validated(params, cp, "Blendful"))
