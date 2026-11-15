"""Regression: modeld and onroad UI share the same Blendful message schema."""

import math

from openpilot.cereal import log, messaging


def test_model_data_v2_sp_blendful_fields_round_trip():
  msg = messaging.new_message("modelDataV2SP")
  data = msg.modelDataV2SP

  expected_floats = {
    "blendfulCurvature": 0.00125,
    "blendfulTrust": 0.75,
    "modelCurvature": -0.0005,
    "blendfulLeftProb": 0.9,
    "blendfulRightProb": 0.8,
    "blendfulLaneWidth": 3.5,
  }

  for field, value in expected_floats.items():
    setattr(data, field, value)
  data.blendfulGoingAround = True

  with log.Event.from_bytes(msg.to_bytes()) as parsed:
    assert parsed.which() == "modelDataV2SP"
    restored = parsed.modelDataV2SP
    for field, value in expected_floats.items():
      assert math.isclose(getattr(restored, field), value, rel_tol=1e-5, abs_tol=1e-6)
    assert restored.blendfulGoingAround is True
