"""Controller selection must remain consistent with Mazda FleetTorque validation."""

from typing import cast
from unittest.mock import patch

from opendbc.car import structs
from opendbc.car.mazda.values import MazdaFlags

from openpilot.cereal import custom
from openpilot.common.params import Params
from openpilot.sunnypilot.selfdrive.controls.controlsd_ext import ControlsExt


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def get(self, key, return_default=False):
    return self.values.get(key)


def torque_cp(*, donor=True, brand="mazda"):
  cp = structs.CarParams(brand=brand, carFingerprint="MAZDA_CX9")
  cp.flags = int(MazdaFlags.STEER_TO_ZERO_EPS) if donor else 0
  cp.lateralTuning.init('torque')
  cp.lateralTuning.torque.latAccelFactor = 2.67
  cp.lateralTuning.torque.friction = 0.161
  return cp


def controller_ext(cp, params, *, validated):
  ext = ControlsExt.__new__(ControlsExt)
  ext.CP = cp
  ext.CP_SP = custom.CarParamsSP()
  ext.params = cast(Params, params)
  ext._donor_eps_mazda = cp.brand == "mazda" and bool(cp.flags & MazdaFlags.STEER_TO_ZERO_EPS)
  ext._zoompilot_mazda = validated and ext._donor_eps_mazda
  return ext


class TestMazdaControllerSelection:
  def test_validated_donor_eps_selects_v2(self):
    cp = torque_cp()
    ext = controller_ext(cp, FakeParams({"EnforceTorqueControl": True, "TorqueControlTune": 2.0}), validated=True)
    expected = object()
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV2", return_value=expected) as v2:
      assert ext.initialize_lateral_control(object(), object(), 0.01) is expected
    v2.assert_called_once()

  def test_unvalidated_donor_eps_fails_closed_to_v0(self):
    cp = torque_cp()
    ext = controller_ext(cp, FakeParams({"EnforceTorqueControl": True, "TorqueControlTune": 2.0}), validated=False)
    expected = object()
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV0", return_value=expected) as v0, \
         patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV2") as v2:
      assert ext.initialize_lateral_control(object(), object(), 0.01) is expected
    v0.assert_called_once()
    v2.assert_not_called()

  def test_validated_donor_eps_respects_explicit_v0(self):
    cp = torque_cp()
    ext = controller_ext(cp, FakeParams({"EnforceTorqueControl": True, "TorqueControlTune": 0.0}), validated=True)
    expected = object()
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV0", return_value=expected) as v0:
      assert ext.initialize_lateral_control(object(), object(), 0.01) is expected
    v0.assert_called_once()

  def test_non_donor_generic_v1_path_is_unchanged(self):
    cp = torque_cp(donor=False)
    legacy_v1 = object()
    ext = controller_ext(cp, FakeParams({"EnforceTorqueControl": True, "TorqueControlTune": 2.0}), validated=False)
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV0") as v0, \
         patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV2") as v2:
      assert ext.initialize_lateral_control(legacy_v1, object(), 0.01) is legacy_v1
    v0.assert_not_called()
    v2.assert_not_called()

  def test_other_brand_cannot_enter_donor_path(self):
    cp = torque_cp(brand="toyota")
    legacy_v1 = object()
    ext = controller_ext(cp, FakeParams({"EnforceTorqueControl": True, "TorqueControlTune": 2.0}), validated=True)
    with patch("openpilot.sunnypilot.selfdrive.controls.controlsd_ext.LatControlTorqueV2") as v2:
      assert ext.initialize_lateral_control(legacy_v1, object(), 0.01) is legacy_v1
    v2.assert_not_called()
