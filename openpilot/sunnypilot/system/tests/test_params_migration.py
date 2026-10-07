"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from openpilot.common.params import Params
from openpilot.common.test import OpenpilotTestCase
from openpilot.sunnypilot.system.params_migration import CAMERA_OFFSET_MIGRATION_VERSION, _migrate_camera_offset, _migrate_model_bundle_slots


class TestCameraOffsetMigration(OpenpilotTestCase):
  def test_migrates_legacy_offset_to_neutral(self):
    params = Params()
    params.put("CameraOffset", -0.08, block=True)

    _migrate_camera_offset(params)

    assert params.get("CameraOffset") == 0.0
    assert params.get("CameraOffsetMigrated") == CAMERA_OFFSET_MIGRATION_VERSION

  def test_preserves_custom_offset(self):
    params = Params()
    params.put("CameraOffset", 0.12, block=True)

    _migrate_camera_offset(params)

    assert params.get("CameraOffset") == 0.12
    assert params.get("CameraOffsetMigrated") == CAMERA_OFFSET_MIGRATION_VERSION

  def test_missing_offset_uses_new_neutral_default(self):
    params = Params()
    params.remove("CameraOffset")

    _migrate_camera_offset(params)

    assert params.get("CameraOffset") is None
    assert params.get("CameraOffset", return_default=True) == 0.0
    assert params.get("CameraOffsetMigrated") == CAMERA_OFFSET_MIGRATION_VERSION

  def test_completed_migration_does_not_rewrite_future_value(self):
    params = Params()
    params.put("CameraOffsetMigrated", CAMERA_OFFSET_MIGRATION_VERSION, block=True)
    params.put("CameraOffset", -0.08, block=True)

    _migrate_camera_offset(params)

    assert params.get("CameraOffset") == -0.08


class TestModelBundleSlotMigration(OpenpilotTestCase):
  """Pre-split, a chestnut user's big-model selection lived in the single ActiveBundle.
  The migration seeds both slots; per-source validation later drops whichever does not
  match its own manifest."""

  def test_seeds_chestnut_slot_from_active_bundle(self):
    params = Params()
    bundle = {"ref": "big", "minimumSelectorVersion": 18}
    params.put("ModelManager_ActiveBundle", bundle, block=True)
    _migrate_model_bundle_slots(params)
    assert params.get("ModelManager_ActiveBundleChestnut") == bundle
    assert params.get("ModelManager_ActiveBundle") == bundle

  def test_noop_when_chestnut_slot_already_set(self):
    params = Params()
    params.put("ModelManager_ActiveBundle", {"ref": "small"}, block=True)
    params.put("ModelManager_ActiveBundleChestnut", {"ref": "big"}, block=True)
    _migrate_model_bundle_slots(params)
    assert params.get("ModelManager_ActiveBundleChestnut") == {"ref": "big"}

  def test_noop_when_no_selection(self):
    params = Params()
    _migrate_model_bundle_slots(params)
    assert params.get("ModelManager_ActiveBundleChestnut") is None
