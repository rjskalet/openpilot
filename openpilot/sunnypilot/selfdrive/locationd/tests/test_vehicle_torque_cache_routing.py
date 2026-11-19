"""Regression tests for sharing one comma across the CX-9 and camera-ACC Suburban.

These pin the proven master cache namespaces while exercising ZoomPilot's
current per-bin learned flags, decays and restore-key validation.
"""
import pytest

from openpilot.selfdrive.locationd.torqued import (
  TorqueEstimator, get_torque_cache_keys, get_torque_cache_pair, save_torque_cache,
)
from openpilot.sunnypilot.selfdrive.locationd.speed_bin_learner import (
  get_speed_dep_cache_key, LIVE_TORQUE_PARAMETERS_SP_KEY,
)
from openpilot.sunnypilot.selfdrive.locationd.tests.speed_dep_helpers import (
  FakePubMaster, in_bounds_values, make_cache, make_cache_sp, make_cp,
)


SUBURBAN = "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"
CX9 = "MAZDA_CX9"
CX9_2021 = "MAZDA_CX9_2021"


@pytest.fixture
def vehicle_params(fake_params):
  # Suburban's measured factor floor is a persistent float default of 1.0.
  fake_params.put("SuburbanFactorFloorScale", 1.0)
  return fake_params


@pytest.mark.parametrize("fingerprint,torque_key,cp_key,sp_key", [
  (SUBURBAN, "LiveTorqueParametersSuburban", "LiveTorqueCarParamsSuburban", "LiveTorqueParametersSPSuburban"),
  (CX9, "LiveTorqueParametersMazda", "LiveTorqueCarParamsMazda", "LiveTorqueParametersSPMazda"),
  (CX9_2021, "LiveTorqueParametersMazda", "LiveTorqueCarParamsMazda", "LiveTorqueParametersSPMazda"),
  ("OTHER_CAR", "LiveTorqueParameters", "CarParamsPrevRoute", "LiveTorqueParametersSP"),
])
def test_vehicle_cache_namespace_matches_previous_master(fingerprint, torque_key, cp_key, sp_key):
  cp = make_cp(fingerprint=fingerprint)
  assert get_torque_cache_keys(cp) == (torque_key, cp_key)
  assert get_speed_dep_cache_key(cp) == sp_key


def healthy_caches(cp, est):
  lafs, frictions = in_bounds_values(est)
  points = [[[0.11, 0.30]] for _ in est.speed_bin_bounds]
  global_fit = make_cache(global_laf=cp.lateralTuning.torque.latAccelFactor,
                          global_friction=cp.lateralTuning.torque.friction, decay=150.0).to_bytes()
  speed_bins = make_cache_sp(
    est.speed_bin_centers, lafs, frictions, points=points,
    seed_version=est.speed_dep_seed_version, learned=[True] * len(est.speed_bin_centers),
    decays=[120.0] * len(est.speed_bin_centers),
  ).to_bytes()
  return global_fit, speed_bins, lafs, frictions


def test_both_vehicle_caches_survive_switching_and_global_overwrite(vehicle_params):
  """Each vehicle restores its own global fit, bin values, learned state and points."""
  saved = {}
  for fingerprint in [SUBURBAN, CX9_2021]:
    cp = make_cp(fingerprint=fingerprint)
    est = TorqueEstimator(cp)
    cache, sp_cache, lafs, frictions = healthy_caches(cp, est)
    torque_key, cp_key = get_torque_cache_keys(cp)
    sp_key = get_speed_dep_cache_key(cp)
    vehicle_params.put(torque_key, cache)
    vehicle_params.put(cp_key, cp.to_bytes())
    vehicle_params.put(sp_key, sp_cache)
    saved[fingerprint] = cp, cache, sp_cache, lafs, frictions

  # The shared cache now belongs to Mazda. The Suburban must ignore it.
  mazda_cp, mazda_fit, mazda_sp, _, _ = saved[CX9_2021]
  vehicle_params.put("CarParamsPrevRoute", mazda_cp.to_bytes())
  vehicle_params.put("LiveTorqueParameters", mazda_fit)
  vehicle_params.put(LIVE_TORQUE_PARAMETERS_SP_KEY, mazda_sp)

  for fingerprint in [SUBURBAN, CX9_2021, SUBURBAN]:
    cp, _, _, lafs, frictions = saved[fingerprint]
    est = TorqueEstimator(cp)
    assert est.decay == pytest.approx(150.0)
    assert est.speed_bin_learned[0]
    assert est.speed_bin_decays[0] == pytest.approx(120.0)
    assert est.speed_bin_filtered[0]["latAccelFactor"].x == pytest.approx(lafs[0], rel=1e-6)
    assert est.speed_bin_filtered[0]["frictionCoefficient"].x == pytest.approx(frictions[0], rel=1e-6)
    assert len(est.speed_bin_points[0]) == 1


@pytest.mark.parametrize("fingerprint", [SUBURBAN, CX9_2021])
def test_matching_global_cache_migrates_once_into_vehicle_namespace(vehicle_params, fingerprint):
  cp = make_cp(fingerprint=fingerprint)
  est = TorqueEstimator(cp)
  torque_blob, sp_blob, lafs, _ = healthy_caches(cp, est)
  vehicle_params.put("CarParamsPrevRoute", cp.to_bytes())
  vehicle_params.put("LiveTorqueParameters", torque_blob)
  vehicle_params.put(LIVE_TORQUE_PARAMETERS_SP_KEY, sp_blob)

  restored = TorqueEstimator(cp)
  torque_key, cp_key = get_torque_cache_keys(cp)
  sp_key = get_speed_dep_cache_key(cp)
  assert get_torque_cache_pair(vehicle_params, cp) == (torque_blob, cp.to_bytes())
  assert vehicle_params.get(torque_key) == torque_blob
  assert vehicle_params.get(cp_key) == cp.to_bytes()
  assert vehicle_params.get(sp_key) == sp_blob
  assert restored.speed_bin_learned[0]
  assert restored.speed_bin_filtered[0]["latAccelFactor"].x == pytest.approx(lafs[0], rel=1e-6)


def test_wrong_vehicle_global_cache_cannot_migrate_into_mazda(vehicle_params):
  suburban_cp = make_cp(fingerprint=SUBURBAN)
  suburban_est = TorqueEstimator(suburban_cp)
  torque_blob, sp_blob, _, _ = healthy_caches(suburban_cp, suburban_est)
  vehicle_params.put("CarParamsPrevRoute", suburban_cp.to_bytes())
  vehicle_params.put("LiveTorqueParameters", torque_blob)
  vehicle_params.put(LIVE_TORQUE_PARAMETERS_SP_KEY, sp_blob)

  mazda_cp = make_cp(fingerprint=CX9)
  restored = TorqueEstimator(mazda_cp)
  torque_key, cp_key = get_torque_cache_keys(mazda_cp)
  assert get_torque_cache_pair(vehicle_params, mazda_cp) == (None, None)
  assert vehicle_params.get(torque_key) is None
  assert vehicle_params.get(cp_key) is None
  assert vehicle_params.get(get_speed_dep_cache_key(mazda_cp)) is None
  assert not any(restored.speed_bin_learned)


@pytest.mark.parametrize("fingerprint", [SUBURBAN, CX9_2021])
def test_new_speed_bin_writer_persists_shared_and_vehicle_copies(vehicle_params, fingerprint):
  cp = make_cp(fingerprint=fingerprint)
  est = TorqueEstimator(cp)
  est._pm = FakePubMaster()
  est.frame += 1
  est.get_msg(valid=True, with_points=True)
  dedicated = vehicle_params.get(get_speed_dep_cache_key(cp))
  assert dedicated is not None
  assert dedicated == vehicle_params.get(LIVE_TORQUE_PARAMETERS_SP_KEY)


@pytest.mark.parametrize("fingerprint", [SUBURBAN, CX9_2021])
def test_global_fit_writer_saves_matching_params_and_rejects_other_vehicle(vehicle_params, fingerprint):
  cp = make_cp(fingerprint=fingerprint)
  other = make_cp(fingerprint=CX9_2021 if fingerprint == SUBURBAN else SUBURBAN)
  fit = make_cache().to_bytes()
  torque_key, cp_key = get_torque_cache_keys(cp)
  vehicle_params.put("CarParams", cp.to_bytes())
  save_torque_cache(vehicle_params, cp, fit)
  assert vehicle_params.get("LiveTorqueParameters") == fit
  assert vehicle_params.get(torque_key) == fit
  assert vehicle_params.get(cp_key) == cp.to_bytes()

  replacement_fit = make_cache(decay=200.0).to_bytes()
  vehicle_params.put("CarParams", other.to_bytes())
  save_torque_cache(vehicle_params, cp, replacement_fit)
  assert vehicle_params.get("LiveTorqueParameters") == replacement_fit
  assert vehicle_params.get(torque_key) == fit
  assert vehicle_params.get(cp_key) == cp.to_bytes()


def test_malformed_carparams_never_overwrites_dedicated_cache(vehicle_params):
  cp = make_cp(fingerprint=SUBURBAN)
  fit = make_cache().to_bytes()
  vehicle_params.put("CarParams", b"not valid capnp")
  save_torque_cache(vehicle_params, cp, fit)
  torque_key, cp_key = get_torque_cache_keys(cp)
  assert vehicle_params.get("LiveTorqueParameters") == fit
  assert vehicle_params.get(torque_key) is None
  assert vehicle_params.get(cp_key) is None
