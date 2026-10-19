"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.structs import car
from openpilot.cereal import log, custom

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

RELAXED_MIN_BUCKET_POINTS = np.array([1, 200, 300, 500, 500, 300, 200, 1])

ALLOWED_CARS = ['toyota', 'hyundai', 'rivian', 'honda']

DEFAULT_SPEED_BIN_BOUNDS = [(5, 8), (8, 12), (12, 18), (18, 24), (24, 29), (29, 35), (35, 40)]
DEFAULT_SPEED_BIN_CENTERS = [6.5, 10.0, 15.0, 21.0, 26.5, 32.0, 37.5]
SUBURBAN_LIVE_TABLE_FINGERPRINT = "CHEVROLET_SUBURBAN_CAMERA_11TH_GEN"
SUBURBAN_MIN_LEARNING_SPEED = 8.0
SUBURBAN_MIN_YAW_LAT_ACCEL = 0.07
SUBURBAN_MAX_LEARNING_SPEED = 38.0
SUBURBAN_FACTOR_MAX_BP = [1.360, 1.360, 1.360, 1.200, 1.200, 1.300, 1.400]
SUBURBAN_FACTOR_FLOOR_SCALE_MIN = 0.90
SUBURBAN_FACTOR_FLOOR_SCALE_MAX = 1.15
MAZDA_CX9_FINGERPRINTS = {"MAZDA_CX9", "MAZDA_CX9_2021"}

LIVE_TORQUE_PARAMETERS_SP_SERVICE = "customReserved19"
LIVE_TORQUE_PARAMETERS_SP_KEY = "LiveTorqueParametersSP"
LIVE_TORQUE_PARAMETERS_SP_SUBURBAN_KEY = "LiveTorqueParametersSPSuburban"
LIVE_TORQUE_PARAMETERS_SP_MAZDA_KEY = "LiveTorqueParametersSPMazda"
LiveTorqueParametersSP = custom.CustomReserved19


def get_speed_dep_cache_key(CP):
  if CP.carFingerprint == SUBURBAN_LIVE_TABLE_FINGERPRINT:
    return LIVE_TORQUE_PARAMETERS_SP_SUBURBAN_KEY
  if CP.carFingerprint in MAZDA_CX9_FINGERPRINTS:
    return LIVE_TORQUE_PARAMETERS_SP_MAZDA_KEY
  return LIVE_TORQUE_PARAMETERS_SP_KEY


class TorqueEstimatorExt:
  """Per-speed-bin torque learning, mixed into TorqueEstimator.

  Each bin runs upstream's total-least-squares fit on the quality-filtered points that fall
  in its speed range and publishes its own latAccelFactor and friction; the controller
  interpolates them by speed. Bins come from speed_dependent.toml, or the defaults above
  seeded with the car's global offline values. Gated by SpeedDependentTorqueToggle.
  """

  def __init__(self, CP: car.CarParams):
    self.CP = CP
    self._params = Params()
    self.frame = -1

    self.enforce_torque_control_toggle = self._params.get_bool("EnforceTorqueControl")
    self.use_params = self.CP.brand in ALLOWED_CARS and self.CP.lateralTuning.which() == 'torque'
    self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
    self.custom_torque_params = self._params.get_bool("CustomTorqueParams")
    self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")
    self.speed_binned = (self.CP.lateralTuning.which() == 'torque'
                         and self._params.get_bool("SpeedDependentTorqueToggle")
                         and self.enforce_torque_control_toggle
                         and self.use_live_torque_params)
    self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS
    self.factor_sanity = 0.0
    self.friction_sanity = 0.0
    self.offline_latAccelFactor = 0.0
    self.offline_friction = 0.0
    self._pm = None
    self._sp_pub_frame = -1

  def initialize_custom_params(self, decimated=False):
    self.update_use_params()

    if self.enforce_torque_control_toggle:
      if self._params.get_bool("LiveTorqueParamsRelaxedToggle"):
        self.min_bucket_points = RELAXED_MIN_BUCKET_POINTS / (10 if decimated else 1)
        self.factor_sanity = 0.5 if decimated else 1.0
        self.friction_sanity = 0.8 if decimated else 1.0

      if self._params.get_bool("CustomTorqueParams"):
        self.offline_latAccelFactor = float(self._params.get("TorqueParamsOverrideLatAccelFactor", return_default=True))
        self.offline_friction = float(self._params.get("TorqueParamsOverrideFriction", return_default=True))

    if self.speed_binned:
      self._post_reset()
      self._restore_ext_cache()

  def _update_params(self):
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.use_live_torque_params = self._params.get_bool("LiveTorqueParamsToggle")
      self.custom_torque_params = self._params.get_bool("CustomTorqueParams")
      self.torque_override_enabled = self._params.get_bool("TorqueParamsOverrideEnabled")

  def update_use_params(self):
    self._update_params()

    if self.enforce_torque_control_toggle:
      if self.custom_torque_params and self.torque_override_enabled:
        self.use_params = False
      else:
        self.use_params = self.use_live_torque_params

    self.frame += 1

  @staticmethod
  def _centers_to_bounds(centers):
    bounds = []
    for i, center in enumerate(centers):
      lo = DEFAULT_SPEED_BIN_BOUNDS[0][0] if i == 0 else (centers[i - 1] + center) / 2
      hi = DEFAULT_SPEED_BIN_BOUNDS[-1][1] if i == len(centers) - 1 else (center + centers[i + 1]) / 2
      bounds.append((lo, hi))
    return bounds

  def _post_reset(self):
    if not self.speed_binned:
      return

    from openpilot.selfdrive.locationd.torqued import TorqueBuckets, STEER_BUCKET_BOUNDS, \
      POINTS_PER_BUCKET, MIN_FILTER_DECAY
    from opendbc.sunnypilot.car.interfaces import get_speed_dep_config_for_car

    cfg = get_speed_dep_config_for_car(self.CP)
    self.speed_dep_seed_version = int(cfg.get('seed_version', 0))

    if 'speed_bp' in cfg:
      self.speed_bin_centers = list(cfg['speed_bp'])
      self.speed_bin_bounds = self._centers_to_bounds(self.speed_bin_centers)
    else:
      self.speed_bin_bounds = list(DEFAULT_SPEED_BIN_BOUNDS)
      self.speed_bin_centers = list(DEFAULT_SPEED_BIN_CENTERS)

    n_bins = len(self.speed_bin_bounds)
    self.speed_bin_points = [self._make_speed_bin_bucket(TorqueBuckets, STEER_BUCKET_BOUNDS, POINTS_PER_BUCKET) for _ in range(n_bins)]
    self._speed_bin_last_len = [0] * n_bins
    self._speed_bin_last_valid = [False] * n_bins

    ref_factors = cfg.get('laf_bp', [self.offline_latAccelFactor] * n_bins)
    ref_frictions = cfg.get('friction_bp', [self.offline_friction] * n_bins)
    self.speed_bin_decays: list[float] = [float(MIN_FILTER_DECAY)] * n_bins
    self.speed_bin_filtered = [
      {'latAccelFactor': FirstOrderFilter(ref_factors[i], self.speed_bin_decays[i], DT_MDL),
       'frictionCoefficient': FirstOrderFilter(ref_frictions[i], self.speed_bin_decays[i], DT_MDL)}
      for i in range(n_bins)
    ]
    self.speed_bin_lat_accel_factor_bounds = self._get_speed_bin_lat_accel_factor_bounds(ref_factors)
    self.speed_bin_friction_bounds = [
      ((1.0 - self.friction_sanity) * friction, (1.0 + self.friction_sanity) * friction)
      for friction in ref_frictions
    ]

  def _get_speed_bin_lat_accel_factor_bounds(self, ref_factors):
    if (self.CP.carFingerprint == SUBURBAN_LIVE_TABLE_FINGERPRINT
        and len(ref_factors) == len(SUBURBAN_FACTOR_MAX_BP)):
      scale = float(self._params.get("SuburbanFactorFloorScale", return_default=True))
      if not np.isfinite(scale):
        scale = 1.0
      scale = float(np.clip(scale, SUBURBAN_FACTOR_FLOOR_SCALE_MIN, SUBURBAN_FACTOR_FLOOR_SCALE_MAX))
      return [
        (factor * scale, max((1.0 + self.factor_sanity) * factor, SUBURBAN_FACTOR_MAX_BP[i]))
        for i, factor in enumerate(ref_factors)
      ]

    return [
      ((1.0 - self.factor_sanity) * factor, (1.0 + self.factor_sanity) * factor)
      for factor in ref_factors
    ]

  def _make_speed_bin_bucket(self, TorqueBuckets, STEER_BUCKET_BOUNDS, POINTS_PER_BUCKET):
    scaled_min = np.maximum(np.asarray(self.min_bucket_points) // len(self.speed_bin_bounds), 1)
    return TorqueBuckets(x_bounds=STEER_BUCKET_BOUNDS,
                         min_points=scaled_min,
                         min_points_total=int(scaled_min.sum()),
                         points_per_bucket=POINTS_PER_BUCKET,
                         rowsize=3)

  def _on_torque_point(self, steer, lateral_acc, vego, yaw_lat_accel=None):
    if not self.speed_binned:
      return
    if self.CP.carFingerprint == SUBURBAN_LIVE_TABLE_FINGERPRINT:
      # The Suburban's near-straight corrections materially flatten the speed-bin fit.
      # Keep the lowest 6.5 m/s bin fixed, but allow learning from the 9.5 m/s bin upward
      # when the vehicle is producing measurable yaw response. Use pre-roll v*yaw_rate
      # for the gate so road/body roll cannot create false excitation. Other vehicles
      # retain the existing point-acceptance behavior.
      if vego < SUBURBAN_MIN_LEARNING_SPEED or vego > SUBURBAN_MAX_LEARNING_SPEED:
        return
      if yaw_lat_accel is None or abs(yaw_lat_accel) < SUBURBAN_MIN_YAW_LAT_ACCEL:
        return
    for i, (lo, hi) in enumerate(self.speed_bin_bounds):
      if lo <= vego < hi:
        self.speed_bin_points[i].add_point(steer, lateral_acc)
        break

  @staticmethod
  def _within_bounds(vals, bounds):
    vals = np.asarray(vals, dtype=float)
    lo, hi = np.asarray(bounds, dtype=float).T
    tol = 1e-5 * np.maximum(np.abs(hi), 1.0)
    return bool(np.all(np.isfinite(vals)) and np.all(vals >= lo - tol) and np.all(vals <= hi + tol))

  def _centers_match(self, cached_centers) -> bool:
    cached = list(cached_centers)
    return len(cached) == len(self.speed_bin_centers) and bool(np.allclose(cached, self.speed_bin_centers, atol=0.01))

  def _restore_ext_cache(self, cache_ltp=None, cache_CP=None, cache_sp=None):
    if not self.speed_binned:
      return
    try:
      from openpilot.selfdrive.locationd.torqued import TorqueEstimator, VERSION, MIN_FILTER_DECAY, MAX_FILTER_DECAY, get_torque_cache_keys
      torque_cache_key, car_params_cache_key = get_torque_cache_keys(self.CP)
      if cache_ltp is None:
        cache = self._params.get(torque_cache_key)
        if not cache and torque_cache_key != "LiveTorqueParameters":
          cache = self._params.get("LiveTorqueParameters")
        if not cache:
          return
        with log.Event.from_bytes(cache) as evt:
          cache_ltp = evt.lateralTorqueParameters
      if cache_CP is None:
        params_cache = self._params.get(car_params_cache_key)
        if not params_cache and car_params_cache_key != "CarParamsPrevRoute":
          params_cache = self._params.get("CarParamsPrevRoute")
        if not params_cache:
          cloudlog.info(f"speed-dep: no {car_params_cache_key}, restarting learning")
          return
        with car.CarParams.from_bytes(params_cache) as msg:
          cache_CP = msg

      if TorqueEstimator.get_restore_key(cache_CP, cache_ltp.version) != TorqueEstimator.get_restore_key(self.CP, VERSION):
        cloudlog.info("speed-dep: cache restore key mismatch, restarting learning")
        return
      if cache_sp is None:
        sp_cache_key = get_speed_dep_cache_key(self.CP)
        cache = self._params.get(sp_cache_key)
        if not cache and sp_cache_key != LIVE_TORQUE_PARAMETERS_SP_KEY:
          cache = self._params.get(LIVE_TORQUE_PARAMETERS_SP_KEY)
        if not cache:
          cloudlog.info(f"speed-dep: no {sp_cache_key} cache, restarting learning")
          return
        with log.Event.from_bytes(cache) as evt:
          cache_sp = getattr(evt, LIVE_TORQUE_PARAMETERS_SP_SERVICE)
      if TorqueEstimator.get_restore_key(cache_CP, cache_sp.version) != TorqueEstimator.get_restore_key(self.CP, VERSION):
        cloudlog.info("speed-dep: fork cache restore key mismatch, restarting learning")
        return
      if cache_sp.seedVersion != self.speed_dep_seed_version:
        cloudlog.info(f"speed-dep: seed version {cache_sp.seedVersion} -> {self.speed_dep_seed_version}, restarting learning")
        return

      n_bins = len(self.speed_bin_bounds)
      if not self._centers_match(cache_sp.speedBinCenters):
        cloudlog.info("speed-dep: config changed, restarting learning")
        return
