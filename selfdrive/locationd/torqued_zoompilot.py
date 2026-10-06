"""ZoomPilot speed-binned torque learning, isolated to verified donor-EPS Mazdas.

This ports the learning/cache half of ZoomPilot's Mazda lateral architecture into StarPilot
without importing sunnypilot's generic extension stack. The controller consumes the same
concepts: fixed 1200-count tune units, per-speed latAccelFactor/friction bins, seed fallback,
and provenance-checked cache restore.
"""
from pathlib import Path
import tomllib

import numpy as np

import cereal.messaging as messaging
from cereal import car, custom, log
from opendbc.car.mazda.values import MazdaFlags
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.locationd.helpers import PointBuckets

MAZDA_TORQUE_SERVICE = "customReserved16"
MAZDA_TORQUE_CACHE_KEY = "LiveTorqueParametersMazda"
MazdaTorqueParameters = custom.CustomReserved16

DEFAULT_SPEED_MIN = 5.0
DEFAULT_SPEED_MAX = 40.0
POINTS_PER_BUCKET = 1500
MIN_FILTER_DECAY = 50.0
MAX_FILTER_DECAY = 250.0
FRICTION_FACTOR = 1.5
STEER_BUCKET_BOUNDS = [(-0.5, -0.3), (-0.3, -0.2), (-0.2, -0.1), (-0.1, 0),
                       (0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.5)]


class SpeedTorqueBuckets(PointBuckets):
  def add_point(self, x, y):
    for bound_min, bound_max in self.x_bounds:
      if bound_min <= x < bound_max:
        self.buckets[(bound_min, bound_max)].append([x, 1.0, y])
        break


def _fit_torque_points(points):
  """ZoomPilot/upstream TLS fit for [steer, 1, lateral_accel] rows."""
  _, _, v = np.linalg.svd(points, full_matrices=False)
  slope, offset = -v.T[0:2, 2] / v.T[2, 2]
  sin = np.sqrt(slope ** 2 / (slope ** 2 + 1))
  cos = np.sqrt(1 / (slope ** 2 + 1))
  rotation = np.array([[cos, -sin], [sin, cos]])
  _, spread = np.matmul(points[:, [0, 2]], rotation).T
  return slope, offset, np.std(spread) * FRICTION_FACTOR


def get_speed_dep_config(CP):
  """Resolve the Mazda speed-dependent table, including donor-rack substitutions."""
  path = Path(__file__).resolve().parents[2] / "opendbc_repo/opendbc/car/torque_data/speed_dependent.toml"
  with path.open("rb") as f:
    tables = tomllib.load(f)

  key = str(CP.carFingerprint)
  cfg = dict(tables.get(key, {}))
  if not cfg:
    return {}

  if cfg.get("requires_steer_to_zero", False) and not (CP.flags & MazdaFlags.STEER_TO_ZERO_EPS):
    return {}

  seen = {key}
  while "substitute" in cfg:
    substitute = str(cfg["substitute"])
    if substitute in seen or substitute not in tables:
      raise ValueError(f"invalid speed-dependent torque substitution: {substitute}")
    seen.add(substitute)
    source = dict(tables[substitute])
    # Keep requirements/minimum speed from the chassis entry when supplied.
    for field in ("requires_steer_to_zero", "min_speed"):
      if field in cfg:
        source[field] = cfg[field]
    cfg = source
  return cfg


class MazdaTorqueBins:
  def __init__(self, CP, *, min_bucket_points, factor_sanity, friction_sanity, fit_points, version):
    self.CP = CP
    self.version = int(version)
    self.fit_points = int(fit_points)
    self.params = Params()
    self.enabled = (
      CP.brand == "mazda"
      and CP.lateralTuning.which() == "torque"
      and bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS)
    )
    self.pm = None

    self.speed_bin_centers = []
    self.speed_bin_bounds = []
    self.speed_bin_points = []
    self.speed_bin_filtered = []
    self.speed_bin_decays = []
    self.speed_bin_valid = []
    self.speed_bin_dirty = []
    self.factor_bounds = []
    self.friction_bounds = []
    self.seed_version = 0
    self.seed_factors = []
    self.seed_frictions = []

    if not self.enabled:
      return

    cfg = get_speed_dep_config(CP)
    centers = list(cfg.get("speed_bp", []))
    factors = list(cfg.get("laf_bp", []))
    frictions = list(cfg.get("friction_bp", []))
    if not centers or len(centers) != len(factors) or len(centers) != len(frictions):
      cloudlog.error("Mazda speed-bin torque config missing or malformed; disabling")
      self.enabled = False
      return

    self.seed_version = int(cfg.get("seed_version", 0))
    self.speed_bin_centers = [float(v) for v in centers]
    self.seed_factors = [float(v) for v in factors]
    self.seed_frictions = [float(v) for v in frictions]
    self.speed_bin_bounds = self._centers_to_bounds(self.speed_bin_centers, cfg.get("min_speed"))

    n_bins = len(self.speed_bin_centers)
    self.scaled_min = np.maximum(np.asarray(min_bucket_points, dtype=float) // n_bins, 1).astype(int)
    self.speed_bin_points = [
      SpeedTorqueBuckets(
        x_bounds=STEER_BUCKET_BOUNDS,
        min_points=self.scaled_min,
        min_points_total=int(self.scaled_min.sum()),
        points_per_bucket=POINTS_PER_BUCKET,
        rowsize=3,
      )
      for _ in range(n_bins)
    ]
    self.speed_bin_decays = [MIN_FILTER_DECAY] * n_bins
    self.speed_bin_filtered = [
      {
        "latAccelFactor": FirstOrderFilter(self.seed_factors[i], MIN_FILTER_DECAY, DT_MDL),
        "frictionCoefficient": FirstOrderFilter(self.seed_frictions[i], MIN_FILTER_DECAY, DT_MDL),
      }
      for i in range(n_bins)
    ]
    self.speed_bin_valid = [False] * n_bins
    self.speed_bin_dirty = [True] * n_bins
    self.factor_bounds = [
      ((1.0 - factor_sanity) * v, (1.0 + factor_sanity) * v)
      for v in self.seed_factors
    ]
    self.friction_bounds = [
      ((1.0 - friction_sanity) * v, (1.0 + friction_sanity) * v)
      for v in self.seed_frictions
    ]
    self._restore_cache()

  @staticmethod
  def _centers_to_bounds(centers, min_speed=None):
    bounds = []
    for i, center in enumerate(centers):
      lo = (DEFAULT_SPEED_MIN if min_speed is None else float(min_speed)) if i == 0 else (centers[i - 1] + center) / 2
      hi = DEFAULT_SPEED_MAX if i == len(centers) - 1 else (center + centers[i + 1]) / 2
      bounds.append((lo, hi))
    return bounds

  @staticmethod
  def _restore_key(CP, version):
    friction = factor = None
    if CP.lateralTuning.which() == "torque":
      friction = CP.lateralTuning.torque.friction
      factor = CP.lateralTuning.torque.latAccelFactor
    return (CP.carFingerprint, CP.lateralTuning.which(), friction, factor, version)

  @staticmethod
  def _within_bounds(values, bounds):
    values = np.asarray(values, dtype=float)
    limits = np.asarray(bounds, dtype=float)
    if values.shape != (len(bounds),) or not np.all(np.isfinite(values)):
      return False
    lo, hi = limits.T
    tol = 1e-5 * np.maximum(np.abs(hi), 1.0)
    return bool(np.all(values >= lo - tol) and np.all(values <= hi + tol))

  def _centers_match(self, centers):
    centers = list(centers)
    return len(centers) == len(self.speed_bin_centers) and bool(
      np.allclose(centers, self.speed_bin_centers, atol=0.01)
    )

  def _restore_cache(self):
    upstream_cache = self.params.get("LiveTorqueParameters")
    car_params_cache = self.params.get("CarParamsPrevRoute")
    bins_cache = self.params.get(MAZDA_TORQUE_CACHE_KEY)
    if not upstream_cache or not car_params_cache or not bins_cache:
      return

    try:
      with log.Event.from_bytes(upstream_cache) as evt:
        upstream = evt.liveTorqueParameters
      with car.CarParams.from_bytes(car_params_cache) as cache_CP:
        if self._restore_key(cache_CP, upstream.version) != self._restore_key(self.CP, self.version):
          return
      with log.Event.from_bytes(bins_cache) as evt:
        bins = getattr(evt, MAZDA_TORQUE_SERVICE)

      if bins.version != self.version or bins.seedVersion != self.seed_version or not self._centers_match(bins.speedBinCenters):
        return

      factors = list(bins.speedBinLatAccelFactors)
      frictions = list(bins.speedBinFrictions)
      if not self._within_bounds(factors, self.factor_bounds) or not self._within_bounds(frictions, self.friction_bounds):
        return

      decay = float(upstream.decay)
      if not np.isfinite(decay):
        return
      decay = float(np.clip(decay, MIN_FILTER_DECAY, MAX_FILTER_DECAY))

      if upstream.liveValid:
        for i in range(len(self.speed_bin_centers)):
          self.speed_bin_filtered[i]["latAccelFactor"].x = factors[i]
          self.speed_bin_filtered[i]["frictionCoefficient"].x = frictions[i]

      cached_points = [[list(point) for point in bucket] for bucket in bins.speedBinPoints]
      if len(cached_points) == len(self.speed_bin_centers) and all(
        not bucket or np.all(np.isfinite(np.asarray(bucket, dtype=float))) for bucket in cached_points
      ):
        for bucket, points in zip(self.speed_bin_points, cached_points, strict=True):
          bucket.load_points(points)

      self.speed_bin_decays = [decay] * len(self.speed_bin_centers)
      for filters in self.speed_bin_filtered:
        filters["latAccelFactor"].update_alpha(decay)
        filters["frictionCoefficient"].update_alpha(decay)
      cloudlog.info("restored Mazda speed-bin torque params from cache")
    except Exception:
      cloudlog.exception("failed to restore Mazda speed-bin torque cache")

  def on_torque_point(self, steer, lateral_acc, v_ego):
    if not self.enabled:
      return
    for i, (lo, hi) in enumerate(self.speed_bin_bounds):
      if lo <= v_ego < hi:
        self.speed_bin_points[i].add_point(steer, lateral_acc)
        self.speed_bin_dirty[i] = True
        break

  def _estimate(self):
    if not self.enabled:
      return

    for i, bucket in enumerate(self.speed_bin_points):
      if not self.speed_bin_dirty[i]:
        continue
      if not bucket.is_calculable():
        self.speed_bin_valid[i] = False
        continue

      try:
        points = bucket.get_points(self.fit_points)
        factor, _, friction = _fit_torque_points(points)
      except np.linalg.LinAlgError:
        factor = friction = np.nan

      if not np.isfinite(factor) or not np.isfinite(friction):
        cloudlog.warning("Mazda speed-bin %d fit invalid; resetting bin", i)
        scaled_min = np.maximum(
          np.asarray(bucket.min_points, dtype=float), 1
        ).astype(int)
        self.speed_bin_points[i] = SpeedTorqueBuckets(
          x_bounds=STEER_BUCKET_BOUNDS,
          min_points=self.scaled_min,
          min_points_total=int(self.scaled_min.sum()),
          points_per_bucket=POINTS_PER_BUCKET,
          rowsize=3,
        )
        self.speed_bin_valid[i] = False
        self.speed_bin_dirty[i] = True
        self.speed_bin_decays[i] = MIN_FILTER_DECAY
        continue

      factor = float(np.clip(factor, *self.factor_bounds[i]))
      friction = float(np.clip(friction, *self.friction_bounds[i]))
      self.speed_bin_decays[i] = min(self.speed_bin_decays[i] + DT_MDL, MAX_FILTER_DECAY)
      self.speed_bin_filtered[i]["latAccelFactor"].update(factor)
      self.speed_bin_filtered[i]["latAccelFactor"].update_alpha(self.speed_bin_decays[i])
      self.speed_bin_filtered[i]["frictionCoefficient"].update(friction)
      self.speed_bin_filtered[i]["frictionCoefficient"].update_alpha(self.speed_bin_decays[i])
      self.speed_bin_valid[i] = bucket.is_valid()
      self.speed_bin_dirty[i] = False

  def _message(self, valid, with_points=False):
    msg = messaging.new_message(MAZDA_TORQUE_SERVICE)
    msg.valid = valid
    bins = getattr(msg, MAZDA_TORQUE_SERVICE)
    bins.version = self.version
    bins.seedVersion = self.seed_version
    if self.enabled:
      bins.speedBinCenters = self.speed_bin_centers
      # On the live wire, invalid bins intentionally expose their static seeds. This is
      # ZoomPilot's controller-side fallback moved to the producer, keeping StarPilot's
      # controlsd independent of the TOML loader. Cache messages retain the raw filters.
      if with_points:
        bins.speedBinLatAccelFactors = [
          float(filters["latAccelFactor"].x) for filters in self.speed_bin_filtered
        ]
        bins.speedBinFrictions = [
          float(filters["frictionCoefficient"].x) for filters in self.speed_bin_filtered
        ]
      else:
        bins.speedBinLatAccelFactors = [
          float(self.speed_bin_filtered[i]["latAccelFactor"].x if self.speed_bin_valid[i] else self.seed_factors[i])
          for i in range(len(self.speed_bin_centers))
        ]
        bins.speedBinFrictions = [
          float(self.speed_bin_filtered[i]["frictionCoefficient"].x if self.speed_bin_valid[i] else self.seed_frictions[i])
          for i in range(len(self.speed_bin_centers))
        ]
      bins.speedBinValid = self.speed_bin_valid
      if with_points:
        bins.speedBinPoints = [
          bucket.get_points()[:, [0, 2]].tolist() for bucket in self.speed_bin_points
        ]
    return msg

  def publish(self, valid=True):
    if not self.enabled:
      return
    self._estimate()
    if self.pm is None:
      self.pm = messaging.PubMaster([MAZDA_TORQUE_SERVICE])
    self.pm.send(MAZDA_TORQUE_SERVICE, self._message(valid, with_points=False))

  def cache(self, valid=True):
    if not self.enabled:
      return
    self._estimate()
    self.params.put_nonblocking(MAZDA_TORQUE_CACHE_KEY, self._message(valid, with_points=True).to_bytes())
