"""Pure speed-profile helpers used by Smart Cruise curve planning."""
import math
import numpy as np

KAPPA_MIN = 1e-5
D_FLOOR = 0.5


def allowed_speed(curvature, a_lat_max: float) -> np.ndarray:
  kappa = np.maximum(np.asarray(curvature, dtype=float), 0.)
  return np.where(kappa > KAPPA_MIN, np.sqrt(a_lat_max / np.maximum(kappa, KAPPA_MIN)), np.inf)


def backward_pass(v_allowed, dist, a_budget: float) -> np.ndarray:
  v_allowed = np.asarray(v_allowed, dtype=float)
  dist = np.asarray(dist, dtype=float)
  v_max = v_allowed.copy()
  for i in range(len(v_max) - 2, -1, -1):
    ds = max(dist[i + 1] - dist[i], 0.)
    if math.isinf(v_max[i + 1]):
      continue
    v_max[i] = min(v_allowed[i], math.sqrt(v_max[i + 1] ** 2 + 2. * a_budget * ds))
  return v_max


def required_decel(v_ego: float, v_allowed, dist, d_lead: float = 0.) -> float:
  v = np.asarray(v_allowed, dtype=float)
  d = np.asarray(dist, dtype=float) - d_lead
  binding = v < v_ego
  if not np.any(binding):
    return 0.
  a = (v_ego ** 2 - v[binding] ** 2) / (2. * np.maximum(d[binding], D_FLOOR))
  return float(np.max(a))


def lead_distance(v_ego: float, t_lead: float, a_budget: float = 0., jerk: float = 0.) -> float:
  d = v_ego * t_lead
  if jerk > 0.:
    d += v_ego * a_budget / (2. * jerk)
  return d


def min_profile_speed(v_max, dist, horizon_d: float) -> float:
  v_max = np.asarray(v_max, dtype=float)
  d = np.asarray(dist, dtype=float)
  mask = d <= horizon_d
  if not np.any(mask):
    return float(v_max[0])
  return float(np.min(v_max[mask]))
