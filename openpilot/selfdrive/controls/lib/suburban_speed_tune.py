import numpy as np

# Route 25 speed-band calibration for CHEVROLET_SUBURBAN_CAMERA_11TH_GEN.
#
# The 15-18, 18-22, and 22-26 m/s bands contained 4,277, 3,875, and
# 978 valid torque-response points respectively. Route 26 independently
# confirmed the 35-45 mph authority loss. Below 30 mph and above 58 mph,
# retain the proven 0.68 baseline until more validation data is collected.
SUBURBAN_V32_SPEED_BP_MS = np.array([
  0.0,
  13.4112,  # 30 mph
  16.5,     # Route 25 midpoint: 15-18 m/s (36.9 mph)
  20.0,     # Route 25 midpoint: 18-22 m/s (44.7 mph)
  24.0,     # Route 25 midpoint: 22-26 m/s (53.7 mph)
  26.0,     # 58.2 mph; blend back to baseline
  40.2336,  # 90 mph
])

SUBURBAN_V32_LAT_ACCEL_FACTOR = np.array([
  0.68,
  0.68,
  0.4437182678,
  0.4759647790,
  0.6728052303,
  0.68,
  0.68,
])


def suburban_v32_lat_accel_factor(v_ego: float) -> float:
  return float(np.interp(v_ego, SUBURBAN_V32_SPEED_BP_MS, SUBURBAN_V32_LAT_ACCEL_FACTOR))
