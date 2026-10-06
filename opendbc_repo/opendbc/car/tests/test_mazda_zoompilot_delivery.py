import pytest

from opendbc.car import structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import CAR, STEER_TO_ZERO_EPS_FW


def donor_params():
  fw = next(iter(STEER_TO_ZERO_EPS_FW))
  return interfaces[CAR.MAZDA_CX9].get_params(
    CAR.MAZDA_CX9,
    {i: {} for i in range(8)},
    [structs.CarParams.CarFw(ecu="eps", fwVersion=fw)],
    alpha_long=False,
    is_release=False,
    docs=False,
    starpilot_toggles=None,
  )


def update_delivery(cs, *, speed, blocked=True, effective=0, request=600, track=False):
  cs.lkas_blocked = blocked
  cs.lkas_effective = effective
  cs.lkas_track_state = track
  cs.update_steer_undelivered(speed, request)


def test_sustained_rolling_zero_delivery_escalates_after_command_suppression():
  cs = CarState(donor_params(), None)
  total = cs.params.STEER_UNDELIVERED_FRAMES + cs.params.STEER_UNDELIVERED_ALERT_FRAMES + 5
  for _ in range(total):
    update_delivery(cs, speed=10.0)

  assert cs.steer_undelivered
  assert cs.steer_undelivered_alert


def test_block_originating_from_standby_does_not_escalate_when_vehicle_accelerates():
  cs = CarState(donor_params(), None)
  update_delivery(cs, speed=0.3, track=True)
  assert cs.steer_first_engage_hold

  total = cs.params.STEER_UNDELIVERED_FRAMES + cs.params.STEER_UNDELIVERED_ALERT_FRAMES + 10
  for _ in range(total):
    update_delivery(cs, speed=10.0, track=False)

  assert cs.steer_undelivered
  assert not cs.steer_undelivered_alert


def test_stock_lkas_off_clears_latch_and_alert():
  cs = CarState(donor_params(), None)
  total = cs.params.STEER_UNDELIVERED_FRAMES + cs.params.STEER_UNDELIVERED_ALERT_FRAMES + 5
  for _ in range(total):
    update_delivery(cs, speed=10.0)
  assert cs.steer_undelivered_alert

  cs.lkas_setting_invalid = True
  update_delivery(cs, speed=10.0)
  assert not cs.steer_undelivered
  assert not cs.steer_undelivered_alert
  assert cs.steer_undelivered_frames == 0


@pytest.mark.parametrize("effective", [-1, 1])
def test_effective_delivery_resets_pre_latch_counter(effective):
  cs = CarState(donor_params(), None)
  for _ in range(cs.params.STEER_UNDELIVERED_FRAMES - 1):
    update_delivery(cs, speed=10.0)
  update_delivery(cs, speed=10.0, effective=effective)

  assert not cs.steer_undelivered
  assert not cs.steer_undelivered_alert
  assert cs.steer_undelivered_frames == 0
