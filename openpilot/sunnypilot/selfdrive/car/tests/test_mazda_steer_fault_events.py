import unittest

from openpilot.cereal import log
from opendbc.car import structs
from opendbc.car.mazda.values import MazdaFlags
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import ET

EventName = log.OnroadEvent.EventName


def car_events(brand: str, flags: int = 0) -> CarSpecificEventsSP:
  CP = structs.CarParams()
  CP.brand = brand
  CP.flags = int(flags)
  return CarSpecificEventsSP(CP, structs.CarParamsSP())


def events(*names: int) -> Events:
  ret = Events()
  for name in names:
    ret.add(name)
  return ret


class TestMazdaSteerFaultEvents(unittest.TestCase):
  def test_steer_to_zero_eps_downgrades_to_silent_warning(self):
    car_specific = car_events("mazda", MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    ev = events(EventName.steerTempUnavailable)

    car_specific.update(structs.CarState(), ev)

    self.assertFalse(ev.has(EventName.steerTempUnavailable))
    self.assertTrue(ev.has(EventName.steerTempUnavailableSilent))
    self.assertTrue(ev.contains(ET.WARNING))
    self.assertFalse(ev.contains(ET.SOFT_DISABLE))
    self.assertFalse(ev.contains(ET.NO_ENTRY))

  def test_older_eps_keeps_upstream_escalation(self):
    car_specific = car_events("mazda", MazdaFlags.GEN1)
    ev = events(EventName.steerTempUnavailable)

    car_specific.update(structs.CarState(), ev)

    self.assertTrue(ev.has(EventName.steerTempUnavailable))
    self.assertFalse(ev.has(EventName.steerTempUnavailableSilent))
    self.assertTrue(ev.contains(ET.SOFT_DISABLE))

  def test_other_brands_are_untouched(self):
    car_specific = car_events("honda", MazdaFlags.STEER_TO_ZERO_EPS)
    ev = events(EventName.steerTempUnavailable)

    car_specific.update(structs.CarState(), ev)

    self.assertTrue(ev.has(EventName.steerTempUnavailable))
    self.assertFalse(ev.has(EventName.steerTempUnavailableSilent))

  def test_other_events_survive_the_swap(self):
    car_specific = car_events("mazda", MazdaFlags.GEN1 | MazdaFlags.STEER_TO_ZERO_EPS)
    ev = events(EventName.steerTempUnavailable, EventName.steerUnavailable, EventName.steerSaturated)

    car_specific.update(structs.CarState(), ev)

    self.assertTrue(ev.has(EventName.steerUnavailable))
    self.assertTrue(ev.has(EventName.steerSaturated))
    self.assertTrue(ev.has(EventName.steerTempUnavailableSilent))
    self.assertFalse(ev.has(EventName.steerTempUnavailable))


if __name__ == "__main__":
  unittest.main()
