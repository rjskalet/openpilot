"""Do not silently downgrade donor-EPS steering fault events after the forward-port."""
import unittest
from opendbc.car import structs
from opendbc.car.mazda.values import MazdaFlags
from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.events import Events
from openpilot.sunnypilot.selfdrive.car.car_specific import CarSpecificEventsSP



class TestMazdaFaultPolicy(unittest.TestCase):
  def test_donor_eps_temporary_steering_fault_keeps_normal_disengage_policy(self):
    cp = structs.CarParams(brand="mazda", flags=int(MazdaFlags.STEER_TO_ZERO_EPS))
    events = Events()
    events.add(log.OnroadEvent.EventName.steerTempUnavailable)
    CarSpecificEventsSP(cp, structs.CarParamsSP()).update(structs.CarState(), events)
    assert events.has(log.OnroadEvent.EventName.steerTempUnavailable)
    assert not events.has(log.OnroadEvent.EventName.steerTempUnavailableSilent)
