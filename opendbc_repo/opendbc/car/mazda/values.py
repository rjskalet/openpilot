from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_DRIVER_ALLOWANCE = 15    # allowed driver torque before start limiting
  STEER_DRIVER_FACTOR = 1        # from dbc
  STEER_STEP = 1                 # 100 Hz

  # ZoomPilot keeps the torque-controller normalization fixed at 1200 counts on the donor EPS.
  # The stock torque database is expressed on upstream Mazda's 800-count scale; interface.py
  # converts the tune once so changing vehicle speed never changes the meaning of normalized torque.
  EPS_STEER_MAX = 1200
  TUNE_STEER_MAX = 800
  TUNE_SCALE = EPS_STEER_MAX / TUNE_STEER_MAX

  # Current ZoomPilot donor-EPS behavior: when the factory LKAS setting returns, the rack
  # spends about 3 s re-arming before its block can be treated as normal steering state.
  LKAS_REARM_T = 3.0

  def __init__(self, CP):
    if CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      self.STEER_MAX = self.EPS_STEER_MAX
      self.STEER_DELTA_UP = 12
      self.STEER_DELTA_DOWN = 12
      self.STEER_DRIVER_MULTIPLIER = 15
      self.STEER_DRIVER_SAMPLES = 10
      self.STEER_DRIVER_MARGIN = 2

      # Keep normalization separate from physical EPS authority. This is ZoomPilot's measured
      # applied-torque ceiling; it clips the wire command without rescaling the lateral tune.
      self.EPS_CEILING_LOOKUP = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5],
                                 [1148, 1132, 1092, 1048, 1012, 920, 808, 676, 620])
      self.STEER_UNDELIVERED_MIN = 200
      self.STEER_UNDELIVERED_FRAMES = 20
      self.STEER_UNDELIVERED_ALERT_FRAMES = 80
      self.STEER_UNDELIVERED_ALERT_MIN_SPEED = 12. * CV.MPH_TO_MS
      self.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED = 1.0
    else:
      # Preserve StarPilot/upstream behavior for non-donor Mazdas.
      self.STEER_MAX = self.TUNE_STEER_MAX
      self.STEER_DELTA_UP = 10
      self.STEER_DELTA_DOWN = 25
      self.STEER_DRIVER_MULTIPLIER = 1
      self.STEER_DRIVER_SAMPLES = 1
      self.STEER_DRIVER_MARGIN = 0


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


@dataclass(frozen=True, kw_only=True)
class MazdaCX5_2022CarSpecs(CarSpecs):
  tireStiffnessFactor: float = 1.0


class MazdaFlags(IntFlag):
  # Gen 1 hardware: same CAN messages and same camera.
  GEN1 = 1
  # EPS firmware that can steer to zero speed (2022 CX-5 donor rack family).
  STEER_TO_ZERO_EPS = 2


class MazdaSafetyFlags(IntFlag):
  # Selects the matching steer-to-zero torque envelope in panda safety.
  STEER_TO_ZERO_EPS = 2


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    # ZoomPilot uses the learned 2022-rack ratio for this shared rack family.
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=18.1)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    # ZoomPilot TC-platform geometry: 2.93 m wheelbase, 17.6 steering ratio.
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6)
  )
  MAZDA_3 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 3 2017-18")],
    MazdaCarSpecs(mass=2875 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=14.0)
  )
  MAZDA_6 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda 6 2017-20")],
    MazdaCarSpecs(mass=3443 * CV.LB_TO_KG, wheelbase=2.83, steerRatio=15.5)
  )
  MAZDA_CX9_2021 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2021-23", video="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    # 18.1 is ZoomPilot's paramsd-learned ratio (15.5 factory nominal).
    MazdaCX5_2022CarSpecs(mass=3728 * CV.LB_TO_KG, wheelbase=2.698, steerRatio=18.1),
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


# Keep this synchronized with ZoomPilot's steer-to-zero EPS firmware set.
STEER_TO_ZERO_EPS_FW = {
  b'K0A1-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  b'KBST-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
}


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
