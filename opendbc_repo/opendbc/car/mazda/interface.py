#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, MazdaFlags, MazdaSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = True

    # The donor 2022 CX-5 EPS carries its steering capability with it. Detect from firmware so
    # an older CX-9 body with a verified donor rack gets the same lateral path as ZoomPilot.
    eps_fw = {fw.fwVersion for fw in car_fw if fw.ecu == 'eps'}
    steer_to_zero = not eps_fw.isdisjoint(STEER_TO_ZERO_EPS_FW)
    if steer_to_zero:
      ret.flags |= MazdaFlags.STEER_TO_ZERO_EPS.value
      ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.STEER_TO_ZERO_EPS.value

    # Preserve StarPilot's supported Mazda bodies, and additionally lift dashcam-only when a
    # verified steer-to-zero donor EPS is detected. Do not broadly enable legacy EPS firmware.
    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021) and not steer_to_zero

    ret.steerActuatorDelay = 0.14 if steer_to_zero else 0.1
    ret.steerLimitTimer = 0.8

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    if not steer_to_zero and candidate not in (CAR.MAZDA_CX5_2022,):
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS
    else:
      ret.minSteerSpeed = 0.0

    ret.centerToFront = ret.wheelbase * 0.41
    ret.enableBsm = True

    return ret
