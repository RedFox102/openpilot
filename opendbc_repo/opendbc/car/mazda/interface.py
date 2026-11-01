#!/usr/bin/env python3
from opendbc.car import get_safety_config, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.mazda.carcontroller import CarController
from opendbc.car.mazda.carstate import CarState
from opendbc.car.mazda.values import CAR, LKAS_LIMITS, STEER_TO_ZERO_EPS_FW, STEER_TO_ZERO_PLATFORMS, TORQUE_TUNES, \
  CarControllerParams, MazdaFlags, MazdaSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def configure_torque_tune(candidate, tune, steering_angle_deadzone_deg=0.0):
    # params.toml's tunes are on upstream's STEER_MAX. An override rather than a conversion in
    # _get_params, so the second call in get_params (NNFF, ForceTorqueController) converts as well.
    CarInterfaceBase.configure_torque_tune(candidate, tune, steering_angle_deadzone_deg)
    lat_accel_factor, friction = TORQUE_TUNES.get(candidate, (tune.torque.latAccelFactor, tune.torque.friction))
    tune.torque.latAccelFactor = lat_accel_factor * CarControllerParams.TUNE_SCALE
    tune.torque.friction = friction / CarControllerParams.TUNE_SCALE

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "mazda"
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.mazda)]
    ret.radarUnavailable = True

    ret.dashcamOnly = candidate not in (CAR.MAZDA_CX5_2022, CAR.MAZDA_CX9_2021)

    # Every gen1 Mazda EPS is the same hardware; only the firmware differs. Steer-to-zero follows
    # the EPS firmware, so a donor-EPS swap carries it and older firmware in a 2022 body loses it.
    # Only an unread EPS (docs, a failed query) falls back to the platform.
    eps_fw = {fw.fwVersion for fw in car_fw if fw.ecu == 'eps'}
    steer_to_zero = not eps_fw.isdisjoint(STEER_TO_ZERO_EPS_FW) or (not eps_fw and candidate in STEER_TO_ZERO_PLATFORMS)
    # The panda enforces the measured envelope the controller runs on every gen1 EPS.
    ret.safetyConfigs[0].safetyParam |= MazdaSafetyFlags.EPS_HW.value
    if steer_to_zero:
      ret.flags |= MazdaFlags.STEER_TO_ZERO_EPS.value
    else:
      ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS
      ret.flags |= MazdaFlags.LEGACY_FW_EPS.value

    # Command-to-torque lag measured on the EPS hardware; lagd learns the remaining delay.
    ret.steerActuatorDelay = 0.14
    ret.steerLimitTimer = 0.8

    CarInterface.configure_torque_tune(candidate, ret.lateralTuning)

    ret.centerToFront = ret.wheelbase * 0.41

    ret.enableBsm = True

    return ret
