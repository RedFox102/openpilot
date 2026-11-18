from collections import deque

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.values import CarControllerParams, MazdaFlags

VisualAlert = structs.CarControl.HUDControl.VisualAlert


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    # The non-delivery latch belongs to the steer-to-zero firmware alone.
    self.steer_to_zero = bool(CP.flags & MazdaFlags.STEER_TO_ZERO_EPS)
    self.driver_torque_samples: deque[float] = deque(maxlen=self.params.STEER_DRIVER_SAMPLES)

  def update(self, CC, CS, now_nanos, starpilot_toggles):
    can_sends = []

    self.driver_torque_samples.append(CS.out.steeringTorque)
    if CS.lkas_rejected:
      # The panda reports every 0x243 it refused back on the can stream (src 192). A rejection
      # zeroes its rate-limit reference, so a controller that keeps ramping is refused on every
      # later frame and the EPS loses its stream. Only a command within one step of zero is
      # accepted next, so the ramp restarts there.
      self.apply_torque_last = 0

    apply_torque = 0

    if CC.latActive:
      # calculate steer and also set limits due to driver torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))

      # Clamp to applied EPS authority so controlsd can detect saturation. Keep this separate
      # from STEER_MAX because the torque parameters are expressed on STEER_MAX.
      eps_ceiling = round(float(np.interp(CS.out.vEgoRaw, self.params.EPS_CEILING_LOOKUP[0],
                                          self.params.EPS_CEILING_LOOKUP[1])))
      new_torque = int(np.clip(new_torque, -eps_ceiling, eps_ceiling))

      # Use the worst sample plus margin to stay inside panda's fresher driver-torque envelope.
      if new_torque >= 0:
        driver_torque = min(self.driver_torque_samples) - self.params.STEER_DRIVER_MARGIN
      else:
        driver_torque = max(self.driver_torque_samples) + self.params.STEER_DRIVER_MARGIN

      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      driver_torque, self.params)

    # Stop requesting torque while carstate says the EPS will not take it: after the
    # non-delivery latch, or through its first engagement of the cycle. Recovery ramps from zero.
    if self.steer_to_zero and (CS.steer_undelivered or CS.steer_first_engage_hold):
      apply_torque = 0

    # No CRZ_BTNS is ever sent, neither CANCEL nor RES. Every injected frame takes the counter of
    # the driver's next real frame from the wheel, and injected CANCELs were causing cruise faults.
    # Need to determine the root cause before we can start injecting the cruise cancel command.

    self.apply_torque_last = apply_torque

    # send HUD alerts
    if self.frame % 50 == 0:
      ldw = CC.hudControl.visualAlert == VisualAlert.ldw
      lkas_blocked = CS.lkas_blocked
      steer_required = CC.hudControl.visualAlert == VisualAlert.steerRequired
      # TODO: find a way to silence audible warnings so we can add more hud alerts
      steer_required = steer_required and CS.lkas_allowed_speed
      can_sends.append(mazdacan.create_alert_command(self.packer, CS.cam_laneinfo, ldw, steer_required, lkas_blocked))

    # send steering command
    can_sends.append(mazdacan.create_steering_control(self.packer, self.CP,
                                                      self.frame, apply_torque, CS.cam_lkas))

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
