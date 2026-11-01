from cereal import custom
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.mazda.values import DBC, LKAS_LIMITS, CarControllerParams, MazdaFlags

ButtonType = structs.CarState.ButtonEvent.Type


class CarState(CarStateBase):
  def __init__(self, CP, FPCP):
    super().__init__(CP, FPCP)

    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["GEAR"]["GEAR"]

    self.crz_btns_counter = 0
    self.acc_active_last = False
    self.lkas_allowed_speed = False
    self.lkas_blocked = False
    self.lkas_effective = 0
    self.lkas_track_state = False
    # LKAS non-delivery state is used only with the steer-to-zero EPS.
    self.params = CarControllerParams(CP)
    self.steer_undelivered_frames = 0
    self.steer_undelivered = False
    self.steer_undelivered_alert = False
    self.lkas_block_origin_speed: float | None = None
    # The EPS's first engagement of the cycle, under standby at a crawl, can fault while delivering
    # nothing: hold the request until it delivers once, the standby lifts or the car is rolling.
    self.lkas_delivered = False
    self.steer_first_engage_hold = False
    # Our 0x243 frames the panda refused since the last cycle, reported back on the can stream
    # with src 192 (bus 0 + 0xC0). Zero-torque refusals while disengaged are not counted.
    self.lkas_rejected = 0
    # Last frame's invalidLkasSetting: with the car's lane keep off the EPS applies nothing by
    # design, so the non-delivery latch has nothing to measure and must not hold or alert.
    self.lkas_setting_invalid = False

    self.distance_button = 0

  def update_steer_undelivered(self, v_ego_raw: float, lkas_request: float) -> None:
    lkas_blocked, lkas_track_state = self.lkas_blocked, self.lkas_track_state
    self.lkas_delivered |= self.lkas_effective != 0
    self.steer_first_engage_hold = (not self.lkas_delivered and lkas_blocked and lkas_track_state and
                                    v_ego_raw < self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED)

    # Latch sustained zero LKAS_EFFECTIVE for a real request before the camera faults. It clears
    # with LKAS_BLOCK (a zeroed command shows no delivery) and while the car's lane keep is off.
    if not lkas_blocked or self.lkas_setting_invalid:
      self.steer_undelivered_frames = 0
      self.steer_undelivered = False
      self.steer_undelivered_alert = False
      self.lkas_block_origin_speed = None
    elif self.lkas_block_origin_speed is None:
      self.lkas_block_origin_speed = v_ego_raw

    if lkas_blocked and not self.steer_undelivered:
      if self.lkas_effective == 0 and abs(lkas_request) > self.params.STEER_UNDELIVERED_MIN:
        self.steer_undelivered_frames += 1
        self.steer_undelivered = self.steer_undelivered_frames >= self.params.STEER_UNDELIVERED_FRAMES
      else:
        self.steer_undelivered_frames = 0

    if self.steer_undelivered:
      # Alert only for a sustained road-speed block that began rolling: LKAS_TRACK_STATE and the
      # origin speed both mark the EPS's standby from a stop.
      self.steer_undelivered_frames += 1
      if (not self.steer_undelivered_alert and not lkas_track_state and
          self.steer_undelivered_frames >= self.params.STEER_UNDELIVERED_FRAMES + self.params.STEER_UNDELIVERED_ALERT_FRAMES and
          v_ego_raw >= self.params.STEER_UNDELIVERED_ALERT_MIN_SPEED and
          self.lkas_block_origin_speed >= self.params.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED):
        self.steer_undelivered_alert = True

  def update(self, can_parsers, starpilot_toggles) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    prev_distance_button = self.distance_button
    self.distance_button = cp.vl["CRZ_BTNS"]["DISTANCE_LESS"]

    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEEDS"]["FL"],
      cp.vl["WHEEL_SPEEDS"]["FR"],
      cp.vl["WHEEL_SPEEDS"]["RL"],
      cp.vl["WHEEL_SPEEDS"]["RR"],
    )

    # Match panda speed reading
    speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1

    can_gear = int(cp.vl["GEAR"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.genericToggle = bool(cp.vl["BLINK_INFO"]["HIGH_BEAMS"])
    ret.leftBlindspot = cp.vl["BSM"]["LEFT_BS_STATUS"] != 0
    ret.rightBlindspot = cp.vl["BSM"]["RIGHT_BS_STATUS"] != 0
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(40, cp.vl["BLINK_INFO"]["LEFT_BLINK"] == 1,
                                                                      cp.vl["BLINK_INFO"]["RIGHT_BLINK"] == 1)

    ret.steeringAngleDeg = cp.vl["STEER"]["STEER_ANGLE"]
    ret.steeringTorque = cp.vl["STEER_TORQUE"]["STEER_TORQUE_SENSOR"]
    ret.steeringPressed = abs(ret.steeringTorque) > LKAS_LIMITS.STEER_THRESHOLD

    ret.steeringTorqueEps = cp.vl["STEER_TORQUE"]["STEER_TORQUE_MOTOR"]
    ret.steeringRateDeg = cp.vl["STEER_RATE"]["STEER_ANGLE_RATE"]

    # TODO: this should be from 0 - 1.
    ret.brakePressed = cp.vl["PEDALS"]["BRAKE_ON"] == 1
    ret.brake = cp.vl["BRAKE"]["BRAKE_PRESSURE"]

    ret.seatbeltUnlatched = cp.vl["SEATBELT"]["DRIVER_SEATBELT"] == 0
    ret.doorOpen = any([cp.vl["DOORS"]["FL"], cp.vl["DOORS"]["FR"],
                        cp.vl["DOORS"]["BL"], cp.vl["DOORS"]["BR"]])

    # TODO: this should be from 0 - 1.
    ret.gasPressed = cp.vl["ENGINE_DATA"]["PEDAL_GAS"] > 0

    # Either due to low speed or hands off
    self.lkas_blocked = cp.vl["STEER_RATE"]["LKAS_BLOCK"] == 1

    # LKAS_EFFECTIVE distinguishes partial delivery from a complete block.
    self.lkas_effective = cp.vl["STEER_RATE"]["LKAS_EFFECTIVE"]
    self.lkas_track_state = cp.vl["STEER_RATE"]["LKAS_TRACK_STATE"] == 1
    # Count the torque requests the panda turned away; a refused zero carries nothing.
    self.lkas_rejected = sum(1 for v in can_parsers[Bus.loopback].vl_all["CAM_LKAS"]["LKAS_REQUEST"] if v != 0)
    if self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      self.update_steer_undelivered(ret.vEgoRaw, cp.vl["STEER_RATE"]["LKAS_REQUEST"])

    if self.CP.minSteerSpeed > 0:
      # LKAS is enabled at 52kph going up and disabled at 45kph going down
      # wait for LKAS_BLOCK signal to clear when going up since it lags behind the speed sometimes
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not self.lkas_blocked:
        self.lkas_allowed_speed = True
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    # TODO: the signal used for available seems to be the adaptive cruise signal, instead of the main on
    #       it should be used for carState.cruiseState.nonAdaptive instead
    ret.cruiseState.available = cp.vl["CRZ_CTRL"]["CRZ_AVAILABLE"] == 1
    ret.cruiseState.enabled = cp.vl["CRZ_CTRL"]["CRZ_ACTIVE"] == 1
    ret.cruiseState.standstill = cp.vl["PEDALS"]["STANDSTILL"] == 1
    ret.cruiseState.speed = cp.vl["CRZ_EVENTS"]["CRZ_SPEED"] * CV.KPH_TO_MS

    # stock lkas should be on
    # TODO: is this needed?
    ret.invalidLkasSetting = cp_cam.vl["CAM_LANEINFO"]["LANE_LINES"] == 0
    self.lkas_setting_invalid = ret.invalidLkasSetting

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True
      else:
        self.low_speed_alert = False
    ret.lowSpeedAlert = self.low_speed_alert

    # Check if LKAS is disabled due to lack of driver torque when all other states indicate
    # it should be enabled (steer lockout). Don't warn until we actually get lkas active
    # and lose it again, i.e, after initial lkas activation
    if not self.CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      ret.steerFaultTemporary = self.lkas_allowed_speed and self.lkas_blocked
    else:
      # Report only sustained road-speed zero delivery after the command has been suppressed.
      ret.steerFaultTemporary = self.steer_undelivered_alert

    self.acc_active_last = ret.cruiseState.enabled

    self.crz_btns_counter = cp.vl["CRZ_BTNS"]["CTR"]

    # camera signals
    self.cam_lkas = cp_cam.vl["CAM_LKAS"]
    self.cam_laneinfo = cp_cam.vl["CAM_LANEINFO"]
    ret.steerFaultPermanent = cp_cam.vl["CAM_LKAS"]["ERR_BIT_1"] == 1

    # TODO: add button types for inc and dec
    ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    fp_ret = custom.StarPilotCarState.new_message()

    fp_ret.dashboardStopSign = 1 if cp_cam.vl["CAM_TRAFFIC_SIGNS"]["STOP_SIGN"] == 9 else 0

    return ret, fp_ret

  @staticmethod
  def get_can_parsers(CP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [("CAM_TRAFFIC_SIGNS", 0)], 2),
      # Our own 0x243 frames the panda refused, reported back with src = bus + 0xC0. Sporadic by
      # nature, so never part of canValid or canTimeout.
      Bus.loopback: CANParser(DBC[CP.carFingerprint][Bus.pt], [("CAM_LKAS", float("nan"))], 192),
    }
