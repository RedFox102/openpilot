from dataclasses import dataclass, field
from enum import IntFlag

from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs, CarParts
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries

Ecu = car.CarParams.Ecu


# Steer torque limits

class CarControllerParams:
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_ERROR_MAX = 350           # max delta between torque cmd and torque motor
  STEER_STEP = 1  # 100 Hz

  def __init__(self, CP):
    # Higher-authority steering tune for cars with the CX-5 2022+ EPS (minSteerSpeed == 0
    # marks cars with full-range, hands-off-capable steering on that EPS).
    if CP.minSteerSpeed == 0:
      self.STEER_MAX = 1200          # theoretical max_steer 2047; EPS clips above ceiling per speed
      # 1200 below 32 mph for full low-speed authority, 800 above for smoother highway steering
      self.STEER_MAX_LOOKUP = ([0., 14.2, 14.5], [1200, 1200, 800])
      self.STEER_DELTA_UP = 12       # EPS hardware rate limit
      self.STEER_DELTA_DOWN = 25
      self.STEER_DRIVER_MULTIPLIER = 15   # weight driver torque (tuned for the CX-5 EPS; stock is 1)
    else:
      self.STEER_MAX = 800           # theoretical max_steer 2047
      self.STEER_DELTA_UP = 10
      self.STEER_DELTA_DOWN = 25
      self.STEER_DRIVER_MULTIPLIER = 1    # stock


# Empirically-learned speed-binned torque feedforward curve for the CX-5 2022+ EPS, ported from
# zoompilot's device-learned data (opendbc torque_data/speed_dependent.toml, MAZDA_CX5_2022).
# Fit to a CX-5 chassis, not necessarily an exact match for other bodies sharing this EPS, but a
# much better starting point than the untuned default for any car running this motor.
MAZDA_CX5_2022_TORQUE_SPEED_BP = [6.5, 9.5, 12.0, 16.4, 21.0, 28.0, 35.0]      # m/s
MAZDA_CX5_2022_TORQUE_LAT_ACCEL_BP = [2.30, 2.53, 2.23, 1.09, 1.10, 1.33, 1.61]
MAZDA_CX5_2022_TORQUE_FRICTION_BP = [0.173, 0.144, 0.147, 0.159, 0.147, 0.130, 0.108]


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.mazda]))


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # not optimized yet


class MazdaFlags(IntFlag):
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('mazda_2017', None))
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=15.5)
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
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
    [MazdaCarDocs("Mazda CX-9 2021-23", video_link="https://youtu.be/dA3duO4a0O4")],
    MAZDA_CX9.specs
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-24")],
    MAZDA_CX5.specs,
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


class Buttons:
  NONE = 0
  SET_PLUS = 1
  SET_MINUS = 2
  RESUME = 3
  CANCEL = 4


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
