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
  # The measured gen1 EPS envelope, mazda.h's MAZDA_EPS_HW_STEERING_LIMITS. Ported from zoompilot,
  # whose docs/zoompilot/mazda-lateral.md holds the measurements behind each constant.
  EPS_STEER_MAX = 1200            # theoretical max_steer 2047
  STEER_DELTA_UP = 12             # torque increase per refresh, the EPS's own slew
  STEER_DELTA_DOWN = 12           # torque decrease per refresh, must equal the panda's max_rate_down
  STEER_DRIVER_ALLOWANCE = 15     # allowed driver torque before start limiting
  STEER_DRIVER_MULTIPLIER = 15    # weight driver torque, tuned for the 2022 EPS
  STEER_DRIVER_FACTOR = 1         # from dbc
  STEER_STEP = 1  # 100 Hz
  # Upstream's STEER_MAX, the scale params.toml's tunes, NNFF models and the manual torque override
  # use: latAccelFactor x TUNE_SCALE, friction and NN torque / TUNE_SCALE.
  TUNE_STEER_MAX = 800
  TUNE_SCALE = EPS_STEER_MAX / TUNE_STEER_MAX

  def __init__(self, CP):
    # Use a sample window and margin to stay inside panda's fresher driver-torque envelope.
    self.STEER_DRIVER_SAMPLES = 10
    self.STEER_DRIVER_MARGIN = 2

    self.STEER_MAX = self.EPS_STEER_MAX
    # Clamp to the measured applied-torque ceiling so controlsd can detect saturation.
    self.EPS_CEILING_LOOKUP = ([8.0, 8.5, 9.4, 10.3, 11.2, 12.1, 13.0, 13.9, 14.5],
                               [1148, 1132, 1092, 1048, 1012,  920,  808,  676,  620])

    if CP.flags & MazdaFlags.STEER_TO_ZERO_EPS:
      # Stop commanding after sustained zero delivery to avoid a camera steering fault. Use
      # LKAS_EFFECTIVE because LKAS_BLOCK may still permit partial delivery.
      self.STEER_UNDELIVERED_MIN = 200      # counts; below this the EPS rounds to zero anyway
      self.STEER_UNDELIVERED_FRAMES = 20    # 200 ms at 100 Hz

      # Alert only after sustained non-delivery above maneuvering speed. Suppress normal
      # low-speed standby blocks identified by LKAS_TRACK_STATE.
      self.STEER_UNDELIVERED_ALERT_FRAMES = 80    # 0.8 s at 100 Hz, on top of the latch's 0.2
      self.STEER_UNDELIVERED_ALERT_MIN_SPEED = 12. * CV.MPH_TO_MS
      # A block that began below this speed is the EPS's standby from a stop; only one that began
      # rolling can be a dropout. carstate's first-engagement hold uses the same boundary.
      self.STEER_UNDELIVERED_ALERT_ORIGIN_SPEED = 1.0  # m/s


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
  # Static flags
  # Gen 1 hardware: same CAN messages and same camera
  GEN1 = 1

  # EPS firmware that steers to zero: no speed floor and LKAS_TRACK_STATE standby semantics.
  STEER_TO_ZERO_EPS = 2
  # Every other gen1 Mazda EPS: the same hardware on firmware that keeps the 45 kph floor.
  LEGACY_FW_EPS = 4
  # Everything keyed on the measured hardware rather than on what the firmware permits.
  EPS_HW = STEER_TO_ZERO_EPS | LEGACY_FW_EPS


class MazdaSafetyFlags(IntFlag):
  # Selects the measured EPS envelope in panda safety; the interface sets it on every gen1 EPS.
  EPS_HW = 2


@dataclass
class MazdaPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'mazda_2017'})
  flags: int = MazdaFlags.GEN1


class CAR(Platforms):
  MAZDA_CX5 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2017-21")],
    MazdaCarSpecs(mass=3655 * CV.LB_TO_KG, wheelbase=2.7, steerRatio=18.1),  # steer ratio from the 2022 CX-5: same rack hardware
  )
  MAZDA_CX9 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-9 2016-20")],
    MazdaCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6),
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
    MazdaCarSpecs(mass=4409 * CV.LB_TO_KG, wheelbase=2.93, steerRatio=17.6),
  )
  MAZDA_CX5_2022 = MazdaPlatformConfig(
    [MazdaCarDocs("Mazda CX-5 2022-25")],
    MazdaCX5_2022CarSpecs(mass=3728 * CV.LB_TO_KG, wheelbase=2.698, steerRatio=18.1),  # learned; factory spec 15.5
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15
  DISABLE_SPEED = 45    # kph
  ENABLE_SPEED = 52     # kph


# (latAccelFactor, friction) on params.toml's scale for a platform whose params.toml entry is
# borrowed: the CX-5 2022's own learned values, not the CX-9 2021's.
TORQUE_TUNES = {
  CAR.MAZDA_CX5_2022: (1.222, 0.154),
}

# Keep steer-to-zero firmware synchronized with the CX-5 2022 EPS entries in fingerprints.py.
STEER_TO_ZERO_EPS_FW = {
  b'K0A1-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',  # CX-8 2023 (Japan)
  b'KBST-3210X-A-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
  b'KSD5-3210X-C-00\x00\x00\x00\x00\x00\x00\x00\x00\x00',
}

# Platforms that ship the steer-to-zero EPS from the factory: what an unread EPS falls back to.
STEER_TO_ZERO_PLATFORMS = frozenset({CAR.MAZDA_CX5_2022})


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
