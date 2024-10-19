from enum import Enum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d

class Alliance(Enum):
  Red = 0
  Blue = 1

class RobotMode(Enum):
  Disabled = auto()
  Auto = auto()
  Teleop = auto()
  Test = auto()

class RobotState(Enum):
  Disabled = auto()
  Enabled = auto()
  EStopped = auto()

class ChassisLocation(Enum):
  FrontLeft = 0
  FrontRight = 1
  RearLeft = 2
  RearRight = 3
  Front = 4
  Rear = 5
  Left = 6
  Right = 7

class MotorDirection(Enum):
  Forward = auto()
  Reverse = auto()
  Stopped = auto()

class MotorIdleMode(Enum):
  Brake = auto()
  Coast = auto()

class MotorControllerType(Enum):
  SparkMax = auto()
  SparkFlex = auto()

class DriveOrientation(Enum):
  Field = auto()
  Robot = auto()

class DriveLockState(Enum):
  Unlocked = auto()
  Locked = auto()

class DriveDriftCorrection(Enum):
  Enabled = auto()
  Disabled = auto()

class ControllerRumbleMode(Enum):
  Both = auto()
  Driver = auto()
  Operator = auto()

class ControllerRumblePattern(Enum):
  Short = auto()
  Long = auto()

class SpeedMode(Enum):
  Competition = auto()
  Demo = auto()

@dataclass(frozen=True)
class PIDConstants:
  P: float
  I: float
  D: float
  FF: float

@dataclass(frozen=True)
class SwerveModuleConfig:
  location: ChassisLocation
  drivingMotorCANId: int
  turningMotorCANId: int
  turningOffset: units.radians
  translation: Translation2d

@dataclass
class TargetInfo:
  distance: units.meters
  heading: units.degrees
  pitch: units.degrees