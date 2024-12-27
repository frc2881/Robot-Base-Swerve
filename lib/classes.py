from enum import Enum, IntEnum, auto
from dataclasses import dataclass
from wpimath import units
from wpimath.geometry import Translation2d, Transform3d
from robotpy_apriltag import AprilTagFieldLayout
from photonlibpy.photonPoseEstimator import PoseStrategy

class Alliance(IntEnum):
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

class OptionState(Enum):
  Enabled = auto()
  Disabled = auto()

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

class SpeedMode(Enum):
  Competition = auto()
  Demo = auto()

class LockState(Enum):
  Unlocked = auto()
  Locked = auto()

class ControllerRumbleMode(Enum):
  Both = auto()
  Driver = auto()
  Operator = auto()

class ControllerRumblePattern(Enum):
  Short = auto()
  Long = auto()

class SwerveModuleLocation(IntEnum):
  FrontLeft = 0,
  FrontRight = 1,
  RearLeft = 2,
  RearRight = 3

class PoseSensorLocation(Enum):
  Front = auto(),
  Rear = auto(),
  Left = auto(),
  Right = auto()

@dataclass(frozen=True)
class PIDConstants:
  P: float
  I: float
  D: float
  FF: float

@dataclass(frozen=True)
class SwerveModuleConfig:
  location: SwerveModuleLocation
  drivingMotorCANId: int
  turningMotorCANId: int
  turningOffset: units.radians
  translation: Translation2d

@dataclass(frozen=True)
class PoseSensorConfig:
  location: PoseSensorLocation
  cameraTransform: Transform3d
  poseStrategy: PoseStrategy
  fallbackPoseStrategy: PoseStrategy
  aprilTagFieldLayout: AprilTagFieldLayout

@dataclass
class TargetInfo:
  distance: units.meters
  heading: units.degrees
  pitch: units.degrees