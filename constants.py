import math
from wpilib import SerialPort
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from photonlibpy.photonPoseEstimator import PoseStrategy
from pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from pathplannerlib.pathfinding import PathConstraints
from pathplannerlib.path import PathPlannerPath
from lib.classes import PIDConstants
from lib.utils import logger
from classes import AutoPath

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(21.5)
    kWheelBase: units.meters = units.inchesToMeters(24.5)
    kDriveBaseRadius: units.meters = Translation2d().distance(Translation2d(kWheelBase / 2, kTrackWidth / 2))

    kTranslationSpeedMax: units.meters_per_second = 4.8
    kRotationSpeedMax: units.radians_per_second = 2 * math.pi

    kInputLimit: units.percent = 0.6
    kInputRateLimit: units.percent = 0.5

    kDriftCorrectionThetaControllerPIDConstants = PIDConstants(0.01, 0, 0, 0)
    kDriftCorrectionThetaControllerPositionTolerance: float = 0.5
    kDriftCorrectionThetaControllerVelocityTolerance: float = 0.5

    kTargetAlignmentThetaControllerPIDConstants = PIDConstants(0.075, 0, 0, 0)
    kTargetAlignmentThetaControllerPositionTolerance: float = 1.0
    kTargetAlignmentThetaControllerVelocityTolerance: float = 1.0
    kTargetAlignmentCarpetFrictionCoeff: float = 0.15
    kTargetAlignmentHeadingInversion: units.degrees = 180.0

    kPathFollowerTranslationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFollowerRotationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFindingConstraints = PathConstraints(2.4, 1.6, units.degreesToRadians(360), units.degreesToRadians(720))

    kSwerveModuleFrontLeftDrivingMotorCANId: int = 2
    kSwerveModuleFrontLeftTurningMotorCANId: int = 3
    kSwerveModuleFrontRightDrivingMotorCANId: int = 4 
    kSwerveModuleFrontRightTurningMotorCANId: int = 5 
    kSwerveModuleRearLeftDrivingMotorCANId: int = 6 
    kSwerveModuleRearLeftTurningMotorCANId: int = 7 
    kSwerveModuleRearRightDrivingMotorCANId: int = 8 
    kSwerveModuleRearRightTurningMotorCANId: int = 9 

    kSwerveModuleFrontLeftTurningOffset: units.radians = -math.pi / 2
    kSwerveModuleFrontRightTurningOffset: units.radians = 0
    kSwerveModuleRearLeftTurningOffset: units.radians = math.pi
    kSwerveModuleRearRightTurningOffset: units.radians = math.pi / 2

    kSwerveModuleFrontLeftTranslation = Translation2d(kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleFrontRightTranslation = Translation2d(kWheelBase / 2, -kTrackWidth / 2)
    kSwerveModuleRearLeftTranslation = Translation2d(-kWheelBase / 2, kTrackWidth / 2)
    kSwerveModuleRearRightTranslation = Translation2d(-kWheelBase / 2, -kTrackWidth / 2)

    kSwerveDriveKinematics = SwerveDrive4Kinematics(
      kSwerveModuleFrontLeftTranslation, 
      kSwerveModuleFrontRightTranslation, 
      kSwerveModuleRearLeftTranslation, 
      kSwerveModuleRearRightTranslation
    )

    class SwerveModule:
      kFreeSpeed: units.revolutions_per_minute = 5676
      kWheelDiameter: units.meters = units.inchesToMeters(3.0)
      kWheelCircumference: units.meters = kWheelDiameter * math.pi
      kDrivingMotorPinionTeeth: int = 14
      kDrivingMotorReduction: float = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15)
      kDrivingMotorFreeSpeedRps: float = kFreeSpeed / 60
      kDriveWheelFreeSpeedRps: float = (kDrivingMotorFreeSpeedRps * kWheelCircumference) / kDrivingMotorReduction 
      kDrivingEncoderPositionConversionFactor: float = (kWheelDiameter * math.pi) / kDrivingMotorReduction
      kDrivingEncoderVelocityConversionFactor: float = ((kWheelDiameter * math.pi) / kDrivingMotorReduction) / 60.0
      kTurningEncoderInverted: bool = True
      kTurningEncoderPositionConversionFactor: float = 2 * math.pi
      kTurningEncoderVelocityConversionFactor: float = (2 * math.pi) / 60.0
      kTurningEncoderPositionPIDMinInput: float = 0
      kTurningEncoderPositionPIDMaxInput: float = kTurningEncoderPositionConversionFactor
      kDrivingMotorCurrentLimit: units.amperes = 80
      kDrivingMotorMaxReverseOutput: units.percent = -1.0
      kDrivingMotorMaxForwardOutput: units.percent = 1.0
      kDrivingMotorPIDConstants = PIDConstants(0.04, 0, 0, 1 / kDriveWheelFreeSpeedRps)
      kTurningMotorCurrentLimit: units.amperes = 20
      kTurningMotorMaxReverseOutput: units.percent = -1.0
      kTurningMotorMaxForwardOutput: units.percent = 1.0
      kTurningMotorPIDConstants = PIDConstants(1, 0, 0, 0)

class Sensors:
  class Gyro:
    class NAVX2:
      kSerialPort = SerialPort.Port.kUSB1

  class Pose:
    kPoseSensors: dict[str, Transform3d] = {
      "Front": Transform3d(
        Translation3d(units.inchesToMeters(0), units.inchesToMeters(0), units.inchesToMeters(0)),
        Rotation3d(units.degreesToRadians(0), units.degreesToRadians(0), units.degreesToRadians(0))
      )
    }
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kVisionSingleTagStandardDeviations: tuple[float, ...] = [1.0, 1.0, 2.0]
    kVisionMultiTagStandardDeviations: tuple[float, ...] = [0.5, 0.5, 1.0]
    kVisionMaxPoseAmbiguity: units.percent = 0.2

  class Camera:
    kStreams: dict[str, str] = {
      "Front": "http://10.28.81.6:1182/?action=stream",
      "Driver": "http://10.28.81.6:1182/?action=stream"
    }

_aprilTagFieldLayout = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

class Game:
  class Field:
    kAprilTagFieldLayout = _aprilTagFieldLayout
    kLength = kAprilTagFieldLayout.getFieldLength()
    kWidth = kAprilTagFieldLayout.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:  
      kBlueTarget = _aprilTagFieldLayout.getTagPose(7) or Pose3d()
      kRedTarget = _aprilTagFieldLayout.getTagPose(4) or Pose3d()

      kTargetTransform = Transform3d(
        units.inchesToMeters(0),
        units.inchesToMeters(0),
        units.inchesToMeters(0),
        Rotation3d()
      )

  class Auto:
    kPaths: dict[AutoPath, PathPlannerPath] = {
      AutoPath.Test: PathPlannerPath.fromPathFile(AutoPath.Test.name)
    }
