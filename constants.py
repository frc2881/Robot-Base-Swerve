import math
from wpilib import SerialPort
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath import units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from photonlibpy.photonPoseEstimator import PoseStrategy
from pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from pathplannerlib.pathfinding import PathConstraints
from rev import ClosedLoopConfig, SparkBaseConfig, SparkFlexConfig, SparkMaxConfig, ClosedLoopSlot, AbsoluteEncoderConfig, EncoderConfig
from lib.classes import PIDConstants, MotorControllerType, ChassisLocation, SwerveModuleConfig
from typing_extensions import Self


class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(24.5)
    kWheelBase: units.meters = units.inchesToMeters(21.5)
    kDriveBaseRadius: units.meters = Translation2d().distance(Translation2d(kWheelBase / 2, kTrackWidth / 2))

    kTranslationSpeedMax: units.meters_per_second = 4.8
    kRotationSpeedMax: units.radians_per_second = 4 * math.pi  # type: ignore

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

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

    kSwerveModules = (
      SwerveModuleConfig(ChassisLocation.FrontLeft, 2, 3, -math.pi / 2, Translation2d(kWheelBase / 2, kTrackWidth / 2)),
      SwerveModuleConfig(ChassisLocation.FrontRight, 4, 5, 0, Translation2d(kWheelBase / 2, -kTrackWidth / 2)),
      SwerveModuleConfig(ChassisLocation.RearLeft, 6, 7, math.pi, Translation2d(-kWheelBase / 2, kTrackWidth / 2)),
      SwerveModuleConfig(ChassisLocation.RearRight, 8, 9, math.pi / 2, Translation2d(-kWheelBase / 2, -kTrackWidth / 2))
    )

    kSwerveDriveKinematics = SwerveDrive4Kinematics(
      kSwerveModules[0].translation,
      kSwerveModules[1].translation,
      kSwerveModules[2].translation,
      kSwerveModules[3].translation
    )

    class SwerveModule:
      kWheelDiameter: units.meters = units.inchesToMeters(3.0)
      kWheelCircumference: units.meters = kWheelDiameter * math.pi
      kWheelBevelGearTeeth: int = 45
      kWheelSpurGearTeeth: int = 22
      kWheelBevelPinionTeeth: int = 15
      kDrivingMotorControllerType = MotorControllerType.SparkMax
      kDrivingMotorFreeSpeed: units.revolutions_per_minute = 5676
      kDrivingMotorPinionTeeth: int = 14
      kDrivingMotorReduction: float = (kWheelBevelGearTeeth * kWheelSpurGearTeeth) / (kDrivingMotorPinionTeeth * kWheelBevelPinionTeeth)
      kDrivingMotorFreeSpeedRps: float = kDrivingMotorFreeSpeed / 60
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

      kDrivingMotorConfig = SparkFlexConfig() \
        .smartCurrentLimit(kDrivingMotorCurrentLimit) \
        .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      # Bug in current version of rev py libraries - we have to explicitly call the base class method.
      SparkBaseConfig.apply(kDrivingMotorConfig, EncoderConfig() \
                            .positionConversionFactor(kDrivingEncoderPositionConversionFactor) \
                            .velocityConversionFactor(kDrivingEncoderVelocityConversionFactor))
      SparkBaseConfig.apply(kDrivingMotorConfig, ClosedLoopConfig() \
                            .outputRange(kDrivingMotorMaxReverseOutput, kDrivingMotorMaxForwardOutput) \
                            .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder) \
                            .pidf(kDrivingMotorPIDConstants.P,
                                  kDrivingMotorPIDConstants.I,
                                  kDrivingMotorPIDConstants.D,
                                  kDrivingMotorPIDConstants.FF))

      kTurningMotorConfig = SparkMaxConfig() \
        .smartCurrentLimit(kTurningMotorCurrentLimit) \
        .setIdleMode(SparkBaseConfig.IdleMode.kBrake) \
        .inverted(kTurningEncoderInverted)
      # Bug in current version of rev py libraries - we have to explicitly call SparkBaseConfig.apply
      SparkBaseConfig.apply(kTurningMotorConfig, AbsoluteEncoderConfig() \
                            .positionConversionFactor(kTurningEncoderPositionConversionFactor)
                            .velocityConversionFactor(kTurningEncoderVelocityConversionFactor))
      SparkBaseConfig.apply(kTurningMotorConfig, ClosedLoopConfig() \
                            .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder) \
                            .pidf(kTurningMotorPIDConstants.P,
                                  kTurningMotorPIDConstants.I,
                                  kTurningMotorPIDConstants.D,
                                  kTurningMotorPIDConstants.FF) \
                            .outputRange(kTurningMotorMaxReverseOutput, kTurningMotorMaxForwardOutput)
                            .positionWrappingInputRange(kTurningEncoderPositionPIDMinInput, kTurningEncoderPositionPIDMaxInput)
                            .positionWrappingEnabled(True))


class Sensors:
  class Gyro:
    class NAVX2:
      kSerialPort = SerialPort.Port.kUSB1

  class Pose:
    kPoseSensors: dict[ChassisLocation, Transform3d] = {
      # ChassisLocation.Front: Transform3d(
      #   Translation3d(units.inchesToMeters(9.62), units.inchesToMeters(4.12), units.inchesToMeters(21.25)),
      #   Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-22.3), units.degreesToRadians(0.0))
      # )
    }
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kSingleTagStandardDeviations: tuple[float, ...] = [1.0, 1.0, 2.0]
    kMultiTagStandardDeviations: tuple[float, ...] = [0.5, 0.5, 1.0]
    kMaxPoseAmbiguity: units.percent = 0.2

  class Camera:
    kStreams: dict[str, str] = {
      # "Front": "http://10.28.81.6:1184/?action=stream",
      # "Driver": "http://10.28.81.6:1188/?action=stream"
    }


class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1


APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)


class Game:
  class Commands:
    kScoringAlignmentTimeout: units.seconds = 0.8
    kScoringLaunchTimeout: units.seconds = 1.0
    kAutoPickupTimeout: units.seconds = 4.0

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kBlueTarget = APRIL_TAG_FIELD_LAYOUT.getTagPose(7) or Pose3d()
      kRedTarget = APRIL_TAG_FIELD_LAYOUT.getTagPose(4) or Pose3d()

      kTargetTransform = Transform3d(
        units.inchesToMeters(6.0),
        units.inchesToMeters(12.0),
        units.inchesToMeters(24),
        Rotation3d()
      )
