from typing import TYPE_CHECKING
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpilib import SmartDashboard
from rev import SparkBase, SparkLowLevel, SparkFlex, SparkMax, SparkAbsoluteEncoder, SparkFlexConfig, SparkMaxConfig, SparkBaseConfig, ClosedLoopConfig, EncoderConfig, AbsoluteEncoderConfig
from lib.classes import SwerveModuleConfig, MotorIdleMode, MotorControllerType
from lib import utils, logger
if TYPE_CHECKING: import constants

class SwerveModule:
  def __init__(
    self,
    config: SwerveModuleConfig,
    constants: "constants.Subsystems.Drive.SwerveModule"
  ) -> None:
    self._config = config
    self._constants = constants

    self._baseKey = f'Robot/Drive/SwerveModules/{self._config.location.name}'
    self._drivingTargetSpeed: units.meters_per_second = 0

    if self._constants.kDrivingMotorControllerType == MotorControllerType.SparkMax:
      self._drivingMotor = SparkMax(self._config.drivingMotorCANId, SparkLowLevel.MotorType.kBrushless)
    else: 
      self._drivingMotor = SparkFlex(self._config.drivingMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._drivingMotor.setCANMaxRetries(10)
    self._drivingMotorConfig = SparkBaseConfig() \
      .smartCurrentLimit(self._constants.kDrivingMotorCurrentLimit) \
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake) \
      .apply(EncoderConfig() \
        .positionConversionFactor(self._constants.kDrivingEncoderPositionConversionFactor) \
        .velocityConversionFactor(self._constants.kDrivingEncoderVelocityConversionFactor)) \
      .apply(ClosedLoopConfig() \
        .outputRange(self._constants.kDrivingMotorMaxReverseOutput, self._constants.kDrivingMotorMaxForwardOutput) \
        .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder) \
        .pidf(self._constants.kDrivingMotorPIDConstants.P, self._constants.kDrivingMotorPIDConstants.I, self._constants.kDrivingMotorPIDConstants.D, self._constants.kDrivingMotorPIDConstants.FF))
    utils.validateREVConfiguration(
      self._drivingMotor.configure(
        self._drivingMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._drivingEncoder = self._drivingMotor.getEncoder()
    self._drivingClosedLoopController = self._drivingMotor.getClosedLoopController()

    self._turningMotor = SparkMax(self._config.turningMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._turningMotor.setCANMaxRetries(10)
    self._turningMotorConfig = SparkBaseConfig() \
      .smartCurrentLimit(self._constants.kTurningMotorCurrentLimit) \
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake) \
      .inverted(self._constants.kTurningEncoderInverted) \
      .apply(AbsoluteEncoderConfig() \
        .positionConversionFactor(self._constants.kTurningEncoderPositionConversionFactor) \
        .velocityConversionFactor(self._constants.kTurningEncoderVelocityConversionFactor)) \
      .apply(ClosedLoopConfig() \
        .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder) \
        .pidf(self._constants.kTurningMotorPIDConstants.P, self._constants.kTurningMotorPIDConstants.I, self._constants.kTurningMotorPIDConstants.D, self._constants.kTurningMotorPIDConstants.FF) \
        .outputRange(self._constants.kTurningMotorMaxReverseOutput, self._constants.kTurningMotorMaxForwardOutput)
        .positionWrappingInputRange(self._constants.kTurningEncoderPositionPIDMinInput, self._constants.kTurningEncoderPositionPIDMaxInput)
        .positionWrappingEnabled(True))
    utils.validateREVConfiguration(
      self._turningMotor.configure(
        self._turningMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._turningEncoder = self._turningMotor.getAbsoluteEncoder()
    self._turningClosedLoopController = self._turningMotor.getClosedLoopController()

    self._drivingEncoder.setPosition(0)

    utils.addRobotPeriodic(self._updateTelemetry)

  def setTargetState(self, targetState: SwerveModuleState) -> None:
    targetState.angle = targetState.angle.__add__(Rotation2d(self._config.turningOffset))
    targetState.optimize(Rotation2d(self._turningEncoder.getPosition()))
    targetState.speed *= targetState.angle.__sub__(Rotation2d(self._turningEncoder.getPosition())).cos()
    self._drivingClosedLoopController.setReference(targetState.speed, SparkBase.ControlType.kVelocity)
    self._turningClosedLoopController.setReference(targetState.angle.radians(), SparkBase.ControlType.kPosition)
    self._drivingTargetSpeed = targetState.speed

  def getState(self) -> SwerveModuleState:
    return SwerveModuleState(self._drivingEncoder.getVelocity(), Rotation2d(self._turningEncoder.getPosition() - self._config.turningOffset))

  def getPosition(self) -> SwerveModulePosition:
    return SwerveModulePosition(self._drivingEncoder.getPosition(), Rotation2d(self._turningEncoder.getPosition() - self._config.turningOffset))

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    idleMode = SparkBaseConfig.IdleMode.kCoast if motorIdleMode == MotorIdleMode.Coast else SparkBaseConfig.IdleMode.kBrake
    utils.validateREVConfiguration(self._drivingMotor.configure(self._drivingMotorConfig.setIdleMode(idleMode), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters))
    utils.validateREVConfiguration(self._turningMotor.configure(self._turningMotorConfig.setIdleMode(idleMode), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters))
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Target', self._drivingTargetSpeed)
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Actual', self._drivingEncoder.getVelocity())
    SmartDashboard.putNumber(f'{self._baseKey}/Turning/AbsolutePosition', self._turningEncoder.getPosition())
