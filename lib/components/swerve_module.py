from typing import TYPE_CHECKING
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkFlex, SparkMax, ClosedLoopConfig
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

    if self._constants.kDrivingMotorControllerType == MotorControllerType.SparkFlex:
      self._drivingMotor = SparkFlex(self._config.drivingMotorCANId, SparkLowLevel.MotorType.kBrushless)
    else: 
      self._drivingMotor = SparkMax(self._config.drivingMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._drivingMotorConfig = SparkBaseConfig()
    (self._drivingMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kDrivingMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kDrivingMotorCurrentLimit))
    (self._drivingMotorConfig.encoder
      .positionConversionFactor(self._constants.kDrivingEncoderPositionConversionFactor)
      .velocityConversionFactor(self._constants.kDrivingEncoderVelocityConversionFactor))
    (self._drivingMotorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
      .pidf(self._constants.kDrivingMotorPIDConstants.P, self._constants.kDrivingMotorPIDConstants.I, self._constants.kDrivingMotorPIDConstants.D, self._constants.kDrivingMotorPIDConstants.FF)
      .outputRange(self._constants.kDrivingMotorMaxReverseOutput, self._constants.kDrivingMotorMaxForwardOutput))
    utils.setSparkConfig(
      self._drivingMotor.configure(
        self._drivingMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._drivingClosedLoopController = self._drivingMotor.getClosedLoopController()
    self._drivingEncoder = self._drivingMotor.getEncoder()
    self._drivingEncoder.setPosition(0)

    self._turningMotor = SparkMax(self._config.turningMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._turningMotorConfig = SparkBaseConfig()
    (self._turningMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kTurningMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kTurningMotorCurrentLimit))
    (self._turningMotorConfig.absoluteEncoder
      .inverted(self._constants.kTurningEncoderInverted)
      .positionConversionFactor(self._constants.kTurningEncoderPositionConversionFactor)
      .velocityConversionFactor(self._constants.kTurningEncoderVelocityConversionFactor))
    (self._turningMotorConfig.closedLoop
      .setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
      .pidf(self._constants.kTurningMotorPIDConstants.P, self._constants.kTurningMotorPIDConstants.I, self._constants.kTurningMotorPIDConstants.D, self._constants.kTurningMotorPIDConstants.FF)
      .outputRange(self._constants.kTurningMotorMaxReverseOutput, self._constants.kTurningMotorMaxForwardOutput)
      .positionWrappingEnabled(True)
      .positionWrappingInputRange(self._constants.kTurningEncoderPositionPIDMinInput, self._constants.kTurningEncoderPositionPIDMaxInput))
    utils.setSparkConfig(
      self._turningMotor.configure(
        self._turningMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )
    self._turningClosedLoopController = self._turningMotor.getClosedLoopController()
    self._turningEncoder = self._turningMotor.getAbsoluteEncoder()

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
    utils.setSparkConfig(self._drivingMotor.configure(SparkBaseConfig().setIdleMode(idleMode), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters))
    utils.setSparkConfig(self._turningMotor.configure(SparkBaseConfig().setIdleMode(idleMode), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters))
    
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Target', self._drivingTargetSpeed)
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Actual', self._drivingEncoder.getVelocity())
    SmartDashboard.putNumber(f'{self._baseKey}/Turning/AbsolutePosition', self._turningEncoder.getPosition())
