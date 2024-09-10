from typing import TYPE_CHECKING
from wpimath import units
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpilib import SmartDashboard
from rev import CANSparkBase, CANSparkLowLevel, CANSparkMax, SparkAbsoluteEncoder
from lib.classes import SwerveModuleLocation, MotorIdleMode
from lib import utils, logger
if TYPE_CHECKING: import constants

class SwerveModule:
  def __init__(
      self,
      location: SwerveModuleLocation,
      drivingMotorCANId: int,
      turningMotorCANId: int,
      turningOffset: units.radians,
      constants: "constants.Subsystems.Drive.SwerveModule"
    ) -> None:
    self._constants = constants
    self._location = location
    self._turningOffset = turningOffset
    
    self._baseKey = f'Robot/Drive/SwerveModules/{self._location.name}'
    self._drivingTargetSpeed: units.meters_per_second = 0

    self._drivingMotor = CANSparkMax(drivingMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._drivingEncoder = self._drivingMotor.getEncoder()
    self._drivingPIDController = self._drivingMotor.getPIDController()
    self._drivingMotor.setCANMaxRetries(10)
    utils.validateParam(self._drivingMotor.restoreFactoryDefaults())
    utils.validateParam(self._drivingPIDController.setFeedbackDevice(self._drivingEncoder))
    utils.validateParam(self._drivingEncoder.setPositionConversionFactor(self._constants.kDrivingEncoderPositionConversionFactor))
    utils.validateParam(self._drivingEncoder.setVelocityConversionFactor(self._constants.kDrivingEncoderVelocityConversionFactor))
    utils.validateParam(self._drivingPIDController.setP(self._constants.kDrivingMotorPIDConstants.P))
    utils.validateParam(self._drivingPIDController.setI(self._constants.kDrivingMotorPIDConstants.I))
    utils.validateParam(self._drivingPIDController.setD(self._constants.kDrivingMotorPIDConstants.D))
    utils.validateParam(self._drivingPIDController.setFF(self._constants.kDrivingMotorPIDConstants.FF))
    utils.validateParam(self._drivingPIDController.setOutputRange(self._constants.kDrivingMotorMaxReverseOutput, self._constants.kDrivingMotorMaxForwardOutput))
    utils.validateParam(self._drivingMotor.setSmartCurrentLimit(self._constants.kDrivingMotorCurrentLimit))
    utils.validateParam(self._drivingMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._drivingMotor.burnFlash())

    self._turningMotor = CANSparkMax(turningMotorCANId, CANSparkLowLevel.MotorType.kBrushless)
    self._turningEncoder = self._turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle)
    self._turningPIDController = self._turningMotor.getPIDController()
    self._turningMotor.setCANMaxRetries(10)
    utils.validateParam(self._turningMotor.restoreFactoryDefaults())
    utils.validateParam(self._turningPIDController.setFeedbackDevice(self._turningEncoder))
    utils.validateParam(self._turningEncoder.setPositionConversionFactor(self._constants.kTurningEncoderPositionConversionFactor))
    utils.validateParam(self._turningEncoder.setVelocityConversionFactor(self._constants.kTurningEncoderVelocityConversionFactor))
    utils.validateParam(self._turningEncoder.setInverted(self._constants.kTurningEncoderInverted))
    utils.validateParam(self._turningPIDController.setP(self._constants.kTurningMotorPIDConstants.P))
    utils.validateParam(self._turningPIDController.setI(self._constants.kTurningMotorPIDConstants.I))
    utils.validateParam(self._turningPIDController.setD(self._constants.kTurningMotorPIDConstants.D))
    utils.validateParam(self._turningPIDController.setFF(self._constants.kTurningMotorPIDConstants.FF))
    utils.validateParam(self._turningPIDController.setOutputRange(self._constants.kTurningMotorMaxReverseOutput, self._constants.kTurningMotorMaxForwardOutput))
    utils.validateParam(self._turningPIDController.setPositionPIDWrappingEnabled(True))
    utils.validateParam(self._turningPIDController.setPositionPIDWrappingMinInput(self._constants.kTurningEncoderPositionPIDMinInput))
    utils.validateParam(self._turningPIDController.setPositionPIDWrappingMaxInput(self._constants.kTurningEncoderPositionPIDMaxInput))
    utils.validateParam(self._turningMotor.setSmartCurrentLimit(self._constants.kTurningMotorCurrentLimit))
    utils.validateParam(self._turningMotor.setIdleMode(CANSparkBase.IdleMode.kBrake))
    utils.validateParam(self._turningMotor.burnFlash())

    self._drivingEncoder.setPosition(0)

    utils.addRobotPeriodic(self._updateTelemetry)

  def setTargetState(self, targetState: SwerveModuleState) -> None:
    targetState.angle = targetState.angle.__add__(Rotation2d(self._turningOffset))
    targetState = SwerveModuleState.optimize(targetState, Rotation2d(self._turningEncoder.getPosition()))
    targetState.speed *= targetState.angle.__sub__(Rotation2d(self._turningEncoder.getPosition())).cos()
    self._drivingPIDController.setReference(targetState.speed, CANSparkBase.ControlType.kVelocity)
    self._turningPIDController.setReference(targetState.angle.radians(), CANSparkBase.ControlType.kPosition)
    self._drivingTargetSpeed = targetState.speed

  def getState(self) -> SwerveModuleState:
    return SwerveModuleState(self._drivingEncoder.getVelocity(), Rotation2d(self._turningEncoder.getPosition() - self._turningOffset))
  
  def getPosition(self) -> SwerveModulePosition:
    return SwerveModulePosition(self._drivingEncoder.getPosition(), Rotation2d(self._turningEncoder.getPosition() - self._turningOffset))
  
  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    idleMode = CANSparkBase.IdleMode.kCoast if motorIdleMode == MotorIdleMode.Coast else CANSparkBase.IdleMode.kBrake
    self._drivingMotor.setIdleMode(idleMode)
    self._turningMotor.setIdleMode(idleMode)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Target', self._drivingTargetSpeed)
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Actual', self._drivingEncoder.getVelocity())
    SmartDashboard.putNumber(f'{self._baseKey}/Turning/AbsolutePosition', self._turningEncoder.getPosition())