from commands2 import Command, cmd
from wpilib import DriverStation, SendableChooser, SmartDashboard
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig, ModuleConfig
from pathplannerlib.controller import PPHolonomicDriveController
from lib import logger, utils
from lib.classes import Alliance
from lib.controllers.game_controller import GameController
from lib.sensors.gyro_sensor_navx2 import GyroSensor_NAVX2
from lib.sensors.pose_sensor import PoseSensor
from commands.auto_commands import AutoCommands
from commands.game_commands import GameCommands
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.localization_subsystem import LocalizationSubsystem
import constants

class RobotContainer:
  def __init__(self) -> None:
    self._setupSensors()
    self._setupSubsystems()
    self._setupCommands()
    self._setupTriggers()
    self._setupControllers()
    utils.addRobotPeriodic(lambda: self._updateTelemetry())

  def _setupSensors(self) -> None:
    self.gyroSensor = GyroSensor_NAVX2(constants.Sensors.Gyro.NAVX2.kSerialPort)
    self.poseSensors: list[PoseSensor] = []
    for location, transform in constants.Sensors.Pose.kPoseSensors.items():
      self.poseSensors.append(PoseSensor(
        location.name,
        transform,
        constants.Sensors.Pose.kPoseStrategy,
        constants.Sensors.Pose.kFallbackPoseStrategy,
        constants.Game.Field.kAprilTagFieldLayout
      ))
    SmartDashboard.putString("Robot/Sensor/Camera/Streams", utils.toJson(constants.Sensors.Camera.kStreams))
    
  def _setupSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(
      lambda: self.gyroSensor.getHeading()
    )
    self.localizationSubsystem = LocalizationSubsystem(
      self.poseSensors,
      lambda: self.gyroSensor.getRotation(),
      lambda: self.driveSubsystem.getSwerveModulePositions()
    )
    
    AutoBuilder.configure(
      lambda: self.localizationSubsystem.getPose(), 
      lambda pose: self.localizationSubsystem.resetPose(pose), 
      lambda: self.driveSubsystem.getSpeeds(), 
      lambda chassisSpeeds: self.driveSubsystem.drive(chassisSpeeds), 
      PPHolonomicDriveController(
        constants.Subsystems.Drive.kPathFollowerTranslationPIDConstants,
        constants.Subsystems.Drive.kPathFollowerRotationPIDConstants
      ),
      RobotConfig(
        constants.Subsystems.Drive.kRobotMass,
        constants.Subsystems.Drive.kRobotMOI,
        ModuleConfig(
          constants.Subsystems.Drive.kDriveBaseRadius,
          constants.Subsystems.Drive.kTranslationSpeedMax,
          constants.Subsystems.Drive.SwerveModule.kWheelCOF,
          constants.Subsystems.Drive.SwerveModule.kDrivingMotorType,
          constants.Subsystems.Drive.SwerveModule.kDrivingMotorCurrentLimit,
          constants.Subsystems.Drive.SwerveModule.kDrivingMotorCount    
        ),
        constants.Subsystems.Drive.kTrackWidth,
        constants.Subsystems.Drive.kWheelBase
      ),
      lambda: utils.getAlliance() == Alliance.Red,
      self.driveSubsystem
    )
    
  def _setupCommands(self) -> None:
    self.gameCommands = GameCommands(self)
    self._autoCommand = cmd.none()
    self._autoChooser = SendableChooser()
    self._autoChooser.setDefaultOption("None", cmd.none)
    self._autoChooser.onChange(lambda command: setattr(self, "_autoCommand", command()))
    self.autoCommands = AutoCommands(self)
    SmartDashboard.putData("Robot/Auto/Command", self._autoChooser)

  def _setupTriggers(self) -> None:
    pass

  def _setupControllers(self) -> None:
    self.driverController = GameController(
      constants.Controllers.kDriverControllerPort, 
      constants.Controllers.kInputDeadband
    )
    self.operatorController = GameController(
      constants.Controllers.kOperatorControllerPort, 
      constants.Controllers.kInputDeadband
    )
    DriverStation.silenceJoystickConnectionWarning(True)

    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveCommand(
        lambda: self.driverController.getLeftY(),
        lambda: self.driverController.getLeftX(),
        lambda: self.driverController.getRightX()
    ))
    self.driverController.rightStick().whileTrue(self.gameCommands.alignRobotToTargetCommand())
    self.driverController.leftStick().whileTrue(self.driveSubsystem.lockCommand())
    # self.driverController.rightTrigger().whileTrue(cmd.none())
    # self.driverController.rightBumper().whileTrue(cmd.none())
    # self.driverController.leftTrigger().whileTrue(cmd.none())
    # self.driverController.leftBumper().whileTrue(cmd.none())
    # self.driverController.povUp().whileTrue(cmd.none())
    # self.driverController.povDown().whileTrue(cmd.none())
    # self.driverController.povLeft().whileTrue(cmd.none())
    # self.driverController.povRight().whileTrue(cmd.none())
    # self.driverController.a().whileTrue(cmd.none())
    # self.driverController.b().whileTrue(cmd.none())
    # self.driverController.y().whileTrue(cmd.none())
    # self.driverController.x().whileTrue(cmd.none())
    self.driverController.start().onTrue(self.gyroSensor.calibrateCommand())
    self.driverController.back().onTrue(self.gyroSensor.resetCommand())

    # self.operatorController.rightTrigger().whileTrue(cmd.none())
    # self.operatorController.rightBumper().whileTrue(cmd.none())
    # self.operatorController.leftTrigger().whileTrue(cmd.none())
    # self.operatorController.leftBumper().whileTrue(cmd.none())
    # self.operatorController.povUp().whileTrue(cmd.none())
    # self.operatorController.povDown().whileTrue(cmd.none())
    # self.operatorController.povLeft().whileTrue(cmd.none())
    # self.operatorController.povRight().whileTrue(cmd.none())
    # self.operatorController.a().whileTrue(cmd.none())
    # self.operatorController.b().whileTrue(cmd.none())
    # self.operatorController.y().whileTrue(cmd.none())
    # self.operatorController.x().whileTrue(cmd.none())
    # self.operatorController.start().whileTrue(cmd.none())
    # self.operatorController.back().whileTrue(cmd.none())

  def _robotHasInitialZeroResets(self) -> bool:
    return utils.isCompetitionMode() or True

  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/HasInitialZeroResets", self._robotHasInitialZeroResets())

  def addAutoOption(self, name: str, command: object) -> None:
    self._autoChooser.addOption(name, command)

  def getAutoCommand(self) -> Command:
    return self._autoCommand

  def autoInit(self) -> None:
    self.resetRobot()

  def autoExit(self) -> None: 
    self.gyroSensor.resetRobotToField(self.localizationSubsystem.getPose())
    
  def teleopInit(self) -> None:
    self.resetRobot()

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()


