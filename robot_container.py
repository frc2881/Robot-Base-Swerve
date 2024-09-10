from commands2 import Command, cmd
from wpilib import DriverStation, SendableChooser, SmartDashboard
import constants
from commands.auto_commands import AutoCommands
from commands.game_commands import GameCommands
from pathplannerlib.auto import AutoBuilder, HolonomicPathFollowerConfig, ReplanningConfig
from lib import logger, utils
from lib.classes import Alliance, RobotState
from lib.controllers.game_controller import GameController
from lib.sensors.gyro_sensor_navx2 import GyroSensor_NAVX2
from lib.sensors.pose_sensor import PoseSensor
from subsystems.drive_subsystem import DriveSubsystem
from subsystems.localization_subsystem import LocalizationSubsystem

class RobotContainer:
  def __init__(self) -> None:
    self._initSensors()
    self._initSubsystems()
    self._initControllers()
    self._initCommands()
    self._setupControllers()
    self._setupTriggers()
    self._setupAutos()

  def _initSensors(self) -> None:
    self.gyroSensor = GyroSensor_NAVX2(constants.Sensors.Gyro.NAVX2.kSerialPort)
    self.poseSensors: list[PoseSensor] = []
    for cameraName, cameraTransform in constants.Sensors.Pose.kPoseSensors.items():
      self.poseSensors.append(PoseSensor(
        cameraName,
        cameraTransform,
        constants.Sensors.Pose.kPoseStrategy,
        constants.Sensors.Pose.kFallbackPoseStrategy,
        constants.Game.Field.kAprilTagFieldLayout
      ))
    
  def _initSubsystems(self) -> None:
    self.driveSubsystem = DriveSubsystem(
      lambda: self.gyroSensor.getHeading()
    )
    self.localizationSubsystem = LocalizationSubsystem(
      self.poseSensors,
      lambda: self.gyroSensor.getRotation(),
      lambda: self.driveSubsystem.getSwerveModulePositions()
    )
    
  def _initControllers(self) -> None:
    DriverStation.silenceJoystickConnectionWarning(True)
    self.driverController = GameController(
      constants.Controllers.kDriverControllerPort, 
      constants.Controllers.kInputDeadband
    )
    self.operatorController = GameController(
      constants.Controllers.kOperatorControllerPort, 
      constants.Controllers.kInputDeadband
    )

  def _initCommands(self) -> None:
    pass
    self.gameCommands = GameCommands(self)
    self.autoCommands = AutoCommands(self.gameCommands)

  def _setupControllers(self) -> None:
    # DRIVER ========================================
    self.driveSubsystem.setDefaultCommand(
      self.driveSubsystem.driveCommand(
        lambda: self.driverController.getLeftY(),
        lambda: self.driverController.getLeftX(),
        lambda: self.driverController.getRightX()
    ))
    # self.driverController.rightTrigger().whileTrue(cmd.none())
    # self.driverController.rightBumper().whileTrue(cmd.none())
    # self.driverController.leftTrigger().whileTrue(cmd.none())
    # self.driverController.leftBumper().whileTrue(cmd.none())
    self.driverController.rightStick().whileTrue(self.gameCommands.alignRobotToTargetCommand())
    self.driverController.leftStick().whileTrue(self.driveSubsystem.lockCommand())
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

    # OPERATOR ========================================
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

  def _setupTriggers(self) -> None:
    pass

  def _setupAutos(self) -> None:
    AutoBuilder.configureHolonomic(
      lambda: self.localizationSubsystem.getPose(), 
      lambda pose: self.localizationSubsystem.resetPose(pose), 
      lambda: self.driveSubsystem.getSpeeds(), 
      lambda chassisSpeeds: self.driveSubsystem.drive(chassisSpeeds), 
      HolonomicPathFollowerConfig(
        constants.Subsystems.Drive.kPathFollowerTranslationPIDConstants,
        constants.Subsystems.Drive.kPathFollowerRotationPIDConstants,
        constants.Subsystems.Drive.kTranslationSpeedMax, 
        constants.Subsystems.Drive.kDriveBaseRadius, 
        ReplanningConfig()
      ),
      lambda: utils.getAlliance() == Alliance.Red,
      self.driveSubsystem
    )

    self._autoChooser = SendableChooser()
    self._autoChooser.setDefaultOption("None", lambda: cmd.none())
    
    self._autoChooser.addOption("Test", lambda: self.autoCommands.test())

    SmartDashboard.putData("Robot/Auto/Command", self._autoChooser)
    
  def getAutonomousCommand(self) -> Command:
    return self._autoChooser.getSelected()()
    
  def autonomousInit(self) -> None:
    self.resetRobot()

  def teleopInit(self) -> None:
    self.resetRobot()
    self.gyroSensor.alignRobotToField(self.localizationSubsystem.getPose())

  def testInit(self) -> None:
    self.resetRobot()

  def resetRobot(self) -> None:
    self.driveSubsystem.reset()
