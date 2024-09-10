#! python3

from commands2 import CommandScheduler, cmd, TimedCommandRobot
from lib import logger, telemetry
from lib.classes import RobotMode
from robot_container import RobotContainer

class Robot(TimedCommandRobot):
  _instance: TimedCommandRobot = None

  @staticmethod
  def getInstance() -> TimedCommandRobot:
    return Robot._instance

  def robotInit(self) -> None:
    Robot._instance = self
    logger.start()
    telemetry.start()
    self._autonomousCommand = cmd.none()
    self._robotContainer = RobotContainer()

  def robotPeriodic(self) -> None:
    try:
      CommandScheduler.getInstance().run()
    except:
      CommandScheduler.getInstance().cancelAll()
      self._robotContainer.resetRobot()
      logger.exception()

  def disabledInit(self) -> None:
    logger.mode(RobotMode.Disabled)

  def disabledPeriodic(self) -> None:
    pass

  def autonomousInit(self) -> None:
    logger.mode(RobotMode.Auto)
    self._robotContainer.autonomousInit()
    self._autonomousCommand = self._robotContainer.getAutonomousCommand()
    if self._autonomousCommand is not None:
      self._autonomousCommand.schedule()

  def autonomousPeriodic(self) -> None:
    pass

  def teleopInit(self) -> None:
    logger.mode(RobotMode.Teleop)
    if self._autonomousCommand is not None:
      self._autonomousCommand.cancel()
    self._robotContainer.teleopInit()

  def teleopPeriodic(self) -> None:
    pass

  def testInit(self) -> None:
    logger.mode(RobotMode.Test)
    CommandScheduler.getInstance().cancelAll()
    self._robotContainer.testInit()

  def testPeriodic(self) -> None:
    pass

  def _simulationInit(self) -> None:
    pass

  def _simulationPeriodic(self) -> None:
    pass
