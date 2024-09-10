from typing import TYPE_CHECKING
from wpilib import RobotBase
from commands2 import Command, cmd
if TYPE_CHECKING: from robot_container import RobotContainer
from lib import utils
from lib.classes import ControllerRumbleMode, ControllerRumblePattern

class GameCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self.robot = robot

  def alignRobotToTargetCommand(self) -> Command:
    return cmd.sequence(
      cmd.parallel(
        self.robot.driveSubsystem.alignToTargetCommand(
          lambda: self.robot.localizationSubsystem.getPose(), 
          lambda: self.robot.localizationSubsystem.getTargetHeading()
        ),
        self.rumbleControllersCommand(ControllerRumbleMode.Operator, ControllerRumblePattern.Short),
        cmd.sequence(
          cmd.waitUntil(lambda: self.robot.driveSubsystem.isAlignedToTarget()),
          self.rumbleControllersCommand(ControllerRumbleMode.Driver, ControllerRumblePattern.Short)
        )
      )
    ).withName("GameCommands:AlignRobotToTarget")

  def rumbleControllersCommand(self, mode: ControllerRumbleMode, pattern: ControllerRumblePattern) -> Command:
    return cmd.parallel(
      self.robot.driverController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Driver or mode == ControllerRumbleMode.Both),
      self.robot.operatorController.rumbleCommand(pattern).onlyIf(lambda: mode == ControllerRumbleMode.Operator or mode == ControllerRumbleMode.Both)
    ).onlyIf(
      lambda: RobotBase.isReal() and not utils.isAutonomousMode()
    ).withName("GameCommands:RumbleControllers")