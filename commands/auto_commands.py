from typing import TYPE_CHECKING
from enum import Enum, auto
from commands2 import Command, cmd
from wpilib import SendableChooser
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
if TYPE_CHECKING: from robot_container import RobotContainer
import constants

class AutoPath(Enum):
  Move0 = auto()
  Move2 = auto()

class AutoCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

    self._paths = { path: PathPlannerPath.fromPathFile(path.name) for path in AutoPath }

  def _move(self, path: AutoPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      self._paths.get(path), 
      constants.Subsystems.Drive.kPathFindingConstraints
    ).withTimeout(
      constants.Game.Commands.kAutoMoveTimeout
    )
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(self._robot.gameCommands.alignRobotToTargetCommand())

  def addAutoOptions(self, chooser: SendableChooser) -> None:
    chooser.setDefaultOption("None", cmd.none)
    chooser.addOption("[0] 0_", self.auto_0_)
    chooser.addOption("[2] 2_", self.auto_2_)

  def auto_0_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move0)
    ).withName("AutoCommands:[0] 0_")

  def auto_2_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move2),
      self._alignToTarget()
    ).withName("AutoCommands:[2] 2_")
  