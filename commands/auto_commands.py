from typing import TYPE_CHECKING
from commands2 import Command, cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
if TYPE_CHECKING: from robot_container import RobotContainer
import constants
from classes import AutoPath

class AutoCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self.robot = robot

  def _getPath(self, autoPath: AutoPath) -> PathPlannerPath:
    return constants.Game.Auto.kPaths.get(autoPath)
  
  def _move(self, path: PathPlannerPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      path, constants.Subsystems.Drive.kPathFindingConstraints
    ).withName("AutoCommands:Move")
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(
      self.robot.gameCommands.alignRobotToTargetCommand()
    ).withName("AutoCommands:AlignToTarget")
  
  # ######################################################################
  # ################################ AUTOS ###############################
  # ######################################################################

  def test(self) -> Command:
    return cmd.sequence(
      self._move(self._getPath(AutoPath.Test)),
      self._alignToTarget()
    ).withName("AutoCommands:Test")
