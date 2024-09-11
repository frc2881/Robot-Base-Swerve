from typing import TYPE_CHECKING
from commands2 import Command, cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
if TYPE_CHECKING: from robot_container import RobotContainer
import constants

class AutoCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

    self._paths: dict[str, PathPlannerPath] = {
      "Test0": PathPlannerPath.fromPathFile("Test0"),
      "Test2": PathPlannerPath.fromPathFile("Test2")
    }

    self._robot.addAutoCommand("[ 0 ] Test", lambda: self.auto_0_test())
    self._robot.addAutoCommand("[ 2 ] Test", lambda: self.auto_2_test())

  def _move(self, path: PathPlannerPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      path, constants.Subsystems.Drive.kPathFindingConstraints
    ).withName("AutoCommands:Move")
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(
      self._robot.gameCommands.alignRobotToTargetCommand()
    ).withName("AutoCommands:AlignToTarget")
  
  # ######################################################################
  # ################################ AUTOS ###############################
  # ######################################################################

  def auto_0_test(self) -> Command:
    return cmd.sequence(
      self._move(self._paths.get("Test0"))
    ).withName("AutoCommands:0_Test")

  def auto_2_test(self) -> Command:
    return cmd.sequence(
      self._move(self._paths.get("Test2")),
      self._alignToTarget()
    ).withName("AutoCommands:2_Test")
  