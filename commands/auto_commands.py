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
      "[0]_Test": PathPlannerPath.fromPathFile("[0]_Test"),
      "[2]_Test": PathPlannerPath.fromPathFile("[2]_Test")
    }

    self._robot.addAutoCommand("[0] Test", self.auto_0_test)
    self._robot.addAutoCommand("[2] Test", self.auto_2_test)
  
  # ######################################################################
  # ################################ AUTOS ###############################
  # ######################################################################

  def auto_0_test(self) -> Command:
    return cmd.sequence(
      self._move(self._paths.get("[0]_Test"))
    ).withName("AutoCommands:[0] Test")

  def auto_2_test(self) -> Command:
    return cmd.sequence(
      self._move(self._paths.get("[2]_Test")),
      self._alignToTarget()
    ).withName("AutoCommands:[2] Test")
  
  # ######################################################################
  # ################################ UTILS ###############################
  # ######################################################################

  def _move(self, path: PathPlannerPath) -> Command:
    return AutoBuilder.pathfindThenFollowPath(
      path, constants.Subsystems.Drive.kPathFindingConstraints
    ).withName("AutoCommands:Move")
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(
      self._robot.gameCommands.alignRobotToTargetCommand()
    ).withName("AutoCommands:AlignToTarget")