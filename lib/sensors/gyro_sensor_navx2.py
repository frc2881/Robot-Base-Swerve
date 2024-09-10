from commands2 import Command, cmd
from wpilib import SerialPort, SmartDashboard, RobotBase
from wpimath import units
from wpimath.geometry import Rotation2d, Pose2d
import navx
from lib import utils

class GyroSensor_NAVX2():
  def __init__(
      self,
      serialPort: SerialPort.Port
    ) -> None:
    self._gyro = navx.AHRS(serialPort)

    self._baseKey = f'Robot/Sensor/Gyro'

    utils.addRobotPeriodic(self._updateTelemetry)
  
  def getHeading(self) -> units.degrees:
    return -utils.wrapAngle(self._gyro.getAngle())
  
  def getRotation(self) -> Rotation2d:
    return Rotation2d.fromDegrees(self.getHeading())
  
  def getPitch(self) -> units.degrees:
    return self._gyro.getPitch()
  
  def getRoll(self) -> units.degrees:
    return self._gyro.getRoll()
  
  def getTurnRate(self) -> units.degrees_per_second:
    return self._gyro.getRate()
  
  def alignRobotToField(self, robotPose: Pose2d) -> None:
    self._reset(utils.wrapAngle(robotPose.rotation().degrees() + utils.getValueForAlliance(0.0, 180.0)))

  def _reset(self, heading: units.degrees) -> None:
    self._gyro.reset()
    self._gyro.setAngleAdjustment(-heading)

  def resetCommand(self) -> Command:
    return cmd.runOnce(
      lambda: self._reset(0)
    ).ignoringDisable(True).withName("GyroSensor:Reset")
  
  def _calibrate(self) -> None:
    if RobotBase.isReal():
      self._gyro.calibrate() # NO-OP as navX2 currently does automatic calibration

  def calibrateCommand(self) -> Command:
    return cmd.sequence(
      cmd.runOnce(
        lambda: [
          SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', True),
          self._calibrate()
        ]
      ),
      cmd.waitSeconds(0.2),
      cmd.runOnce(
        lambda: SmartDashboard.putBoolean(f'{self._baseKey}/IsCalibrating', False)
      )
    ).ignoringDisable(True).withName("GyroSensor:Calibrate")
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Heading', self.getHeading())
    SmartDashboard.putNumber(f'{self._baseKey}/Pitch', self.getPitch())
    SmartDashboard.putNumber(f'{self._baseKey}/Roll', self.getRoll())
    SmartDashboard.putNumber(f'{self._baseKey}/TurnRate', self.getTurnRate())