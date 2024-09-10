from wpilib import SmartDashboard
from wpimath.geometry import Transform3d
from robotpy_apriltag import AprilTagFieldLayout
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy, EstimatedRobotPose
from lib import utils

class PoseSensor:
  def __init__(
      self, 
      cameraName: str,
      cameraTransform: Transform3d,
      poseStrategy: PoseStrategy,
      fallbackPoseStrategy: PoseStrategy,
      aprilTagFieldLayout: AprilTagFieldLayout
    ) -> None:
    self._cameraName = cameraName
    self._baseKey = f'Robot/Sensor/Pose/{self._cameraName}'
    self._photonCamera = PhotonCamera(cameraName)
    self._photonCamera.setDriverMode(False)
    self._photonPoseEstimator = PhotonPoseEstimator(aprilTagFieldLayout, poseStrategy, self._photonCamera, cameraTransform)
    self._photonPoseEstimator.multiTagFallbackStrategy = fallbackPoseStrategy
    self._hasTarget = False
    self._targetCount: int = 0

    utils.addRobotPeriodic(self._updateTelemetry)

  def getEstimatedRobotPose(self) -> EstimatedRobotPose | None:
    if self._photonCamera.isConnected():
      photonPipelineResult = self._photonCamera.getLatestResult()
      self._hasTarget = photonPipelineResult.hasTargets()
      self._targetCount = len(photonPipelineResult.getTargets()) if self._hasTarget else 0
      return self._photonPoseEstimator.update(photonPipelineResult)
    self._hasTarget = False
    return None
  
  def hasTarget(self) -> bool:
    return self._hasTarget
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean(f'{self._baseKey}/IsConnected', self._photonCamera.isConnected())
    SmartDashboard.putBoolean(f'{self._baseKey}/HasTarget', self._hasTarget)
    SmartDashboard.putNumber(f'{self._baseKey}/TargetCount', self._targetCount)