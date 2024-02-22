from commands2 import Subsystem
from wpimath.geometry import Pose3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from FROGlib.limelight import FROGPositioning, FROGTargeting
from robotpy_apriltag import loadAprilTagLayoutField, AprilTagField
from configs import robotToLimeLightTransform
from constants import kPhotonCameraName
from wpilib import SmartDashboard


class PositioningSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.estimator = FROGPositioning("limelight-field")

    def periodic(self) -> None:
        self.latestPose = self.estimator.getBotPoseEstimateBlue()
        if self.latestPose:
            SmartDashboard.putString("Vision Estimate", self.latestPose[0].__str__())
        self.latestTransform = self.estimator.getTargetTransform()
        if self.latestTransform:
            SmartDashboard.putString("Target Transform", self.latestTransform.__str__())

    def getLatestPoseEstimate(self) -> tuple[Pose3d, float]:
        if self.latestPose:
            return (self.latestPose[0], self.latestPose[1])


class TargetingSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.camera = FROGTargeting

    def periodic(self) -> None:
        self.target = self.camera.getTarget()
