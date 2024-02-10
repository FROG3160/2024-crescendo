from commands2 import Subsystem
from wpimath.geometry import Pose3d
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from photonlibpy.photonTrackedTarget import PhotonTrackedTarget
from FROGlib.limelight import FROGLLField
from robotpy_apriltag import loadAprilTagLayoutField, AprilTagField
from configs import robotToLimeLightTransform
from constants import kPhotonCameraName
from wpilib import SmartDashboard


class VisionSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.estimator = PhotonPoseEstimator(
            loadAprilTagLayoutField(AprilTagField.k2024Crescendo),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PhotonCamera(kPhotonCameraName),
            robotToLimeLightTransform,
        )

    def periodic(self) -> None:
        self.latestPose = self.estimator.update()
        if self.latestPose:
            SmartDashboard.putString('Vision Estimate', self.latestPose.estimatedPose.__str__())

    def getLatestPoseEstimate(self) -> tuple[Pose3d, float]:
        if self.latestPose:
            return (self.latestPose.estimatedPose, self.latestPose.timestampSeconds)

    def getTargetsUsed(self) -> list[PhotonTrackedTarget]:
        if self.latestPose:
            return self.latestPose.targetsUsed
        

class AprilTagSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.estimator = FROGLLField('limelight-field')

    def periodic(self) -> None:
        self.latestPose = self.estimator.getBotPoseEstimateBlue()
        if self.latestPose:
            SmartDashboard.putString('Vision Estimate', self.latestPose[0].__str__())
        self.latestTransform = self.estimator.getTargetTransform()
        if self.latestTransform:
            SmartDashboard.putString('Target Transform', self.latestTransform.__str__())
    
    def getLatestPoseEstimate(self) -> tuple[Pose3d, float]:
        if self.latestPose:
            return (self.latestPose[0], self.latestPose[1])