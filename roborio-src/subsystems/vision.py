import os
import wpilib
from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import AprilTagFieldLayout
from configs import robotToLimeLightTransform
from constants import kAprilTagsFilename, kPhotonCameraName

apriltagsLayoutPath = os.path.join(
    wpilib.getOperatingDirectory(), kAprilTagsFilename
)

class VisionSystem(Subsystem):
    def __init__(self):
        self.estimator = PhotonPoseEstimator(
            AprilTagFieldLayout(apriltagsLayoutPath),
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            PhotonCamera(kPhotonCameraName),
            robotToLimeLightTransform
        )