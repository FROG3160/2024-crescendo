import os
import wpilib
from commands2 import Subsystem
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
from robotpy_apriltag import loadAprilTagLayoutField, AprilTagField
from configs import robotToLimeLightTransform
from constants import kPhotonCameraName

class VisionSystem(Subsystem):
    def __init__(self):
        self.estimator = PhotonPoseEstimator(
            loadAprilTagLayoutField(AprilTagField.k2024Crescendo),
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            PhotonCamera(kPhotonCameraName),
            robotToLimeLightTransform
        )