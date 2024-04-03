from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Pose2d
from wpimath.kinematics import ChassisSpeeds
from FROGlib.limelight import FROGPositioning, FROGTargeting, BotPoseResult
from constants import (
    kLimelightPositioning,
    kLimelightTargeting,
    kTargetSizeThreshold,
    kMaxChassisRadiansPerSec,
    kMaxMetersPerSecond,
)
from wpilib import SmartDashboard
from commands2.button import Trigger
import math
from wpimath.filter import MedianFilter


class PositioningSubsystem(Subsystem):
    def __init__(
        self,
        parent_nt: str = "Subsystems",
    ):
        super().__init__()
        self.setName("VisionPose")
        nt_table = f"{parent_nt}/{self.getName()}"
        self.estimator = FROGPositioning(
            kLimelightPositioning,
        )
        self._visionPosePub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/pose2d", Pose2d)
            .publish()
        )
        self.validAprilTagFoundCount = 0
        self.readyToInitializePose = False

    def periodic(self) -> None:
        self.latestData = self.estimator.getBotPoseEstimateBlue()
        # self.latestTransform = self.estimator.getTargetTransform()

        if self.latestData.tagCount > 0:
            SmartDashboard.putString(
                "Vision Estimate", self.latestData.botPose.__str__()
            )
            self._visionPosePub.set(self.latestData.botPose.toPose2d())
            if (
                self.latestData.tagData[0].distanceToCamera < 4
                and not self.readyToInitializePose
            ):
                self.validAprilTagFoundCount += 1
                if self.validAprilTagFoundCount > 25:
                    self.readyToInitializePose = True

        # if self.latestTransform:
        #     SmartDashboard.putString("Target Transform", self.latestTransform.__str__())

    def getLatestData(self) -> BotPoseResult:
        return self.latestData


class TargetingSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.camera = FROGTargeting(kLimelightTargeting)
        self.filterVX = MedianFilter(9)  # MedianFilter(5)
        self.filterVT = MedianFilter(
            9
        )  # Apparently have to create new filter objects for every input stream.

    def getTargetInRange(self):
        """Returns true if ta is more than 18"""

        return float(self.camera.ta or 0) > kTargetSizeThreshold

    def hasSeenTarget(self):
        """Returns true if tv is 1 (there is a valid target)"""
        return self.camera.hasObjectTarget()

    def calculate_vx(self):
        """Calculate X robot-oriented speed from the Y value of the target in the camera frame.

        Args:
            targetY (Float):  The target Y value determined by limelight.

        Returns:
            Float: Velocity (m/s) in the X direction (robot oriented)
        """
        if ty := self.camera.ty:
            return self.filterVX.calculate(
                min(
                    2.0, ((0.45 * math.log(ty + 11.5))) + 0.4
                )  # (0.020833 * (14.7 * math.exp(0.0753 * ty) * 2))
            )
        else:
            return self.filterVX.calculate(0)

    def calculate_vt(self):
        """Calculate the rotational speed from the X value of the target in the camera frame.
        Return is inverted to make left rotation positive from a negative X value, meaning the
        target is to the left of center of the camera's view.

        Args:
            targetX (Float): The X value of the target in the camera frame, 0 is straight ahead,
            to the left is negative, to the right is positive.

        Returns:
            Float: Rotational velocity (radians/sec) with CCW (left, robot oriented) positive.
        """
        if tx := self.camera.tx:
            return self.filterVT.calculate(-(tx / 25))
        else:
            return self.filterVT.calculate(0)

    def getChassisSpeeds(self):
        """Get calculated velocities from vision target data"""
        return ChassisSpeeds(self.calculate_vx(), 0, self.calculate_vt())

    def periodic(self) -> None:
        self.camera.getTarget()
        SmartDashboard.putBoolean("Note In Range", self.getTargetInRange())
