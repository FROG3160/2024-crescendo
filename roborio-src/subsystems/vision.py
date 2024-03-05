from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Pose2d
from wpimath.kinematics import ChassisSpeeds
from FROGlib.limelight import FROGPositioning, FROGTargeting
from robotpy_apriltag import loadAprilTagLayoutField, AprilTagField
from constants import kLimelightPositioning, kLimelightTargeting
from wpilib import SmartDashboard
from FROGlib.utils import ShootingSolution


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
        self.fieldLayout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

    def periodic(self) -> None:
        self.latestPose = self.estimator.getBotPoseEstimateBlue()
        # self.latestTransform = self.estimator.getTargetTransform()
        pose, latency = self.latestPose
        if latency != -1:
            SmartDashboard.putString("Vision Estimate", pose.__str__())
            self._visionPosePub.set(pose.toPose2d())
            tagrange, azimuth = self.getRangeAzimuth(8)
            SmartDashboard.putNumber("Calculated Range", tagrange)
            SmartDashboard.putNumber("Calculated Azimuth", azimuth)

        # if self.latestTransform:
        #     SmartDashboard.putString("Target Transform", self.latestTransform.__str__())

    def getLatestPoseEstimate(self) -> tuple[Pose3d, float]:
        if self.latestPose:
            return (self.latestPose[0], self.latestPose[1])

    def getRangeAzimuth(self, tag):
        tagPose = self.fieldLayout.getTagPose(tag)
        transform = tagPose - self.latestPose[0]
        solution = ShootingSolution(transform.translation())
        return solution.calculateRange(), solution.calculateAzimuth()


class TargetingSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.camera = FROGTargeting(kLimelightTargeting)

    def getTargetInRange(self):
        """Returns true if ta is more than 18"""
        return float(self.camera.ta or 0) > 18.0

    def calculate_vx(self):
        """Calculate X robot-oriented speed from the size of the target.  Return is inverted
        since we need the robot to drive backwards toward the target to pick it up.

        Args:
            targetArea (Float):  The target area determined by limelight.

        Returns:
            Float: Velocity in the X direction (robot oriented)
        """
        if targetVertical := self.camera.ty:
            return min(-0.20, -(targetVertical * -0.0811 + 0.7432))

    def calculate_vt(self):
        """Calculate the rotational speed from the X value of the target in the camera frame.
        Return is inverted to make left rotation positive from a negative X value, meaning the
        target is to the left of center of the camera's view.

        Args:
            targetX (Float): The X value of the target in the camera frame, 0 is straight ahead,
            to the left is negative, to the right is positive.

        Returns:
            Float: Rotational velocity with CCW (left, robot oriented) positive.
        """
        if targetX := self.camera.tx:
            return -(targetX / 25)

    def getChassisSpeeds(self):
        """Get calculated velocities from vision target data"""
        return ChassisSpeeds(self.calculate_vx, 0, self.calculate_vt)

    def periodic(self) -> None:
        self.camera.getTarget()
        SmartDashboard.putBoolean("TargetInRange", self.getTargetInRange())
