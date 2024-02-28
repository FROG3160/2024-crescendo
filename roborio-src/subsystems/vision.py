from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Pose2d
from wpimath.kinematics import ChassisSpeeds
from FROGlib.limelight import FROGPositioning, FROGTargeting
from robotpy_apriltag import loadAprilTagLayoutField, AprilTagField
from constants import kLimelightPositioning, kLimelightTargeting
from wpilib import SmartDashboard


class PositioningSubsystem(Subsystem):
    def __init__(
        self,
        parent_nt: str = "Subsystems",
    ):
        super().__init__()
        self.setName("LimePositioning")
        nt_table = f"{parent_nt}/{self.getName()}"
        self.estimator = FROGPositioning(
            kLimelightPositioning,
        )
        self._visionPosePub = (
            NetworkTableInstance.getDefault()
            .getStructTopic(f"{nt_table}/command_value", Pose2d)
            .publish()
        )

    def periodic(self) -> None:
        self.latestPose = self.estimator.getBotPoseEstimateBlue()
        if self.latestPose:
            pose, timestamp = self.latestPose
            if pose:
                SmartDashboard.putString("Vision Estimate", pose.__str__())
                self._visionPosePub.set(pose.toPose2d())
        self.latestTransform = self.estimator.getTargetTransform()
        if self.latestTransform:
            SmartDashboard.putString("Target Transform", self.latestTransform.__str__())

    def getLatestPoseEstimate(self) -> tuple[Pose3d, float]:
        if self.latestPose:
            return (self.latestPose[0], self.latestPose[1])


class TargetingSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.camera = FROGTargeting(kLimelightTargeting)

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
