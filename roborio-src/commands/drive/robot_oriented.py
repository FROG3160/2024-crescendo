import math
from commands2 import Command
from subsystems.drivetrain import DriveTrain
from subsystems.vision import TargetingSubsystem
import constants
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ntcore import NetworkTableInstance
from FROGlib.xbox import FROGXboxDriver
from wpimath.kinematics import ChassisSpeeds


class DriveToTarget(Command):

    def __init__(
        self, drive: DriveTrain, targeting: TargetingSubsystem, table: str = "Undefined"
    ) -> None:
        """Allows robot to drive in a robot oriented towards the target

        Args:
            drive (DriveTrain): The drive to control.
            targeting (TargetingSubsystem): The targeting subsystem that uses a limelight
            to calculate chassis speeds to drive towards the target
            table (str, optional): The name of the network table telemetry data will go into.
        """

        self.drive = drive
        self.targeting = targeting
        self.addRequirements(self.drive)
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vX = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vX")
            .publish()
        )
        self._caluclated_vT = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def execute(self):
        self.drive.robotOrientedDrive(*self.targeting.getChassisSpeeds())
        self._calculated_vX = self.targeting.calculate_vx()
        self._caluclated_vT = self.targeting.calculate_vt()


class ThrottledDriveToTarget(Command):

    def __init__(
        self,
        drive: DriveTrain,
        targeting: TargetingSubsystem,
        drive_controller: FROGXboxDriver,
        table: str = "Undefined",
    ) -> None:
        """Moves the robot using robot-oriented speeds.  The speed forward is controlled
        by the left Y axis and trigger, rotation to the target is calculated from the vision
        system.

        Args:
            drive (DriveTrain): The drive to control.
            targeting (TargetingSubsystem): The targeting subsystem that uses a limelight
            to calculate chassis speeds to drive towards the target
            table (str, optional): The name of the network table telemetry data will go into.
        """

        self.drive = drive
        self.targeting = targeting
        self.controller = drive_controller
        self.addRequirements(self.drive)
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vX = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vX")
            .publish()
        )
        self._calculated_vT = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def execute(self):
        visionChassisSpeeds = self.targeting.getChassisSpeeds()
        controllerVx = (
            self.controller.getSlewLimitedFieldForward()
            * self.controller.getFieldThrottle()
        )

        self.drive.robotOrientedDrive(
            controllerVx * self.drive.max_speed, 0, visionChassisSpeeds.omega
        )
        self._throttle_vX = controllerVx
        self._calculated_vT = self.targeting.calculate_vt()


class ManualRobotOrientedDrive(Command):
    def __init__(
        self, controller: FROGXboxDriver, drive: DriveTrain, table: str = "Undefined"
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.addRequirements(self.drive)

    def execute(self) -> None:
        throttle = self.controller.getFieldThrottle()
        max_speed = self.drive.max_speed

        self.drive.robotOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            self.controller.getSlewLimitedFieldForward() * max_speed * throttle,
            self.controller.getSlewLimitedFieldLeft() * max_speed * throttle,
            self.controller.getSlewLimitedFieldRotation()
            * self.drive.max_rotation_speed
            * throttle,
        )


class FindTargetAndDrive(Command):
    def __init__(
        self, controller: FROGXboxDriver, drive: DriveTrain, table: str = "Undefined"
    ) -> None:
        """Rotates to find a target and drives to it.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.addRequirements(self.drive)

    def execute(self) -> None:
        # set rotation to what is seen with the targeting system,
        # or if no target is found rotate toward the amp side.
        # might also want to make this rotate the other direction if
        # on the source side.
        if self.drive.onRedAlliance():
            vT = -0.25
        else:
            vT = 0.25
        throttle = self.controller.getFieldThrottle()
        max_speed = self.drive.max_speed

        self.drive.robotOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            self.controller.getSlewLimitedFieldForward() * max_speed * throttle,
            self.controller.getSlewLimitedFieldLeft() * max_speed * throttle,
            self.controller.getSlewLimitedFieldRotation()
            * self.drive.max_rotation_speed
            * throttle,
        )
