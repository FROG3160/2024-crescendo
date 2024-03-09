import math
from commands2 import Command
from subsystems.drivetrain import DriveTrain
from subsystems.vision import TargetingSubsystem
import constants
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ntcore import NetworkTableInstance


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
