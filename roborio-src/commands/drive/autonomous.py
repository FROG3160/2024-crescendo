from commands2 import Command
from subsystems.drivetrain import DriveTrain


class RotateInPlaceTowardsSpeaker(Command):

    def __init__(self, drive: DriveTrain, table: str = "Undefined") -> None:
        """Allows manual control of the lateral movement of the drivetrain through use of the specified
        controller.  Rotation is calculated to aim the shooter at the speaker.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.drive = drive
        self.addRequirements(self.drive)
        self.firstCall = True

    def execute(self) -> None:
        if self.firstCall:
            # this is the first time we hit this conditional, so
            # reset the controller
            self.firstCall = False
            self.drive.resetRotationController()

        # we need the change in rotation to point at the speaker
        robotRelativeFiringHeading = self.drive.getFiringHeadingForSpeaker()
        currentRotation = self.drive.getRotation2d()

        vT = self.drive.profiledRotationController.calculate(
            currentRotation.radians(),
            (currentRotation + robotRelativeFiringHeading).radians(),
        )

        self.drive.fieldOrientedAutoRotateDrive(
            # self._vX, self._vY, self._vT, self._throttle
            0,
            0,
            vT,
        )

    def isFinished(self):
        return self.drive.profiledRotationController.atGoal()
