from commands2 import Command
from subsystems.elevation import ElevationSubsystem
from subsystems.drivetrain import DriveTrain


class AutoElevation(Command):
    def __init__(
        self,
        elevation: ElevationSubsystem,
        drive: DriveTrain,
    ) -> None:
        self.elevation = elevation
        self.drive = drive
        self.addRequirements(self.elevation)

    def getPositionToSpeaker(self):
        distance, heading, vT = self.drive.getTargeting()
        return distance * 3.9877 - 6.3804

    def execute(self) -> None:
        self.elevation.setLeadscrewPosition(self.getPositionToSpeaker())
        self.elevation.moveLeadscrewToPosition()
