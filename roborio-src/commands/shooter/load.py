from commands2 import SequentialCommandGroup, ParallelCommandGroup, StartEndCommand
from commands2.cmd import runOnce, startEnd, waitUntil
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem
from subsystems.elevation import ElevationSubsystem


# class LoadShooterCommand(StartEndCommand):
#     def __init__(self,
#                  shooter,
#                  intake):
#         self.shooter = shooter
#         self.intake = intake
# loadShooterCommand = startEnd(self.shooter.loadWithSequencer, self.shooter.stopSequencer, self.shooter
#     )


def loadShooterCommand(
    shooter: ShooterSubsystem, intake: IntakeSubsystem
) -> StartEndCommand:
    return (
        StartEndCommand(shooter.loadWithSequencer, shooter.stopSequencer, shooter)
        .until(shooter.noteInShooter)
        .withName("RunSequencer")
        .deadlineWith(intake.transferCommand())
        .andThen(shooter.homeNoteCommand())
    )


class IntakeAndLoad(SequentialCommandGroup):

    def __init__(
        self,
        intake: IntakeSubsystem,
        shooter: ShooterSubsystem,
        elevation: ElevationSubsystem,
    ):
        self.intake = intake
        self.shooter = shooter
        self.elevation = elevation
        command_list = [
            # we'll start with running the intake.  This command will be triggered
            # by another event, such as seeing a note that will require this
            # first
            self.intake.intakeCommand(),
            runOnce(self.intake.disallowIntake, self.intake),
            runOnce(self.elevation.moveToLoadPosition),
            waitUntil(self.elevation.readyToLoad),
            loadShooterCommand(self.shooter, self.intake),
        ]
        super().__init__(command_list)


moveShooter = ParallelCommandGroup()
