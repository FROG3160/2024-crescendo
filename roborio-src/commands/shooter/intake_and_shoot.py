from commands2 import SequentialCommandGroup, ParallelCommandGroup
from commands2.cmd import runOnce
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem
from subsystems.elevation import ElevationSubsystem


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
            self.shooter.loadShooterCommand(),
        ]
        super().__init__(command_list)


moveShooter = ParallelCommandGroup()
