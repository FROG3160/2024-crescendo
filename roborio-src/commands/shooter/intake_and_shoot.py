from commands2 import SequentialCommandGroup, ParallelCommandGroup
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem

intakeAndLoad = SequentialCommandGroup(
    [
        # we'll start with running the intake.  This command will be triggered
        # by another even, such as seeing a note that will require this
        # first
        IntakeSubsystem.intakeCommand(),
        ShooterSubsystem.loadShooterCommand(),
    ]
)

moveShooter = ParallelCommandGroup()
