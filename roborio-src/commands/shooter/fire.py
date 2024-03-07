from commands2 import SequentialCommandGroup, ParallelCommandGroup, StartEndCommand
from commands2.cmd import runOnce, startEnd, waitUntil, waitSeconds
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem
from subsystems.elevation import ElevationSubsystem


class Fire(SequentialCommandGroup):

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
            self.shooter.runFlywheelsCommand(),
            waitUntil(self.readyToFire),
            waitUntil(self.elevation.leadscrewAtPosition),
            self.shooter.fireSequencerCommand(),
            waitUntil(self.shooter.shooterSensor.get),
            self.shooter.stopSequencerCommand(),
            waitSeconds(0.5),
            runOnce(self.intake.allowIntake, self.intake),
            self.shooter.stopFlywheelsCommand(),
            self.elevation.moveToLoadPositionCommand(),
        ]
        super().__init__(command_list)

    def readyToFire(self):
        return self.elevation.leadscrewAtPosition() and self.shooter.flywheelAtSpeed()


# self.runFlywheelsCommand()  # run the flywheel at the commanded speed
#             .andThen(waitUntil(self.flywheelAtSpeed))
#             .andThen(
#                 self.fireSequencerCommand()
#             )  # run the sequencer to move the note into the flywheel
#             .andThen(
#                 waitUntil(self.shooterSensor.get)
#             )  # wait until we no longer detect the note
#             .andThen(
#                 self.stopSequencerCommand()
#             )  # could also call stopShootingCommand at the very end
#             .andThen(waitSeconds(1))  # wait 1 second
#             .andThen(self.stopFlywheelsCommand())
