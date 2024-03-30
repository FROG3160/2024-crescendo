from FROGlib.led import FROGLED
from commands2 import Command


class LEDSubsystem(FROGLED):
    def __init__(self, canID):
        super().__init__(canID)
        # set initial sequence to show robot isn't
        # set with an initial pose yet
        self.red()

    def ledIntakeCommand(self) -> Command:
        return self.runOnce(lambda: self.orange())

    def ledShooterCommand(self) -> Command:
        return self.runOnce(lambda: self.larsonAnimation(252, 157, 3, 1))

    def ledDefaultCommand(self) -> Command:
        return self.runOnce(lambda: self.default())

    def ledErrorCommand(self) -> Command:
        return self.runOnce(lambda: self.red())
