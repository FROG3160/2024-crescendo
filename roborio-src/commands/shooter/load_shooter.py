from subsystems import intake, shooter, elevation
from commands2 import Command


class LoadShooter(Command):
    def __init__(self, intake, shooter, elevation):
        self.intake = intake
        self.shooter = shooter
        self.elvation = elevation
