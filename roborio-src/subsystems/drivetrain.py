# High level objects that control our drivetrain
from FROGlib.swerve import SwerveChassis
from commands2.subsystem import Subsystem

class DriveTrain(SwerveChassis, Subsystem):
    def __init__(self):
        SwerveChassis.__init__()
        Subsystem.__init__()