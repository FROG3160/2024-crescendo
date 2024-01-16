# High level objects that control our drivetrain
from FROGlib.swerve import SwerveChassis, SwerveModule
from commands2.subsystem import Subsystem
from constants import kSwerveDriveGearing
from FROGlib.sensors import FROGGyro
from constants import kMaxChassisRadiansPerSec, kMaxChassisRadiansPerSec

class DriveTrain(SwerveChassis, Subsystem):
    def __init__(self):
        SwerveChassis.__init__(
            modules=(
                SwerveModule('FrontLeft'),
                SwerveModule('FrontRight'),
                SwerveModule('BackLeft'),
                SwerveModule('BackRight')
            ),
            gyro=FROGGyro(),
            max_speed=self.max_speed,
            max_rotation_speed = kMaxChassisRadiansPerSec
        )
        Subsystem.__init__()