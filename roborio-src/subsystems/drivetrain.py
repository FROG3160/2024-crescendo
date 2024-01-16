# High level objects that control our drivetrain
from FROGlib.swerve import SwerveChassis, SwerveModule
from FROGlib.sensors import FROGGyro
from constants import kMaxChassisRadiansPerSec, kMaxMetersPerSecond

class DriveTrain(SwerveChassis):
    def __init__(self):
        SwerveChassis.__init__(
            modules=(
                SwerveModule('FrontLeft'),
                SwerveModule('FrontRight'),
                SwerveModule('BackLeft'),
                SwerveModule('BackRight')
            ),
            gyro=FROGGyro(),
            max_speed=kMaxMetersPerSecond,
            max_rotation_speed = kMaxChassisRadiansPerSec
        )
