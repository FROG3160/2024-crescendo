# High level objects that control our drivetrain
from FROGlib.swerve import SwerveChassis, SwerveModule
from FROGlib.sensors import FROGGyro
from constants import kMaxChassisRadiansPerSec, kMaxMetersPerSecond
import configs

class DriveTrain(SwerveChassis):
    def __init__(self):
        SwerveChassis.__init__(
            modules=(
                SwerveModule(**configs.swerveModuleFrontLeft),
                SwerveModule(**configs.swerveModuleFrontRight),
                SwerveModule(**configs.swerveModuleBackLeft),
                SwerveModule(**configs.swerveModuleBackRight)
            ),
            gyro=FROGGyro(),
            max_speed=kMaxMetersPerSecond,
            max_rotation_speed = kMaxChassisRadiansPerSec
        )
