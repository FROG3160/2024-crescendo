# High level objects that control our drivetrain
from FROGlib.swerve import SwerveChassis, SwerveModule
from FROGlib.sensors import FROGGyro
from constants import kMaxChassisRadiansPerSec, kMaxMetersPerSecond, kDriveBaseRadius, kSteerP, kSteerI
import configs
# Objects needed for Auto setup (AutoBuilder)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation

class DriveTrain(SwerveChassis):
    def __init__(self):
        super().__init__(
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
        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.robotOrientedDrive, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                PIDConstants(0.0, 0.0, 0.0), # Translation PID constants
                #ISSUE #37 Replace filler steer PID constants with the calculated ones
                PIDConstants(kSteerP, kSteerI, 0.0), # Rotation PID constants
                kMaxMetersPerSecond, # Max module speed, in m/s
                kDriveBaseRadius, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed