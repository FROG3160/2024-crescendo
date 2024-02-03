# High level objects that control our drivetrain
import math
from FROGlib.swerve import SwerveChassis, SwerveModule
from FROGlib.sensors import FROGGyro
from constants import kMaxChassisRadiansPerSec, kMaxMetersPerSecond, kDriveBaseRadius, kSteerP, kSteerI
import configs
# Objects needed for Auto setup (AutoBuilder)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants
from wpilib import DriverStation
from wpimath.geometry import Pose2d
from subsystems.vision import VisionSubsystem
from wpilib import SmartDashboard


class DriveTrain(SwerveChassis):
    def __init__(self, vision: VisionSubsystem):
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
        self.vision = vision
        
        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getPose, # Robot pose supplier
            self.resetPose, # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds, # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.setChassisSpeeds, # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig( # HolonomicPathFollowerConfig, this should likely live in your Constants class
                configs.holonomicTranslationPID, # Translation PID constants
                configs.holonomicTranslationPID, # Rotation PID constants
                kMaxMetersPerSecond, # Max module speed, in m/s
                kDriveBaseRadius, # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig() # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath, # Supplier to control path flipping based on alliance color
            self # Reference to this subsystem to set requirements
        )

    def setFieldPosition(self, pose: Pose2d):
        self.estimator.resetPosition(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions()),
            pose,
        )

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed
    
    def periodic(self):
        self.estimatorPose = self.estimator.update(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions())
        )
        if visionEstimate := self.vision.getLatestPoseEstimate():
            self.estimator.addVisionMeasurement(
                visionEstimate[0].toPose2d(),
                visionEstimate[1], 
                (0.3, 0.3, math.pi/2)
            )
        SmartDashboard.putString(self.estimator.getEstimatedPosition().__str__())

        #run periodic method of the superclass, in this case SwerveChassis.periodic()
        super().periodic()