# High level objects that control our drivetrain
import math

from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField
from FROGlib.swerve import SwerveChassis, SwerveModule
from FROGlib.sensors import FROGGyro
from constants import (
    kMaxChassisRadiansPerSec,
    kMaxMetersPerSecond,
    kDriveBaseRadius,
    kSteerP,
    kSteerI,
)
import configs

# Objects needed for Auto setup (AutoBuilder)
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)
from wpilib import DriverStation
from wpimath.geometry import Pose2d
from subsystems.vision import PositioningSubsystem
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from FROGlib.utils import RobotRelativeTarget


class DriveTrain(SwerveChassis):
    def __init__(self, vision: PositioningSubsystem, parent_nt: str = "Subsystems"):
        super().__init__(
            swerve_module_configs=(
                configs.swerveModuleFrontLeft,
                configs.swerveModuleFrontRight,
                configs.swerveModuleBackLeft,
                configs.swerveModuleBackRight,
            ),
            # modules=(
            #     SwerveModule(**configs.s,werveModuleFrontLeft),
            #     SwerveModule(**configs.swerveModuleFrontRight),
            #     SwerveModule(**configs.swerveModuleBackLeft),
            #     SwerveModule(**configs.swerveModuleBackRight),
            # ),
            gyro=FROGGyro(),
            max_speed=kMaxMetersPerSecond,
            max_rotation_speed=kMaxChassisRadiansPerSec,
            parent_nt=parent_nt,
        )
        self.vision = vision
        self.fieldLayout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getAutoPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.setChassisSpeeds,  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                configs.holonomicTranslationPID,  # Translation PID constants
                configs.holonomicTranslationPID,  # Rotation PID constants
                self.max_speed,  # Max module speed, in m/s
                kDriveBaseRadius,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig(),  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self,  # Reference to this subsystem to set requirements
        )
        self.isBlueAlliance = not self.shouldFlipPath()

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def getSpeakerTagNum(self):
        # should return 7 is false and 4 if True
        return [7, 4][self.shouldFlipPath()]

    def setFieldPosition(self, pose: Pose2d):
        self.estimator.resetPosition(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions()),
            pose,
        )

    def resetGyroCommand(self) -> Command:
        return self.runOnce(self.gyro.resetGyro)

    def getTargeting(self):
        tagPose = self.fieldLayout.getTagPose(self.getSpeakerTagNum())
        robotToTarget = RobotRelativeTarget(
            self.estimatorPose, tagPose, not self.shouldFlipPath()
        )
        return (
            robotToTarget.distance,
            robotToTarget.firingHeading,
            robotToTarget.driveVT,
        )

    def getvTtoTag(self):
        tagPose = self.fieldLayout.getTagPose(self.getSpeakerTagNum())
        robotToTarget = RobotRelativeTarget(
            self.estimatorPose, tagPose, not self.shouldFlipPath()
        )
        return robotToTarget.driveVT

    def periodic(self):
        self.estimatorPose = self.estimator.update(
            self.gyro.getRotation2d(), tuple(self.getModulePositions())
        )
        visionPose, visionTimestamp = self.vision.getLatestPoseEstimate()
        if visionPose:
            self.estimator.addVisionMeasurement(
                visionPose.toPose2d(), visionTimestamp, (0.3, 0.3, math.pi / 8)
            )
        SmartDashboard.putString(
            "Drive Estimator", self.estimator.getEstimatedPosition().__str__()
        )
        distance, azimuth, vt = self.getTargeting()
        SmartDashboard.putNumber("Calculated Distance", distance)
        SmartDashboard.putNumber("Calculated Firing Heading", azimuth.degrees())
        SmartDashboard.putNumber("Calculated VT", vt)

        # run periodic method of the superclass, in this case SwerveChassis.periodic()
        super().periodic()
