# High level objects that control our drivetrain
import math

from robotpy_apriltag import AprilTagField, loadAprilTagLayoutField
from FROGlib.swerve import SwerveChassis, SwerveModule
from FROGlib.sensors import FROGGyro
from constants import (
    AprilTagPlacement,
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
from pathplannerlib.path import PathPlannerPath, PathConstraints
from wpilib import DriverStation, Field2d
from wpimath.geometry import Pose2d, Rotation2d, Transform2d
from subsystems.vision import PositioningSubsystem
from subsystems.elevation import ElevationSubsystem
from wpilib import SmartDashboard
from commands2 import Subsystem, Command
from FROGlib.utils import RobotRelativeTarget
import constants
from wpimath.units import degreesToRadians


class DriveTrain(SwerveChassis):
    def __init__(
        self,
        vision: PositioningSubsystem,
        elevation: ElevationSubsystem,
        parent_nt: str = "Subsystems",
    ):
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
        # We need data from the vision system
        self.vision = vision
        # We need to send data to the elevation system
        self.elevation = elevation
        self.fieldLayout = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)

        self.estimatorPose = Pose2d(0, 0, Rotation2d(0))

        # Configure the AutoBuilder last
        AutoBuilder.configureHolonomic(
            self.getPose,  # Robot pose supplier
            self.resetPose,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            self.setChassisSpeeds,  # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            HolonomicPathFollowerConfig(  # HolonomicPathFollowerConfig, this should likely live in your Constants class
                configs.holonomicTranslationPID,  # Translation PID constants
                configs.holonomicRotationPID,  # Rotation PID constants
                self.max_speed,  # Max module speed, in m/s
                constants.kDriveBaseRadius,  # Drive base radius in meters. Distance from robot center to furthest module.
                ReplanningConfig(),  # Default path replanning config. See the API for the options here
            ),
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self,  # Reference to this subsystem to set requirements
        )
        self.robotToSpeaker = None
        self.robotToAmp = None
        self.field = Field2d()
        SmartDashboard.putData("DrivePose", self.field)

    def onRedAlliance(self):
        # Returns boolean that equals true if we are on the Red Alliance
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return self.onRedAlliance()

    def getSpeakerTagNum(self):
        # should return 7 is false and 4 if True
        return [AprilTagPlacement.Blue.SPEAKER, AprilTagPlacement.Red.SPEAKER][
            self.onRedAlliance()
        ]

    def getAmpTagNum(self):
        return [AprilTagPlacement.Blue.AMP, AprilTagPlacement.Red.AMP][
            self.onRedAlliance()
        ]

    def getStageTagNum(self):
        # if our X value on the field falls between the blue and red
        # wing lines
        if 5.320792 < self.estimatorPose.x < 11.220196:
            return [
                AprilTagPlacement.Blue.STAGE_CENTER,
                AprilTagPlacement.Red.STAGE_CENTER,
            ][self.onRedAlliance()]
        # if we are inside our wing, determine if we are on the amp or
        # source side of the stage
        elif self.estimatorPose.y > 4.105148:
            return [
                AprilTagPlacement.Blue.STAGE_AMP,
                AprilTagPlacement.Red.STAGE_AMP,
            ][self.onRedAlliance()]
        else:
            return [
                AprilTagPlacement.Blue.STAGE_SOURCE,
                AprilTagPlacement.Red.STAGE_SOURCE,
            ][self.onRedAlliance()]

    def getStagePose(self):
        # get the tag's pose and transform it for the robot position
        # in this case we need the robot facing the opposite direction
        # of the tag and about 0.5 meters in front of it.
        return (
            self.fieldLayout.getTagPose(self.getStageTagNum())
            .toPose2d()
            .transformBy(
                Pose2d(0.5, 0, Rotation2d(math.pi)) - Pose2d(0, 0, Rotation2d(0))
            )
        )

    def getPathToStage(self) -> str:

        if (self.estimatorPose.x > 6 and not self.onRedAlliance()) or (
            self.estimatorPose.x < 11 and self.onRedAlliance()
        ):
            return "Center Stage Approach"
        elif self.estimatorPose.y > 4.105148:
            return "Amp Side Stage Approach"
        else:
            return "Source Side Stage Approach"

    def driveToStageCommand(self) -> Command:
        # return AutoBuilder.pathfindToPose(
        #     self.getStagePose(), self.pathfindingConstraints, 0.0
        # )
        pathname = self.getPathToStage()
        return AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathname)).withName(
            pathname
        )

    def setFieldPositionFromVision(self):
        self.resetPose(self.vision.getLatestPoseEstimate()[0].toPose2d())
        # self.estimator.resetPosition(
        #     self.gyro.getRotation2d(),
        #     tuple(self.getModulePositions()),
        #     pose,
        # )

    # def resetGyroCommand(self) -> Command:
    #     return self.runOnce(self.gyro.resetGyro(self.onRedAlliance())

    def calculateRobotRelativeTargets(self):
        speakerPose = self.fieldLayout.getTagPose(self.getSpeakerTagNum())
        self.robotToSpeaker = RobotRelativeTarget(
            self.estimatorPose, speakerPose, not self.onRedAlliance()
        )
        ampPose = self.fieldLayout.getTagPose(self.getAmpTagNum())
        self.robotToAmp = RobotRelativeTarget(
            self.estimatorPose, ampPose, not self.onRedAlliance()
        )

    def getvTtoTag(self):
        tagPose = self.fieldLayout.getTagPose(self.getSpeakerTagNum())
        robotToTarget = RobotRelativeTarget(
            self.estimatorPose, tagPose, not self.onRedAlliance()
        )
        return robotToTarget.driveVT

    def periodic(self):
        self.estimatorPose = self.estimator.update(
            self.gyro.getRotation2d(), tuple(self.getModulePositions())
        )
        visionPose, visionTimestamp = self.vision.getLatestPoseEstimate()
        if visionPose:
            if (
                abs(visionPose.x - self.estimatorPose.x) < 0.5
                and abs(visionPose.y - self.estimatorPose.y) < 0.5
            ):
                self.estimator.addVisionMeasurement(
                    visionPose.toPose2d(), visionTimestamp, (0.5, 0.5, math.pi / 8)
                )
        self.field.setRobotPose(self.estimator.getEstimatedPosition())
        # SmartDashboard.putValue("Drive Pose", self.estimatorPose)
        # SmartDashboard.putData(
        #     "Drive Pose w/Vision ", self.estimator.getEstimatedPosition()
        # )
        self.calculateRobotRelativeTargets()
        speakerDistance = self.robotToSpeaker.distance
        # update elevation with the needed distance
        self.elevation.setSpeakerDistance(speakerDistance)
        # SmartDashboard.putNumber("Calculated Distance", distance)
        # SmartDashboard.putNumber("Calculated Firing Heading", azimuth.degrees())
        # SmartDashboard.putNumber("Calculated VT", vt)

        # Gyro data
        # SmartDashboard.putNumber("Gyro Angle", self.gyro.getRotation2d().degrees())
        # SmartDashboard.putNumber("Gyro Adjustment", self.gyro.getAngleAdjustment())
        # SmartDashboard.putNumber("RAW Gyro Angle CCW", -self.gyro.gyro.getYaw())

        # run periodic method of the superclass, in this case SwerveChassis.periodic()
        super().periodic()
