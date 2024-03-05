#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#


# This is where we hold the basic settings/configuration of our robot.
# This should only hold numerical, string, or boolean constants, using
# the C++ naming convention.

import math
import wpilib
from wpimath.geometry import Pose3d, Rotation3d
from wpimath.units import feetToMeters, inchesToMeters

# CANCoder offsets
kFrontLeftOffset = -0.246338
kFrontRightOffset = 0.036377
kBackLeftOffset = 0.478027
kBackRightOffset = 0.207031

# steer motor gains
kSteerP = 2.402346
kSteerI = 0.200195

# drive motor gains
kDriveFeedForward = 0.53
kDutyCycleDriveV = 0.00916
kDutyCycleDriveS = 0.01125
kVoltageDriveV = 0.11
kVoltageDriveS = 0.135

# Swerve Drive Motor/Encoder IDs
kFrontLeftDriveID = 11
kFrontLeftSteerID = 21
kFrontLeftSensorID = 31
kFrontRightDriveID = 12
kFrontRightSteerID = 22
kFrontRightSensorID = 32
kBackLeftDriveID = 13
kBackLeftSteerID = 23
kBackLeftSensorID = 33
kBackRightDriveID = 14
kBackRightSteerID = 24
kBackRightSensorID = 34

# Intake Controller IDs
kIntakeRollerControllerID = 41
kTransferWheelsID = 42

# Shooter Controller IDs
kLeadscrewControllerID = 53
kFlyWheelControllerLeftID = 51
kFlyWheelCOntrollerRightID = 52
kSequencerControllerID = 54

# IR Sensor DIO Channel
kIntakeSensorChannel = 0
kShooterSensorChannel = 1
kShooterPositionSensorChannel = 2

# Shooter Motor Gains
# TODO: Tune these gains
kLeadscrewSlot0P = 0.5  # slot 0 for position
kLeadscrewSlot1S = 0.3  # slot 1 for velocity
kLeadscrewSlot1V = 0.5  # slot 1 for velocity
kLeadscrewSlot1P = 32
kLeadscrewSlot1A = 0.01
kLeadscrewMMA = 40
kLeadscrewMMV = 20

kClimberSlot0S = 0.2
kClimberSlot0V = 0.105
kClimberSlot0A = 0.01
kClimberSlot0P = 2
kClimberMMV = 25
kClimberMMA = 50

kleftFlywheelVoltageS = 0.14
kleftFlywheelVoltageV = 0.11
kleftFlywheelVoltageP = 0.16
kLeftFlywheelVoltageA = 0.01

kRightFlywheelVoltageS = 0.16
kRightFlywheelVoltageV = 0.1075
kRightFlywheelVoltageP = 0.16
kRightFlywheelVoltageA = 0.01

kFlywheelMMA = 400
kFlywheelMMV = 100

# Climber Motor IDs
kLeftClimberControllerID = 7
kRightClimberControllerID = 8

# Falcon 500 max rps
kFalconMaxRps = 106

# Lead Screw Rotations
kLeadscrewRotations = 21
kLeadscrewPositionTolerance = 0.01

# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(21.5)
kCenterToFrontModulesMeters = inchesToMeters(8.75)
kBackModulesToCenterMeters = inchesToMeters(12.75)
kWheelBaseMeters = kCenterToFrontModulesMeters + kBackModulesToCenterMeters
kWheelDiameter = 0.1000125  # 3 15/16 inches in meters
kSwerveDriveGearing = [(14 / 50), (25 / 19), (15 / 45)]
# kDriveBaseRadius is the distance from the center of the robot
# to the farthest module. This is needed for the construction
# of Autobuilder in the drivetrain init method.
kDriveBaseRadius = math.sqrt(
    ((kTrackWidthMeters / 2) ** 2) + ((kBackModulesToCenterMeters) ** 2)
)

# Swerve Drive Performance
kMaxMetersPerSecond = feetToMeters(13)  # max fps for L1=13.5, L2=16.3, L3=18
kMaxChassisRadiansPerSec = 2 * math.tau  # revolutions per sec * tau

# Swerve Drive Trajectory Constraints
kMaxTrajectorySpeed = feetToMeters(
    5
)  # Unit: m/s, around 1.524 m/s, same value used 2023
kMaxTrajectoryAccel = feetToMeters(
    5
)  # Unit: m/s/s, around 1.524 m/s/s, same value used 2023

# Xbox controller ports
kDriverControllerPort = 0
kOperatorControllerPort = 1

# Xbox controller constants
kDeadband = 0.15
kDebouncePeriod = 0.5
kTranslationSlew = 2
kRotSlew = 2

kLimelightTargeting = "limelight-targets"
kLimelightPositioning = "limelight-field"

kProfiledMaxVelocity = math.pi * 8
kProfiledMaxAccel = math.pi * 4

kProfiledP = 0.4
kProfiledI = 0.0
kProfiledD = 0.0

kRollerVoltage = 3
kRollerTransferVoltage = 1
kTransferPercent = 0.75
kSequencerShootPercent = 0.75
kSequencerTransferPercent = 0.75
kSequencerHomeReversePercent = 0.2
kSequencerHomeForwardPercent = 0.1

kSpeakerPosition = Pose3d(0, 5.554, 2.07, Rotation3d(0, 0, 0))
