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
kLeadScrewControllerID = 51
kFlyWheelControllerLeftID = 52
kFlyWheelCOntrollerRightID = 53
kSequencerControllerID = 54

# Shooter Motor Gains
# TODO: Tune these gains
kLeadScrewSlot0P = 0.5 # slot 0 for position
kLeadScrewSlot1S = 0.14 # slot 1 for velocity
kLeadScrewSlot1V = 0.42 # slot 1 for velocity
kFlywheelVoltageS = 0.34
kFlywheelVoltageV = 0.11

# Falcon 500 max rps
kFalconMaxRps = 106

# Lead Screw Rotations
kLeadScrewRotations = 21

# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(21.5)
kWheelBaseMeters = inchesToMeters(21.5)
kWheelDiameter = 0.1000125 # 3 15/16 inches in meters
kSwerveDriveGearing = [(14 / 50), (25 / 19), (15 / 45)]
# kDriveBaseRadius is the distance from the center of the robot
# to the farthest module. This is needed for the construction 
# of Autobuilder in the drivetrain init method.
kDriveBaseRadius = math.sqrt(((kTrackWidthMeters / 2) ** 2) + ((kWheelBaseMeters / 2) ** 2))

# Swerve Drive Performance
kMaxMetersPerSecond = feetToMeters(13) #max fps for L1=13.5, L2=16.3, L3=18
kMaxChassisRadiansPerSec = 2 * math.tau # revolutions per sec * tau

# Swerve Drive Trajectory Constraints
kMaxTrajectorySpeed = feetToMeters(5) # Unit: m/s, around 1.524 m/s, same value used 2023
kMaxTrajectoryAccel = feetToMeters(5) # Unit: m/s/s, around 1.524 m/s/s, same value used 2023

# Xbox controller ports
kDriverControllerPort = 0
kOperatorControllerPort = 1

#Xbox controller constants
kDeadband = 0.15
kDebouncePeriod = 0.5
kTranslationSlew = 0.5
kRotSlew = 0.5

kLimelightGrabber = 'limelight'
kLimelightUpper = 'limelight-at'

kPhotonCameraName = 'FROGLimelight'

kProfiledMaxVelocity = math.pi*8
kProfiledMaxAccel = math.pi*4

kProfiledP = 0.4
kProfiledI = 0.0
kProfiledD = 0.0

kRollerSpeed = -0.5
kTransferSpeed = 0.5
kSequencerSpeed = 0.3

kSpeakerPosition = Pose3d(0, 5.554, 2.07, Rotation3d(0, 0, 0))