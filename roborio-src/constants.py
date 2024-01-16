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
from wpimath.units import feetToMeters, inchesToMeters

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

# Swerve Drive Physical Attributes
kTrackWidthMeters = inchesToMeters(21.5)
kWheelBaseMeters = inchesToMeters(21.5)
kWheelDiameter = 0.1000125 # 3 15/16 inches in meters
kSwerveDriveGearing = [(14 / 50), (25 / 19), (15 / 45)]

# Swerve Drive Performance
kMaxMetersPerSecond = feetToMeters(12) #max fps for L1=13.5, L2=16.3, L3=18
# kMinMetersPerSecond = 0.55 #TODO:  Determine if this is needed
kMaxChassisRadiansPerSec = 2 * math.tau # revolutions per sec * tau
kVoltageCompensation = 10.5

# Swerve Drive Trajectory Constraints
kMaxTrajectorySpeed = feetToMeters(5) # Unit: m/s, around 1.524 m/s, same value used 2023
kMaxTrajectoryAccel = feetToMeters(5) # Unit: m/s/s, around 1.524 m/s/s, same value used 2023

# Xbox controller ports
kDriverControllerPort = 0
kOperatorControllerPort = 1

#Xbox controller constants
kDeadband = 0.15
kDebouncePeriod = 0.5
kTranslationSlew = 1.5 
kRotSlew = 1.5