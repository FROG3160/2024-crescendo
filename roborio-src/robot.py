#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
from phoenix6 import SignalLogger
import wpilib
import rev
import commands2
import constants
import configs
from commands2.cmd import runOnce
from wpimath.geometry import Pose2d
from wpilib import DataLogManager, DriverStation
from wpilib import RobotController

from robotcontainer import RobotContainer

# Temporary falcon motor control
from phoenix6.controls import VelocityDutyCycle, VelocityVoltage


class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        # Start recording to log
        DataLogManager.start()
        # Set log path for ctre on the first USB drive found
        # TODO: https://github.com/FROG3160/2024-crescendo/issues/162 this doesn't work, check the Roborio to see if USB
        #   is really on /media/sda1
        # SignalLogger.set_path("/media/sda1/ctre-logs/")
        # start SignalLogger if the robot is running it
        if self.isReal():
            SignalLogger.start()

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

        self.startingPose2d = Pose2d(0, 0, 0)

        wpilib.SmartDashboard.putNumber("flyspeed", 8.8)
        wpilib.SmartDashboard.putNumber("rotations", -1.5)
        wpilib.SmartDashboard.putData("Shooter", self.container.shooterSubsystem)
        wpilib.SmartDashboard.putData("DriveTrain", self.container.driveSubsystem)

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    # I think the extra logging in robotPeriod is causing loop overruns
    # def robotPeriodic(self) -> None:
    #     super().robotPeriodic()
    #     canStatus = RobotController.

    #     SignalLogger.write_integer("CAN/busOffCount", canStatus.busOffCount)
    #     SignalLogger.write_float(
    #         "CAN/percentBusUtilization", canStatus.percentBusUtilization
    #     )
    #     SignalLogger.write_integer("CAN/receiveErrorCount", canStatus.receiveErrorCount)
    #     SignalLogger.write_integer(
    #         "CAN/transmitErrorCount", canStatus.transmitErrorCount
    #     )
    #     SignalLogger.write_integer("CAN/txFullCount", canStatus.txFullCount)

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()
        # when autonomous starts if we have an autonomous routine selected,
        # set the starting pose for the robot to it.
        if self.autonomousCommand:
            print("Auto command found, setting pose.")
            self.startingPose2d = self.autonomousCommand.getStartingPoseFromAutoFile(
                self.autonomousCommand.getName()
            )
            print("Starting pose: " + self.startingPose2d.__str__())
            self.container.driveSubsystem.resetPose(self.startingPose2d)

        self.container.driveSubsystem.enable()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

        self.container.elevationSubsystem.homeShooterCommand().schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.

        # start logging DS and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())

        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.container.driveSubsystem.enable()
        self.container.elevationSubsystem.homeShooterCommand().schedule()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

        # reset pose
        # if self.container.operatorController.getBackButtonPressed():
        #     self.autonomousCommand = self.container.getAutonomousCommand()
        #     if self.autonomousCommand:
        #         print("Auto command found, setting pose.")
        #         self.startingPose2d = (
        #             self.autonomousCommand.getStartingPoseFromAutoFile(
        #                 self.autonomousCommand.getName()
        #             )
        #         )
        #         print("Starting pose: " + self.startingPose2d().__str__())
        #         self.container.driveSubsystem.resetPose(self.startingPose2d)

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
