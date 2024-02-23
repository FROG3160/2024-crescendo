#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import typing
import wpilib
import rev
import commands2
import constants
import configs
from wpimath.geometry import Pose2d
from wpilib import DataLogManager, DriverStation

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

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()
        self.container.driveSubsystem.enable()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

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

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

        # reset pose
        if self.container.operatorController.getBackButtonPressed():
            self.autonomousCommand = self.container.getAutonomousCommand()
            if self.autonomousCommand:
                print("Auto command found, setting pose.")
                self.startingPose2d = (
                    self.autonomousCommand.getStartingPoseFromAutoFile(
                        self.autonomousCommand.getName()
                    )
                )
                print("Starting pose: " + self.startingPose2d().__str__())
                self.container.driveSubsystem.resetPose(self.startingPose2d)

        if self.container.operatorController.getAButton():
            self.container.shooterSubsystem.setLeadscrewPosition(
                wpilib.SmartDashboard.getNumber("rotations", 0)
            )
        if self.container.operatorController.getBButton():
            self.container.shooterSubsystem.setLeadscrewPosition(8.5)
        if self.container.operatorController.getXButton():
            self.container.shooterSubsystem.setLeadscrewPosition(0)

        self.container.shooterSubsystem.setFlywheelSpeed(
            wpilib.SmartDashboard.getNumber("flyspeed", 0)
        )

        # self.container.shooter.sequencer.set(self.container.operatorController.runSequencer())

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
