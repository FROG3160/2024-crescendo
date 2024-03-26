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
from FROGlib.led import LedSubsystem
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

        wpilib.SmartDashboard.putNumber("Flywheel Speed", 100)
        wpilib.SmartDashboard.putNumber("Shooter Pos", 0)
        wpilib.SmartDashboard.putData("Shooter", self.container.shooterSubsystem)
        wpilib.SmartDashboard.putData("DriveTrain", self.container.driveSubsystem)
        wpilib.SmartDashboard.putData("Intake", self.container.intakeSubsystem)
        wpilib.SmartDashboard.putData("Elevation", self.container.elevationSubsystem)

        self.led = LedSubsystem(9) # name change
        # ADD CAN ID
        self.led.default()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()
        # self.autonomousCommand = (
        #     self.container.elevationSubsystem.homeShooterCommand().andThen(
        #         self.container.getAutonomousCommand()
        #     )
        # )

        self.container.driveSubsystem.enable()
        if self.container.shooterSubsystem.noteInShooter():
            self.container.intakeSubsystem.disallowIntake()
        if self.autonomousCommand:
            if self.isReal():
                self.container.elevationSubsystem.homeShooterCommand().andThen(
                    self.autonomousCommand
                ).withName(self.autonomousCommand.getName()).schedule()
            else:
                self.autonomousCommand.schedule()

            # self.autonomousCommand.schedule()

        # self.container.elevationSubsystem.homeShooterCommand().schedule()
        self.container.climberSubsystem.get_homeLeftClimber().andThen(
            self.container.climberSubsystem.get_homeRightClimber()
        ).schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.

        # start logging DS and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())

        self.container.ledSubsystem.ledCommand().schedule()

        if self.container.shooterSubsystem.noteInShooter():
            self.container.intakeSubsystem.disallowIntake()
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        self.container.driveSubsystem.enable()
        self.container.elevationSubsystem.homeShooterCommand().schedule()
        # Run the elevation homing routine while we are running teleop at home
        if self.isReal():
            self.container.elevationSubsystem.homeShooterCommand().schedule()
            self.container.climberSubsystem.get_homeLeftClimber().andThen(
                self.container.climberSubsystem.get_homeRightClimber()
            ).schedule()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
