#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import os
import wpilib
from wpilib.interfaces import GenericHID

import commands2
import commands2.button
import commands2.cmd

from constants import kDriverControllerPort, kDeadband, kDebouncePeriod, kTranslationSlew, kRotSlew

from FROGlib.xbox import FROGXboxDriver
from subsystems.drivetrain import DriveTrain
from pathplannerlib.auto import PathPlannerAuto, NamedCommands
from subsystems.vision import VisionSystem
from subsystems.intake import Intake


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        # The robot's subsystems


        # Retained command handles

        # A simple auto routine that drives forward a specified distance, and then stops.
        # self.simpleAuto = commands.autos.Autos.simpleAuto(self.driveSubsystem)

        # A complex auto routine that drives forward, drops a hatch, and then drives backward.
        # self.complexAuto = commands.autos.Autos.complexAuto(
        #     self.driveSubsystem, self.hatchSubsystem
        # )

        # The driver's controller
        self.driverController = FROGXboxDriver(kDriverControllerPort, kDeadband, kDebouncePeriod, kTranslationSlew, kRotSlew)
        self.driveSubsystem = DriveTrain()
        self.visionSubsystem = VisionSystem()

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.driveSubsystem.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.cmd.run(
                lambda: self.driveSubsystem.fieldOrientedDrive(
                    self.driverController.getFieldForward(),
                    self.driverController.getFieldLeft(),
                    self.driverController.getFieldRotation()
                ),
                self.driveSubsystem, 
            )
        )

        # Chooser
        self.chooser = wpilib.SendableChooser()

        autosPath = os.path.join(wpilib.getDeployDirectory(), 'pathplanner', 'autos')
        for autoFile in os.listdir(autosPath):
            autoName = autoFile.split('.')[0]
            ppAuto = PathPlannerAuto(autoName)
            wpilib.SmartDashboard.putData(f"autos/{autoName}", ppAuto)
            self.chooser.addOption(autoName, ppAuto)


        # Add commands to the autonomous command chooser
        # self.chooser.setDefaultOption("Simple Auto", self.simpleAuto)
        # self.chooser.addOption("Complex Auto", self.complexAuto)

        # Put the chooser on the dashboard
        wpilib.SmartDashboard.putData("PathPlanner Autos", self.chooser)

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # # Grab the hatch when the Circle button is pressed.
        # self.driverController.circle().onTrue(self.hatchSubsystem.grabHatch())

        # # Release the hatch when the Square button is pressed.
        # self.driverController.square().onTrue(self.hatchSubsystem.releaseHatch())

        # # While holding R1, drive at half speed
        # self.driverController.R1().onTrue(
        #     commands2.cmd.runOnce(lambda: self.driveSubsystem.setMaxOutput(0.5))
        # ).onFalse(commands2.cmd.runOnce(lambda: self.driveSubsystem.setMaxOutput(1)))

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()
# %%
wpilib.getDeployDirectory()
# %%
import wpilib
# %%
