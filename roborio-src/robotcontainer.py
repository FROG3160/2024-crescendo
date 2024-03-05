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
import constants
import configs
from constants import (
    kDriverControllerPort,
    kOperatorControllerPort,
    kDeadband,
    kDebouncePeriod,
    kTranslationSlew,
    kRotSlew,
)
from commands2.cmd import runOnce
from FROGlib.xbox import FROGXboxDriver, FROGXboxOperator
from subsystems.drivetrain import DriveTrain
from pathplannerlib.auto import PathPlannerAuto, NamedCommands
from subsystems.vision import PositioningSubsystem, TargetingSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem
from subsystems.climber import ClimberSubsystem
from subsystems.elevation import ElevationSubsystem
from commands.drive.field_oriented import ManualDrive


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:

        # The driver's controller
        self.driverController = FROGXboxDriver(
            kDriverControllerPort,
            kDeadband,
            kDebouncePeriod,
            kTranslationSlew,
            kRotSlew,
        )
        self.operatorController = FROGXboxOperator(kOperatorControllerPort, kDeadband)

        # Create all subsystems here.  If a subsystem is needed by other subsystems, create it first,
        # then pass it in to the subsystems needing it.
        self.positioningSubsystem = PositioningSubsystem()
        self.targetingSubsystem = TargetingSubsystem()
        self.intakeSubsystem = IntakeSubsystem()
        self.climberSubsystem = ClimberSubsystem()
        self.elevationSubsystem = ElevationSubsystem()
        self.driveSubsystem = DriveTrain(self.positioningSubsystem)
        self.shooterSubsystem = ShooterSubsystem(self.intakeSubsystem)

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.driveSubsystem.setDefaultCommand(
            ManualDrive(self.driverController, self.driveSubsystem)
        )

        # Chooser
        self.chooser = wpilib.SendableChooser()

        autosPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for autoFile in os.listdir(autosPath):
            autoName = autoFile.split(".")[0]
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

        # Driver Controller Bindings

        self.driverController.a().onTrue(self.intakeSubsystem.intakeCommand())
        self.driverController.x().onTrue(self.intakeSubsystem.stopIntakeCommand())
        self.driverController.b().onTrue(self.shooterSubsystem.loadShooterCommand())
        self.driverController.rightBumper().onTrue(self.shooterSubsystem.shootCommand())
        self.driverController.y().onTrue(self.shooterSubsystem.stopShootingCommand())
        self.driverController.start().onTrue(self.driveSubsystem.resetGyroCommand())

        # Operator Controller Bindings
        self.operatorController.axisLessThan(
            wpilib.XboxController.Axis.kLeftY, -0.5
        ).whileTrue(self.climberSubsystem.get_ExtendCommand())
        self.operatorController.axisGreaterThan(
            wpilib.XboxController.Axis.kLeftY, 0.5
        ).whileTrue(self.climberSubsystem.get_RetractCommand())
        self.operatorController.leftBumper().onTrue(
            self.climberSubsystem.get_homeLeftClimber()
        )
        self.operatorController.rightBumper().onTrue(
            self.climberSubsystem.get_homeRightClimber()
        )
        self.operatorController.y().onTrue(
            self.elevationSubsystem.homeShooterCommand().withInterruptBehavior(
                commands2.InterruptionBehavior.kCancelIncoming
            )
        )
        self.container.operatorController.a().onTrue(
            runOnce(
                lambda: self.container.elevationSubsystem.setLeadscrewPosition(
                    wpilib.SmartDashboard.getNumber("rotations", 0)
                )
            ).andThen(self.container.elevationSubsystem.setLeadscrewCommand())
        )
        self.container.operatorController.b().onTrue(
            runOnce(
                lambda: self.container.elevationSubsystem.setLeadscrewPosition(8.5)
            ).andThen(self.container.elevationSubsystem.setLeadscrewCommand())
        )
        self.container.operatorController.x().onTrue(
            runOnce(
                lambda: self.container.elevationSubsystem.setLeadscrewPosition(0)
            ).andThen(self.container.elevationSubsystem.setLeadscrewCommand())
        )

        self.targetingSubsystem.getTargetInRangeTrigger().onTrue(
            self.intakeSubsystem.intakeCommand()
        )

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
