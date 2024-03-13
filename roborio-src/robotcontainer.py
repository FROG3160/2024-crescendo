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
from pathplannerlib.auto import PathPlannerAuto, NamedCommands, AutoBuilder
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)
from subsystems.vision import PositioningSubsystem, TargetingSubsystem
from subsystems.intake import IntakeSubsystem
from subsystems.shooter import ShooterSubsystem
from subsystems.climber import ClimberSubsystem
from subsystems.elevation import ElevationSubsystem
from commands.drive.field_oriented import ManualDrive
from commands.drive.robot_oriented import DriveToTarget
from commands.shooter.load import IntakeAndLoad, loadShooterCommand
from commands.shooter.fire import Fire
from commands.drive.robot_oriented import DriveToTarget

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
        self.intakeSubsystem = IntakeSubsystem(self.targetingSubsystem)
        self.climberSubsystem = ClimberSubsystem()
        self.elevationSubsystem = ElevationSubsystem()
        self.driveSubsystem = DriveTrain(
            self.positioningSubsystem, self.elevationSubsystem
        )
        self.shooterSubsystem = ShooterSubsystem(self.intakeSubsystem)

        self.registerNamedCommands()

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.driveSubsystem.setDefaultCommand(
            ManualDrive(self.driverController, self.driveSubsystem)
        )

        # Keep elevation at home/load, when it's not doing anything else
        self.elevationSubsystem.setDefaultCommand(
            self.elevationSubsystem.moveToLoadPositionCommand()
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

    def registerNamedCommands(self):

        NamedCommands.registerCommand(
            "Fire",
            Fire(self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem),
        )
        NamedCommands.registerCommand(
            "Intake And Load",
            IntakeAndLoad(
                self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
            ),
        )
        NamedCommands.registerCommand(
            "Drive To Note", DriveToTarget(self.driveSubsystem, self.targetingSubsystem)
        )
        NamedCommands.registerCommand(
            "Home Shooter", self.elevationSubsystem.homeShooterCommand()
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        """DRIVER CONTROLS"""

        self.driverController.a().onTrue(
            self.elevationSubsystem.autoMoveRunWithDistanceCommand()
        )
        self.driverController.b().whileTrue(
            DriveToTarget(self.driveSubsystem, self.targetingSubsystem)
        )
        self.driverController.x().onTrue(
            self.elevationSubsystem.moveToLoadPositionCommand()
        )
        self.driverController.y().onTrue(self.shooterSubsystem.stopShootingCommand())

        self.driverController.leftTrigger().onTrue(
            Fire(self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem)
        )
        # self.driverController.start().onTrue(self.driveSubsystem.resetGyroCommand())

        """OPERATOR CONTROLS"""

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

        self.operatorController.a().onTrue(self.intakeSubsystem.intakeCommand())
        self.operatorController.b().onTrue(
            loadShooterCommand(self.shooterSubsystem, self.intakeSubsystem)
        )
        self.operatorController.x().onTrue(self.intakeSubsystem.stopIntakeCommand())
        self.operatorController.y().onTrue(
            self.elevationSubsystem.homeShooterCommand().withInterruptBehavior(
                commands2.InterruptionBehavior.kCancelIncoming
            )
        )
        # self.operatorController.a().onTrue(
        #     runOnce(
        #         lambda: self.elevationSubsystem.setLeadscrewPosition(
        #             wpilib.SmartDashboard.getNumber("Shooter Pos", 0)
        #         )
        #     ).andThen(self.elevationSubsystem.setLeadscrewCommand())
        # )
        # self.operatorController.b().onTrue(
        #     runOnce(lambda: self.elevationSubsystem.setLeadscrewPosition(8.5)).andThen(
        #         self.elevationSubsystem.setLeadscrewCommand()
        #     )
        # )
        # self.operatorController.x().onTrue(
        #     runOnce(lambda: self.elevationSubsystem.setLeadscrewPosition(0)).andThen(
        #         self.elevationSubsystem.setLeadscrewCommand()
        #     )
        # )
        if not self.shooterSubsystem.noteInShooter():
            self.intakeSubsystem.getTargetInRangeTrigger().onTrue(
                IntakeAndLoad(
                    self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
                )
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
