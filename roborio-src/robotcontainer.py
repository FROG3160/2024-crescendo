#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import os
import wpilib
from wpilib.interfaces import GenericHID
from wpimath.units import degreesToRadians
from wpimath.geometry import Pose2d, Rotation2d

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
from commands2 import DeferredCommand
from FROGlib.xbox import FROGXboxDriver, FROGXboxOperator
from subsystems.drivetrain import DriveTrain
from pathplannerlib.auto import PathPlannerAuto, NamedCommands, AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
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
from commands.drive.field_oriented import (
    ManualDrive,
    AutoRotateShooterToSpeaker,
    AutoRotateShooterTowardsAmpCorner,
)
from commands.drive.robot_oriented import DriveToTarget
from commands.shooter.load import IntakeAndLoad, loadShooterCommand
from commands.shooter.fire import Fire
from commands.drive.robot_oriented import (
    DriveToTarget,
    ThrottledDriveToTarget,
    ManualRobotOrientedDrive,
    FindTargetAndDrive,
)


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
        self.shooterSubsystem = ShooterSubsystem()

        self.registerNamedCommands()

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        self.driveSubsystem.setDefaultCommand(
            ManualDrive(self.driverController, self.driveSubsystem)
        )

        # Keep elevation at home/load, when it's not doing anything else
        # self.elevationSubsystem.setDefaultCommand(
        #     self.elevationSubsystem.moveToLoadPositionCommand().withName(
        #         "Moving to load position."
        #     )
        # )

        # Chooser
        self.chooser = wpilib.SendableChooser()

        autosPath = os.path.join(wpilib.getDeployDirectory(), "pathplanner", "autos")
        for autoFile in os.listdir(autosPath):
            autoName = autoFile.split(".")[0]
            ppAuto = PathPlannerAuto(autoName)
            self.chooser.addOption(autoName, ppAuto)

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
        NamedCommands.registerCommand(
            "Move to Amp Elevation", self.elevationSubsystem.moveToAmpPositionCommand()
        )
        NamedCommands.registerCommand(
            "Drive to Target",
            DriveToTarget(
                self.driveSubsystem,
                self.targetingSubsystem,
            ),
        )
        NamedCommands.registerCommand(
            "Extend Arms", self.climberSubsystem.get_ExtendCommand()
        )
        NamedCommands.registerCommand(
            "Retract Arms", self.climberSubsystem.get_RetractCommand()
        )
        NamedCommands.registerCommand("Auto Aim", self.autoAimAtSpeakerCommand())

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        """DRIVER CONTROLS"""

        self.driverController.a().and_(self.shooterSubsystem.hasNote()).whileTrue(
            self.autoAimAtSpeakerCommand()
        )  # .whileFalse(self.elevationSubsystem.moveToLoadPositionCommand())

        self.driverController.start().and_(self.shooterSubsystem.hasNote()).whileTrue(
            self.autoAimTowardsAmpCommand()
        ).whileFalse(self.elevationSubsystem.moveToLoadPositionCommand())

        self.driverController.b().whileTrue(
            ThrottledDriveToTarget(
                self.driveSubsystem, self.targetingSubsystem, self.driverController
            )
        )
        self.driverController.x().onTrue(
            self.elevationSubsystem.moveToLoadPositionCommand()
        )
        self.driverController.y().onTrue(self.shooterSubsystem.stopShootingCommand())

        self.driverController.leftTrigger().onTrue(
            Fire(self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem)
        )

        self.driverController.povRight().whileTrue(
            AutoBuilder.followPath(PathPlannerPath.fromPathFile("Amp Approach"))
            .withName("Approach Amp Red")
            .alongWith(self.elevationSubsystem.moveToAmpPositionCommand())
            .andThen(self.shooterSubsystem.setFlywheelSpeedForAmpCommand())
            .andThen(
                Fire(
                    self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
                )
            )
            .withName("FollowPath")
        )
        self.driverController.povLeft().whileTrue(
            AutoBuilder.pathfindToPoseFlipped(
                Pose2d(1.84, 7.72, Rotation2d().fromDegrees(-90)),
                PathConstraints(
                    2.0,
                    2.0,
                    constants.kMaxChassisRadiansPerSec,
                    constants.kMaxChassisRadiansPerSec * 2,
                ),
                #     PathPlannerPath.fromPathFile("Amp Approach"))
                # .withName("Approach Amp Blue")
                # .alongWith(self.elevationSubsystem.moveToAmpPositionCommand())
                # .andThen(self.shooterSubsystem.setFlywheelSpeedForAmpCommand())
                # .andThen(
                #     Fire(
                #         self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
                #     )
                # )
            ).withName("PathFindToPoseFlipped")
        )

        self.driverController.povUp().whileTrue(
            # we use a "deffered command" so that driveToStageCommand can assess the robot's location
            # at the time the command is run instead of when the key binding occurs.
            DeferredCommand(lambda: self.driveSubsystem.driveToStageCommand())
        )

        self.driverController.povDown().whileTrue(
            # AutoBuilder.pathfindToPoseFlipped(
            #     Pose2d(1.83, 7.60, Rotation2d().fromDegrees(-90)),
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Amp Approach"),
                PathConstraints(
                    2.0,
                    3.0,
                    constants.kMaxChassisRadiansPerSec,
                    constants.kMaxChassisRadiansPerSec * 2,
                ),
            ).withName("PahtFindThenFollowPath")
        )

        # followPath(
        #     PathPlannerPath.fromPathFile("Straight Run")
        # ).withName("Straight Run Test")

        self.driverController.leftBumper().whileTrue(
            ManualRobotOrientedDrive(self.driverController, self.driveSubsystem)
        )

        self.driverController.start().onTrue(
            runOnce(lambda: self.driveSubsystem.setFieldPositionFromVision())
        )

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
            loadShooterCommand(
                self.shooterSubsystem, self.intakeSubsystem, self.elevationSubsystem
            )
        )
        self.operatorController.x().onTrue(self.intakeSubsystem.stopIntakeCommand())
        self.operatorController.y().onTrue(
            self.elevationSubsystem.homeShooterCommand().withInterruptBehavior(
                commands2.InterruptionBehavior.kCancelIncoming
            )
        )
        self.operatorController.povDown().whileTrue(
            FindTargetAndDrive(
                self.operatorController, self.targetingSubsystem, self.driveSubsystem
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
        self.intakeSubsystem.getTargetInRangeTrigger().not_().onTrue(
            IntakeAndLoad(
                self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
            )
        )

    def getAutonomousCommand(self) -> commands2.Command:
        return self.chooser.getSelected()

    def autoAimAtSpeakerCommand(self):
        return (
            self.shooterSubsystem.setFlywheelSpeedForSpeakerCommand()
            .andThen(self.driveSubsystem.resetRotationControllerCommand())
            .andThen(
                self.elevationSubsystem.autoMoveRunWithDistanceCommand().alongWith(
                    AutoRotateShooterToSpeaker(
                        self.driverController, self.driveSubsystem
                    )
                )
            )
        )

    def autoAimTowardsAmpCommand(self):
        return (
            self.shooterSubsystem.setFlywheelSpeedForSpeakerCommand()
            .andThen(self.driveSubsystem.resetRotationControllerCommand())
            .andThen(
                self.elevationSubsystem.moveToLoadPositionCommand().alongWith(
                    AutoRotateShooterTowardsAmpCorner(
                        self.driverController, self.driveSubsystem
                    )
                )
            )
        )
