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
from commands2.cmd import runOnce, startEnd, waitUntil
from commands2 import DeferredCommand, PrintCommand

from FROGlib.xbox import FROGXboxDriver, FROGXboxOperator

from subsystems.drivetrain import DriveTrain
from pathplannerlib.auto import PathPlannerAuto, NamedCommands, AutoBuilder
from pathplannerlib.path import PathPlannerPath, PathConstraints
from pathplannerlib.config import (
    HolonomicPathFollowerConfig,
    ReplanningConfig,
    PIDConstants,
)

from subsystems.leds import LEDSubsystem
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
from commands.shooter.load import IntakeAndLoad, loadShooterCommand
from commands.shooter.fire import Fire, FireOnlyOptimized
from commands.drive.robot_oriented import (
    DriveToTarget,
    ThrottledDriveToTarget,
    ManualRobotOrientedDrive,
    FindTarget,
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
        self.ledSubsystem = LEDSubsystem(9)
        self.driveSubsystem = DriveTrain(
            self.positioningSubsystem, self.elevationSubsystem, self.ledSubsystem
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
        # self.chooser = wpilib.SendableChooser()
        self.autochooser = AutoBuilder.buildAutoChooser()

        # Put the chooser on the dashboard0.5
        wpilib.SmartDashboard.putData("PathPlanner Autos", self.autochooser)

    def registerNamedCommands(self):

        NamedCommands.registerCommand(
            "Fire",
            self.shooterSubsystem.homeNoteCommand().andThen(
                Fire(
                    self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
                ),
            ),
        )
        NamedCommands.registerCommand(
            "Fire Only",
            self.shooterSubsystem.homeNoteCommand().andThen(
                FireOnlyOptimized(
                    self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
                ),
            ),
        )
        NamedCommands.registerCommand(
            "Intake and Load",
            IntakeAndLoad(
                self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
            ),
        )
        NamedCommands.registerCommand("Intake", self.intakeCommand())
        NamedCommands.registerCommand("Load", self.loadCommand())
        NamedCommands.registerCommand(
            "Seek and Drive to Target", self.seekAndDriveToTargetCommand()
        )
        NamedCommands.registerCommand(
            "Home Shooter", self.elevationSubsystem.homeShooterCommand()
        )
        NamedCommands.registerCommand(
            "Move to Amp Elevation", self.elevationSubsystem.moveToAmpPositionCommand()
        )
        NamedCommands.registerCommand(
            "Move to Load Position", self.elevationSubsystem.moveToLoadPositionCommand()
        )
        NamedCommands.registerCommand("Drive to Target", self.driveToTargetCommand())
        NamedCommands.registerCommand(
            "Extend Arms", self.climberSubsystem.get_ExtendCommand()
        )
        NamedCommands.registerCommand(
            "Retract Arms", self.climberSubsystem.get_RetractCommand()
        )
        NamedCommands.registerCommand(
            "Aim At Speaker",
            self.autoAimAtSpeakerCommand().until(self.shooterAimed),
        )

        NamedCommands.registerCommand(
            "Set Flywheel for Speaker",
            self.shooterSubsystem.setFlywheelSpeedForSpeakerCommand(),
        )

        NamedCommands.registerCommand(
            "Wait Until Loaded", self.waitUntilShooterLoaded()
        )

        NamedCommands.registerCommand("After Path", PrintCommand("Launched After Path"))
        NamedCommands.registerCommand(
            "Source Side Speaker Approach", self.moveToSpeakerSourceSideCommand()
        )

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        self.configureDriverControls()
        self.configureOperatorControls()
        self.configureTriggers()

    def configureDriverControls(self):
        """DRIVER CONTROLS"""

        self.driverController.a().and_(self.shooterSubsystem.hasNote()).whileTrue(
            self.autoAimAtSpeakerCommand()
        )  # .whileFalse(self.elevationSubsystem.moveToLoadPositionCommand())

        self.driverController.back().and_(self.shooterSubsystem.hasNote()).whileTrue(
            self.autoAimTowardsAmpCommand()
        )  # .whileFalse(self.elevationSubsystem.moveToLoadPositionCommand())

        self.driverController.b().whileTrue(self.driveToTargetCommand())
        self.driverController.x().onTrue(
            self.elevationSubsystem.moveToLoadPositionCommand()
        )
        self.driverController.y().onTrue(self.shooterSubsystem.stopShootingCommand())

        self.driverController.leftTrigger().onTrue(
            FireOnlyOptimized(
                self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
            ).andThen(self.elevationSubsystem.moveToLoadPositionCommand())
        )

        # self.driverController.povLeft().whileTrue(
        #     AutoBuilder.followPath(PathPlannerPath.fromPathFile("Amp Approach"))
        #     .withName("Approach Amp")
        #     .alongWith(self.elevationSubsystem.moveToAmpPositionCommand())
        #     .andThen(self.shooterSubsystem.setFlywheelSpeedForAmpCommand())
        #     .andThen(
        #         Fire(
        #             self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
        #         )
        #     )
        #     .withName("FollowPath")
        # )

        self.driverController.povUp().whileTrue(
            # we use a "deffered command" so that driveToStageCommand can assess the robot's location
            # at the time the command is run instead of when the key binding occurs.
            DeferredCommand(lambda: self.driveSubsystem.driveToStageCommand())
            # DeferredCommand(lambda: self.testFollowPathCommand())
        )

        self.driverController.rightBumper().and_(
            self.shooterSubsystem.hasNote()
        ).whileTrue(self.placeInAmpCommand())
        # self.driverController.povRight().and_(
        #     self.shooterSubsystem.hasNote()
        # ).whileTrue(self.placeInAmpCommand())

        self.driverController.leftBumper().whileTrue(
            ManualRobotOrientedDrive(self.driverController, self.driveSubsystem)
        )

        self.driverController.start().onTrue(
            runOnce(lambda: self.driveSubsystem.setFieldPositionFromVision())
        )

    def configureOperatorControls(self):
        """OPERATOR CONTROLS"""

        # Operator Controller Bindings
        self.operatorController.axisLessThan(
            wpilib.XboxController.Axis.kLeftY, -0.5
        ).onTrue(self.climberSubsystem.get_ExtendCommand())
        self.operatorController.axisGreaterThan(
            wpilib.XboxController.Axis.kLeftY, 0.5
        ).onTrue(self.climberSubsystem.get_RetractCommand())

        self.operatorController.leftBumper().onTrue(
            self.climberSubsystem.get_homeLeftClimber().andThen(
                self.climberSubsystem.get_homeRightClimber()
            )
        )
        self.operatorController.rightBumper().onTrue(
            self.elevationSubsystem.homeShooterCommand().withInterruptBehavior(
                commands2.InterruptionBehavior.kCancelIncoming
            )
        )

        self.operatorController.a().onTrue(self.intakeSubsystem.intakeCommand())
        self.operatorController.b().onTrue(
            loadShooterCommand(
                self.shooterSubsystem, self.intakeSubsystem, self.elevationSubsystem
            )
        )
        self.operatorController.x().onTrue(self.intakeSubsystem.stopIntakeCommand())
        self.operatorController.y().onTrue(
            self.elevationSubsystem.moveToLoadPositionCommand()
        )
        self.operatorController.povDown().whileTrue(self.seekAndDriveToTargetCommand())
        self.operatorController.start().whileTrue(
            self.intakeSubsystem.reverseTransferCommand()
        )  # temporary mapping to test how well the command works
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

    def configureTeleopTriggers(self):
        # Thesse triggers are created by teleopInit in robot.py
        self.intakeSubsystem.getTargetInRangeTrigger().not_().onTrue(
            IntakeAndLoad(
                self.intakeSubsystem, self.shooterSubsystem, self.elevationSubsystem
            )
        )

        self.intakeSubsystem.getNoteInIntakeTrigger().onTrue(
            runOnce(lambda: self.driverController.rightRumble()).alongWith(
                self.ledSubsystem.ledIntakeCommand()
            )
        ).onFalse(runOnce(lambda: self.driverController.stopRightRumble()))

        self.shooterSubsystem.hasNote().onTrue(
            startEnd(
                lambda: self.driverController.leftRumble(),
                lambda: self.driverController.stopLeftRumble(),
            )
            .withTimeout(1)
            .alongWith(self.ledSubsystem.ledShooterCommand())
        ).onFalse(
            runOnce(lambda: self.driverController.stopLeftRumble()).alongWith(
                self.ledSubsystem.ledDefaultCommand()
            )
        )

        # self.noteNotAtHomeTrigger().onTrue(self.shooterSubsystem.homeNoteCommand())

    def configureTriggers(self):
        pass

    def getAutonomousCommand(self):
        return self.autochooser.getSelected()

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

    def driveToTargetCommand(self):
        return DriveToTarget(self.driveSubsystem, self.targetingSubsystem)

    def seekAndDriveToTargetCommand(self):
        return (
            FindTarget(self.targetingSubsystem, self.driveSubsystem)
            .until(self.targetingSubsystem.hasSeenTarget)
            .andThen(self.driveToTargetCommand())
        )

    # def adjustToSpeaker(self):
    #     return (
    #         self.shooterSubsystem.setFlywheelSpeedForSpeakerCommand()
    #         .andThen(self.driveSubsystem.resetRotationControllerCommand())
    #         .andThen(
    #             self.elevationSubsystem.autoMoveRunWithDistanceCommand().alongWith(

    #             )
    #         )
    #     )

    def waitUntilShooterLoaded(self):
        return waitUntil(self.shooterSubsystem.noteInShooter)

    def waitUntilAimed(self):
        return waitUntil(
            self.elevationSubsystem.leadscrewAtPosition()
            and self.driveSubsystem.profiledRotationController.atGoal()
        )

    def shooterAimed(self):
        return (
            self.elevationSubsystem.leadscrewAtPosition()
            and self.driveSubsystem.profiledRotationController.atGoal()
        )

    # def testFollowPathCommand(self):
    #     pathCommand = AutoBuilder.followPath(
    #         PathPlannerPath.fromPathFile("Rotation Tests")
    #     ).withName("Rotation Tests")

    #     # The follow path command uses PPHolonomicDriveController
    #     # at pathCommand._command._controller

    #     pathCommand._command._controller._xController.setP(
    #         wpilib.SmartDashboard.getNumber("TranslationP", 1)
    #     )
    #     pathCommand._command._controller._yController.setP(
    #         wpilib.SmartDashboard.getNumber("TranslationP", 1)
    #     )
    #     pathCommand._command._controller._rotationController.setP(
    #         wpilib.SmartDashboard.getNumber("RotationP", 1)
    #     )
    #     return pathCommand

    def placeInAmpCommand(self):
        return (
            # AutoBuilder.pathfindThenFollowPath(
            AutoBuilder.followPath(
                PathPlannerPath.fromPathFile("Amp Approach"),
                # PathConstraints(
                #     constants.kMaxTrajectorySpeed / 2,
                #     constants.kMaxTrajectoryAccel / 2,
                #     constants.kProfiledRotationMaxVelocity,
                #     constants.kProfiledRotationMaxAccel,
                # ),
            )
            .alongWith(self.elevationSubsystem.moveToAmpPositionCommand())
            .andThen(self.shooterSubsystem.setFlywheelSpeedForAmpCommand())
            .withName("Place in Amp")
        )

    def moveToSpeakerSourceSideCommand(self):
        return AutoBuilder.pathfindThenFollowPath(
            PathPlannerPath.fromPathFile("Source Side Speaker Approach"),
            PathConstraints(
                constants.kMaxTrajectorySpeed,
                constants.kMaxTrajectoryAccel,
                constants.kProfiledRotationMaxVelocity,
                constants.kProfiledRotationMaxAccel,
            ),
        ).withName("PathFindToSpeakerApproach")

    def intakeCommand(self):
        return (
            self.intakeSubsystem.intakeCommand()
            .andThen(runOnce(self.intakeSubsystem.disallowIntake, self.intakeSubsystem))
            .andThen(runOnce(self.elevationSubsystem.moveToLoadPosition))
        )

    def loadCommand(self):
        return waitUntil(self.elevationSubsystem.readyToLoad).andThen(
            loadShooterCommand(
                self.shooterSubsystem, self.intakeSubsystem, self.elevationSubsystem
            )
        )

    def noteNotAtHomeTrigger(self):
        return commands2.button.Trigger(
            lambda: not self.intakeSubsystem.intakeAllowed()
            and not self.shooterSubsystem.noteInShooter()
        )
