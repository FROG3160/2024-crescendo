from commands2 import Subsystem, Command, cmd
from FROGlib.motors import FROGTalonFX, FROGTalonFXConfig, FROGSparkMax
from phoenix6.controls import (
    PositionDutyCycle,
    VelocityDutyCycle,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
    MotionMagicVoltage,
)
import constants
from configs import (
    leadScrewConfig,
    leftFlywheelConfig,
    rightFlywheelConfig,
    sequencerMotorType,
)
from phoenix6.signals.spn_enums import NeutralModeValue, InvertedValue
from ntcore import NetworkTableInstance
from wpilib import DigitalInput, SmartDashboard
from commands2.cmd import waitSeconds, waitUntil
from subsystems.intake import IntakeSubsystem


class ShooterSubsystem(Subsystem):

    flyWheelSpeed: float
    leadScrewPosition: float

    def __init__(
        self,
        intake: IntakeSubsystem,
        parent_nt: str = "Subsystems",
    ):
        super().__init__()
        self.setName("Shooter")
        nt_table = f"{parent_nt}/{self.getName()}"
        # Very rudimentary system that allows speed control  of
        # the lead screw and flywheel on the operator Xbox controller triggers.
        # The NEO 550 will be used to pull and push the note out of
        # the intake and into the flywheel.
        self.leadScrew = FROGTalonFX(
            constants.kLeadScrewControllerID,
            leadScrewConfig,
            parent_nt=f"{nt_table}",
            motor_name="LeadScrew",
        )
        self.leftFlyWheel = FROGTalonFX(
            constants.kFlyWheelControllerLeftID,
            leftFlywheelConfig,
            parent_nt=f"{nt_table}",
            motor_name="LeftFlywheel",
        )
        self.rightFlyWheel = FROGTalonFX(
            constants.kFlyWheelCOntrollerRightID,
            rightFlywheelConfig,
            parent_nt=f"{nt_table}",
            motor_name="RightFlywheel",
        )
        self.sequencer = FROGSparkMax(
            constants.kSequencerControllerID,
            sequencerMotorType,
            parent_nt=f"{nt_table}",
            motor_name="Sequencer",
        )
        self.shooterSensor = DigitalInput(constants.kShooterSensorChannel)
        self.intake = intake

        self._flywheelCommandedVelocity = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/flywheel_velocity")
            .publish()
        )
        self._shooterCommandedPosition = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/shooter_position")
            .publish()
        )
        self._sequencerCommandedSpeed = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/transfer_speed")
            .publish()
        )
        self._shooterSensorCurrentState = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/note_detected")
            .publish()
        )
        self._leadScrewAtPosition = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/leadscrew_at_position")
            .publish()
        )
        self.flyWheelSpeed = 0
        self.leadScrewPosition = self.leadScrew.get_position().value
        self.sequencerCommandedSpeed = 0

    def setFlywheelSpeed(self, flywheelSpeed: float):
        self.flyWheelSpeed = flywheelSpeed

    def runFlywheels(self):
        self.leftFlyWheel.set_control(
            VelocityVoltage(velocity=self.flyWheelSpeed, slot=0)
        )
        self.rightFlyWheel.set_control(
            VelocityVoltage(velocity=self.flyWheelSpeed, slot=0)
        )

    def runFlywheelsCommand(self) -> Command:
        return self.runOnce(self.runFlywheels).withName("RunFlywheels")

    def stopFlywheels(self):
        self.leftFlyWheel.set_control(VelocityVoltage(velocity=0, slot=0))
        self.rightFlyWheel.set_control(VelocityVoltage(velocity=0, slot=0))

    def stopFlywheelsCommand(self) -> Command:
        return self.runOnce(self.stopFlywheels).withName("StopFlywheels")

    def flywheelAtSpeedIsTrue(self) -> bool:
        if (
            abs(self.flyWheelSpeed - self.leftFlyWheel.get_velocity().value) < 5
            and abs(self.flyWheelSpeed - abs(self.rightFlyWheel.get_velocity().value))
            < 5
        ):
            return True
        else:
            return False

    def runSequencer(self):
        self.sequencerCommandedSpeed = constants.kSequencerSpeed
        self.sequencer.set(self.sequencerCommandedSpeed)

    def stopSequencer(self):
        self.sequencer.stopMotor()

    def loadShooterCommand(self) -> Command:
        return (
            self.startEnd(self.runSequencer, self.stopSequencer)
            .until(self.noteInShooter)
            .withName("RunSequencer")
            .deadlineWith(self.intake.transferCommand())
        )

    def stopSequencerCommand(self) -> Command:
        return self.runOnce(self.stopSequencer).withName("StopSequencer")

    def runSequencerCommand(self) -> Command:
        return self.runOnce(self.runSequencer).withName("RunSequencer")

    def setLeadscrewCommand(self) -> Command:
        return self.runOnce(self.setLeadscrewPosition).withName("SetLeadscrew")

    def shootCommand(self) -> Command:
        return (
            self.runFlywheelsCommand()  # run the flywheel at the commanded speed
            .andThen(
                self.moveLeadscrewToPosition()
            )  # wait until the flywheel is at speed
            .andThen(
                waitUntil(self.flywheelAtSpeedIsTrue)
            )  # sets the leadscrew at the commanded position
            .andThen(
                waitUntil(self.getLeadscrewPositionIsTrue)
            )  # wait until the leadscrew is at position
            .andThen(
                self.runSequencerCommand()
            )  # run the sequencer to move the note into the flywheel
            .andThen(
                waitUntil(self.shooterSensor.get)
            )  # wait until we no longer detect the note
            .andThen(
                self.stopSequencerCommand()
            )  # could also call stopShootingCommand at the very end
            .andThen(waitSeconds(1))  # wait 1 second
            .andThen(self.stopFlywheelsCommand())
        )

    def stopShootingCommand(self) -> Command:
        return self.runOnce(self.stopFlywheelsCommand) and self.runOnce(
            self.stopSequencerCommand
        )

    def setLeadscrewPosition(self, leadscrewPosition: float):
        self.leadScrewPosition = leadscrewPosition

    def moveLeadscrewToPosition(self):
        self.leadScrew.set_control(
            MotionMagicVoltage(position=self.leadScrewPosition, slot=1)
        )

    def getLeadscrewPositionIsTrue(self) -> bool:
        if self.leadScrewPosition == self.leadScrew.get_position():
            return True
        else:
            return False

    def zeroLeadScrew(self):
        self.leadScrew.set_position(0)

    def noteInShooter(self) -> bool:
        return not self.shooterSensor.get()

    def periodic(self) -> None:
        self.logTelemetry()
        SmartDashboard.putBoolean("ShooterDioSensor", self.noteInShooter())
        SmartDashboard.putBoolean("FlywheelAtSpeed", self.flywheelAtSpeedIsTrue())

    def logTelemetry(self):
        self._flywheelCommandedVelocity.set(self.flyWheelSpeed)
        self._shooterCommandedPosition.set(self.leadScrewPosition)
        self._sequencerCommandedSpeed.set(self.sequencerCommandedSpeed)
        self._shooterSensorCurrentState.set(self.noteInShooter())
        self._leadScrewAtPosition.set(self.getLeadscrewPositionIsTrue())
