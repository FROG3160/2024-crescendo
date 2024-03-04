from commands2 import Subsystem, Command, cmd
from phoenix5 import StatusFrameEnhanced, TalonSRXControlMode
from FROGlib.motors import (
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGSparkMax,
    FROGTalonSRX,
    FROGTalonSRXConfig,
)
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
    leadscrewConfig,
    leftFlywheelConfig,
    rightFlywheelConfig,
)
from phoenix6.signals.spn_enums import NeutralModeValue, InvertedValue
from ntcore import NetworkTableInstance
from wpilib import DigitalInput, SmartDashboard
from commands2.cmd import waitSeconds, waitUntil
from subsystems.intake import IntakeSubsystem


class ShooterSubsystem(Subsystem):

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
        self.sequencer = FROGTalonSRX(
            constants.kSequencerControllerID,
            FROGTalonSRXConfig(),
            parent_nt=f"{nt_table}",
            motor_name="Sequencer",
        )
        self.sequencer.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 200)

        self.shooterSensor = DigitalInput(constants.kShooterSensorChannel)

        self.intake = intake

        self._flywheelCommandedVelocity = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/FlywheelVelocity")
            .publish()
        )
        self._sequencerCommandedPercent = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/TransferSpeed")
            .publish()
        )
        self._shooterSensorCurrentState = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/NoteDetected")
            .publish()
        )
        self.flyWheelSpeed = 0.0
        self.sequencerCommandedPercent = 0.0
        self.leftFlyWheelSpeedFactor = 0.8
        self.rightFlyWheelSpeedFactor = 1.0

    def setFlywheelSpeed(self, flywheelSpeed: float):
        self.flyWheelSpeed = flywheelSpeed

    def runFlywheels(self):
        self.leftFlyWheel.set_control(
            VelocityVoltage(velocity=self.flyWheelSpeed, slot=0)
        )
        self.rightFlyWheel.set_control(
            VelocityVoltage(velocity=self.flyWheelSpeed * 0.6, slot=0)
        )

    def runFlywheelsCommand(self) -> Command:
        return self.runOnce(self.runFlywheels).withName("RunFlywheels")

    def stopFlywheels(self):
        self.leftFlyWheel.set_control(VelocityVoltage(velocity=0, slot=0))
        self.rightFlyWheel.set_control(VelocityVoltage(velocity=0, slot=0))

    def stopFlywheelsCommand(self) -> Command:
        return self.runOnce(self.stopFlywheels).withName("StopFlywheels")

    def flywheelAtSpeed(self) -> bool:
        if (
            abs(
                self.flyWheelSpeed * self.leftFlyWheelSpeedFactor
                - self.leftFlyWheel.get_velocity().value
            )
            < 5
            and abs(
                self.flyWheelSpeed * self.rightFlyWheelSpeedFactor
                - abs(self.rightFlyWheel.get_velocity().value)
            )
            < 5
        ):
            return True
        else:
            return False

    def controlSequencer(self, percent):
        self.sequencerCommandedPercent = percent
        self.sequencer.set(
            TalonSRXControlMode.PercentOutput, self.sequencerCommandedPercent
        )

    def runSequencer(self):
        self.controlSequencer(constants.kSequencerShootPercent)

    def stopSequencer(self):
        self.controlSequencer(0)

    def stopShooting(self):
        self.stopSequencer()
        self.stopFlywheels()

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

    def shootCommand(self) -> Command:
        return (
            self.runFlywheelsCommand()  # run the flywheel at the commanded speed
            .andThen(waitUntil(self.flywheelAtSpeed))
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
        return self.runOnce(self.stopShooting)

    def noteInShooter(self) -> bool:
        return not self.shooterSensor.get()

    def periodic(self) -> None:
        # self.logTelemetry()
        self.publishOnSmartDashboard()

    def publishOnSmartDashboard(self):
        SmartDashboard.putBoolean("ShooterDioSensor", self.noteInShooter())
        SmartDashboard.putBoolean("FlywheelAtSpeed", self.flywheelAtSpeed())

    def logTelemetry(self):
        self.leftFlyWheel.logData()
        self.rightFlyWheel.logData()
        self.sequencer.logData()
        self._flywheelCommandedVelocity.set(self.flyWheelSpeed)
        self._sequencerCommandedPercent.set(self.sequencerCommandedPercent)
        self._shooterSensorCurrentState.set(self.noteInShooter())
