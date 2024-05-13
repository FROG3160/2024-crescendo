from commands2 import Subsystem, Command, cmd, PrintCommand
from phoenix5 import StatusFrameEnhanced, TalonSRXControlMode, NeutralMode
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
    DutyCycleOut,
)
import constants
from configs import (
    leftFlywheelConfig,
    rightFlywheelConfig,
)
from phoenix6.signals.spn_enums import NeutralModeValue, InvertedValue
from ntcore import NetworkTableInstance
from wpilib import DigitalInput, SmartDashboard
from commands2.cmd import waitSeconds, waitUntil
from commands2.button import Trigger
from FROGlib.sensors import FROGColor


class ShooterSubsystem(Subsystem):

    def __init__(
        self,
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
        self.sequencer.setNeutralMode(NeutralMode.Brake)
        self.sequencer.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 200)

        self.shooterSensor = DigitalInput(constants.kShooterSensorChannel)
        self.shooterLoaded = DigitalInput(constants.kShooterSwitchChannel)

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
        SmartDashboard.putNumber("Flywheel Speed Override", 0)
        SmartDashboard.putNumber("Left Flywheel Speed Factor Override", 0)
        SmartDashboard.putNumber("Right Flywheel Speed Factor Override", 0)

    # Flywheel Bools
    def flywheelAtSpeed(self) -> bool:
        flywheelTolerance = self.flyWheelSpeed * 0.1
        if (
            abs(
                self.flyWheelSpeed * self.leftFlyWheelSpeedFactor
                - self.leftFlyWheel.get_velocity().value
            )
            < flywheelTolerance
            and abs(
                self.flyWheelSpeed * self.rightFlyWheelSpeedFactor
                - abs(self.rightFlyWheel.get_velocity().value)
            )
            < flywheelTolerance
        ):
            return True
        else:
            return False

    # Flywheel commands
    def setFlywheelSpeedForAmpCommand(self) -> Command:
        return self.runOnce(lambda: self.setFlywheelSpeed(20)).withName(
            "Set Flywheel Speed for Amp"
        )

    def setFlywheelSpeedForSpeakerCommand(self) -> Command:
        return self.runOnce(lambda: self.setFlywheelSpeed(20)).withName(
            "Set Flywheel Speed for Speaker"
        )

    def runFlywheelsCommand(self) -> Command:
        return self.runOnce(self.runFlywheels).withName("RunFlywheels")

    def stopFlywheelsCommand(self) -> Command:
        return self.runOnce(self.stopFlywheels).withName("StopFlywheels")

    # Flywheel Methods
    def setFlywheelSpeed(self, flywheelSpeed: float):
        self.flyWheelSpeed = flywheelSpeed

    def runFlywheels(self):
        # self.flyWheelSpeed = SmartDashboard.getNumber("Flywheel Speed", 0)
        speedOverride = SmartDashboard.getNumber("Flywheel Speed Override", 0)
        leftFactor = SmartDashboard.getNumber("Left Flywheel Speed Factor Override", 0)
        rightFactor = SmartDashboard.getNumber(
            "Right Flywheel Speed Factor Override", 0
        )
        if speedOverride > 0:
            self.flyWheelSpeed = speedOverride
        if leftFactor != 0:
            self.leftFlyWheelSpeedFactor = leftFactor
        if rightFactor != 0:
            self.rightFlyWheelSpeedFactor = rightFactor
        self.leftFlyWheel.set_control(
            VelocityVoltage(
                velocity=self.flyWheelSpeed * self.leftFlyWheelSpeedFactor, slot=0
            )
        )
        self.rightFlyWheel.set_control(
            VelocityVoltage(
                velocity=self.flyWheelSpeed * self.rightFlyWheelSpeedFactor, slot=0
            )
        )

    def stopFlywheels(self):
        self.leftFlyWheel.set_control(DutyCycleOut(0))
        self.rightFlyWheel.set_control(DutyCycleOut(0))

    def runFlywheelsCommandForDemo(self):
        return self.runOnce(self.runFlywheelsForDemo).withName("RunFlywheelsForDemo")

    def runFlywheelsForDemo(self):
        self.flyWheelSpeed = 20
        self.leftFlyWheel.set_control(
            VelocityVoltage(
                velocity=self.flyWheelSpeed * self.leftFlyWheelSpeedFactor, slot=0
            )
        )
        self.rightFlyWheel.set_control(
            VelocityVoltage(
                velocity=self.flyWheelSpeed * self.rightFlyWheelSpeedFactor, slot=0
            )
        )

    def leftFlywheelActualSpeed(self):
        return self.leftFlyWheel.get_velocity().value

    def rightFlywheelActualSpeed(self):
        return self.rightFlyWheel.get_velocity().value

    # Sequencer Commands
    def homeNoteCommand(self) -> Command:
        return PrintCommand("homeNoteCommand called.")
        # return (
        #     self.startEnd(
        #         lambda: self.controlSequencer(-0.5), lambda: self.controlSequencer(0)
        #     )
        #     .onlyWhile(self.noteInShooter)
        #     .andThen(
        #         self.startEnd(
        #             lambda: self.controlSequencer(0.2), lambda: self.controlSequencer(0)
        #         )
        #     )
        #     .until(self.noteInShooter)
        # )

    def stopSequencerCommand(self) -> Command:
        return self.runOnce(self.stopSequencer).withName("StopSequencer")

    def fireSequencerCommand(self) -> Command:
        return self.runOnce(self.fireWithSequencer).withName("RunSequencer")

    # sequencer methods
    def controlSequencer(self, percent):
        self.sequencerCommandedPercent = percent
        self.sequencer.set(
            TalonSRXControlMode.PercentOutput, self.sequencerCommandedPercent
        )

    def loadWithSequencer(self):
        self.controlSequencer(constants.kLoadWheelsPercent)

    def fireWithSequencer(self):
        self.controlSequencer(constants.kSequencerShootPercent)

    def stopSequencer(self):
        self.controlSequencer(0)

    # Shooter Bools
    def noteInShooter(self) -> bool:
        # return not self.shooterSensor.get()
        return not self.shooterLoaded.get()

    # Shooter Triggers
    def hasNote(self):
        return Trigger(lambda: self.noteInShooter())

    # Shooting Commands
    def stopShootingCommand(self) -> Command:
        return self.runOnce(self.stopShooting)

    def shootCommand(self) -> Command:
        return (
            self.runFlywheelsCommand()  # run the flywheel at the commanded speed
            .andThen(waitUntil(self.flywheelAtSpeed))
            .andThen(
                self.fireSequencerCommand()
            )  # run the sequencer to move the note into the flywheel
            .andThen(waitSeconds(0.5))  # wait until we no longer detect the note
            .andThen(
                self.stopSequencerCommand()
            )  # could also call stopShootingCommand at the very end
            .andThen(waitSeconds(1))  # wait 1 second
            .andThen(self.stopFlywheelsCommand())
        )

    # shooting Methods
    def stopShooting(self):
        self.stopSequencer()
        self.stopFlywheels()

    def periodic(self) -> None:
        self.logTelemetry()
        self.publishOnSmartDashboard()

    # Smart Dashboard publishing
    def publishOnSmartDashboard(self):
        SmartDashboard.putBoolean("Note in Shooter", self.noteInShooter())
        SmartDashboard.putBoolean("Flywheel At Speed", self.flywheelAtSpeed())
        SmartDashboard.putNumber("Left Flywheel Speed", self.leftFlywheelActualSpeed())
        SmartDashboard.putNumber(
            "Right Flywheel Speed", self.rightFlywheelActualSpeed()
        )
        # color = self.colorSensor.getColor()
        # SmartDashboard.putNumber("Color:Red", color.red)
        # SmartDashboard.putNumber("Color:Green", color.green)
        # SmartDashboard.putNumber("Color:Blue", color.blue)

    def logTelemetry(self):
        # self.leftFlyWheel.logData()
        # self.rightFlyWheel.logData()
        # self.sequencer.logData()
        self._flywheelCommandedVelocity.set(self.flyWheelSpeed)
        self._sequencerCommandedPercent.set(self.sequencerCommandedPercent)
        self._shooterSensorCurrentState.set(self.noteInShooter())
