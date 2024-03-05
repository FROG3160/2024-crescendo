# The intake consists of a roller bar (TalonSRX), an intake wheel(SparkMax), and a transfer wheel(SparkMax
from enum import Enum, auto

from phoenix5 import StatusFrameEnhanced, TalonSRXControlMode
from FROGlib.motors import (
    FROGSparkMax,
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGTalonSRX,
    FROGTalonSRXConfig,
)
from commands2 import Subsystem, Command
from constants import (
    kIntakeRollerControllerID,
    kTransferWheelsID,
    kRollerVoltage,
    kIntakeSensorChannel,
    kRollerTransferVoltage,
    kLoadWheelsPercent,
)
from wpilib import DigitalInput, SmartDashboard
from ntcore import NetworkTableInstance
import configs
from phoenix6.controls import DutyCycleOut, VoltageOut


class IntakeSubsystem(Subsystem):
    class State(Enum):
        Disabled = auto()  # Intake shouldn't operate.
        Waiting = auto()  # No note, and not running intake
        Intaking = auto()  # running intake roller
        Holding = auto()  # Note loaded, but not yet transferred
        Transferring = auto()  # moving note to shooter

    def __init__(self, parent_nt: str = "Subsystems"):
        """A Subsystem that takes notes from the field and delivers them to the shooter

        Args:
            parent_nt (str, optional): The parent network table to put this systems log data under
        """
        super().__init__()
        self.setName("Intake")
        nt_table = f"{parent_nt}/{self.getName()}"

        self.intakeMotor = FROGTalonFX(
            kIntakeRollerControllerID,
            configs.intakeMotorConfig,
            parent_nt=f"{nt_table}",
            motor_name="IntakeRoller",
        )
        self.transferMotor = FROGTalonSRX(
            kTransferWheelsID,
            FROGTalonSRXConfig(),
            parent_nt=f"{nt_table}",
            motor_name="TransferMotor",
        )
        self.transferMotor.setStatusFramePeriod(
            StatusFrameEnhanced.Status_2_Feedback0, 200
        )
        self.intakeEmptySensor = DigitalInput(kIntakeSensorChannel)

        self._intakeMotorCommandedVoltage = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/intakeRoller/speed")
            .publish()
        )
        self._transferMotorCommandedPercent = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/transferMotor/speed")
            .publish()
        )
        self._intakeSensorCurrentState = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/noteDetected/state")
            .publish()
        )
        self.intakeMotorCommandedVoltage = 0
        self.transferMotorCommandedPercent = 0

        self.state = self.State.Disabled

    def intakeCommand(self) -> Command:
        # return a command that starts the intakeMotor
        # and then waits for noteDetected() goes True
        return (
            self.startEnd(self.runIntake, self.stopIntake)
            .until(self.noteInIntake)
            .withName("RunIntake")
        )

    def stopIntakeCommand(self) -> Command:
        # used to kill the intake outside normal operations
        return self.runOnce(self.stopIntake).withName("StopIntake")

    def transferCommand(self) -> Command:
        # return a command that starts the transferMotor
        # and then waits for noteDetected() goes True
        return self.startEnd(self.runTransfer, self.stopTransfer).withName(
            "RunTransfer"
        )

    def stopTransferCommand(self) -> Command:
        # used to kill the transfer motor outside normal operations
        return self.runOnce(self.stopIntake).withName("StopTransfer")

    def runIntake(self) -> None:
        self.state = self.State.Intaking
        self.controlIntake(kRollerVoltage)

    def controlIntake(self, voltage):
        # setting it to an attribute so we can log the value if needed
        self.intakeMotorCommandedVoltage = voltage
        self.intakeMotor.set_control(VoltageOut(self.intakeMotorCommandedVoltage))

    def controlTransfer(self, percent):
        self.transferMotorCommandedPercent = percent
        self.transferMotor.set(
            TalonSRXControlMode.PercentOutput, self.transferMotorCommandedPercent
        )

    def runTransfer(self):
        self.state = self.State.Transferring
        self.controlTransfer(kLoadWheelsPercent)
        self.controlIntake(kRollerTransferVoltage)

    def stopIntake(self):
        if self.noteInIntake():
            self.state = self.State.Holding
        else:
            self.state = self.State.Waiting
        self.controlIntake(0)

    def stopTransfer(self):
        if self.noteInIntake():
            self.state = self.State.Holding
        else:
            self.state = self.State.Waiting
        self.controlTransfer(0)
        self.stopIntake()

    def isIntakeRunning(self):
        return abs(self.intakeMotor.getMotorVoltage()) > 0.0

    def isTransferRunning(self):
        return abs(self.transferMotor.getMotorVoltage()) > 0.0

    def noteInIntake(self) -> bool:
        # intakeEmptySensor returns True when it's not detecting
        # anything, so we negate the boolean to determine when a
        # note is detected.
        return not self.intakeEmptySensor.get()

    def periodic(self) -> None:
        #  as a method of subystem, this is run every loop
        # self.logTelemetry()
        self.publishOnSmartDashboard()

    def publishOnSmartDashboard(self):
        SmartDashboard.putBoolean("IntakeDioSensor", self.noteInIntake())

    def logTelemetry(self):
        self.intakeMotor.logData()
        self.transferMotor.logData()
        self._intakeMotorCommandedVoltage.set(self.intakeMotorCommandedVoltage)
        self._transferMotorCommandedPercent.set(self.transferMotorCommandedPercent)
        self._intakeSensorCurrentState.set(self.noteInIntake())
