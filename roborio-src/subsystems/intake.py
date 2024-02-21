# The intake consists of a roller bar (TalonSRX), an intake wheel(SparkMax), and a transfer wheel(SparkMax
from enum import Enum, auto
from FROGlib.motors import FROGSparkMax
from commands2 import Subsystem, Command
from constants import (
    kIntakeRollerControllerID,
    kTransferWheelsID,
    kRollerSpeed,
    kTransferSpeed,
    kIntakeSensorChannel,
)
from wpilib import DigitalInput, SmartDashboard
from ntcore import NetworkTableInstance


class IntakeSubsystem(Subsystem):
    class State(Enum):
        Disabled = auto()  # Intake shouldn't operate.
        Waiting = auto()  # No note, and not running intake
        Intaking = auto()  # running intake roller
        Holding = auto()  # Note loaded, but not yet transferred
        Transferring = auto()  # moving note to shooter

    def __init__(self, table: str = "Undefined"):
        super().__init__()
        self.setName("Intake_Subsystem")

        self.intakeMotor = FROGSparkMax(
            kIntakeRollerControllerID, FROGSparkMax.MotorType.kBrushless
        )
        self.transferMotor = FROGSparkMax(
            kTransferWheelsID, FROGSparkMax.MotorType.kBrushless
        )
        self.intakeEmptySensor = DigitalInput(kIntakeSensorChannel)

        nt_table = f"{table}/{self.getName()}"

        self._intakeMotorCommandedSpeed = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/intakeRoller/speed")
            .publish()
        )
        self._transferMotorCommandedSpeed = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/transferMotor/speed")
            .publish()
        )
        self._intakeSensorCurrentState = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/noteDetected/state")
            .publish()
        )
        self.intakeMotorCommandedSpeed = 0
        self.transferMotorCommandedSpeed = 0

        self.state = self.State.Disabled

    def intakeCommand(self) -> Command:
        # return a command that starts the intakeMotor
        # and then waits for noteDetected() goes True
        return (
            self.startEnd(self.runIntake, self.stopIntake)
            .until(self.noteDetected)
            .withName("RunIntake")
        )

    def stopIntakeCommand(self) -> Command:
        return self.runOnce(self.stopIntake).withName("StopIntake")

    def runIntake(self) -> None:
        self.state = self.State.Intaking
        self.intakeMotorCommandedSpeed = kRollerSpeed
        self.intakeMotor.set(self.intakeMotorCommandedSpeed)

    def transfer(self):
        self.transferMotorCommandedSpeed = kTransferSpeed
        self.transferMotor.set(self.transferMotorCommandedSpeed)

    def stopIntake(self):
        self.intakeMotorCommandedSpeed = 0
        self.intakeMotor.set(self.intakeMotorCommandedSpeed)
        if self.noteDetected():
            self.state = self.State.Holding
        else:
            self.state = self.State.Waiting

    def stopTransfer(self):
        self.transferMotorCommandedSpeed = 0

    def isIntakeRunning(self):
        return abs(self.intakeMotor.getAppliedOutput()) > 0.0

    def isTransferRunning(self):
        return abs(self.transferMotor.getAppliedOutput()) > 0.0

    def noteDetected(self) -> bool:
        # intakeEmptySensor returns True when it's not detecting
        # anything, so we negate the boolean to determine when a
        # note is detected.
        return not self.intakeEmptySensor.get()

    def periodic(self) -> None:
        self.logTelemetry()
        SmartDashboard.putBoolean("IntakeDioSensor", self.noteDetected())

    def logTelemetry(self):
        self._intakeMotorCommandedSpeed.set(self.intakeMotorCommandedSpeed)
        self._transferMotorCommandedSpeed.set(self.transferMotorCommandedSpeed)
        self._intakeSensorCurrentState.set(self.noteDetected())
