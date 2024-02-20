#The intake consists of a roller bar (TalonSRX), an intake wheel(SparkMax), and a transfer wheel(SparkMax
from FROGlib.motors import FROGSparkMax
from commands2 import Subsystem
from constants import (
    kIntakeRollerControllerID,
    kTransferWheelsID,
    kRollerSpeed,
    kTransferSpeed,
    kIntakeSensorChannel,
)
from wpilib import DigitalInput, SmartDashboard
from ntcore import NetworkTableInstance


class Intake(Subsystem):   
    def __init__(self):
        self.intakeMotor = FROGSparkMax(kIntakeRollerControllerID, FROGSparkMax.MotorType.kBrushless)
        self.transferMotor = FROGSparkMax(kTransferWheelsID, FROGSparkMax.MotorType.kBrushless)
        self.sensor = DigitalInput(0)

    def __init__(self, table: str = "Undefined"):

        self.intakeMotor = CANSparkMax(
            kIntakeRollerControllerID, CANSparkMax.MotorType.kBrushless
        )
        self.transferMotor = CANSparkMax(
            kTransferWheelsID, CANSparkMax.MotorType.kBrushless
        )
        self.intakeEmptySensor = DigitalInput(kIntakeSensorChannel)
        nt_table = f"{table}/{type(self).__name__}"

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

    def intake(self):
        self.intakeMotorCommandedSpeed = kRollerSpeed
        self.intakeMotor.set(self.intakeMotorCommandedSpeed)

    def transfer(self):
        self.transferMotorCommandedSpeed = kTransferSpeed
        self.transferMotor.set(self.transferMotorCommandedSpeed)

    def stopIntake(self):
        self.intakeMotorCommandedSpeed = 0

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
        self.intakeMotor.set(self.intakeMotorCommandedSpeed)
        self.transferMotor.set(self.transferMotorCommandedSpeed)
        self.logTelemetry()
        SmartDashboard.putBoolean("IntakeDioSensor", self.noteDetected())

    def logTelemetry(self):
        self._intakeMotorCommandedSpeed.set(self.intakeMotorCommandedSpeed)
        self._transferMotorCommandedSpeed.set(self.transferMotorCommandedSpeed)
        self._intakeSensorCurrentState.set(self.noteDetected())
