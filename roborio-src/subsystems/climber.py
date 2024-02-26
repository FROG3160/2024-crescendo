from commands2 import Command, Subsystem
from FROGlib.motors import FROGTalonFX, FROGTalonFXConfig
from phoenix6.controls import (
    PositionDutyCycle,
    VelocityDutyCycle,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
    MotionMagicVoltage,
)
from ntcore import NetworkTableInstance
from constants import kLeftClimberControllerID, kRightClimberControllerID
from configs import leftClimberMotorConfig, rightClimberMotorConfig


class ClimberSubsystem(Subsystem):

    def __init__(
        self,
        parent_nt: str = "Subsystems",
    ):
        super().__init__()
        self.setName("Climber")
        nt_table = f"{parent_nt}/{self.getName()}"
        self.leftClimber = FROGTalonFX(
            kLeftClimberControllerID,
            leftClimberMotorConfig,
            parent_nt=f"{nt_table}",
            motor_name="LeftMotor",
        )
        self.rightClimber = FROGTalonFX(
            kRightClimberControllerID,
            rightClimberMotorConfig,
            parent_nt=f"{nt_table}",
            motor_name="RightMotor",
        )
        self.motorCommandValue = 0
        self.motorControlType = ""

        self._motorCommandedValuePub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/command_value")
            .publish()
        )
        self._motorControlTypePub = (
            NetworkTableInstance.getDefault()
            .getStringTopic(f"{nt_table}/command_type")
            .publish()
        )
        self._leftMotorTorque = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/torque/left")
            .publish()
        )
        self._rightMotorTorque = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/torque/right")
            .publish()
        )

    def getTorque(self, motor: FROGTalonFX) -> float:
        return motor.get_torque_current().value

    def setVoltage(self, climberVoltage: float):
        motorControl = VoltageOut(climberVoltage)
        self.motorCommandValue = motorControl.output
        self.motorControlType = motorControl.name
        self.leftClimber.set_control(motorControl)
        self.rightClimber.set_control(motorControl)

    def extend(self):
        self.setVoltage(-3)

    def retract(self):
        self.setVoltage(3)

    def retractSlow(self):
        self.setVoltage(0.5)

    def stop(self):
        self.setVoltage(0)

    def periodic(self) -> None:
        self.logTelemetry()

    def get_ExtendCommand(self) -> Command:
        return self.startEnd(self.extend, self.stop)

    def get_RetractCommand(self) -> Command:
        return self.startEnd(self.retract, self.stop)

    def logTelemetry(self):
        self.leftClimber.logData()
        self.rightClimber.logData()
        self._motorCommandedValuePub.set(self.motorCommandValue)
        self._motorControlTypePub.set(self.motorControlType)
        self._leftMotorTorque.set(self.getTorque(self.leftClimber))
        self._rightMotorTorque.set(self.getTorque(self.rightClimber))
