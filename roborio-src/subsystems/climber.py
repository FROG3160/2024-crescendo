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

    def isAtHome(self, motor: FROGTalonFX) -> bool:
        return self.getTorque(motor) > 8

    def leftClimberAtHome(self) -> bool:
        return self.isAtHome(self.leftClimber)

    def rightClimberAtHome(self) -> bool:
        return self.isAtHome(self.rightClimber)

    def homeMotor(self, motor):
        """Runs motor with a low voltage in order to home it.

        Args:
            motor (FROGTalonFX): the motor to run
        """
        motorControl = VoltageOut(0.5)
        motor.set_control(motorControl)

    def homeLeft(self):
        self.homeMotor(self.leftClimber)

    def homeRight(self):
        self.homeMotor(self.rightClimber)

    def resetMotorPosition(self, motor: FROGTalonFX, position: float):
        motor.set_position(position)

    def resetLeftClimberPosition(self):
        self.resetMotorPosition(self.leftClimber, 85)

    def resetRightClimberPosition(self):
        self.resetMotorPosition(self.rightClimber, 85)

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

    def get_homeLeftClimber(self) -> Command:
        return self.startEnd(self.homeLeft, self.resetLeftClimberPosition).until(
            self.leftClimberAtHome
        )

    def get_homeRightClimber(self) -> Command:
        return self.startEnd(self.homeRight, self.resetRightClimberPosition).until(
            self.rightClimberAtHome
        )

    def logTelemetry(self):
        self._motorCommandedValuePub.set(self.motorCommandValue)
        self._motorControlTypePub.set(self.motorControlType)
        self._leftMotorTorque.set(self.getTorque(self.leftClimber))
        self._rightMotorTorque.set(self.getTorque(self.rightClimber))
