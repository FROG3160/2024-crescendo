from commands2 import Subsystem
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


class ClimberSubsystem(Subsystem):

    def __init__(
        self,
        left_climber_id: int,
        left_climber_config: FROGTalonFXConfig,
        right_climber_id: int,
        right_climber_config: FROGTalonFXConfig,
        parent_nt: str = "Subsystems",
    ):
        super().__init__()
        self.setName("Climber")
        self.leftClimber = FROGTalonFX(left_climber_id, left_climber_config)
        self.rightClimber = FROGTalonFX(right_climber_id, right_climber_config)
        nt_table = f"{parent_nt}/{self.getName()}"
        self.motorCommandedSpeed = 0

        self._motorCommandedSpeedPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/motor/speed")
            .publish()
        )

    def setVoltage(self, climberVoltage: float):
        self.climberVoltage = climberVoltage
        self.leftClimber.set_control(VoltageOut(volt=self.climberVoltage))
        self.rightClimber.set_control(VoltageOut(volt=-self.climberVoltage))

    def periodic(self) -> None:
        self.logTelemetry()

    def logTelemetry(self):
        self._motorCommandedSpeedPub.set(self.motorCommandedSpeed)
