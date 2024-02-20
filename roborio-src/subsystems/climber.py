from commands2 import Subsystem
from FROGlib.motors import FROGTalonFX, FROGTalonFXConfig
from phoenix6.controls import (
    PositionDutyCycle,
    VelocityDutyCycle,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
    MotionMagicVoltage
)

class Climber(Subsystem):
    
    def __init__(
        self,
        left_climber_id: int,
        left_climber_config: FROGTalonFXConfig,
        right_climber_id: int,
        right_climber_config: FROGTalonFXConfig
    ):

        self.leftClimber = FROGTalonFX(left_climber_id, left_climber_config)
        self.rightClimber = FROGTalonFX(right_climber_id, right_climber_config)

    def setVoltage(self, climberVoltage: float):
        self.climberVoltage = climberVoltage
        self.leftClimber.set_control(
            VoltageOut(volt=self.climberVoltage)
        )
        self.rightClimber.set_control(
            VoltageOut(volt=-self.climberVoltage)
        )

    