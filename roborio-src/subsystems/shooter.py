from commands2 import Subsystem
from FROGlib.motors import FROGTalonFX, FROGTalonFXConfig
from phoenix6.controls import (
    PositionDutyCycle,
    VelocityDutyCycle,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
)
from rev import CANSparkMax
import constants
import configs


class Shooter(Subsystem):

    flyWheelSpeed: float
    leadScrewPosition: float

    def __init__(
        self,
        lead_screw_id: int,
        lead_screw_config: FROGTalonFXConfig,
        left_flywheel_id: int,
        left_flywheel_config: FROGTalonFXConfig,
        right_flywheel_id: int,
        right_flywheel_config: FROGTalonFXConfig,
        sequencer_id: int,
        sequencer_motor_type,
    ):

        # Very rudimentary system that allows speed control  of
        # the lead screw and flywheel on the operator Xbox controller triggers.
        # The NEO 550 will be used to pull and push the note out of
        # the intake and into the flywheel.

        self.leadScrew = FROGTalonFX(lead_screw_id, lead_screw_config)
        self.leftFlyWheel = FROGTalonFX(left_flywheel_id, left_flywheel_config)
        self.rightFlyWheel = FROGTalonFX(right_flywheel_id, right_flywheel_config)
        self.sequencer = CANSparkMax(sequencer_id, sequencer_motor_type)

    def setFlywheelSpeed(self, flywheelSpeed: float):
        self.flywheelSpeed = flywheelSpeed

    def runFlywheels(self):
        self.leftFlyWheel.set_control(
            VelocityVoltage(velocity=self.flywheelSpeed, slot=0)
        )
        self.rightFlyWheel.set_control(
            VelocityVoltage(velocity=-self.flywheelSpeed, slot=0)
        )

    def getFlyWheelSpeedsIsTrue(self) -> bool:
        if (
            self.flywheelSpeed == self.leftFlyWheel.get_velocity()
            and self.flywheelSpeed == abs(self.rightFlyWheel.get_velocity())
        ):
            return True
        else:
            return False

    def runSequencer(self):
        self.sequencer.set(constants.kSequencerSpeed)

    def stopSequencer(self):
        self.sequencer.stopMotor()

    def setLeadscrewPosition(self, leadscrewPosition: float):
        self.leadscrewPosition = leadscrewPosition
        self.leadScrew.set_position(self.leadscrewPosition)

    def getLeadscrewPositionIsTrue(self) -> bool:
        if self.leadscrewPosition == self.leadScrew.get_position():
            return True
        else:
            return False
