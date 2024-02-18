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
from rev import CANSparkMax
import constants
import configs
from ntcore import NetworkTableInstance
from wpilib import DigitalInput


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
        sequencer_motor_type
    ):

        # Very rudimentary system that allows speed control  of
        # the lead screw and flywheel on the operator Xbox controller triggers.
        # The NEO 550 will be used to pull and push the note out of
        # the intake and into the flywheel.

        self.leadScrew = FROGTalonFX(lead_screw_id, lead_screw_config)
        self.leftFlyWheel = FROGTalonFX(left_flywheel_id, left_flywheel_config)
        self.rightFlyWheel = FROGTalonFX(right_flywheel_id, right_flywheel_config)
        self.sequencer = CANSparkMax(sequencer_id, sequencer_motor_type)
        self.shooterSensor = DigitalInput(constants.kShooterSensorChannel)

        self._leftFlywheelCommandedSpeed = NetworkTableInstance.getDefault().getFloatTopic(
            f'/Left_Flywheel: ID 52/velocity').publish()
        self._rightFlywheelCommandedSpeed = NetworkTableInstance.getDefault().getFloatTopic(
            f'/Right_Flywheel: ID 53/velocity').publish()
        self._leadscrewCommandedSpeed = NetworkTableInstance.getDefault().getFloatTopic(
            f'/Lead_Screw: ID 51/position').publish()
        self._sequencerCommandedSpeed = NetworkTableInstance.getDefault().getFloatTopic(
            f'/Sequencer: ID 54/velocity').publish()
        self._shooterSensorCurrentState = NetworkTableInstance.getDefault().getBooleanTopic(
            f'/Note_In_Shooter_Is_True: Channel 1/boolean').publish()

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
        self.sequencerCommandedSpeed = constants.kSequencerSpeed
        self.sequencer.set(self.sequencerCommandedSpeed)

    def stopSequencer(self):
        self.sequencer.stopMotor()

    def setLeadscrewPosition(self, leadscrewPosition: float):
        self.leadscrewPosition = leadscrewPosition
        self.leadScrew.set_control(
            MotionMagicVoltage(
                position=leadscrewPosition,
                slot=1
            )
        )

    def getLeadscrewPositionIsTrue(self) -> bool:
        if self.leadscrewPosition == self.leadScrew.get_position():
            return True
        else:
            return False

    def zeroLeadScrew(self):
        self.leadScrew.set_position(0)

    def noteInShooterIsTrue(self) -> bool:
        self.noteNotInSensor = self.shooterSensor.get()
        return not self.noteNotInSensor
    
    def logShooterComponentValues(self):
        self._leftFlywheelCommandedSpeed.set(self.flywheelSpeed)
        self._rightFlywheelCommandedSpeed.set(self.flywheelSpeed)
        self._leadscrewCommandedSpeed.set(self.leadscrewPosition)
        self._sequencerCommandedSpeed.set(self.sequencerCommandedSpeed)
        self._shooterSensorCurrentState.set(self.noteInShooterIsTrue())
