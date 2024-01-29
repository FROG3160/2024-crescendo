from commands2 import Subsystem
from FROGlib.motors import FROGTalonFX, FROGTalonFXConfig
from phoenix6.controls import PositionDutyCycle, VelocityDutyCycle, VelocityVoltage, PositionVoltage
from rev import CANSparkMax
from constants import kLeadScrewControllerID, kFlyWheelControllerID, kSequencerControllerID
import configs

class Shooter(Subsystem):

    def __init__(
        self,
        lead_screw_id: int,
        lead_screw_config: FROGTalonFXConfig,
        flywheel_id: int,
        flywheel_config: FROGTalonFXConfig,
        sequencer_id: int,
        sequencer_motor_type
    ):
        
        # Very rudimentary system that allows speed control  of 
        # the lead screw and flywheel on the operator Xbox controller triggers.
        # The NEO 550 will be used to pull and push the note out of
        # the intake and into the flywheel.
        
        self.leadScrew = FROGTalonFX(lead_screw_id, lead_screw_config)
        self.flyWheel = FROGTalonFX(flywheel_id, flywheel_config)
        self.sequencer = CANSparkMax(sequencer_id, sequencer_motor_type)
        


        









