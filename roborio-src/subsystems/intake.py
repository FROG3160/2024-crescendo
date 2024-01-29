#The intake consists of a roller bar (TalonSRX), an intake wheel(SparkMax), and a transfer wheel(SparkMax
from rev import CANSparkMax
from commands2 import Subsystem
from constants import kIntakeRollerControllerID, kTransferWheelsID

class Intake(Subsystem):
    
    def __init__(self):
        
        self.intakeWheel = CANSparkMax(kIntakeRollerControllerID, CANSparkMax.MotorType.kBrushless)
        self.transferWheel = CANSparkMax(kTransferWheelsID, CANSparkMax.MotorType.kBrushless)



