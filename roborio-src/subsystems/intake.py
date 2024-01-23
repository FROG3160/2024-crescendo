#The intake consists of a roller bar (TalonSRX), an intake wheel(SparkMax), and a transfer wheel(SparkMax
from rev import CANSparkMax

from commands2 import Subsystem
from constants import kIntakeWheelsControllerID, kRollerBarControllerID, kTransferWheelsID

class Intake(Subsystem):
    def __init__(self):
        
        self.intakeWheelController = CANSparkMax(kIntakeWheelsControllerID, CANSparkMax.MotorType.kBrushless)
        self.transferWheelController = CANSparkMax(kTransferWheelsID, CANSparkMax.MotorType.kBrushless)

