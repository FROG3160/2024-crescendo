#The intake consists of a roller bar (TalonSRX), an intake wheel(SparkMax), and a transfer wheel(SparkMax
from rev import CANSparkMax
from commands2 import Subsystem
from constants import kIntakeRollerControllerID, kTransferWheelsID, kRollerSpeed, kTransferSpeed
from wpilib import DigitalInput, SmartDashboard
class Intake(Subsystem):
    
    def __init__(self):
        
        self.intakeMotor = CANSparkMax(kIntakeRollerControllerID, CANSparkMax.MotorType.kBrushless)
        self.transferMotor = CANSparkMax(kTransferWheelsID, CANSparkMax.MotorType.kBrushless)
        self.sensor = DigitalInput(0)

    def runIntake(self):
        self.intakeMotor.set(kRollerSpeed)

    def runTransfer(self):
        self.transferMotor.set(kTransferSpeed)

    def stopIntake(self):
        self.intakeMotor.set(0)

    def stopTransfer(self):
        self.transferMotor.set(0)

    def isIntakeRunning(self):
        return abs(self.intakeMotor.getAppliedOutput()) > 0.0
    
    def isTransferRunning(self):
        return abs(self.transferMotor.getAppliedOutput()) > 0.0
    
    def periodic(self) -> None:
        SmartDashboard.putBoolean("dioSensor", self.sensor.get())
