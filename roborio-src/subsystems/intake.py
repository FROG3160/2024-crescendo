#The intake consists of a roller bar (TalonSRX), an intake wheel(SparkMax), and a transfer wheel(SparkMax
from rev import CANSparkMax
from commands2 import Subsystem
from constants import kIntakeRollerControllerID, kTransferWheelsID, kRollerSpeed, kTransferSpeed, kIntakeSensorChannel
from wpilib import DigitalInput, SmartDashboard
from ntcore import NetworkTableInstance
class Intake(Subsystem):
    
    def __init__(self):
        
        self.intakeMotor = CANSparkMax(kIntakeRollerControllerID, CANSparkMax.MotorType.kBrushless)
        self.transferMotor = CANSparkMax(kTransferWheelsID, CANSparkMax.MotorType.kBrushless)
        self.intakeSensor = DigitalInput(kIntakeSensorChannel)

        self._intakeMotorCommandedSpeed = NetworkTableInstance.getDefault().getFloatTopic(
            f'/Intake_Motor{kIntakeRollerControllerID}/velocity').publish()
        self._transferMotorCommandedSpeed = NetworkTableInstance.getDefault().getFloatTopic(
            f'/Transfer_Motor{kTransferWheelsID}/velocity').publish()
        self._intakeSensorCurrentState = NetworkTableInstance.getDefault().getBooleanTopic(
            f'/Note_In_Intake_Is_True: Channel 0/boolean').publish()

    def runIntake(self):
        self.intakeMotorCommandedSpeed = kRollerSpeed
        self.intakeMotor.set(self.intakeMotorCommandedSpeed)

    def runTransfer(self):
        self.transferMotorCommandedSpeed = kTransferSpeed
        self.transferMotor.set(self.transferMotorCommandedSpeed)

    def stopIntake(self):
        self.intakeMotor.set(0)

    def stopTransfer(self):
        self.transferMotor.set(0)

    def isIntakeRunning(self):
        return abs(self.intakeMotor.getAppliedOutput()) > 0.0
    
    def isTransferRunning(self):
        return abs(self.transferMotor.getAppliedOutput()) > 0.0
    
    def noteInIntakeIsTrue(self) -> bool:
        self.noteNotInIntake = self.intakeSensor.get()
        return not self.noteNotInIntake
    
    def periodic(self) -> None:
        SmartDashboard.putBoolean("ShooterDioSensor", self.noteInIntakeIsTrue())

    def logIntakeComponentValues(self):
        self._intakeMotorCommandedSpeed.set(self.intakeMotorCommandedSpeed)
        self._transferMotorCommandedSpeed.set(self.transferMotorCommandedSpeed)
        self._intakeSensorCurrentState.set(self.noteInIntakeIsTrue())