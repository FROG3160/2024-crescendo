from commands2 import Subsystem, Command
from ntcore import NetworkTableInstance
from wpilib import DigitalInput, SmartDashboard
from FROGlib.motors import (
    FROGTalonFX,
    FROGTalonFXConfig,
    FROGSparkMax,
    FROGTalonSRX,
    FROGTalonSRXConfig,
)
from phoenix6.controls import (
    PositionDutyCycle,
    VelocityDutyCycle,
    VelocityVoltage,
    PositionVoltage,
    VoltageOut,
    MotionMagicVoltage,
)
import constants
from configs import leadscrewConfig


class ElevationSubsystem(Subsystem):

    def __init__(self, parent_nt: str = "Subsystems"):
        super().__init__()
        self.setName("Climber")
        nt_table = f"{parent_nt}/{self.getName()}"

        self.leadscrew = FROGTalonFX(
            constants.kLeadscrewControllerID,
            leadscrewConfig,
            parent_nt=f"{nt_table}",
            motor_name="Leadscrew",
        )
        self.shooterPositionSensor = DigitalInput(
            constants.kShooterPositionSensorChannel
        )
        self.leadscrewPosition = 0.0

        self._shooterCommandedPosition = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/CommandedPosition")
            .publish()
        )
        self._shooterActualPositionPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{nt_table}/ActualPosition")
            .publish()
        )
        self._leadscrewAtPosition = (
            NetworkTableInstance.getDefault()
            .getBooleanTopic(f"{nt_table}/AtPosition")
            .publish()
        )

    def getLeadscrewPosition(self) -> float:
        return self.leadscrew.get_position().value

    def homeShooterCommand(self) -> Command:
        return (
            # move with positive voltage until sensor no longer reading
            # move negative voltage until sensor reads
            # set position
            self.startEnd(self.runLeadscrewForward, self.stopLeadscrew)
            .onlyWhile(self.shooterAtHome)
            .andThen(
                self.startEnd(self.runLeadscrewBackward, self.stopLeadscrew).until(
                    self.shooterAtHome
                )
            )
            .finallyDo(lambda interrupted: self.resetPosition(0))
        )

    def leadscrewAtPosition(self) -> bool:
        position_diff = self.leadscrewPosition - self.getLeadscrewPosition()
        if abs(position_diff) < constants.kLeadscrewPositionTolerance:
            return True
        else:
            return False

    def moveLeadscrewToPosition(self):
        self.leadscrew.set_control(
            MotionMagicVoltage(position=self.leadscrewPosition, slot=1)
        )

    def moveToLoad(self):
        self.setLeadscrewPosition(0)
        self.moveLeadscrewToPosition()

    def readyToLoad(self):
        return self.leadscrewPosition == 0 and self.leadscrewAtPosition()

    def periodic(self):
        self.logTelemetry()
        SmartDashboard.putBoolean("ShooterPositionDioSensor", self.shooterAtHome())

    def runLeadscrewForward(self):
        self.leadscrew.set_control(VoltageOut(0.5))

    def runLeadscrewBackward(self):
        self.leadscrew.set_control(VoltageOut(-0.35))

    def runLeadscrewForwardCommand(self) -> Command:
        return self.run(self.runLeadscrewForward)

    def runLeadscrewBackwardCommand(self) -> Command:
        return self.run(self.runLeadscrewBackward)

    def resetPosition(self, position):
        self.leadscrew.set_position(position)

    def setLeadscrewCommand(self) -> Command:
        return self.runOnce(self.moveLeadscrewToPosition).withName("SetLeadscrew")

    def setLeadscrewPosition(self, leadscrewPosition: float):
        self.leadscrewPosition = leadscrewPosition

    def shooterAtHome(self) -> bool:
        return not self.shooterPositionSensor.get()

    def shooterNotAtHome(self) -> bool:
        return self.shooterPositionSensor.get()

    def stopLeadscrew(self):
        self.leadscrew.set_control(VoltageOut(0))

    def zeroLeadscrew(self):
        self.leadscrew.set_position(0)

    def logTelemetry(self):
        self._shooterCommandedPosition.set(self.leadscrewPosition)
        self._shooterActualPositionPub.set(self.getLeadscrewPosition())
        self._leadscrewAtPosition.set(self.leadscrewAtPosition())
