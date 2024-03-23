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
from enum import Enum, auto


def clamp(v, minval, maxval):
    return max(min(v, maxval), minval)


class ElevationSubsystem(Subsystem):

    class State(Enum):
        Disabled = auto()
        AtHome = auto()
        AutoAdjusting = auto()
        AtPosition = auto()

    def __init__(self, parent_nt: str = "Subsystems"):
        super().__init__()
        self.setName("Elevation")
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
        self.speakerDistance = None
        self.state = self.State.Disabled
        SmartDashboard.putNumber("Elevation Position Override", 0)

    def getCurrentLeadscrewPosition(self) -> float:
        return self.leadscrew.get_position().value

    def calcPositionForSpeaker(self):
        # calculatedPosition = self.tagDistance * 3.9877 - 6.3804
        calculatedPosition = (
            (self.speakerDistance**2 * -1.4822)
            + (self.speakerDistance * 13.764)
            - 15.187
        )
        return clamp(calculatedPosition, 1, 17)

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
        position_diff = self.leadscrewPosition - self.getCurrentLeadscrewPosition()
        if abs(position_diff) < constants.kLeadscrewPositionTolerance:
            self.state = self.State.AtPosition
            return True
        else:
            return False

    def runMotorWithControl(self):
        positionOverride = SmartDashboard.getNumber("Elevation Position Override", 0)
        if positionOverride > 0:
            self.leadscrewPosition = positionOverride
        self.leadscrew.set_control(
            MotionMagicVoltage(position=self.leadscrewPosition, slot=1)
        )

    def autoMoveWithDistance(self):
        self.state = self.State.AutoAdjusting
        self.setLeadscrewPosition(self.calcPositionForSpeaker())
        self.runMotorWithControl()

    def autoMoveRunWithDistanceCommand(self) -> Command:
        return self.run(self.autoMoveWithDistance)

    def moveToAmpPosition(self):
        self.setLeadscrewPosition(56)
        self.runMotorWithControl()

    def moveToAmpPositionCommand(self):
        return self.runOnce(self.moveToAmpPosition)

    def moveToLoadPosition(self):
        self.setLeadscrewPosition(0)
        self.runMotorWithControl()

    def moveToLoadPositionCommand(self):
        return self.runOnce(self.moveToLoadPosition)

    def readyToLoad(self):
        return self.leadscrewPosition == 0 and self.leadscrewAtPosition()

    def periodic(self):
        self.logTelemetry()
        SmartDashboard.putBoolean("ShooterPositionAtHome", self.shooterAtHome())

    def publishOnSmartDashboard(self):
        SmartDashboard.putString("Shooter State", str(self.state.name))

    def runLeadscrewForward(self):
        self.leadscrew.set_control(VoltageOut(1.0))

    def runLeadscrewBackward(self):
        self.leadscrew.set_control(VoltageOut(-0.7))

    def runLeadscrewForwardCommand(self) -> Command:
        return self.run(self.runLeadscrewForward)

    def runLeadscrewBackwardCommand(self) -> Command:
        return self.run(self.runLeadscrewBackward)

    def resetPosition(self, position):
        self.leadscrew.set_position(position)
        self.leadscrew.config.software_limit_switch.with_forward_soft_limit_threshold(
            constants.kLeadscrewForwardLimit
        ).with_forward_soft_limit_enable(True)
        self.leadscrew.config.software_limit_switch.with_reverse_soft_limit_threshold(
            constants.kLeadscrewReverseLimit
        ).with_reverse_soft_limit_enable(True)

    def setLeadscrewCommand(self) -> Command:
        return self.runOnce(self.runMotorWithControl).withName("SetLeadscrew")

    def setLeadscrewPosition(self, leadscrewPosition: float):
        self.leadscrewPosition = leadscrewPosition

    def setSpeakerDistance(self, distance):
        self.speakerDistance = distance

    def shooterAtHome(self) -> bool:
        self.shooterPositionSensorNotTripped = self.shooterPositionSensor.get()
        if not self.shooterPositionSensorNotTripped:
            self.state = self.State.AtHome
        return not self.shooterPositionSensorNotTripped

    def shooterNotAtHome(self) -> bool:
        return self.shooterPositionSensorNotTripped

    def stopLeadscrew(self):
        self.leadscrew.set_control(VoltageOut(0))

    def zeroLeadscrew(self):
        self.leadscrew.set_position(0)

    def logTelemetry(self):
        self._shooterCommandedPosition.set(self.leadscrewPosition)
        self._shooterActualPositionPub.set(self.getCurrentLeadscrewPosition())
        self._leadscrewAtPosition.set(self.leadscrewAtPosition())
