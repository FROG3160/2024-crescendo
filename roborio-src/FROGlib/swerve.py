from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from phoenix6 import TalonFX, CANcoder, PositionDutyCycle, VelocityDutyCycle, TalonFXConfiguration
from utils import DriveUnit
from wpimath.units import radiansToRotations

class SwerveModule:
    def __init__(
        self,
        name: str,
        location: Translation2d,
        gearing: list,
        wheel_diameter: float
    ):
        # set initial states for the component

        #create/configure drive motor
        #create/configure steer motor
        #create/configure cancoder

        #set module location
        self.location = location
    
        #set module name
        self.name = name

        #
        self.drive_unit = DriveUnit(
            gearing, wheel_diameter
        )
        self.configModuleComponents()
        self.useMinSpeed = True

        # self.velocity = 0
        # self.angle = 0
        self.enabled = False

    def disable(self):
        self.enabled = False

    def enable(self):
        self.enabled = True

    def getEncoderAzimuthDegrees(self) -> float:
        """gets the absolute position from the CANCoder
        Returns:
            float: position of the sensor in degrees (-180 to 180)
        """
        return self.encoder.getAbsolutePosition()

    def getCurrentAzimuth(self) -> Rotation2d:
        """Gets the Azimuth of the swerve wheel.

        Returns:
            Rotation2d: The robot-relative Azimuth of the swerve wheel.
        """        
        if degrees := self.getEncoderAzimuthDegrees():
            return Rotation2d.fromDegrees(degrees)
        else:
            return Rotation2d.fromDegrees(0)

    def getCurrentDistance(self) -> float:
        """Gets distance traveled by the system.

        Returns:
            float: distance in meters
        """
        return self.drive_unit.positionToDistance(
            self.drive.getSelectedSensorPosition()
        )

    def getCurrentSpeed(self) -> float:
        return self.drive_unit.velocityToSpeed(self.drive.getSelectedSensorVelocity())

    def getCurrentState(self):
        return SwerveModuleState(
            self.getCurrentSpeed(),
            self.getCurrentAzimuth(),
        )

    def getCurrentPosition(self):
        return SwerveModulePosition(
            self.getCurrentDistance(), self.getCurrentAzimuth()
        )

    def configModuleComponents(self):
        # configure CANCoder
        pass

    def setState(self, requested_state: SwerveModuleState):
        if self.enabled:
            self.requestedState = SwerveModuleState.optimize(
                requested_state, self.getCurrentAzimuth()
            )

            self.steer.set(
                PositionDutyCycle,
                radiansToRotations(requested_state.angle.radians)
            )

            self.drive.set(
                VelocityDutyCycle,
                self.drive_unit.speedToVelocity(self.requestedState.speed),
            )
        else:
            self.drive.set(0)