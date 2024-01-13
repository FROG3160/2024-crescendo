from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from phoenix6.controls import PositionDutyCycle, VelocityDutyCycle
from utils import DriveUnit
from wpimath.units import radiansToRotations, rotationsToRadians
from motors import FROGTalonFX, FROGTalonFXConfig
from sensors import FROGCANCoderConfig, FROGCanCoder
class SwerveModule:
    def __init__(
        self,
        name: str,
        location: Translation2d,
        drive_gearing: list,
        wheel_diameter: float,
        drive_id: int,
        drive_config: FROGTalonFXConfig,
        steer_id: int,
        steer_confing: FROGTalonFXConfig,
        cancoder_id: int,
        cancoder_config: FROGCANCoderConfig
    ):
        # set initial states for the component

        #create/configure drive motor
        self.drive = FROGTalonFX(drive_id, drive_config)
        #create/configure steer motor
        self.steer = FROGTalonFX(steer_id, steer_confing)
        #create/configure cancoder
        self.encoder = FROGCanCoder(cancoder_id, cancoder_config)

        #set module location
        self.location = location
    
        #set module name
        self.name = name

        #
        self.drive_unit = DriveUnit(
            drive_gearing, wheel_diameter
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

    def getEncoderAzimuthRotations(self) -> float:
        """gets the absolute position from the CANCoder
        Returns:
            float: position of the sensor in degrees (-180 to 180)
        """
        return self.encoder.get_absolute_position()

    def getCurrentAzimuth(self) -> Rotation2d:
        """Gets the Azimuth of the swerve wheel.

        Returns:
            Rotation2d: The robot-relative Azimuth of the swerve wheel.
        """        
        if rotations := self.getEncoderAzimuthRotations():
            return Rotation2d(rotationsToRadians(rotations))
        else:
            return Rotation2d(0)

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