import math
from logging import Logger
from typing import Tuple
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition, ChassisSpeeds, SwerveDrive4Kinematics
from phoenix6.controls import PositionDutyCycle, VelocityDutyCycle
from wpimath.units import radiansToRotations, rotationsToRadians
from motors import FROGTalonFX, FROGTalonFXConfig, DriveUnit
from sensors import FROGCANCoderConfig, FROGCanCoder, FROGGyro
from constants import kMaxMetersPerSecond, kMaxChassisRadiansPerSec


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


class SwerveChassis:

    def __init__(self, modules: Tuple[SwerveModule], gyro:FROGGyro):
        # need each of the swerve modules
        self.enabled = False

        self.center = Translation2d(0, 0)
        self.modules = modules
        self.gyro = gyro


        self.moduleStates = (
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
        )
        self.current_speeds = ChassisSpeeds(0, 0, 0)

        self.kinematics = SwerveDrive4Kinematics(
            # the splat operator (asterisk) below expands
            # the list into positional arguments for the
            # kinematics object.  We are taking the location
            # property of each swerveModule object and passing
            # it to SwerveDrive4Kinematics the order defined by
            # self.modules above.  Order is critical here.
            # We will receive back drive and steer values for
            # each SwerveModule in the same order we use here.
            *[m.location for m in self.modules]
        )

        self.gyro.resetGyro()

        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)

    def disable(self):
        self.enabled = False
        for module in self.modules:
            module.disable()

    def enable(self):
        self.enabled = True
        for module in self.modules:
            module.enable()

    def fieldOrientedDrive(self, vX: float, vY: float, vT: float, throttle=1.0):
        """Calculates the necessary chassis speeds given the commanded field-oriented
        x, y, and rotational speeds.  An optional throttle value adjusts all inputs
        proportionally.

        Args:
            vX (float): velocity requested in the X direction, downfield, away from
                the driver station.  A proportion of the maximum speed.  (-1 to 1)
            vY (float): velocity requested in the Y direction, to the left when at the
                driver station facing the field.  A proportion of the maximum speed.  (-1 to 1)
            vT (float): rotational velocity requested, CCW positive (-1 to 1)
                throttle (float, optional): a proportion of all 3 speeds commanded. Defaults to 1.0.
        """        
        xSpeed = vX * kMaxMetersPerSecond * throttle
        ySpeed = vY * kMaxMetersPerSecond * throttle
        rotSpeed = vT * kMaxChassisRadiansPerSec * throttle
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, self.gyro.getRotation2d()
        )
    
    def getChassisVelocityFPS(self):
        return math.sqrt( self.chassisSpeeds.vx_fps**2 + self.chassisSpeeds.vy_fps**2)
    
    def getHeadingRadians(self):
        return math.atan2( self.chassisSpeeds.vy, self.chassisSpeeds.vx )
    
    def getModulePositions(self):
        return [module.getCurrentPosition() for module in self.modules]
    
    def getModuleStates(self):
        return [module.getCurrentState() for module in self.modules]

    def lockChassis(self):
        moduleAngles = [y.location.angle() for y in self.modules]
        for module, moduleAngle in zip(self.modules, moduleAngles):
                module.setState(SwerveModuleState(0, moduleAngle))
 
    # def lockChassis(self):
    #      """Commands each SwerveModule to point it's wheel in toward the center of the robot
    #      to keep it from moving on the field.
    #      """        
    #      self.moduleFrontLeft.setState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
    #      self.moduleBackRight.setState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
    #      self.moduleFrontRight.setState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
    #      self.moduleBackLeft.setState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))

    def robotOrientedDrive(self, vX, vY, vT):
        self.logger.info(f'Velocities: {vX}, {vY}, {vT}')
        self.chassisSpeeds = ChassisSpeeds(vX, vY, vT)

    def setModuleStates(self, states):
        self.moduleStates = states

    def setStatesFromSpeeds(self):
        states = self.kinematics.toSwerveModuleStates(self.chassisSpeeds, self.center)
        states = self.kinematics.desaturateWheelSpeeds(
            states, kMaxMetersPerSecond
        )
        self.moduleStates = states


    def holonomicDrive(self, chassisSpeeds) -> None:
        self.chassisSpeeds = chassisSpeeds


    def execute(self):
        if self.enabled:
            self.setStatesFromSpeeds()#apply chassis Speeds

            for module, state in zip(self.modules, self.moduleStates):
                module.setState(state)
