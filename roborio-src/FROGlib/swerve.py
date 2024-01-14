from logging import Logger
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from phoenix6.controls import PositionDutyCycle, VelocityDutyCycle
from utils import DriveUnit
from wpimath.units import radiansToRotations, rotationsToRadians
from motors import FROGTalonFX, FROGTalonFXConfig
from sensors import FROGCANCoderConfig, FROGCanCoder, FROGGyro


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
    moduleFrontLeft: SwerveModule
    moduleFrontRight: SwerveModule
    moduleBackLeft: SwerveModule
    moduleBackRight: SwerveModule

    logger: Logger

    # fieldLayout: FROGFieldLayout *not used for final 2023 code
    
    # limelight: FROGLimeLightVision

    gyro: FROGGyro

    def __init__(self):
        self.enabled = False

        self.center = Translation2d(0, 0)


    def setup(self):

        self.modules = (
            self.moduleFrontLeft,
            self.moduleFrontRight,
            self.moduleBackLeft,
            self.moduleBackRight,
        )

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

        self.trajectoryConfig = TrajectoryConfig(
            MAX_TRAJECTORY_SPEED, MAX_TRAJECTORY_ACCEL
        )
        self.trajectoryConfig.setKinematics(self.kinematics)

        self.gyro.resetGyro()

        self.chassisSpeeds = ChassisSpeeds(0, 0, 0)
        self.startingPose2d = Pose2d()

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            tuple(
                [SwerveModulePosition(0, x.getCurrentRotation()) for x in self.modules]
            ),
            self.startingPose2d,
        )
        # TODO: Adjust the stdDevs
        self.estimator.setVisionMeasurementStdDevs((0.5, 0.5, math.pi/2))
        self.field = Field2d()

    def disable(self):
        self.enabled = False
        for module in self.modules:
            module.disable()

    def disableMinSpeed(self):
        for module in self.modules:
            module.disableMinSpeed()

    def enableMinSpeed(self):
        for module in self.modules:
            module.enableMinSpeed()

    def enable(self):
        self.enabled = True
        for module in self.modules:
            module.enable()

    def lockChassis(self):
        self.moduleFrontLeft.setState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleBackRight.setState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.moduleFrontRight.setState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.moduleBackLeft.setState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))

    def setModuleStates(self, states):
        self.moduleStates = states

    def setFieldPosition(self, pose: Pose2d):
        self.estimator.resetPosition(
            self.gyro.getRotation2d(),
            tuple(self.getModulePositions()),
            pose,
        )

    def getModuleStates(self):
        return [module.getCurrentState() for module in self.modules]

    def getModulePositions(self):
        return [module.getCurrentPosition() for module in self.modules]

    def setStatesFromSpeeds(self):
        states = self.kinematics.toSwerveModuleStates(self.chassisSpeeds, self.center)
        states = self.kinematics.desaturateWheelSpeeds(
            states, config.MAX_METERS_PER_SEC
        )
        self.moduleStates = states

    def fieldOrientedDrive(self, vX: float, vY: float, vT: float, throttle=1.0):
        xSpeed = vX * config.MAX_METERS_PER_SEC * throttle
        ySpeed = vY * config.MAX_METERS_PER_SEC * throttle
        rotSpeed = vT * config.MAX_CHASSIS_RADIANS_SEC * throttle
        self.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, self.gyro.getRotation2d()
        )

    def getChassisVelocityFPS(self):
        return math.sqrt( self.chassisSpeeds.vx_fps**2 + self.chassisSpeeds.vy_fps**2)
    
    def getHeadingRadians(self):
        return math.atan2( self.chassisSpeeds.vy, self.chassisSpeeds.vx )

    def holonomicDrive(self, chassisSpeeds) -> None:
        self.chassisSpeeds = chassisSpeeds

    def robotOrientedDrive(self, vX, vY, vT):
        self.logger.info(f'Velocities: {vX}, {vY}, {vT}')
        self.chassisSpeeds = ChassisSpeeds(vX, vY, vT)

    def execute(self):
        if self.enabled:
            self.setStatesFromSpeeds()#apply chassis Speeds

            for module, state in zip(self.modules, self.moduleStates):
                module.setState(state)
        self.periodic()

    def periodic(self) -> None:
        self.estimatorPose = self.estimator.update(
            Rotation2d.fromDegrees(self.gyro.getYawCCW()),
            tuple(self.getModulePositions()),
        )
        visionPose, visionTime = self.limelight.getBotPoseEstimateForAlliance()
        if visionPose:
            if (
                abs(visionPose.x - self.estimatorPose.x) < 0.5
                and abs(visionPose.y - self.estimatorPose.y) < 0.5
            ):
                stddevupdate = remap(visionPose.x,2.0, 8.0, 0.3, 2.0)
                self.estimator.addVisionMeasurement(
                    visionPose.toPose2d(), visionTime,
                    (stddevupdate, stddevupdate, math.pi/2)
                )
