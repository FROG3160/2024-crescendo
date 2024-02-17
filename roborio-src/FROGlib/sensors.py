import math
from navx import AHRS
from wpimath.geometry import Rotation2d
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.signals.spn_enums import AbsoluteSensorRangeValue, SensorDirectionValue
from phoenix6.hardware.cancoder import CANcoder


class FROGGyro:
    """Gyro class that creates and instance of the NavX gyro and uses it to get AHRS data,
    converting it for use by the swerve drivetrain.  All swerve calculations use radians
    with CCW rotation being positive, and field-oriented driving uses moving forward and
    moving left as positive."""

    def __init__(self):
        # TODO Make sure if we need this.
        self.gyro = AHRS.create_spi()
        self.starting_angle = 0.0
        self.offset = 0
        # self.field_heading = 360-242
        # self.gyro.reset()
        self.gyro.setAngleAdjustment(self.offset)

    def getYawCCW(self):
        # returns gyro heading +180 to -180 degrees
        # and inverts it to change from bearing to
        # cartesian angles with CCW positive.
        # return -self.gyro.getYaw()
        return -self.gyro.getAngle()

    def getRoll(self):
        return self.gyro.getRoll()

    def getPitch(self):
        return self.gyro.getPitch()

    def setOffset(self, offset):
        self.offset = offset

    def getDegreesPerSecCCW(self):
        return -self.gyro.getRate()

    def getRadiansPerSecCCW(self):
        return math.radians(self.getDegreesPerSecCCW())

    def getRotation2d(self):
        return Rotation2d.fromDegrees(self.getAngleCCW())

    def getOffsetYaw(self):
        chassisYaw = self.getYawCCW()
        fieldYaw = Rotation2d.fromDegrees(chassisYaw + self.starting_angle)
        # Adding an angle to the current reading can cause the result
        # to be outside -180 to +180 degrees, so we utilize the atan2
        # function to give us the angle back inside the limits.
        return math.degrees(math.atan2(fieldYaw.sin(), fieldYaw.cos()))

    def resetGyro(self):
        # sets yaw reading to 0
        self.setAngleAdjustment(self.starting_angle)
        self.gyro.reset()

    def getAngle(self):
        return self.gyro.getAngle()

    def getAngleCCW(self):
        return -self.gyro.getAngle()

    def getAngleConstrained(self):
        angle = self.getAngle()
        return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))

    def setAngleAdjustment(self, angle):
        self.gyro.setAngleAdjustment(angle)

    def getRadiansCCW(self):
        return math.radians(self.getYawCCW())

    def getAngleAdjustment(self):
        return self.gyro.getAngleAdjustment()


class FROGCANCoderConfig(CANcoderConfiguration):
    """Inheretis from CANcoderConfiguration and add the ability to pass in steer offset
    during instantiation."""

    def __init__(self, steer_offset):
        super().__init__()
        self.magnet_sensor.absolute_sensor_range = (
            AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        )
        self.magnet_sensor.magnet_offset = steer_offset
        self.magnet_sensor.sensor_direction = (
            SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )


class FROGCanCoder(CANcoder):
    def __init__(self, id, config: FROGCANCoderConfig):
        super().__init__(id)
        self.configurator.apply(config)
