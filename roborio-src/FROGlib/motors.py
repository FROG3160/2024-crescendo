import math
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue
from phoenix6.configs.config_groups import Slot0Configs, Slot1Configs, FeedbackConfigs
from phoenix6.signals.spn_enums import GravityTypeValue


class FROGFeedbackConfig(FeedbackConfigs):
    def __init__(
        self, remote_sensor_id=0, sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR
    ):
        super().__init__()
        self.feedback_remote_sensor_id = remote_sensor_id
        self.feedback_sensor_source = sensor_source
        # TODO: Add this in if it makes sense
        # self.rotor_to_sensor_ratio =
        # self.sensor_to_mechanism_ratio =


class FROGTalonFXConfig(TalonFXConfiguration):
    """A subclass of TalonFXConfiguration that adds the ability to pass parameters to __init__
    during instantiation instead of creating an instance and then setting attributes."""

    def __init__(
        self,
        feedback_config=FROGFeedbackConfig(),
        slot0gains=Slot0Configs(),
        slot1gains=Slot1Configs(),
    ):
        super().__init__()
        self.feedback = feedback_config
        self.slot0 = slot0gains
        self.slot1 = slot1gains


class FROGTalonFX(TalonFX):
    """A subclass of TalonFX that allows us to pass in the config and apply it during
    instantiation.
    """

    def __init__(
        self, id: int = 0, motor_config: FROGTalonFXConfig = FROGTalonFXConfig()
    ):
        """Creates a TalonFX motor object with applied configuration

        Args:
            id (int, required): The CAN ID assigned to the motor.
            motor_config (FROGTalonFXConfig, required): The configuration to apply to the motor.
        """
        super().__init__(device_id=id)
        self.config = motor_config
        self.configurator.apply(self.config)


class GearStages:
    def __init__(self, gear_stages: list):
        """
        Constructs a GearStages object that stores data about the gear stages.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
        """
        self.gearing = math.prod(gear_stages)

    # TODO: need to re-evalute the names of these methods.  How can we make it more
    # explicit?
    def toMotor(self, rotations):
        """Calculates motor rotations given the rotation at the other end of the gears."""
        return rotations / self.gearing

    def fromMotor(self, rotations):
        """Calculates final gear rotation given the motor's rotation"""
        return rotations * self.gearing


class DriveUnit:
    def __init__(self, gear_stages: list, diameter: float):
        """Constructs a DriveUnit object that stores data about the motor, gear stages, and wheel
           that makes up a complete power train.
        Args:
            gear_stages (list): list of gear stages expressed as tuples of two integers e.g. [(10, 32), (9, 24)]
            diameter (float): Diameter of the attached wheel in meters
        """
        self.gearing = GearStages(gear_stages)
        self.circumference = math.pi * diameter

    def speedToVelocity(self, speed: float) -> float:
        """Converts the system linear speed to a motor velocity
        Args:
            speed (float): desired linear speed in meters per second
        Returns:
            float: motor rotations per second
        """
        wheel_rotations_sec = speed / self.circumference
        motor_rotations_sec = self.gearing.toMotor(wheel_rotations_sec)
        return motor_rotations_sec

    def velocityToSpeed(self, rotations_per_sec: float) -> float:
        """Converts motor velocity to the system linear speed

        Args:
            velocity (float): motor velocity in encoder counts per 100ms
        Returns:
            float: system linear speed in meters per second
        """
        wheel_rotations_sec = self.gearing.fromMotor(rotations_per_sec)
        return wheel_rotations_sec * self.circumference

    def positionToDistance(self, rotations: float) -> float:
        """Takes encoder count and returns distance

        Args:
            position (int): number of encoder counts

        Returns:
            float: distance in meters
        """
        wheel_rotations = self.gearing.fromMotor(rotations)
        return wheel_rotations * self.circumference


# %%
