import math
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import FeedbackSensorSourceValue

class FROGTalonFXConfig(TalonFXConfiguration):
    '''A subclass of TalonFXConfiguration that adds the ability to pass parameters to __init__
    during instantiation instead of creating an instance and then setting attributes.'''
    def __init__(self, feedback_sensor_source=FeedbackSensorSourceValue.ROTOR_SENSOR,
                 feedback_remote_sensor_id=None, k_p=0, k_i=0, k_d=0, k_v=0):
        super.__init__()
        self.feedback.feedback_sensor_source = feedback_sensor_source
        self.feedback.feedback_remote_sensor_id = feedback_remote_sensor_id
        self.slot0.k_p = k_p
        self.slot0.k_i = k_i
        self.slot0.k_d = k_d
        self.slot0.k_v = k_v
        #TODO Research and test setting allowableClosedloopError for phoenix6 Library #33
        # self.allowableClosedloopError

class FROGTalonFX(TalonFX):
    '''A subclass of TalonFX that allows us to pass in the config and apply it during
    instantiation.
    '''
    def __init__(self, id:int=None, motor_config:FROGTalonFXConfig=None):
        super.__init__(device_id=id)
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