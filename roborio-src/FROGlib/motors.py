from phoenix6 import TalonFXConfiguration, TalonFX
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
    def __init__(self, id=None, motor_config=FROGTalonFXConfig()):
        super.__init__(device_id=id)
        self.config = motor_config
        self.configurator.apply(self.config)