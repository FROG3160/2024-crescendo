import constants
from wpimath.geometry import Translation2d
from FROGlib.motors import FROGTalonFXConfig, FROGFeedbackConfig
from FROGlib.sensors import FROGCANCoderConfig
from phoenix6.configs.config_groups import Slot0Configs, Slot1Configs
from phoenix6.signals.spn_enums import FeedbackSensorSourceValue

steerGains = Slot0Configs.with_k_p(constants.kSteerP).with_k_i(constants.kSteerI)
driveDutyCycleGains = Slot0Configs()
driveVoltageGains = Slot1Configs()

swerveModuleFrontLeft = {
    'name':'FrontLeft',
    'location':Translation2d(
        constants.kWheelBaseMeters/2,
        constants.kTrackWidthMeters/2),
    'drive_gearing':constants.kSwerveDriveGearing,
    'wheel_diameter':constants.kWheelDiameter,
    'drive_id':constants.kFrontLeftDriveID,
    'drive_config':FROGTalonFXConfig(),
    'steer_id':constants.kFrontLeftSteerID,
    'steer_config':FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(remote_sensor_id=constants.kFrontLeftSensorID,
                               sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER)
        ),
        slot0gains=steerGains
    ),
    'cancoder_id':constants.kFrontLeftSensorID,
    'cancoder_config':FROGCANCoderConfig(constants.kFrontLeftOffset)
}
swerveModuleFrontRight = {
    'name':'FrontRight',
    'location':Translation2d(
        constants.kWheelBaseMeters/2,
        -constants.kTrackWidthMeters/2),
    'drive_gearing':constants.kSwerveDriveGearing,
    'wheel_diameter':constants.kWheelDiameter,
    'drive_id':constants.kFrontRightDriveID,
    'drive_config':FROGTalonFXConfig(),
    'steer_id':constants.kFrontRightSteerID,
    'steer_config':FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(remote_sensor_id=constants.kFrontRightSensorID,
                               sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER)
        ),
        slot0gains=steerGains
    ),
    'cancoder_id':constants.kFrontRightSensorID,
    'cancoder_config':FROGCANCoderConfig(constants.kFrontRightOffset)
}
swerveModuleBackLeft = {
    'name':'BackLeft',
    'location':Translation2d(
        -constants.kWheelBaseMeters/2,
        constants.kTrackWidthMeters/2),
    'drive_gearing':constants.kSwerveDriveGearing,
    'wheel_diameter':constants.kWheelDiameter,
    'drive_id':constants.kBackLeftDriveID,
    'drive_config':FROGTalonFXConfig(),
    'steer_id':constants.kBackLeftSteerID,
    'steer_config':FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(remote_sensor_id=constants.kBackLeftSensorID,
                               sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER)
        ),
        slot0gains=steerGains
    ),
    'cancoder_id':constants.kBackLeftSensorID,
    'cancoder_config':FROGCANCoderConfig(constants.kBackLeftOffset)
}
swerveModuleBackRight = {
    'name':'BackRight',
    'location':Translation2d(
        -constants.kWheelBaseMeters/2,
        -constants.kTrackWidthMeters/2),
    'drive_gearing':constants.kSwerveDriveGearing,
    'wheel_diameter':constants.kWheelDiameter,
    'drive_id':constants.kBackRightDriveID,
    'drive_config':FROGTalonFXConfig(),
    'steer_id':constants.kBackRightSteerID,
    'steer_config':FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(remote_sensor_id=constants.kBackRightSensorID,
                               sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER)
        ),
        slot0gains=steerGains
    ),
    'cancoder_id':constants.kBackRightSensorID,
    'cancoder_config':FROGCANCoderConfig(constants.kBackRightOffset)
}