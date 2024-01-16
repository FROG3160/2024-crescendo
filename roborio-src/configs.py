import constants
from wpimath.geometry import Translation2d
from FROGlib.motors import FROGTalonFXConfig
from FROGlib.sensors import FROGCANCoderConfig

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
    'steer_config':FROGTalonFXConfig(),
    'cancoder_id':constants.kFrontLeftSensorID,
    'cancoder_config':FROGCANCoderConfig()
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
    'steer_config':FROGTalonFXConfig(),
    'cancoder_id':constants.kFrontRightSensorID,
    'cancoder_config':FROGCANCoderConfig()
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
    'steer_config':FROGTalonFXConfig(),
    'cancoder_id':constants.kBackLeftSensorID,
    'cancoder_config':FROGCANCoderConfig()
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
    'steer_config':FROGTalonFXConfig(),
    'cancoder_id':constants.kBackRightSensorID,
    'cancoder_config':FROGCANCoderConfig()
}