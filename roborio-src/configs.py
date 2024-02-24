import math
import constants
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d
from FROGlib.motors import FROGTalonFXConfig, FROGFeedbackConfig
from FROGlib.sensors import FROGCANCoderConfig
from phoenix6.configs.config_groups import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
)
from phoenix6.signals.spn_enums import FeedbackSensorSourceValue
from pathplannerlib.config import PIDConstants
from wpimath.units import inchesToMeters
from rev import CANSparkMax
from phoenix6.signals.spn_enums import NeutralModeValue, InvertedValue


steerGains = Slot0Configs().with_k_p(constants.kSteerP).with_k_i(constants.kSteerI)
driveDutyCycleGains = (
    Slot0Configs()
    .with_k_s(constants.kDutyCycleDriveS)
    .with_k_v(constants.kDutyCycleDriveV)
)
driveVoltageGains = (
    Slot1Configs().with_k_s(constants.kVoltageDriveS).with_k_v(constants.kVoltageDriveV)
)


holonomicTranslationPID = PIDConstants(5.0, 0.0, 0.0)
holonomicRotationPID = PIDConstants(5.0, 0.0, 0.0)

swerveModuleFrontLeft = {
    "name": "FrontLeft",
    "location": Translation2d(
        constants.kCenterToFrontModulesMeters, constants.kTrackWidthMeters / 2
    ),
    "drive_gearing": constants.kSwerveDriveGearing,
    "wheel_diameter": constants.kWheelDiameter,
    "drive_id": constants.kFrontLeftDriveID,
    "drive_config": FROGTalonFXConfig(
        slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
    ),
    "steer_id": constants.kFrontLeftSteerID,
    "steer_config": FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(
                remote_sensor_id=constants.kFrontLeftSensorID,
                sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
            )
        ),
        slot0gains=steerGains,
    ),
    "cancoder_id": constants.kFrontLeftSensorID,
    "cancoder_config": FROGCANCoderConfig(constants.kFrontLeftOffset),
}
swerveModuleFrontRight = {
    "name": "FrontRight",
    "location": Translation2d(
        constants.kCenterToFrontModulesMeters, -constants.kTrackWidthMeters / 2
    ),
    "drive_gearing": constants.kSwerveDriveGearing,
    "wheel_diameter": constants.kWheelDiameter,
    "drive_id": constants.kFrontRightDriveID,
    "drive_config": FROGTalonFXConfig(
        slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
    ),
    "steer_id": constants.kFrontRightSteerID,
    "steer_config": FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(
                remote_sensor_id=constants.kFrontRightSensorID,
                sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
            )
        ),
        slot0gains=steerGains,
    ),
    "cancoder_id": constants.kFrontRightSensorID,
    "cancoder_config": FROGCANCoderConfig(constants.kFrontRightOffset),
}
swerveModuleBackLeft = {
    "name": "BackLeft",
    "location": Translation2d(
        -constants.kBackModulesToCenterMeters, constants.kTrackWidthMeters / 2
    ),
    "drive_gearing": constants.kSwerveDriveGearing,
    "wheel_diameter": constants.kWheelDiameter,
    "drive_id": constants.kBackLeftDriveID,
    "drive_config": FROGTalonFXConfig(
        slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
    ),
    "steer_id": constants.kBackLeftSteerID,
    "steer_config": FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(
                remote_sensor_id=constants.kBackLeftSensorID,
                sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
            )
        ),
        slot0gains=steerGains,
    ),
    "cancoder_id": constants.kBackLeftSensorID,
    "cancoder_config": FROGCANCoderConfig(constants.kBackLeftOffset),
}
swerveModuleBackRight = {
    "name": "BackRight",
    "location": Translation2d(
        -constants.kBackModulesToCenterMeters, -constants.kTrackWidthMeters / 2
    ),
    "drive_gearing": constants.kSwerveDriveGearing,
    "wheel_diameter": constants.kWheelDiameter,
    "drive_id": constants.kBackRightDriveID,
    "drive_config": FROGTalonFXConfig(
        slot0gains=driveDutyCycleGains, slot1gains=driveVoltageGains
    ),
    "steer_id": constants.kBackRightSteerID,
    "steer_config": FROGTalonFXConfig(
        feedback_config=(
            FROGFeedbackConfig(
                remote_sensor_id=constants.kBackRightSensorID,
                sensor_source=FeedbackSensorSourceValue.REMOTE_CANCODER,
            )
        ),
        slot0gains=steerGains,
    ),
    "cancoder_id": constants.kBackRightSensorID,
    "cancoder_config": FROGCANCoderConfig(constants.kBackRightOffset),
}

# Assuming that the control output for the lead screw would be Duty Cycle
# and the control output for the flywheel would be Voltage
leadScrewPositionGains = Slot0Configs().with_k_p(constants.kLeadScrewSlot0P)
leadScrewVelocityGains = (
    Slot1Configs()
    .with_k_s(constants.kLeadScrewSlot1S)
    .with_k_v(constants.kLeadScrewSlot1V)
)
leadScrewVelocityGains.k_a = 0.01
leadScrewVelocityGains.k_p = 4

flywheelVoltageGains = (
    Slot0Configs()
    .with_k_s(constants.kFlywheelVoltageS)
    .with_k_v(constants.kFlywheelVoltageV)
)
leadScrewConfig = FROGTalonFXConfig(
    slot0gains=leadScrewPositionGains,
    slot1gains=leadScrewVelocityGains,
    feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(4),
).with_motor_output(
    MotorOutputConfigs()
    .with_neutral_mode(NeutralModeValue.BRAKE)
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
)
leadScrewConfig.motion_magic.motion_magic_acceleration = 40
leadScrewConfig.motion_magic.motion_magic_cruise_velocity = 20
flywheelConfig = FROGTalonFXConfig(slot0gains=flywheelVoltageGains)
sequencerMotorType = CANSparkMax.MotorType.kBrushless

climberMotorConfig = FROGTalonFXConfig(
    feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(1)
).with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE))
leftClimberMotorConfig = climberMotorConfig
rightClimberMotorConfig = climberMotorConfig
rightClimberMotorConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

""" Don't think the following is needed anymore.  Leaving for reference."""
# robotToLimeLightTransform = Transform3d(
#     Translation3d(
#         inchesToMeters(13),  # Forward from center
#         inchesToMeters(11.5),  # Left from center
#         inchesToMeters(22.625),  # Up from the floor
#     ),
#     Rotation3d(0, 0, math.pi),
# )
