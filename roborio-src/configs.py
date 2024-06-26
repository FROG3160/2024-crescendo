import math
import constants
from wpimath.geometry import Translation2d, Translation3d, Transform3d, Rotation3d
from FROGlib.motors import FROGTalonFXConfig, FROGFeedbackConfig
from FROGlib.sensors import FROGCANCoderConfig
from phoenix6.configs.config_groups import (
    Slot0Configs,
    Slot1Configs,
    MotorOutputConfigs,
    MotionMagicConfigs,
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
    Slot1Configs()
    .with_k_s(constants.kVoltageDriveS)
    .with_k_v(constants.kVoltageDriveV)
    .with_k_p(constants.kVoltageDriveP)
    .with_k_a(constants.kVoltageDriveA)
)


autobuilderHolonomicTranslationPID = PIDConstants(1.0, 0.0, 0.0)
autobuilderHolonomicRotationPID = PIDConstants(0.4, 0.0, 0.0)

motorOutputCCWPandBrake = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.BRAKE)
)
motorOutputCCWPandCoast = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.COAST)
)
motorOutputCWPandBrake = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.BRAKE)
)
motorOutputCWPandCoast = (
    MotorOutputConfigs()
    .with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
    .with_neutral_mode(NeutralModeValue.COAST)
)


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

leadscrewPositionGains = Slot0Configs().with_k_p(constants.kLeadscrewSlot0P)
leadscrewVelocityGains = (
    Slot1Configs()
    .with_k_s(constants.kLeadscrewSlot1S)
    .with_k_v(constants.kLeadscrewSlot1V)
    .with_k_p(constants.kLeadscrewSlot1P)
    .with_k_a(constants.kLeadscrewSlot1A)
)

climberMotorGains = (
    Slot0Configs()
    .with_k_s(constants.kClimberSlot0S)
    .with_k_v(constants.kClimberSlot0V)
    .with_k_a(constants.kClimberSlot0A)
)

leftFlywheelVoltageGains = (
    Slot0Configs()
    .with_k_s(constants.kleftFlywheelVoltageS)
    .with_k_v(constants.kleftFlywheelVoltageV)
    .with_k_p(constants.kleftFlywheelVoltageP)
    .with_k_a(constants.kLeftFlywheelVoltageA)
)
rightFlywheelVoltageGains = (
    Slot0Configs()
    .with_k_s(constants.kRightFlywheelVoltageS)
    .with_k_v(constants.kRightFlywheelVoltageV)
    .with_k_p(constants.kRightFlywheelVoltageP)
    .with_k_a(constants.kRightFlywheelVoltageA)
)

mmLeadscrew = (
    MotionMagicConfigs()
    .with_motion_magic_acceleration(constants.kLeadscrewMMA)
    .with_motion_magic_cruise_velocity(constants.kLeadscrewMMV)
)
leadscrewConfig = (
    FROGTalonFXConfig(
        slot0gains=leadscrewPositionGains,
        slot1gains=leadscrewVelocityGains,
        feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(4),
    )
    .with_motor_output(motorOutputCWPandBrake)
    .with_motion_magic(mmLeadscrew)
)

mmFlywheel = (
    MotionMagicConfigs()
    .with_motion_magic_acceleration(constants.kFlywheelMMA)
    .with_motion_magic_cruise_velocity(constants.kFlywheelMMV)
)

leftFlywheelConfig = (
    FROGTalonFXConfig(slot0gains=leftFlywheelVoltageGains)
    .with_motor_output(motorOutputCWPandCoast)
    .with_motion_magic(mmFlywheel)
)
rightFlywheelConfig = (
    FROGTalonFXConfig(slot0gains=rightFlywheelVoltageGains)
    .with_motor_output(motorOutputCCWPandCoast)
    .with_motion_magic(mmFlywheel)
)

mmClimber = (
    MotionMagicConfigs()
    .with_motion_magic_acceleration(constants.kClimberMMA)
    .with_motion_magic_cruise_velocity(constants.kClimberMMV)
)

leftClimberMotorConfig = (
    FROGTalonFXConfig(
        slot0gains=climberMotorGains,
        feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(1),
    )
    .with_motor_output(motorOutputCCWPandBrake)
    .with_motion_magic(mmClimber)
)

rightClimberMotorConfig = (
    FROGTalonFXConfig(
        slot0gains=climberMotorGains,
        feedback_config=FROGFeedbackConfig().with_sensor_to_mechanism_ratio(1),
    )
    .with_motor_output(motorOutputCWPandBrake)
    .with_motion_magic(mmClimber)
)

intakeMotorConfig = FROGTalonFXConfig().with_motor_output(
    MotorOutputConfigs().with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
)

""" Don't think the following is needed anymore.  Leaving for reference."""
# robotToLimeLightTransform = Transform3d(
#     Translation3d(
#         inchesToMeters(13),  # Forward from center
#         inchesToMeters(11.5),  # Left from center
#         inchesToMeters(22.625),  # Up from the floor
#     ),
#     Rotation3d(0, 0, math.pi),
# )
