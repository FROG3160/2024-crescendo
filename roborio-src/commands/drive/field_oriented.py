import math
from commands2 import Command
from wpilib import DriverStation
from subsystems.drivetrain import DriveTrain
from subsystems.vision import TargetingSubsystem
from FROGlib.xbox import FROGXboxDriver
import constants
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ntcore import NetworkTableInstance
from wpilib import SmartDashboard

# povSpeed = 0.1
# povSpeeds = {
#     0: (povSpeed, 0),
#     45: (povSpeed, -povSpeed),
#     90: (0, -povSpeed),
#     135: (-povSpeed, -povSpeed),
#     180: (-povSpeed, 0),
#     225: (-povSpeed, povSpeed),
#     270: (0, povSpeed),
#     315: (povSpeed, povSpeed),
# }


class ManualDrive(Command):
    def __init__(
        self, controller: FROGXboxDriver, drive: DriveTrain, table: str = "Undefined"
    ) -> None:
        """Allows manual control of the drivetrain through use of the specified
        controller.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.addRequirements(self.drive)
        self.resetController = True
        profiledRotationConstraints = TrapezoidProfileRadians.Constraints(
            constants.kProfiledRotationMaxVelocity, constants.kProfiledRotationMaxAccel
        )
        self.profiledRotationController = ProfiledPIDControllerRadians(
            constants.kProfiledRotationP,
            constants.kProfiledRotationI,
            constants.kProfiledRotationD,
            profiledRotationConstraints,
        )
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )
        self._rotation_degreesPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/rotation_degrees")
            .publish()
        )

    def resetRotationController(self):
        self.profiledRotationController.reset(
            self.drive.getRotation2d().radians(),
            self.drive.gyro.getRadiansPerSecCCW(),
        )

    def execute(self) -> None:
        # read right joystick Y to see if we are using it
        rightStickY = self.controller.getRightY()
        RightstickPressed = self.controller._hid.getRightStickButton()
        driveRotation2d = self.drive.getRotation2d()
        if rightStickY > 0.5:
            # if self.resetController:
            #     # this is the first time we hit this conditional, so
            #     # reset the controller
            #     self.resetController = False
            #     self.resetRotationController()
            # # Rotate to 0 degrees, point downfield
            # vT = self.profiledRotationController.calculate(
            #     math.radians(gyroYawCCW), math.radians(0)
            # )
            vT = self.drive.getvTtoTag()
            self._calculated_vTPub.set(vT)
        elif rightStickY < -0.5:
            if self.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.resetController = False
                self.resetRotationController()
            # Rotate to 180 degrees
            vT = self.profiledRotationController.calculate(
                driveRotation2d.radians(), math.radians(180)
            )
            self._calculated_vTPub.set(vT)
        elif RightstickPressed == True:
            if self.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.resetController = False
                self.resetRotationController()
            # Rotate to -90 degrees
            vt = self.profiledRotationController.calculate(
                driveRotation2d.radians(), math.radians(-90)
            )
            self._calculated_vTPub.set(vT)
        else:
            # set to true so the first time the other if conditionals evaluate true
            # the controller will be reset
            self.resetController = True
            vT = self.controller.getSlewLimitedFieldRotation()
        self._rotation_degreesPub.set(driveRotation2d.degrees())

        # pov = self.controller.getPOVDebounced()
        # if pov != -1:
        #     vX, vY = povSpeeds[pov]
        # else:

        # We are now using the blue alliance coordinate system all the time,
        # so if we are on the red side invert x and y
        if self.drive.onRedAlliance():
            vX = -self.controller.getSlewLimitedFieldForward()
            vY = -self.controller.getSlewLimitedFieldLeft()
        else:
            vX = self.controller.getSlewLimitedFieldForward()
            vY = self.controller.getSlewLimitedFieldLeft()

        self.drive.fieldOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )


class AutoRotateShooterToSpeaker(Command):

    def __init__(
        self, controller: FROGXboxDriver, drive: DriveTrain, table: str = "Undefined"
    ) -> None:
        """Allows manual control of the lateral movement of the drivetrain through use of the specified
        controller.  Rotation is calculated to aim the shooter at the speaker.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.addRequirements(self.drive)

        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def execute(self) -> None:
        if self.drive.resetController:
            # this is the first time we hit this conditional, so
            # reset the controller
            self.drive.resetController = False
            self.drive.resetRotationController()

        # we need the change in rotation to point at the speaker
        robotRelativeFiringHeading = self.drive.getFiringHeadingForSpeaker()
        currentRotation = self.drive.getRotation2d()

        vT = self.drive.profiledRotationController.calculate(
            currentRotation.radians(),
            (currentRotation + robotRelativeFiringHeading).radians(),
        )

        self._calculated_vTPub.set(vT)
        # Only rotation is calculated/automated.  The driver still needs
        # to use the controller to move the joystick across the field
        if self.drive.onRedAlliance():
            vX = -self.controller.getSlewLimitedFieldForward()
            vY = -self.controller.getSlewLimitedFieldLeft()
        else:
            vX = self.controller.getSlewLimitedFieldForward()
            vY = self.controller.getSlewLimitedFieldLeft()
        # vT is calculated by the controller and should not be throttled
        self.drive.fieldOrientedAutoRotateDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )


class AutoRotateShooterTowardsAmpCorner(Command):

    def __init__(
        self, controller: FROGXboxDriver, drive: DriveTrain, table: str = "Undefined"
    ) -> None:
        """Allows manual control of the lateral movement of the drivetrain through use of the specified
        controller.  Rotation is calculated to rotate towards the amp corner of our alliance.

        Args:
            controller (FROGXboxDriver): The controller used to control the drive.
            drive (DriveTrain): The drive to be controlled.
            table (str): The name of the network table telemetry data will go into
        """
        self.controller = controller
        self.drive = drive
        self.addRequirements(self.drive)

        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vTPub = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def execute(self) -> None:

        driveRotation2d = self.drive.getRotation2d()

        self.current_x_pos = self.drive.getPose().x
        self.current_y_pos = self.drive.getPose().y
        if self.drive.onRedAlliance():
            if self.drive.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.drive.resetController = False
                self.drive.resetRotationController()
            vT = self.drive.profiledRotationController.calculate(
                driveRotation2d.radians(),
                math.atan2(8.21 - self.current_y_pos, 16.54 - self.current_x_pos)
                - math.pi,
            )
        else:
            if self.drive.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.drive.resetController = False
                self.drive.resetRotationController()
            vT = self.drive.profiledRotationController.calculate(
                driveRotation2d.radians(),
                -math.atan2(8.21 - self.current_y_pos, self.current_x_pos),
            )
        self._calculated_vTPub.set(vT)

        if self.drive.onRedAlliance():
            vX = -self.controller.getSlewLimitedFieldForward()
            vY = -self.controller.getSlewLimitedFieldLeft()
        else:
            vX = self.controller.getSlewLimitedFieldForward()
            vY = self.controller.getSlewLimitedFieldLeft()

        self.drive.fieldOrientedAutoRotateDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )
