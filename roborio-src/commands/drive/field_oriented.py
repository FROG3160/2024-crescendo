import math
from commands2 import Command
from subsystems.drivetrain import DriveTrain
from subsystems.vision import TargetingSubsystem
from FROGlib.xbox import FROGXboxDriver
import constants
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians
from ntcore import NetworkTableInstance

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
            constants.kProfiledMaxVelocity, constants.kProfiledMaxAccel
        )
        self.profiledRotationController = ProfiledPIDControllerRadians(
            constants.kProfiledP,
            constants.kProfiledI,
            constants.kProfiledD,
            profiledRotationConstraints,
        )
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vT = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )
        self._gyro_yaw = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/gyro_yaw")
            .publish()
        )

    def resetRotationController(self):
        self.profiledRotationController.reset(
            math.radians(self.drive.gyro.getYawCCW()),
            self.drive.gyro.getRadiansPerSecCCW(),
        )

    def execute(self) -> None:
        # read right joystick Y to see if we are using it
        rightStickY = self.controller.getRightY()
        gyroYawCCW = self.drive.gyro.getYawCCW()
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
            self._calculated_vT.set(vT)
        elif rightStickY < -0.5:
            if self.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.resetController = False
                self.resetRotationController()
            # Rotate to 180 degrees
            vT = self.profiledRotationController.calculate(
                math.radians(gyroYawCCW), math.radians(180)
            )
            self._calculated_vT.set(vT)
        else:
            # set to true so the first time the other if conditionals evaluate true
            # the controller will be reset
            self.resetController = True
            vT = self.controller.getSlewLimitedFieldRotation()
        self._gyro_yaw.set(gyroYawCCW)

        # pov = self.controller.getPOVDebounced()
        # if pov != -1:
        #     vX, vY = povSpeeds[pov]
        # else:
        vX = self.controller.getSlewLimitedFieldForward()
        vY = self.controller.getSlewLimitedFieldLeft()

        self.drive.fieldOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )


class DriveToTarget(Command):

    def __init__(
        self, drive: DriveTrain, targeting: TargetingSubsystem, table: str = "Undefined"
    ) -> None:
        """Allows robot to drive in a robot oriented towards the target

        Args:
            drive (DriveTrain): The drive to control.
            targeting (TargetingSubsystem): The targeting subsystem that uses a limelight
            to calculate chassis speeds to drive towards the target
            table (str, optional): The name of the network table telemetry data will go into.
        """

        self.drive = drive
        self.targeting = targeting
        self.addRequirements(self.drive)
        self.nt_table = f"{table}/{type(self).__name__}"
        self._calculated_vX = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vX")
            .publish()
        )
        self._caluclated_vT = (
            NetworkTableInstance.getDefault()
            .getFloatTopic(f"{self.nt_table}/calculated_vT")
            .publish()
        )

    def execute(self):
        self.drive.robotOrientedDrive(*self.targeting.getChassisSpeeds())
        self._calculated_vX = self.targeting.calculate_vx()
        self._caluclated_vT = self.targeting.calculate_vt()
