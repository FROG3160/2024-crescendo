import math
from commands2 import Command
from subsystems.drivetrain import DriveTrain
from FROGlib.xbox import FROGXboxDriver
import constants
from wpimath.controller import ProfiledPIDControllerRadians
from wpimath.trajectory import TrapezoidProfileRadians

povSpeed = 0.1
povSpeeds = {
    0: (povSpeed, 0),
    45: (povSpeed, -povSpeed),
    90: (0, -povSpeed),
    135: (-povSpeed, -povSpeed),
    180: (-povSpeed, 0),
    225: (-povSpeed, povSpeed),
    270: (0, povSpeed),
    315: (povSpeed, povSpeed),
}


class ManualDrive(Command):
    def __init__(self, controller: FROGXboxDriver, drive: DriveTrain) -> None:
        super().__init__()
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

    def resetRotationController(self):
        self.profiledRotationController.reset(
            math.radians(self.drive.gyro.getYawCCW()), self.drive.gyro.getRadiansPerSecCCW()
        )

    def execute(self) -> None:
        # read right joystick Y to see if we are using it
        rightStickY = self.controller.getRightY()
        if rightStickY > 0.5:
            if self.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.resetController = False
                self.resetRotationController()
            # Rotate to 0 degrees, point downfield
            vT = self.profiledRotationController.calculate(
                math.radians(self.drive.gyro.getYawCCW()), math.radians(0)
            )
        elif rightStickY < -0.5:
            if self.resetController:
                # this is the first time we hit this conditional, so
                # reset the controller
                self.resetController = False
                self.resetRotationController()
            # Rotate to 180 degrees
            vT = self.profiledRotationController.calculate(
                math.radians(self.drive.gyro.getYawCCW()), math.radians(180)
            )
        else:
            # set to true so the first time the other if conditionals evaluate true
            # the controller will be reset
            self.resetController = True
            vT = self.controller.getFieldRotation()
        pov = self.controller.getPOV()
        if pov != -1:
            vX, vY = povSpeeds[pov]
        else:
            vX = self.controller.getFieldForward()
            vY = self.controller.getFieldLeft()

        self.drive.fieldOrientedDrive(
            # self._vX, self._vY, self._vT, self._throttle
            vX,
            vY,
            vT,
            self.controller.getFieldThrottle(),
        )
