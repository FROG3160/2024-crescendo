import math
from wpilib import XboxController, Timer
from wpimath.filter import SlewRateLimiter
from wpimath import applyDeadband
from wpilib.interfaces import GenericHID

RIGHT_RUMBLE = GenericHID.RumbleType.kRightRumble

class FROGXboxDriver(XboxController):
    """Custom Xbox Controller class for the driver controller specifically
    for field-oriented swerve drive control.
    """
    DEADBAND = 0.15
    DEBOUNCE_PERIOD = 0.5
    MODE = 0  # run auto routines

    def __init__(self, channel):
        super().__init__(channel)
        self.button_latest = {}
        self.timer = Timer()
        self.xSlew = SlewRateLimiter(1.5)
        self.ySlew = SlewRateLimiter(1.5)
        self.rotSlew = SlewRateLimiter(1.5)

    def getFieldHeading(self) -> float:
        """Get the desired robot heading from the Xbox's right
        stick.  

        Returns:
            float: heading in radians with CCW positive
        """
        left = -self.getRightX()
        forward = -self.getRightY()
        # convert to radians
        return math.atan2(left, forward)

    def getFieldRotation(self) -> float:
        """Get the speed/rate of rotation from the Xbox's right
        stick X axis, with applied deadband.

        Returns:
            float: rotational speed factor from -1 to 1 with CCW being positive
        """
        return applyDeadband(-self.getRightX(), self.DEADBAND)
        
    def getSlewLimitedFieldRotation(self) -> float:
        return self.rotSlew.calculate(
            self.getFieldRotation()
        )

    def getFieldForward(self):
        return applyDeadband(-self.getLeftY(), self.DEADBAND)
    
    def getSlewLimitedFieldForward(self):
        return self.xSlew.calculate(
            self.getFieldForward()
        )

    def getFieldLeft(self):
        return applyDeadband(-self.getLeftX(), self.DEADBAND)
    
    def getSlewLimitedFieldLeft(self):
        return self.ySlew.calculate(
            self.getFieldLeft()
        )

    def getFieldThrottle(self):
        return applyDeadband(self.getRightTriggerAxis(), 0)

    def getPOVDebounced(self):
        val = -1
        now = self.timer.getFPGATimestamp()
        pov = self.getPOV()
        if pov > -1:
            if (now - self.button_latest.get("POV", 0)) > self.DEBOUNCE_PERIOD:
                self.button_latest["POV"] = now
                val = pov
        if (now - self.button_latest.get("POV", 0)) < self.DEBOUNCE_PERIOD:
            self.setRumble(RIGHT_RUMBLE, 1)
        else:
            self.setRumble(RIGHT_RUMBLE, 0)
        # self.update_nt("button_pov", val)
        return val