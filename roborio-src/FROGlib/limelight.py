from typing import Tuple, Any

import constants
import wpilib
from ntcore import NetworkTableInstance


class FROGLLObjects:
    # pipeline stuff and lists go here
    def __init__(self, name):
        self.ll_objectTable = NetworkTableInstance.getDefault().getTable(key=name)
        self.objectTclass = self.ll_objectTable.getStringTopic("tclass").subscribe(
            "None"
        )
        self.objectTa = self.ll_objectTable.getFloatTopic("ta").subscribe(0)
        self.objectTx = self.ll_objectTable.getFloatTopic("tx").subscribe(-999)
        self.objectTv = self.ll_objectTable.getIntegerTopic("tv").subscribe(0)
        self.objectPipe = self.ll_objectTable.getIntegerTopic("getpipe").subscribe(-1)

        self.objectCl = self.ll_objectTable.getFloatTopic("cl").subscribe(0)
        self.objectT1 = self.ll_objectTable.getFloatTopic("tl").subscribe(0)

        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()
        # self.txFilter = MedianFilter(8)
        # self.taFilter = MedianFilter(8)
        # self.poseXFilter = MedianFilter(8)
        # self.poseYFilter = MedianFilter(8)
        # self.poseZFilter = MedianFilter(8)
        # self.poseRollFilter = MedianFilter(8)
        # self.posePitchFilter = MedianFilter(8)
        # self.poseYawFilter = MedianFilter(8)

    def getLatency(self):
        return (self.objectCl.get() + self.objectT1.get()) / 1000

    """
    def findCubes(self):
        self.setGrabberPipeline(LL_CUBE)
    probably dont need this
    def findCones(self):
        self.setGrabberPipeline(LL_CONE)
    """

    def getObjectPipeline(self):
        return self.objectPipe.get()

    def getTarget(self):
        if self.objectTv.get():
            self.tClass = self.objectTclass.get()
            self.ta = self.objectTa.get()
            self.tx = self.objectTx.get()
            self.tv = self.objectTv.get()
            self.drive_vRotate = self.calculateRotation(self.tx)
            self.drive_vX = self.calculateX(self.ta)
            self.drive_vY = 0
        else:
            self.tClass = self.ta = self.tx = self.tv = None
            self.drive_vRotate = self.drive_vX = self.drive_vY = 0
            # self.txFilter.reset()
            # self.taFilter.reset()
            # should probably be put into the apriltag class

    def calculateX(self, targetArea):
        """Calculate X robot-oriented speed from the size of the target.  Return is inverted
        since we need the robot to drive backwards toward the target to pick it up.

        Args:
            targetArea (Float):  The target area determined by limelight.

        Returns:
            Float: Velocity in the X direction (robot oriented)
        """
        return min(-0.20, -(targetArea * -0.0125 + 1.3125))
        # calcX = -(-0.0002*(targetArea**2) + 0.0093*targetArea+1)
        # return max(-1, calcX)

    # the calculates should probalby be put into the apriltag class

    def calculateRotation(self, targetX):
        """Calculate the rotational speed from the X value of the target in the camera frame.
        Return is inverted to make left rotation positive from a negative X value, meaning the
        target is to the left of center of the camera's view.

        Args:
            targetX (Float): The X value of the target in the camera frame, 0 is straight ahead,
            to the left is negative, to the right is positive.

        Returns:
            Float: Rotational velocity with CCW (left, robot oriented) positive.
        """
        return -(targetX / 25)

    def getVelocities(self):
        """Get calculated velocities from vision target data

        Returns:
            Tuple(vX, vY, vT): X, Y, and rotation velocities as a tuple.
        """
        # most likeley to be put into apriltag class
        return (self.drive_vX, self.drive_vY, self.drive_vRotate)

    def hasObjectTarget(self):
        return self.tv

    def execute(self) -> None:
        self.getTarget()
        # might need that in apriltag class

    def setObjectPipeline(self, objectInt: int):
        self.ll_objectTable.putNumber("pipeline", objectInt)
