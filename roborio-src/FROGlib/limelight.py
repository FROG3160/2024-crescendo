from typing import Tuple, Any

import constants
import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Translation3d, Rotation3d


class FROGLLObjects:
    #pipeline stuff and lists go here
    def __init__(self, name):
        self.ll_objectTable = NetworkTableInstance.getDefault().getTable(
            key=name
        )
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

    '''
    def findCubes(self):
        self.setGrabberPipeline(LL_CUBE)
    probably dont need this
    def findCones(self):
        self.setGrabberPipeline(LL_CONE)
    '''

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
        #most likeley to be put into apriltag class
        return (self.drive_vX, self.drive_vY, self.drive_vRotate)

    def hasObjectTarget(self):
        return self.tv

    def execute(self) -> None:
        self.getTarget()
        #might need that in apriltag class

    def setObjectPipeline(self, objectInt: int):
        self.ll_objectTable.putNumber("pipeline", objectInt)


class FROGLLField:
    # fieldLayout: FROGFieldLayout

    def __init__(self, table_name: str):
        # self.fieldLayout = fieldLayout
        self.network_table = NetworkTableInstance.getDefault().getTable(
            key=table_name
        )
        self.botpose = self.network_table.getFloatArrayTopic(
            "botpose").subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.botpose_blue = self.network_table.getFloatArrayTopic(
            "botpose_wpiblue").subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.botpose_red = self.network_table.getFloatArrayTopic(
            "botpose_wpired").subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.targetpose_robotspace = self.network_table.getFloatArrayTopic(
            "targetpose_robotspace").subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.pipeline_num = self.network_table.getIntegerTopic("getpipe").subscribe(-1)
        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()

    def getPipelineNum(self):
        return self.pipeline_num.get()

    # def getBotPoseEstimateForAlliance(self) -> Tuple[Pose3d, Any]:
    #     if self.fieldLayout.alliance == RED_ALLIANCE:
    #         return *self.getBotPoseEstimateRed(),
    #     elif self.fieldLayout.alliance == BLUE_ALLIANCE:
    #         return *self.getBotPoseEstimateBlue(),
        
    def getBotPoseEstimate(self) -> Tuple[Pose3d, Any]:
        return *self.arrayToBotPoseEstimate(self.botpose.get()),

    def getBotPoseEstimateBlue(self) -> Tuple[Pose3d, Any]:
        return *self.arrayToBotPoseEstimate(self.botpose_blue.get()),

    def getBotPoseEstimateRed(self) ->Tuple[Pose3d, Any] :
        return *self.arrayToBotPoseEstimate(self.botpose_red.get()),

    def getTargetTransform(self):
        transform = self.targetpose_robotspace.get()
        if transform[0] != -99:
            return self.targetpose_robotspace.get()

    def getVelocities(self):
        """Get calculated velocities from vision target data

        Returns:
            Tuple(vX, vY, vT): X, Y, and rotation velocities as a tuple.
        """
        return (self.drive_vX, self.drive_vY, self.drive_vRotate)

    def execute(self) -> None:
        self.getTarget()

    def setUpperPipeline(self, pipeNum: int):
        self.network_table.putNumber("pipeline", pipeNum)

    def arrayToBotPoseEstimate(self, poseArray) -> Tuple[Pose3d, Any]:
        """Takes limelight array data and creates a Pose3d object for 
           robot position and a timestamp reprepresenting the time
           the position was observed.

        Args:
            poseArray (_type_): An array from the limelight network tables.

        Returns:
            Tuple[Pose3d, Any]: Returns vision Pose3d and timestamp.
        """
        pX, pY, pZ, pRoll, pPitch, pYaw, msLatency = poseArray
        if msLatency == -1:
            return None, None
        else:
            return Pose3d(
                        Translation3d(pX, pY, pZ),
                        Rotation3d.fromDegrees(pRoll, pPitch, pYaw)
                    ), self.timer.getFPGATimestamp() - (msLatency/1000)
