from typing import Tuple, Any

import config #? what does that mean
import wpilib
from components.field import FROGFieldLayout #? what
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Rotation3d, Translation3d
from wpimath.filter import MedianFilter

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

class FROGLLAprilTags:
    '''stuff with position fiding and calculating goes here
    '''
    #TODO #22 create this class
    
    pass

class FROGLLObjects:
    #pipeline stuff and lists go here
    
    pass

class FROGLimeLightVision:
    #fieldLayout: this is now differnt still need it though

    def __init__(self):
        # self.fieldLayout = fieldLayout
        self.ll_grabberTable = NetworkTableInstance.getDefault().getTable(
            key=config.LIMELIGHT_GRABBER
        )
        self.ll_upperTable = NetworkTableInstance.getDefault().getTable(
            key=config.LIMELIGHT_UPPER
        )
        self.grabberTclass = self.ll_grabberTable.getStringTopic("tclass").subscribe(
            "None"
        )
        self.grabberTa = self.ll_grabberTable.getFloatTopic("ta").subscribe(0)
        self.grabberTx = self.ll_grabberTable.getFloatTopic("tx").subscribe(-999)
        self.grabberTv = self.ll_grabberTable.getIntegerTopic("tv").subscribe(0)
        self.grabberPipe = self.ll_grabberTable.getIntegerTopic("getpipe").subscribe(-1)

        self.grabberCl = self.ll_grabberTable.getFloatTopic("cl").subscribe(0)
        self.grabberTl = self.ll_grabberTable.getFloatTopic("tl").subscribe(0)

        self.upperPose = self.ll_upperTable.getFloatArrayTopic("botpose").subscribe(
            [-99, -99, -99, 0, 0, 0, -1]
        )
        self.upperPoseBlue = self.ll_upperTable.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.upperPoseRed = self.ll_upperTable.getFloatArrayTopic(
            "botpose_wpired"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.upperPipe = self.ll_upperTable.getIntegerTopic("getpipe").subscribe(-1)
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
        return (self.grabberCl.get() + self.grabberTl.get()) / 1000

    '''
    def findCubes(self):
        self.setGrabberPipeline(LL_CUBE)
    probably dont need this
    def findCones(self):
        self.setGrabberPipeline(LL_CONE)
    '''

    def getGrabberPipeline(self):
        return self.grabberPipe.get()

    def getUpperPipeline(self):
        return self.upperPipe.get()

    def getBotPoseEstimateForAlliance(self) -> Tuple[Pose3d, Any]:
        if self.fieldLayout.alliance == RED_ALLIANCE:
            return *self.getBotPoseEstimateRed(),
        elif self.fieldLayout.alliance == BLUE_ALLIANCE:
            return *self.getBotPoseEstimateBlue(),
        
    def getBotPoseEstimate(self) -> Tuple[Pose3d, Any]:
        return *self.arrayToBotPoseEstimate(self.upperPose.get()),

    def getBotPoseEstimateBlue(self) -> Tuple[Pose3d, Any]:
        return *self.arrayToBotPoseEstimate(self.upperPoseBlue.get()),

    def getBotPoseEstimateRed(self) ->Tuple[Pose3d, Any] :
        return *self.arrayToBotPoseEstimate(self.upperPoseRed.get()),

    def getTID(self) -> float:
        return self.ll_grabberTable.getNumber("tid", -1.0)

    def getTarget(self):
        if self.grabberTv.get():
            self.tClass = self.grabberTclass.get()
            self.ta = self.grabberTa.get()
            self.tx = self.grabberTx.get()
            self.tv = self.grabberTv.get()
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

    def hasGrabberTarget(self):
        return self.tv

    def execute(self) -> None:
        self.getTarget()
        #might need that in apriltag class

    def setGrabberPipeline(self, objectInt: int):
        self.ll_grabberTable.putNumber("pipeline", objectInt)

    def setUpperPipeline(self, pipeNum: int):
        self.ll_upperTable.putNumber("pipeline", pipeNum)

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
        # pX = self.poseXFilter.calculate(poseArray[0])
        # pY = self.poseYFilter.calculate(poseArray[1])
        # pZ = self.poseZFilter.calculate(poseArray[2])
        # pRoll = self.poseRollFilter.calculate(poseArray[3])
        # pPitch = self.posePitchFilter.calculate(poseArray[4])
        # pYaw = self.poseYawFilter.calculate(poseArray[5])
        if msLatency == -1:
            return None, None
        else:
            return Pose3d(
                        Translation3d(pX, pY, pZ),
                        Rotation3d.fromDegrees(pRoll, pPitch, pYaw)
                    ), self.timer.getFPGATimestamp() - (msLatency/1000)
        #might need that in apriltag class
