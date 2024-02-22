from typing import Tuple, Any

import constants
import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Translation3d, Rotation3d


class FROGTargeting:
    """Custom FROG class for reading network tables data from limelight configured to identify targets"""

    def __init__(self, limelight_name: str = "limelight"):
        # get Network Tables table that the limelight is publishing to.
        self.network_table = NetworkTableInstance.getDefault().getTable(
            key=limelight_name
        )
        self.nt_tclass = self.network_table.getStringTopic("tclass").subscribe("None")
        self.nt_ta = self.network_table.getFloatTopic("ta").subscribe(0)
        self.nt_tx = self.network_table.getFloatTopic("tx").subscribe(-999)
        self.nt_tv = self.network_table.getIntegerTopic("tv").subscribe(0)
        self.nt_pipeline = self.network_table.getIntegerTopic("getpipe").subscribe(-1)

        self.nt_cl = self.network_table.getFloatTopic("cl").subscribe(0)
        self.nt_tl = self.network_table.getFloatTopic("tl").subscribe(0)

        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()

        # these filters can be used to reduce "noise" in the readings
        # self.txFilter = MedianFilter(8)
        # self.taFilter = MedianFilter(8)
        # self.poseXFilter = MedianFilter(8)
        # self.poseYFilter = MedianFilter(8)
        # self.poseZFilter = MedianFilter(8)
        # self.poseRollFilter = MedianFilter(8)
        # self.posePitchFilter = MedianFilter(8)
        # self.poseYawFilter = MedianFilter(8)

    def getLatency(self) -> float:
        return (self.nt_cl.get() + self.nt_tl.get()) / 1000

    def setPipeline(self, pipeline_num: int):
        self.network_table.putNumber("pipeline", pipeline_num)

    def getPipeline(self):
        return self.nt_pipeline.get()

    def getTarget(self):
        if self.nt_tv.get():
            self.tClass = self.nt_tclass.get()
            self.ta = self.nt_ta.get()
            self.tx = self.nt_tx.get()
            self.tv = self.nt_tv.get()
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
        return self.tv == 1

    def execute(self) -> None:
        self.getTarget()


class FROGPositioning:
    """FROG Custom class handling network tables data from limelight configured to use AprilTags"""

    def __init__(self, limelight_name: str = "limelight"):
        self.network_table = NetworkTableInstance.getDefault().getTable(
            key=limelight_name
        )
        self.nt_botpose = self.network_table.getFloatArrayTopic("botpose").subscribe(
            [-99, -99, -99, 0, 0, 0, -1]
        )
        self.nt_botpose_blue = self.network_table.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.nt_botpose_red = self.network_table.getFloatArrayTopic(
            "botpose_wpired"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.nt_targetpose_robotspace = self.network_table.getFloatArrayTopic(
            "targetpose_robotspace"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1])
        self.nt_pipeline = self.network_table.getIntegerTopic("getpipe").subscribe(-1)
        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()

    def getPipelineNum(self):
        return self.nt_pipeline.get()

    def setPipeline(self, pipeline_num: int):
        self.network_table.putNumber("pipeline", pipeline_num)

    # def getBotPoseEstimateForAlliance(self) -> Tuple[Pose3d, Any]:
    #     if self.fieldLayout.alliance == RED_ALLIANCE:
    #         return *self.getBotPoseEstimateRed(),
    #     elif self.fieldLayout.alliance == BLUE_ALLIANCE:
    #         return *self.getBotPoseEstimateBlue(),

    def getBotPoseEstimate(self) -> Tuple[Pose3d, Any]:
        return (*self.arrayToBotPoseEstimate(self.nt_botpose.get()),)

    def getBotPoseEstimateBlue(self) -> Tuple[Pose3d, Any]:
        return (*self.arrayToBotPoseEstimate(self.nt_botpose_blue.get()),)

    def getBotPoseEstimateRed(self) -> Tuple[Pose3d, Any]:
        return (*self.arrayToBotPoseEstimate(self.nt_botpose_red.get()),)

    def getTargetTransform(self):
        transform = self.nt_targetpose_robotspace.get()
        if transform[0] != -99:
            return self.nt_targetpose_robotspace.get()

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
                Translation3d(pX, pY, pZ), Rotation3d.fromDegrees(pRoll, pPitch, pYaw)
            ), self.timer.getFPGATimestamp() - (msLatency / 1000)
