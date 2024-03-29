from functools import reduce
from itertools import chain
from typing import Tuple, Any

import wpilib
from ntcore import NetworkTableInstance
from wpimath.geometry import Pose3d, Translation3d, Rotation3d, Transform3d
from FROGlib.utils import partitionArray, arrayToPose3d


class TagMetrics:
    def __init__(self, tagData):
        if len(tagData) != 7:
            self.id = 0
            self.txnc = 0
            self.tync = 0
            self.ta = 0
            self.distanceToCamera = 0
            self.distanceToRobot = 0
            self.ambiguity = 0
        else:
            (
                self.id,
                self.txnc,
                self.tync,
                self.ta,
                self.distanceToCamera,
                self.distanceToRobot,
                self.ambiguity,
            ) = tagData


class BotPoseResult:
    def __init__(self, array: list[float], system_timestamp: float):
        # split the array up into list with the specified lengths
        # This assumes that there is a maximum of 3 sets of tag
        # details at the end all of length 7.
        splitArray = partitionArray(array, [6, 1, 4, 7, 7, 7])
        self.botPose = arrayToPose3d(splitArray[0])
        self.latency = splitArray[1][0] / 1000
        self.timestamp = system_timestamp - self.latency
        self.tagCount = splitArray[2][0]
        self.tagSpan = splitArray[2][1]
        self.avgTagDistance = splitArray[2][2]
        self.avgTagArea = splitArray[2][3]
        self.tagData = [
            TagMetrics(splitArray[3]),
            TagMetrics(splitArray[4]),
            TagMetrics(splitArray[5]),
        ]


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
        self.nt_ty = self.network_table.getFloatTopic("ty").subscribe(-999)
        self.nt_tv = self.network_table.getIntegerTopic("tv").subscribe(0)
        self.nt_pipeline = self.network_table.getIntegerTopic("getpipe").subscribe(-1)

        self.nt_cl = self.network_table.getFloatTopic("cl").subscribe(0)
        self.nt_tl = self.network_table.getFloatTopic("tl").subscribe(0)

        # create the timer that we can use to the the FPGA timestamp
        self.timer = wpilib.Timer()
        self.zeroValues()

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
            self.ty = self.nt_ty.get()
            self.tv = self.nt_tv.get()
        else:
            self.zeroValues()
            # self.txFilter.reset()
            # self.taFilter.reset()

    def zeroValues(self):
        self.tClass = self.ta = self.tx = self.ty = self.tv = None

    # the calculates should probalby be put into the apriltag class

    def hasObjectTarget(self):
        return self.tv == 1

    def execute(self) -> None:
        self.getTarget()


class FROGPositioning:
    """FROG Custom class handling network tables data from limelight configured to use AprilTags"""

    # For every tag used by megatag localization, the above arrays now include
    # (tagID, txnc, tync, ta, distanceToCamera, distanceToRobot, ambiguity)

    def __init__(self, limelight_name: str = "limelight"):
        self.network_table = NetworkTableInstance.getDefault().getTable(
            key=limelight_name
        )
        # self.nt_botpose = self.network_table.getFloatArrayTopic("botpose").subscribe(
        #     [-99, -99, -99, 0, 0, 0, -1]
        # )
        # returns a list of 25 zeros to zero everything out if we aren't getting anything from NT
        self.nt_botpose_blue = self.network_table.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([0] * 25)
        # self.nt_botpose_red = self.network_table.getFloatArrayTopic(
        #     "botpose_wpired"
        # ).subscribe([-99, -99, -99, 0, 0, 0, -1])

        self.nt_pipeline = self.network_table.getIntegerTopic("getpipe").subscribe(-1)
        # create the timer that we can use to the the FPGA timestamp
        self.nt_tv = self.network_table.getFloatTopic("tv").subscribe(-1)
        self.nt_tid = self.network_table.getFloatTopic("tid").subscribe(-1)
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

    # def getBotPoseEstimate(self) -> Tuple[Pose3d, Any]:
    #     if self.nt_tv.get() > 0.0:
    #         return (*self.arrayToBotPoseEstimate(self.nt_botpose.get()),)
    #     else:
    #         return (None, -1)

    def getBotPoseEstimateBlue(self) -> BotPoseResult:
        # The subscriber returns all zeros if no data is on NT
        # we can just send that right into BotPoseResult. All
        # systems using the result should check for tagCount or tagData[0].tagID > 0
        return BotPoseResult(self.nt_botpose_blue.get(), self.timer.getFPGATimestamp())

    def getBotPoseEstimateRed(self) -> Tuple[Pose3d, Any]:
        if self.nt_tv.get() > 0.0:
            return (*self.arrayToBotPoseEstimate(self.nt_botpose_red.get()),)
        else:
            return (None, -1)

    def getTargetTransform(self):
        transform_array = self.nt_targetpose_robotspace.get()
        if self.nt_tv.get() > 0.0:
            # timestamp = transform_array[6]
            transform = Transform3d(
                Translation3d(
                    transform_array[0], transform_array[1], transform_array[2]
                ),
                Rotation3d(transform_array[3], transform_array[4], transform_array[5]),
            )
            return transform  # , timestamp

    def arrayToBotPoseEstimate(self, poseArray) -> Tuple[Pose3d, Any]:
        """Takes limelight array data and creates a Pose3d object for
           robot position and a timestamp reprepresenting the time
           the position was observed.

        Args:
            poseArray (_type_): An array from the limelight network tables.

        Returns:
            Tuple[Pose3d, Any]: Returns vision Pose3d and timestamp.
        """
        (
            pX,
            pY,
            pZ,
            pRoll,
            pPitch,
            pYaw,
            msLatency,
        ) = poseArray
        return Pose3d(
            Translation3d(pX, pY, pZ), Rotation3d.fromDegrees(pRoll, pPitch, pYaw)
        ), self.timer.getFPGATimestamp() - (msLatency / 1000)


# testarray = [
#     1,
#     2,
#     3,
#     4,
#     5,
#     6,
#     7,
#     8,
#     9,
#     10,
#     11,
#     12,
#     13,
#     14,
#     15,
#     16,
#     17,
#     18,
# ]  # , 19, 20, 21, 22, 23, 24, 25, 26]
# relative_indexes = [6, 1, 4, 7, 7, 7]
# print(relative_indexes)

# from utils import partitionArray

# print(partitionArray(testarray, relative_indexes))
