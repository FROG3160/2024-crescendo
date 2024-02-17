import logging
import os
from robotpy_apriltag import AprilTagFieldLayout
import wpilib
from wpimath.geometry import Pose3d, Transform3d


apriltagsFilename = r"apriltags_layout.json"
# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue


class FROGFieldLayout(AprilTagFieldLayout):
    def __init__(self):
        self.logger = logging.getLogger("FROGFieldLayout")
        super().__init__(apriltagsLayoutPath)
        self.alliance = wpilib.DriverStation.Alliance.kInvalid
        # set layout to be specific to the alliance end

    def getTagtoRobotTransform(self, fieldPose: Pose3d, tagID: int) -> Transform3d:
        return fieldPose - self.getTagPose(tagID)

    def getPosition(self, position: int) -> Pose3d:
        """Returns the field pose for the robot to use to be in front
        of the given grid position

        Args:
            position (int): Grid position, from 1 to 9 with 1 being the furthest right

        Returns:
            Pose3d: Field pose
        """
        return self.getTagRelativePosition(*self.gridPositions[position])

    def getAlliance(self):
        return wpilib.DriverStation.getAlliance()

    # set alliance/change origin
    def syncAlliance(self, alliance=getAlliance()):
        self.logger.info(f"FROGFieldLayout.setAlliance() called with {alliance}")
        if alliance == RED_ALLIANCE:
            self.setOrigin(self.OriginPosition.kRedAllianceWallRightSide)
            self.alliance = RED_ALLIANCE
        elif alliance == BLUE_ALLIANCE:
            self.setOrigin(self.OriginPosition.kBlueAllianceWallRightSide)
            self.alliance = BLUE_ALLIANCE
