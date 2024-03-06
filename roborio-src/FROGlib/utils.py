import math
from wpimath.geometry import Transform3d, Translation3d


def constrain_radians(rads):
    """Returns radians between -pi and pi
    Args:
        rads (float): angle in radians"""
    return math.atan2(math.sin(rads), math.cos(rads))


def remap(val, OldMin, OldMax, NewMin, NewMax):
    """take a value in the old range and return a value in the new range"""
    return (((val - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin


def getRangeFromTransform(transform: Transform3d) -> float:
    translation = transform.translation()
    return math.sqrt(translation.x**2 + translation.y**2 + translation.z**2)


def getAngleFromTransform(transform: Transform3d) -> float:
    translation = transform.translation()
    return math.atan2(translation.y, translation.x)


class ShootingSolution:
    def __init__(self, translation: Translation3d):
        self.x = translation.x
        self.y = translation.y
        self.z = translation.z
        self.range = self.calculateRange()
        self.azimuth = self.calculateAzimuth()
        self.elevation = self.calculateElevation()

    def calculateRange(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def calculateAzimuth(self):
        return math.atan2(self.y, self.x)

    def calculateElevation(self):
        return math.acos(math.sqrt(self.x**2 + self.y**2) / self.range)


# from robotpy_apriltag import loadAprilTagLayoutField, AprilTagField
# from wpimath.geometry import Pose3d, Rotation3d
# field = loadAprilTagLayoutField(AprilTagField.k2024Crescendo)
# field.setOrigin(field.OriginPosition.kBlueAllianceWallRightSide)
# robotPose = Pose3d(13, 0, 0, Rotation3d(0, 0, 0))
# tagPose =  field.getTagPose(7)
# print(tagPose.__str__())
# test = tagPose - robotPose
# print(test.__str__())
# ss = ShootingSolution(test.translation())
# from math import degrees
# print(degrees(ss.calculateAzimuth()))
