import math

def constrain_radians(rads):
    """Returns radians between -pi and pi
    Args:
        rads (float): angle in radians"""
    return math.atan2(math.sin(rads), math.cos(rads))