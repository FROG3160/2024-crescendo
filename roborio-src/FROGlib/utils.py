import math

def constrain_radians(rads):
    """Returns radians between -2*pi and 2*pi
    Args:
        rads (float): angle in radians"""
    return math.atan2(math.sin(rads), math.cos(rads))