import numpy as np
import math  


def unit_vector(vector):
    """ Returns the unit vector of the vector """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    dot = v1_u[0]*v2_u[0] + v1_u[1]*v2_u[1]      # dot product
    det = v1_u[0]*v2_u[1] - v1_u[1]*v2_u[0]      # determinant
    aux = math.atan2(det, dot)
    if aux < 0:
        aux += 2*math.pi
    return aux  # atan2(y, x) or atan2(sin, cos)

def rotate_vector(x, angle):
    """Rotate vector x anticlockwise around the origin by angle degrees, return angle in format [x, y]"""
    y1 = math.cos(angle)*x[0] - math.sin(angle)*x[1]
    y2 = math.sin(angle)*x[0] + math.cos(angle)*x[1]
    return [y1, y2]


def unit_convert(input_unit, whidth_conv, height_conv):
    """ Convert input data, in centimeters, read from ROS, for virtual field data, in pixels """
    return (int(input_unit[0]*whidth_conv),int(input_unit[1]*height_conv))


def position_from_origin(position_tuple, origin):
    """ calculate the pixel of the center of a robot from origin as reference """

    return (origin[0]+position_tuple[0],(origin[1]-position_tuple[1]))