"""\
Active triangulation 3D range imaging sensor planning optimizer.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, sin, cos

from adolphus.geometry import Point, Rotation, Pose

def camera_pose_from_xhdb(x, h, d, beta):
    y = d * sin(beta)
    z = d * cos(beta) + h
    T = Point((x, y, z))
    R = Rotation.from_axis_angle(pi - beta, Point((1, 0, 0)))
    return Pose(T, R)
