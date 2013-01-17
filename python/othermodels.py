"""\
Alternate coverage models from the literature, for comparison.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import atan2, pi
from copy import copy

from adolphus.coverage import Task, Camera, Model
from adolphus.geometry import Point, Angle, Rotation, Pose
from adolphus.yamlparser import modeltypes


class HorsterLienhartCamera(Camera):
    """\
        * E. Horster and R. Lienhart, "Optimal Placement of Multiple Visual
          Sensors," in Multi-Camera Networks: Principles and Applications, H.
          Aghajan and A. Cavallaro, Eds. Academic Press, 2009, pp. 117-138.
    """
    def _flatten_pose(self):
        du = self.pose.map(Point(0, 0, 1)) - self.pose.T
        R = \
            Rotation.from_axis_angle(Angle(atan2(du.y, du.x) - pi / 2.0), Point(0, 0, 1)) + \
            Rotation.from_axis_angle(-pi / 2.0, Point(1, 0, 0))
        flat_pose = Pose(T=Point(self.pose.T.x, self.pose.T.y, 50), R=R)
        self.set_absolute_pose(flat_pose)
        self._flatten_pose = lambda: None

    def strength(self, point, task_params):
        """\
        Return the coverage strength for a directional point. Note that since
        the L{Camera} object is not internally aware of the scene it inhabits,
        occlusion is computed in the L{Model} object instead.

        Assumptions:
        
            * High walls (infinite planes induced by M{x}-M{y} line segments).
            * Cameras are rotated only about M{z}.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        self._flatten_pose()
        cp = self.pose.inverse().map(point)
        d = self.zres(task_params['res_min'][1])
        a = self.fov['tah'] / 2.0
        if cp.z > 0 and cp.z < d and cp.x < a * cp.z and cp.x > -a * cp.z:
            return 1.0
        return 0.0


class HorsterLienhartModel(Model):
    yaml = {'cameras': HorsterLienhartCamera, 'tasks': Task}


modeltypes['horsterlienhart'] = HorsterLienhartModel


class ZhaoTask(Task):
    """\
    Range imaging task model class.

    The L{ZhaoTask} adds to the set of task parameters:

        - C{feature_length}: minimum height resolution (mm/pixel), ideal/acceptable.
        - C{projected_length_min}: min. projected length of a feature (pixels).
    """
    defaults = copy(Task.defaults)
    defaults['feature_length'] = 0.0
    defaults['projected_length_min'] = 0.0


class ZhaoCamera(Camera):
    """\
        * J. Zhao, S.-C. Cheung, and T. Nguyen, "Optimal Camera Network
          Configurations for Visual Tagging," IEEE J. Sel. Topics Signal
          Processing, vol. 2, no. 4, pp. 464-479, 2008.
    """
    def strength(self, point, task_params):
        """\
        Return the coverage strength for a directional point. Note that since
        the L{Camera} object is not internally aware of the scene it inhabits,
        occlusion is computed in the L{Model} object instead.

        Assumptions:

            * Vertical occlusions (planes project to M{x}-M{y} line segments).

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        cp = self.pose.inverse().map(point)
        try:
            sigma = -(cp.unit()).dot(cp.direction_unit())
        except (ValueError, AttributeError):
            sigma = 1.0
        try:
            zl = self.zres(task_params['feature_length'] * sigma / \
                task_params['projected_length_min'])
        except ZeroDivisionError:
            zl = float('inf')
        return self.cv(cp, Task.defaults) * self.cd(cp, Task.defaults) \
            if cp.z < zl else 0.0


class ZhaoModel(Model):
    yaml = {'cameras': ZhaoCamera, 'tasks': ZhaoTask}


modeltypes['zhao'] = ZhaoModel


class ParkCamera(Camera):
    """\
        * J. Park, P. C. Bhat, and A. C. Kak, "A Look-Up Table Based Approach
          for Solving the Camera Selection Problem in Large Camera Networks,"
          in Proc. Int. Wkshp. Distributed Smart Cameras, 2006.
    """
    def strength(self, point, task_params):
        """\
        Return the coverage strength for a directional point. Note that since
        the L{Camera} object is not internally aware of the scene it inhabits,
        occlusion is computed in the L{Model} object instead.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        cp = self.pose.inverse().map(point)
        zn, zf = self.zc(task_params['blur_max'][1] * min(self._params['s']))
        vfc = Point(0, 0, (zf - zn) / 2.0 + zn)
        bn, bf = [self.fov['tah'] * self.fov['tav'] * z ** 2 for z in zn, zf]
        # FIXME: this does not return a value in [0, 1]
        try:
            return vfc.euclidean(cp) / ((zf * bf - zn * bn) / 3.0)
        except ZeroDivisionError:
            return 0.0
