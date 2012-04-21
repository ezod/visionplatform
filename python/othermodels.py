"""\
Alternate coverage models from the literature, for comparison.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from adolphus.coverage import Camera
from adolphus.geometry import Point


class HorsterLienhartCamera(Camera):
    """\
        * E. Horster and R. Lienhart, "Optimal Placement of Multiple Visual
          Sensors," in Multi-Camera Networks: Principles and Applications, H.
          Aghajan and A. Cavallaro, Eds. Academic Press, 2009, pp. 117-138.
    """
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
        cp = self.pose.inverse().map(point)
        d = self.zres(task_params['res_min'][1])
        a = self.fov['tah'] / 2.0
        if cp.z > 0 and cp.z < d and cp.x < a * cp.z and cp.x > -a * cp.z:
            return 1.0
        return 0.0


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
