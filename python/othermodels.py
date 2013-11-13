"""\
Alternate coverage models from the literature, for comparison.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import atan2, pi, cos
from copy import copy

from adolphus.coverage import Task, Camera, Model
from adolphus.laser import RangeTask, RangeCamera, RangeModel, LineLaser
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

class ScottTask(RangeTask):
    """\
    Range imaging task model class.

    The L{ScottTask} adds to the set of task parameters:

        - C{sample_density}: The number of samples along the transport direction
                             in one range image.
    """
    defaults = copy(RangeTask.defaults)
    defaults['sample_density'] = 512

class ScottCamera(RangeCamera):
    """\
        * W. R. Scott, "Model-Based View Planning," Machine Vision and Applications,
          vol. 20, no. 1, pp. 47-69, 2009.
        * W. R. Scott, G. Roth, and J. F. Rivest, "Pose Error Effects on Range Sensing,"
          In 15th Int. Conf. on Vision Interface, Calgary, pp. 331-338, 2002.
        * S. El-Hakim, J. A. Beraldin, "Configuration Design for Sensor Integration,"
          In Proc. Videometrics IV, SPIE, Philadelphia, vol. 2598, pp. 274-285, 1995.

        The general model is presented in [Scott 2009], however many implementation
        details are drawn from [Scott 2002] and [El-Hakim 1995].
    """
    def unit_step(self, x):
        """\
        This function implements the unit step, and is implemented mainly
        for ease of readability.

        @param x: The input value.
        @type x: C{float}
        @return: The state of the step function.
        @rtype: C{bool}
        """
        return 0 if x <= 0 else 1

    def mp(self, cp, theta_xz, theta_yz):
        """\
        Measurement precision.

        @param cp: The point to test, already in camera coordinates.
        @type cp: L{Point}
        @param theta_xz: The incidence angle in the x-z plane.
        @type theta_xz: L{Angle}
        @param theta_yz: The incidence angle in the y-z plane.
        @type theta_yz: L{Angle}
        @return: The measurement precision.
        @rtype: C{bool}
        """
        # C_z is a curve fitting coefficient of the response of the 3D sensor's
        # accuracy versus the "z" distance variation with respect to "zS",
        # see El-Hakim 1995, Section 5.2. The numerator of sigma_z is effectively
        # modeling focus and the depth of field. A mayor flaw is that C_z is obtained
        # from calibration of the camera system, however this calibration assumes
        # a lens and therefore a standoff distance has been selected. Since the
        # standoff distance is part of the viewpoint selection process itself, this
        # method faces the "Chicken and egg problem".
        # Since neither Scott nor El-Hakim explain more about C_z, we set it here to 1.
        C_z = 1

        # t_yz is the cut-off angle on the y-z plane at which the sensor can not
        # produce an output, Scott 2009 Section 2.3.1.
        t_yz = (60. * pi) / 180.
        t_xz = (70. * pi) / 180.
        try:
            sigma_z = (C_z * cp.z**2) / \
                (cos(theta_yz) * (1 - self.unit_step(abs(theta_yz) - t_yz)) * \
                (1 - self.unit_step(abs(theta_xz) - t_xz)))
        except ZeroDivisionError:
            sigma_z = 0
        return sigma_z

    def sd(self, cp, laser_pose, task_params, theta_xz, theta_yz):
        """\
        Sampling density.

        @param cp: The point to test, already in camera coordinates.
        @type cp: L{Point}
        @param laser_pose: The laser's origin point.
        @type laser_pose: L{Pose}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @param theta_xz: The incidence angle in the x-z plane.
        @type theta_xz: L{Angle}
        @param theta_yz: The incidence angle in the y-z plane.
        @type theta_yz: L{Angle}
        @return: The sampling density.
        @rtype: C{bool}
        """
        # phi_x is sensor's angular field of view in the x-z plane.
        phi_x = self.fov['ah']

        # f_d is a standoff distance adjustment, f_d = 1 + d, d << 1.
        # See Scott 2009, Section 4.4.3.
        f_d = 1.02

        # R_o is the optimum sensor scanning range.
        # See Scott 2009, Section 4.4.3.
        R_o = max(self.zres(task_params['res_max'][1]), \
                  self.zc(task_params['blur_max'][1] * \
                  min(self._params['s']))[0])

        # phi_xz is the angle between the z axis in the wcs and the line between the
        # scene point and the laser.
        point = self.pose.map(cp)
        lp = laser_pose.inverse().map(point)
        phi_xz = Point(0, 0, -1).angle(lp)

        # R_xz is the slant range. See Scott 2009, Section 2.3.1.
        R_xz = cp.z / cos(phi_xz)

        N_x = self._params['dim'][0]
        N_y = task_params['sample_density']

        rho_z = ((N_x - 1)**2 * (N_y - 1)**2 * cos(theta_xz)**2 * cos(theta_yz)**2) / \
            (phi_x**2 * ((R_xz**2 * (N_y - 1)**2 * cos(theta_yz)**2) + \
            (f_d**2 * R_o**2 * (N_x - 1)**2 * cos(theta_xz)**2)))

        return rho_z

    def strength(self, point, laser_pose, task_params):
        """\
        Return the coverage strength for a directional point. Note that since
        the L{Camera} object is not internally aware of the scene it inhabits,
        occlusion is computed in the L{Model} object instead.

        @param point: The (directional) point to test.
        @type point: L{Point}
        @param laser_pose: The laser's origin point.
        @type laser_pose: L{Pose}
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: The coverage strength of the point.
        @rtype: C{float}
        """
        cp = self.pose.inverse().map(point)
#         try:
#             if abs(cp.direction_unit().x) > 1e-4:
#                 raise ValueError('point is not aligned for range coverage')
#         except AttributeError:
#             raise TypeError('point must be directional for range coverage')

        # theta_yz is the incidence angle in the y-z plane of the laser with respect to
        # the normal of the point's surface (in the global reference frame).
        # See Scott 2009, Section 4.4.3.
        temp_p = Point(0., point.direction_unit().y, point.direction_unit().z)
        # This makes the assumption that the laser plane is orthogonal to the transport
        # direction.
        theta_yz = Point(0., 0., 1.).angle(temp_p)
        temp_p = Point(point.direction_unit().x, 0., point.direction_unit().z)
        theta_xz = Point(0., 0., 1.).angle(temp_p)

        return self.cv(cp, task_params) * \
            self.unit_step(self.mp(cp, theta_xz, theta_yz)) * \
            self.unit_step(self.sd(cp, laser_pose, task_params, theta_xz, theta_yz))

class ScottModel(RangeModel):
    yaml = {'cameras': ScottCamera, 'lasers': LineLaser, 'tasks': ScottTask}


modeltypes['scott'] = ScottModel
