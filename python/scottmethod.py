"""
Implementation of Scott's viewpoint selection method.

* W. R. Scott, "Model-Based View Planning," Machine Vision and Applications,
vol. 20, no. 1, pp. 47-69, 2009.

@author: Jose Alarcon
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""
import yaml
import argparse
from time import time
from copy import deepcopy
from pprint import pprint
from numpy import array, zeros, matrix, linalg
from math import pi, sqrt, sin, cos, asin, acos, atan2

from adolphus.interface import Experiment
from adolphus.coverage import PointCache, Camera, Task
from adolphus.geometry import gaussian_pose_error, DirectionalPoint
from adolphus.geometry import Angle, Point, Quaternion, Rotation, Pose


def avg_points(X):
    """
    Average a list of 3D points.

    @param X: The 3D points.
    @type X: C{List} of L{Point}
    @return: The average.
    @rtype: L{Point}
    """
    avg = Point(0,0,0)
    for p in X:
        avg += p
    return avg / float(len(X))

def read_csv(filename, field_names=None):
    """\
    Read the contents of a coma separated file. The elements in the first row are
    considered as the field names and the keys of the dictionary if no field names
    are passed.

    @param filename: The name of the csv file.
    @type filename: C{str}
    @param field_names: The list of field names.
    @type field_names: C{list} of C{str}
    @return: The dictionary listing the contents of the csv file and the original
        order of the field names.
    @rtype: C{dict} of {list} and C{list}
    """
    separator = None
    with open(filename, 'rU') as file:
        if not field_names:
            field_names = file.readline()[0:-1]
            if ',' in field_names:
                separator = ','
            elif '\t' in field_names:
                separator = '\t'
            field_names = field_names.split(separator)
            if field_names[0][0] == "\"" and field_names[0][-1] == "\"":
                for field in field_names:
                    field_names[field_names.index(field)] = field[1:-1]
        csv_dict = {}
        for line in file:
            if line[-1] == '\n':
                items = line[0:-1]
            else:
                items = line
            if not separator:
                if ',' in items:
                    separator = ','
                elif '\t' in items:
                    separator = '\t'
            items = items.split(separator)
            for field in field_names:
                item = items[field_names.index(field)]
                if not item:
                    item = ''
                else:
                    if item[0] == "\"" and item[-1] == "\"":
                        item = item[1:-1]
                try:
                    csv_dict[field].append(float(item))
                except KeyError:
                    csv_dict[field] = [float(item)]
    return csv_dict, field_names

class ScottMethod(object):
    """
    Scott's viewpoint selection method and experiment class.
    """
    def __init__(self, model_file, lens_lut, vis=False):
        """
        Constructor.

        @param model_file: The YAML file to load the camera model.
        @type model_file: C{str}
        @param lens_list: List of lenses for optimization.
        @type lens_list: C{list} of C{dict}
        @param vis: Enable visualization.
        @type vis: C{bool}
        """
        csv_dict, field_names = read_csv(lens_lut)
        self.lens_dict = csv_dict
        self.vis = vis

        # setup the camera model.
        self.exp = Experiment(zoom=False)
        # TODO: Load Scott's range model and coverage model.
        self.exp.execute('loadmodel %s' % model_file)
        self.exp.execute('loadconfig %s' % '')

        # Some shortcuts.
        self.model = self.exp.model
        self.cam = self.exp.model['C']
        self.laser = self.exp.model['L']
        self.tasks = self.exp.tasks
        self.task_par = self.tasks['T'].params
        self.cam_par = self.model['C'].params

        # start the experiment.
        if self.vis:
            self.exp.start()

    def run(self):
        """
        The main experiment.
        """
        # Generate the scene point from the scene directly.
        scene_points = self.gen_scene_points(list(self.model['CAD'].triangles))
        scene_points_c = PointCache()
        for point in scene_points:
            scene_points_c[point] = 1.0
        self.tasks['T'] = Task(self.task_par, scene_points_c)

        # Visualize scene points.
        if self.vis:
            self.tasks['T'].visualize()
            self.tasks['T'].update_visualization()

        # Generate the viewpoint solution space.
        view_points, view_point_poses = self.gen_view_points(scene_points)

        # Visualize view points.
        if self.vis:
            view_points_c = PointCache()
            for point in view_points:
                view_points_c[point] = 1.0
            view_points_c.visualize()

        # Generate the measurability matrix.
        m_matrix = self.gen_visual_matrix(view_points, view_point_poses, scene_points)

    def update_camera(self, index):
        """
        Update the camera parameters.

        @param index: The location of the camera parameters in the parameter's list.
        @type index: C{int}
        """
        A = self.lens_dict['f'][index] / self.lens_dict['stop'][index]
        o = [self.lens_dict['ou'][index], self.lens_dict['ov'][index]]
        self.cam.setparam('A', A)
        self.cam.setparam('f', self.lens_dict['f'][index])
        self.cam.setparam('o', o)
        self.cam.setparam('zS', self.lens_dict['zS'][index])

    def gen_scene_points(self, triangles):
        """\
        Generate the directional points that model the task from the list of
        triangles of the associated CAD file as given by the raw triangle
        representation.

        @param triangles: The collection of triangles in the model representation.
        @type triangles: C{list} of L{adolphus.geometry.Triangle}
        @return: A list of directional points.
        @rtype: C{list} of L{DirectionalPoint}
        """
        points = []
        for occ_triangle in triangles:
            triangle = occ_triangle.mapped_triangle()
            average = avg_points(triangle.vertices)
            normal = triangle.normal()
            points.append(DirectionalPoint(average.x, average.y, average.z, \
                normal.angle(Point(0, 0, 1)), atan2(normal.y, normal.x)))
        return points

    def gen_view_points(self, scene_points, mode='flat'):
        """\
        Generate the solution space as the list of viewpoints.

        @param scene_points: The list of scene points in the task model.
        @type scene_points: C{list}
        @param mode: Select whether to position all cameras flat, vertical, or
            alternate between flat and vertical. Options: C{flat}, C{alternate},
            C{vertical}
        @type mode: C{str}
        @return: A pair of lists containing the viewpoints and their poses.
        @rtype: C{list}
        """
        view_points = []
        view_point_poses = []
        flat = True
        # Standoff calculation as described in Scott 2009, Section 4.4.3.
        f_d = 1.02
        R_o = max(self.cam.zres(self.task_par['res_max'][1]), \
            self.cam.zc(self.task_par['blur_max'][1] * \
            min(self.cam.getparam('s')))[0])
        e = 1e9
        for zS in self.lens_dict['zS']:
            if abs(zS - (f_d * R_o)) < e:
                e = abs(zS - (f_d * R_o))
                index = self.lens_dict['zS'].index(zS)
        cam_standoff = self.lens_dict['zS'][index]
        # Updating the camera after computing the camera standoff distance simulates
        # the action of focusing the lens.
        self.update_camera(index)
        # This implementation assumes that the camera is mounted on the laser
        # and it is in fact the laser that moves.
        self.cam.mount = self.laser
        T = Point(0, cam_standoff * sin(0.7853), 0)
        R = Rotation.from_euler('zyx', (Angle(-0.7853), Angle(0), Angle(0)))
        self.cam.set_relative_pose(Pose(T, R))
        # The laser standoff.
        standoff = cam_standoff * cos(0.7853)
        for point in scene_points:
            x = point.x + (standoff * sin(point.rho) * cos(point.eta))
            y = point.y + (standoff * sin(point.rho) * sin(point.eta))
            z = point.z + (standoff * cos(point.rho))
            rho = point.rho + pi
            eta = point.eta
            point = DirectionalPoint(x, y, z, rho, eta)
            view_points.append(point)
            normal = point.direction_unit()
            angle = Point(0, 0, 1).angle(normal)
            axis = Point(0, 0, 1).cross(normal)
            pose = Pose(Point(x, y, z), Rotation.from_axis_angle(angle, axis))
            angles = pose.R.to_euler_zyx()
            if mode == 'flat':
                new_angles = (angles[0], angles[1], Angle(1.5707))
            elif mode == 'vertical':
                new_angles = (angles[0], angles[1], Angle(0))
            elif mode == 'alternate':
                if flat:
                    new_angles = (angles[0], angles[1], Angle(1.5707))
                    flat = False
                elif not flat:
                    new_angles = (angles[0], angles[1], Angle(0))
                    flat = True
            new_pose = Pose(Point(x,y,z), Rotation.from_euler('zyx', new_angles))
            view_point_poses.append(new_pose)
        return view_points, view_point_poses

    def gen_visual_matrix(self, view_points, view_point_poses, scene_points):
        """\
        Compute the measurability matrix.

        @param view_points: The list of viewpoints in the solution space.
        @type view_points: C{list}
        @param view_point_poses: The list of poses of the viewpoints.
        @type view_point_poses: C{list}
        @param scene_points: the list of scene points in the task model.
        @type scene_points: C{list}
        @return: the visual matrix whose entries represent coverage strength.
        @rtype: L{numpy.array}
        """
        num_points = len(view_points)
        vis_matrix = zeros((num_points, num_points))
        for i in range(num_points):
            for j in range(num_points):
                # This assumes that the camera is mounted on the laser and both
                # move simultaneously. See Scott 2009, Section 2.3.1.
                self.laser.pose = view_point_poses[j]
                vis_matrix[i,j] = self.cam.strength(scene_points[i], \
                    self.laser.pose, self.task_par)
        return vis_matrix

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('model', help='Yaml model file.')
    parser.add_argument('lut', help='Lens calibration file.')
    parser.add_argument('--vis', type=bool, default=False, \
        help='Eneble the visualization.')
    args = parser.parse_args()

    experiment = ScottMethod(args.model, args.lut, args.vis)
    x = raw_input('Press Enter to begin.\n')
    experiment.run()
