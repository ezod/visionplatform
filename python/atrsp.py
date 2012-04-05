"""\
Active triangulation 3D range imaging sensor planning optimizer.

Assumptions:

    * Only one camera is being optimized at a time.
    * The task represents the goal for that camera.
    * The laser projects along M{-z} and across M{x}.
    * The target is reasonably positioned for scanning.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import argparse
import csv
from bisect import bisect
from math import pi, sin, cos
from random import gauss

try:
    import cPickle as pickle
except ImportError:
    import pickle

import warnings
with warnings.catch_warnings():
    warnings.filterwarnings('ignore', category=UserWarning)
    from adolphus.geometry import Angle, Point, DirectionalPoint, Rotation, Pose, gaussian_pose_error
    from adolphus.laser import RangeModel
    from adolphus.yamlparser import YAMLParser
    try:
        from adolphus.interface import Experiment
    except ImportError:
        Experiment = None

import psoerr as pso


class DummyExperiment(object):
    def __init__(self):
        self.model = {}
        self.tasks = {}


class LensLUT(object):
    """\
    Lens intrinsic parameter lookup table.
    """
    def __init__(self, filename, fnumber):
        """\
        Constructor.

        @param filename: The path to the sorted CSV file specifying the LUT.
        @type filename: C{str}
        @param fnumber: The f-number (for computing effective aperture).
        @type fnumber: C{float}
        """
        lines = csv.reader(open(filename, 'r'))
        self.values = []
        # zS, f, ou, ov
        for line in lines:
            self.values.append([float(n) for n in line])
        self.fnumber = fnumber

    @property
    def bounds(self):
        """\
        Bounds of the subject distance in the LUT.
        """
        return (self.values[0][0], self.values[0][-1])

    def parameters(self, zS):
        """\
        Interpolate and return the intrinsic parameters for a given subject
        distance.

        @param zS: The subject distance.
        @type zS: C{float}
        @return: The corresponding interpolated intrinsic parameters.
        @rtype: C{tuple} of C{float}
        """
        assert zS >= self.bounds[0] and zS <= self.bounds[1]
        params = []
        i = bisect(self.values[0], zS)
        if i == 0:
            for j in range(1, len(self.values)):
                params.append(self.values[j][0])
        elif i == len(self.values[0]):
            for j in range(1, len(self.values)):
                params.append(self.values[j][-1])
        else:
            prop = (zS - self.values[0][i - 1]) / \
                (self.values[0][i] - self.values[0][i - 1])
            for j in range(1, len(self.values)):
                params.append(self.values[j][i - 1] + \
                    prop * (self.values[j][i] - self.values[j][i - 1]))
        # A
        params.append(params[0] / self.fnumber)
        return params
        

def load_model(modelfile, cameras, fnumber, visualize=False):
    if visualize:
        ex = Experiment()
        ex.execute('loadmodel %s' % modelfile)
        ex.execute('loadconfig')
    else:
        ex = DummyExperiment()
        ex.model, ex.tasks = YAMLParser(modelfile).experiment
    # situate cameras on a side of the laser
    for camera in cameras:
        if ex.model[camera[0]].pose.T.y < \
           ex.model[ex.model.active_laser].pose.T.y:
            ex.model[camera[0]].negative_side = True
        else:
            ex.model[camera[0]].negative_side = False
    # disable unspecified cameras
    for camera in ex.model.cameras:
        if not camera in [c[0] for c in cameras]:
            ex.model[camera].active = False
    # load lens lookup tables
    lut = []
    for camera in cameras:
        lut.append(LensLUT(camera[1], fnumber))
    return ex, lut


def modify_camera(model, camera, lut, x, h, d, beta):
    if model[camera].negative_side:
        y = -d * sin(beta) + model[model.active_laser].pose.T.y
        z = d * cos(beta) + h
        R = Rotation.from_axis_angle(pi + beta, Point(1, 0, 0))
    else:
        y = d * sin(beta) + model[model.active_laser].pose.T.y
        z = d * cos(beta) + h
        R = Rotation.from_axis_angle(pi, Point(0, 1, 0)) + \
            Rotation.from_axis_angle(-beta, Point(-1, 0, 0))
    T = Point(x, y, z)
    model[camera].set_absolute_pose(Pose(T, R))
    f, ou, ov, A = lut.parameters(d)
    model[camera].setparam('zS', d)
    model[camera].setparam('f', f)
    model[camera].setparam('o', [ou, ov])
    model[camera].setparam('A', A)


def get_bounds(model, task, cameras, lut):
    # compute bounds on x and h
    xmin, xmax = float('inf'), -float('inf')
    zmin, zmax = float('inf'), -float('inf')
    for point in task.mapped:
        lp = model[model.active_laser].triangle.intersection(point,
            point + Point(0, 1, 0), False)
        if lp:
            if lp.x < xmin:
                xmin = lp.x
            if lp.x > xmax:
                xmax = lp.x
            if lp.z < zmin:
                zmin = lp.z
            if lp.z > zmax:
                zmax = lp.z
    # compute bounds on d
    dbounds = []
    Ra = task.getparam('res_min')[1]
    Ha = task.getparam('hres_min')[1]
    for c, camera in enumerate(cameras):
        modify_camera(model, camera, lut[c], 0, 0, lut[c].bounds[1],
            task.getparam('angle_max')[1])
        p = (-model[camera].pose).map(DirectionalPoint(0, 0, 0, 0, 0))
        angle = p.direction_unit().angle(-p)
        du = min(model[camera].zres(Ra),
                 model[camera].zhres(Ha, angle))
        dbounds.append((lut[c].bounds[0], min(lut[c].bounds[1], du)))
    # return bounds
    bounds = []
    for i in range(len(cameras)):
        bounds += [(xmin, xmax), (zmin, zmax), dbounds[i],
                   (0, task.getparam('angle_max')[1])]
    return bounds


def gaussian_yz_pose_error(pose, tsigma, rsigma):
    T, R = pose.T, pose.R
    T = Point(gauss(T.x, tsigma), gauss(T.y, tsigma), gauss(T.z, tsigma))
    R += Rotation.from_axis_angle(Angle(gauss(0, rsigma)), Point(1, 0, 0))
    return Pose(T=T, R=R)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fnumber', dest='fnumber', type=float, default=1.0)
    parser.add_argument('-S', '--size', dest='size', type=int)
    parser.add_argument('-w', '--omega', dest='omega', type=float)
    parser.add_argument('-p', '--phip', dest='phip', type=float)
    parser.add_argument('-n', '--phin', dest='phin', type=float)
    parser.add_argument('-l', '--clamp', dest='clamp', type=float, default=1.0)
    parser.add_argument('-i', '--iterations', dest='it', type=int, default=None)
    parser.add_argument('-a', '--accept', dest='af', type=float, default=float('inf'))
    parser.add_argument('-u', '--cluster', dest='cluster', nargs=2, type=float, default=(1.0, 0.0))
    parser.add_argument('-t', '--topology', dest='topology', choices=pso.topologies.keys())
    parser.add_argument('-c', '--constraint', dest='constraint', choices=pso.constraints.keys())
    parser.add_argument('-C', '--cameras', dest='cameras', nargs=2, action='append')
    parser.add_argument('-v', '--visualize', dest='visualize', action='store_true', default=False)
    parser.add_argument('-F', '--fvalues', dest='fvalues', action='store_true', default=False)
    parser.add_argument('-R', '--report', dest='report', action='store_true', default=False)
    parser.add_argument('--cerror', dest='cerror', type=float, nargs=2, default=(0.0, 0.0))
    parser.add_argument('--terror', dest='terror', type=float, nargs=2, default=(0.0, 0.0))
    parser.add_argument('-e', '--nerror', dest='nerror', type=int, default=0)
    parser.add_argument('modelfile')
    parser.add_argument('task')
    parser.add_argument('datafile')
    args = parser.parse_args()
    if not Experiment:
        args.visualize = False

    ex, lut = load_model(args.modelfile, args.cameras, args.fnumber, args.visualize)

    # get solution space bounds
    bounds = get_bounds(ex.model, ex.tasks[args.task],
        [camera[0] for camera in args.cameras], lut)

    # create transport context
    transport = RangeModel.LinearTargetTransport(ex.model)

    Ftimes = 0

    # define fitness function
    def fitness(particle):
        globals()['Ftimes'] += 1
        for i in range(len(args.cameras)):
            x, h, d, beta = particle[4 * i: 4 * (i + 1)]
            if d < lut[i].bounds[0] or d > lut[i].bounds[1]:
                return -float('inf')
            modify_camera(ex.model, args.cameras[i][0], lut[i],
                x, h, d, beta)
        coverage = ex.model.range_coverage(ex.tasks[args.task], transport)
        return ex.model.performance(ex.tasks[args.task], coverage=coverage)

    # define fitness with error function
    def fitness_e(particle):
        if not args.nerror:
            return -float('inf')
        globals()['Ftimes'] += args.nerror
        for i in range(len(args.cameras)):
            x, h, d, beta = particle[4 * i: 4 * (i + 1)]
            if d < lut[i].bounds[0] or d > lut[i].bounds[1]:
                return -float('inf')
            modify_camera(ex.model, args.cameras[i][0], lut[i],
                x, h, d, beta)
        original_pose = {}
        for obj in [ex.tasks[args.task].mount.name] + [c[0] for c in args.cameras]:
            original_pose[obj] = ex.model[obj].pose
        perf = []
        for i in range(args.nerror):
            for obj in original_pose:
                if obj in ex.model.cameras:
                    ex.model[obj].set_absolute_pose(gaussian_yz_pose_error(original_pose[obj], *args.cerror))
                else:
                    ex.model[obj].set_absolute_pose(gaussian_pose_error(original_pose[obj], *args.terror))
            coverage = ex.model.range_coverage(ex.tasks[args.task], transport)
            perf.append(ex.model.performance(ex.tasks[args.task], coverage=coverage))
        for obj in original_pose:
            ex.model[obj].set_absolute_pose(original_pose[obj])
        return sum(perf) / float(args.nerror)

    # load visualization
    if args.visualize:
        ex.start()
        ex.event.wait()

    # optimize
    F, Fe = [], []
    try:
        for best, Fi, Fei in pso.particle_swarm_optimize(fitness, fitness_e,
            4 * len(args.cameras), bounds, args.size, args.omega, args.phip,
            args.phin, args.clamp, args.it, args.af, args.cluster,
            topology_type=args.topology, constraint_type=args.constraint):
            F.append(Fi)
            Fe.append(Fei)
            if args.fvalues:
                print('%g\t%g' % (Fi, Fei))
            if args.visualize:
                for c, camera in enumerate(args.cameras):
                    modify_camera(ex.model, camera[0], lut[c],
                        *best[4 * c: 4 * (c + 1)])
                    ex.model[camera[0]].update_visualization()
    except KeyboardInterrupt:
        pass

    # dump data
    result = {'args': args, 'best': best, 'F': F, 'Fe': Fe, 'Ftimes': Ftimes}
    pickle.dump(result, open(args.datafile, 'w'))

    if args.report:
        print('-' * 80)
        print('SENSOR PLANNING REPORT')
        print('-' * 80)
        print('Global best after %d iterations: %f' % (len(F), F[-1]))
        print('-' * 80)
        for c, camera in enumerate(args.cameras):
            params = best[4 * c: 4 * (c + 1)]
            print('Camera %s: x = %g, h = %g, d = %g, beta = %g' \
                % ((camera[0],) + params))
            modify_camera(ex.model, camera[0], lut[c], *params)
