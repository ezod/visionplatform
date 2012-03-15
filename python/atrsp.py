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

import warnings
with warnings.catch_warnings():
    warnings.filterwarnings('ignore', category=UserWarning)
    from adolphus.geometry import Point, DirectionalPoint, Rotation, Pose, \
        gaussian_pose_error
    from adolphus.laser import RangeModel
    from adolphus.yamlparser import YAMLParser
    try:
        from adolphus.interface import Experiment
    except ImportError:
        Experiment = None

import pso


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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--fnumber', dest='fnumber', type=float,
        default=1.0)
    parser.add_argument('-s', '--size', dest='size', type=int)
    parser.add_argument('-o', '--omega', dest='omega', type=float)
    parser.add_argument('-p', '--phip', dest='phip', type=float)
    parser.add_argument('-g', '--phig', dest='phig', type=float)
    parser.add_argument('-i', '--iterations', dest='it', type=int,
        default=None)
    parser.add_argument('-a', '--accept', dest='af', type=float,
        default=float('inf'))
    parser.add_argument('-t', '--topology', dest='topology',
        choices=pso.topologies.keys())
    parser.add_argument('-c', '--constraint', dest='constraint',
        choices=pso.constraints.keys())
    parser.add_argument('-C', '--cameras', dest='cameras', nargs=2,
        action='append')
    parser.add_argument('-v', '--visualize', dest='visualize',
        action='store_true', default=False)
    parser.add_argument('-R', '--report', dest='report',
        action='store_true', default=False)
    parser.add_argument('-E', '--error', dest='error',
        action='store_true', default=False)
    parser.add_argument('-F', '--fvalues', dest='fvalues',
        action='store_true', default=False)
    parser.add_argument('modelfile')
    parser.add_argument('task')
    args = parser.parse_args()
    if not Experiment:
        args.visualize = False

    if args.visualize:
        ex = Experiment()
        ex.execute('loadmodel %s' % args.modelfile)
        ex.execute('loadconfig')
    else:
        ex = DummyExperiment()
        ex.model, ex.tasks = YAMLParser(args.modelfile).experiment

    lut = []
    for camera in args.cameras:
        lut.append(LensLUT(camera[1], args.fnumber))
        if ex.model[camera[0]].pose.T.y < \
           ex.model[ex.model.active_laser].pose.T.y:
            ex.model[camera[0]].negative_side = True
        else:
            ex.model[camera[0]].negative_side = False
    for camera in ex.model.cameras:
        if not camera in [c[0] for c in args.cameras]:
            ex.model[camera].active = False

    # compute bounds on x and h
    xmin, xmax = float('inf'), -float('inf')
    zmin, zmax = float('inf'), -float('inf')
    for point in ex.tasks[args.task].mapped:
        lp = ex.model[ex.model.active_laser].triangle.intersection(point,
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
    Ra = ex.tasks[args.task].getparam('res_min')[1]
    Ha = ex.tasks[args.task].getparam('hres_min')[1]
    for c, camera in enumerate(args.cameras):
        modify_camera(ex.model, camera[0], lut[c], 0, 0, lut[c].bounds[1],
            ex.tasks[args.task].getparam('angle_max')[1])
        p = (-ex.model[camera[0]].pose).map(DirectionalPoint(0, 0, 0, 0, 0))
        angle = p.direction_unit().angle(-p)
        du = min(ex.model[camera[0]].zres(Ra),
                 ex.model[camera[0]].zhres(Ha, angle))
        dbounds.append((lut[c].bounds[0], min(lut[c].bounds[1], du)))

    bounds = []
    for i in range(len(args.cameras)):
        bounds += [(xmin, xmax), (zmin, zmax), dbounds[i],
                   (0, ex.tasks[args.task].getparam('angle_max')[1])]

    def fitness(particle):
        for i in range(len(args.cameras)):
            x, h, d, beta = particle[4 * i: 4 * (i + 1)]
            if d < lut[i].bounds[0] or d > lut[i].bounds[1]:
                return -float('inf')
            modify_camera(ex.model, args.cameras[i][0], lut[i], x, h, d, beta)
        coverage = ex.model.range_coverage(ex.tasks[args.task],
            RangeModel.LinearTargetTransport)
        return ex.model.performance(ex.tasks[args.task], coverage=coverage)

    if args.visualize:
        ex.start()
        ex.event.wait()

    if args.fvalues:
        print('%d,%g,%g,%g,%s,%s' % (args.size, args.omega, args.phip,
            args.phig, args.topology, args.constraint))
    i = 0
    try:
        for best, F in pso.particle_swarm_optimize(fitness,
            4 * len(args.cameras), bounds, args.size, args.omega, args.phip,
            args.phig, args.it, args.af, topology_type=args.topology,
            constraint_type=args.constraint):
            if args.fvalues:
                print(F)
            if args.visualize:
                for c, camera in enumerate(args.cameras):
                    modify_camera(ex.model, camera[0], lut[c],
                        *best[4 * c: 4 * (c + 1)])
                    ex.model[camera[0]].update_visualization()
            i += 1
    except KeyboardInterrupt:
        print('')

    if args.report:
        print('-' * 80)
        print('SENSOR PLANNING REPORT')
        print('-' * 80)
        print('Global best after %d iterations: %f' % (i, F))
        print('-' * 80)
        for c, camera in enumerate(args.cameras):
            params = best[4 * c: 4 * (c + 1)]
            print('Camera %s: x = %g, h = %g, d = %g, beta = %g' \
                % ((camera[0],) + params))
            modify_camera(ex.model, camera[0], lut[c], *params)

    if args.error:
        original_pose = {}
        for obj in [ex.model.active_laser, ex.tasks[args.task].mount.name] + \
            [c[0] for c in args.cameras]:
            original_pose[obj] = ex.model[obj].pose
        print('-' * 80)
        print('Error Values - (T, R) for Camera, Laser, Target')
        while True:
            errs = raw_input('> ')
            if errs == 'q':
                break
            errv = [float(v) for v in errs.split(' ')]
            for obj in original_pose:
                if obj in ex.model.cameras:
                    t, r = errv[0:2]
                elif obj in ex.model.lasers:
                    t, r = errv[2:4]
                else:
                    t, r = errv[4:6]
                if not t and not r:
                    continue
                ex.model[obj].set_absolute_pose(gaussian_pose_error(\
                original_pose[obj], t, r))
                print('%s (orig.): %s' % (obj, original_pose[obj]))
                print('%s (error): %s' % (obj, ex.model[obj].pose))
            coverage = ex.model.range_coverage(ex.tasks[args.task],
                RangeModel.LinearTargetTransport)
            print(ex.model.performance(ex.tasks[args.task], coverage=coverage))
