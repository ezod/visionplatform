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
    from adolphus.geometry import Point, Rotation, Pose
    from adolphus.interface import Experiment

from pso import particle_swarm_optimize


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
        try:
            prop = (zS - self.values[0][i - 1]) / \
                (self.values[0][i] - self.values[0][i - 1])
        except IndexError:
            # f, ou, ov
            for j in range(1, len(self.values)):
                params.append(self.values[j][0])
        else:
            # f, ou, ov
            for j in range(1, len(self.values)):
                params.append(self.values[j][i - 1] + \
                    prop * (self.values[j][i] - self.values[j][i - 1]))
        # A
        params.append(params[0] / self.fnumber)
        return params
        

def camera_pose_from_xhdb(x, h, d, beta):
    y = -d * sin(beta)
    z = d * cos(beta) + h
    T = Point((x, y, z))
    R = Rotation.from_axis_angle(pi + beta, Point((1, 0, 0)))
    return Pose(T, R)

def modify_camera(model, camera, lut, h, d, beta):
    model[camera].set_absolute_pose(\
        camera_pose_from_xhdb(model[camera].pose.T.x, h, d, beta))
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
    parser.add_argument('-t', '--topology', dest='topology')
    parser.add_argument('-c', '--constraint', dest='constraint')
    parser.add_argument('-v', '--visualize', dest='visualize',
        action='store_true', default=False)
    parser.add_argument('lutfile')
    parser.add_argument('modelfile')
    parser.add_argument('task')
    parser.add_argument('cameras', nargs='+')
    args = parser.parse_args()

    lut = LensLUT(args.lutfile, args.fnumber)
    ex = Experiment()
    ex.execute('loadmodel %s' % args.modelfile)
    ex.execute('loadconfig')

    # compute bounds on h
    zmin, zmax = float('inf'), -float('inf')
    for point in ex.tasks[args.task].mapped:
        lp = ex.model[ex.model.active_laser].triangle.intersection(point,
            point + Point((0, 1, 0)), limit=False)
        if lp:
            if lp.z < zmin:
                zmin = lp.z
            if lp.z > zmax:
                zmax = lp.z

    bounds = ((zmin, zmax), lut.bounds,
              (0, ex.tasks[args.task].getparam('angle_max_acceptable')))

    def fitness(particle):
        h, d, beta = particle
        if d < lut.bounds[0] or d > lut.bounds[1]:
            return -float('inf')
        modify_camera(ex.model, args.cameras[0], lut, h, d, beta)
        coverage = ex.model.range_coverage_linear(ex.tasks[args.task])
        return ex.model.performance(ex.tasks[args.task], coverage=coverage)

    if args.visualize:
        ex.start()

    i = 0
    for best, performance in particle_swarm_optimize(fitness, 3, bounds,
        args.size, args.omega, args.phip, args.phig, args.it, args.af,
        topology_type=args.topology, constraint_type=args.constraint):
        print('Global best for iteration %d: %s @ %f' % (i, best, performance))
        if args.visualize:
            modify_camera(ex.model, args.cameras[0], lut, best[0], best[1],
                best[2])
            ex.model[args.cameras[0]].update_visualization()
        i += 1
