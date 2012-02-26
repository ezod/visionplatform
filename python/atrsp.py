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

import csv
from bisect import bisect
from math import pi, sin, cos
from optparse import OptionParser

from adolphus.geometry import Point, Rotation, Pose
from adolphus.yamlparser import YAMLParser

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
    parser = OptionParser()
    parser.add_option('-f', '--fnumber', dest='fnumber', action='store',
        type='float', default=1.0)
    parser.add_option('-s', '--size', dest='size', action='store',
        type='int', default=1)
    parser.add_option('-o', '--omega', dest='omega', action='store',
        type='float', default=0.0)
    parser.add_option('-p', '--phip', dest='phip', action='store',
        type='float', default=0.0)
    parser.add_option('-g', '--phig', dest='phig', action='store',
        type='float', default=0.0)
    parser.add_option('-i', '--iterations', dest='it', action='store',
        type='int', default=1000)
    parser.add_option('-a', '--accept', dest='af', action='store',
        type='float', default=1.0)
    parser.add_option('-t', '--topology', dest='topology', action='store')
    parser.add_option('-c', '--constraint', dest='constraint', action='store')
    opts, args = parser.parse_args()

    lutfile, modelfile, task, camera = args[:4]

    lut = LensLUT(lutfile, opts.fnumber)
    model, tasks = YAMLParser(modelfile).experiment

    # compute bounds on h
    zmin, zmax = float('inf'), -float('inf')
    for point in tasks[task].mapped:
        lp = model[model.active_laser].triangle.intersection(point,
            point + Point((0, 1, 0)), limit=False)
        if lp:
            if lp.z < zmin:
                zmin = lp.z
            if lp.z > zmax:
                zmax = lp.z

    bounds = ((zmin, zmax), lut.bounds,
              (0, tasks[task].getparam('angle_max_acceptable')))

    def fitness(particle):
        h, d, beta = particle
        if d < lut.bounds[0] or d > lut.bounds[1]:
            return -float('inf')
        modify_camera(model, camera, lut, h, d, beta)
        coverage = model.range_coverage_linear(tasks[task])
        return model.performance(tasks[task], coverage=coverage)

    best, performance = particle_swarm_optimize(fitness, 3, bounds, opts.size,
        opts.omega, opts.phip, opts.phig, opts.it, opts.af,
        topology_type=opts.topology, constraint_type=opts.constraint)
