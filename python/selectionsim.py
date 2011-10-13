"""\
Camera selection simulation.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import numpy
from scipy.interpolate import interp1d
from optparse import OptionParser

from adolphus import Pose, Point, Experiment


def interpolate_points(points):
    split = [numpy.array([p[i] for p in points]) for i in range(3)]
    tm = [float(i) for i in range(len(points))]
    f = [interp1d(tm, p, kind='cubic') for p in split]
    return lambda t: Point([f[i](t) for i in range(3)])


def best_view(model, relevance, ocular=1, current=None, threshold=0,
              candidates=None, time=None):
    if candidates:
        scores = dict.fromkeys(candidates)
    else:
        scores = dict.fromkeys(model.views(ocular=ocular))
    for view in scores:
        scores[view] = model.performance(relevance, subset=view)
    if current:
        scores[current] += threshold
    if time is not None:
        print('%d,' % time + ('%f,' * len(scores))[:-1] % tuple([scores[view] \
            for view in [frozenset([chr(i)]) for i in range(65,88)]]))
    return sorted(scores.keys(), key=scores.__getitem__)[-1]


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-c', '--conf', dest='conf', default=None,
        help='custom configuration file to load')
    parser.add_option('-t', '--threshold', dest='threshold', action='store',
        type='float', default=0.0, help='hysteresis threshold')
    parser.add_option('-z', '--zoom', dest='zoom', default=False,
        action='store_true', help='disable camera view and use visual zoom')
    parser.add_option('-p', '--printvals', dest='printvals', default=False,
        action='store_true', help='print coverage values')
    opts, args = parser.parse_args()
    points = []
    for line in open(args[1], 'r'):
        points.append(Point([float(s) for s in line.rstrip().split(',')]))
    f = interpolate_points(points)
    experiment = Experiment(zoom=opts.zoom)
    experiment.execute('loadmodel %s' % args[0])
    experiment.execute('loadconfig %s' % opts.conf)
    experiment.start()
    if opts.printvals:
        print('Time,' + ('%s,' * 23)[:-1] % tuple([chr(i) for i in range(65,88)]))
    best = experiment.model.views().pop()
    for t in range(100 * (len(points) - 1)):
        experiment.model[args[2]].set_absolute_pose(Pose(T=f(t / 100.0)))
        experiment.model[args[2]].update_visualization()
        best = best_view(experiment.model, experiment.relevance_models[args[3]],
            current=best, threshold=opts.threshold,
            time=opts.printvals and t or None)
        experiment.execute('select %s' % set(best).pop())
