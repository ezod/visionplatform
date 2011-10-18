"""\
Camera selection simulation.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

try:
    import cPickle as pickle
except ImportError:
    import pickle

import numpy
from scipy.interpolate import interp1d
from optparse import OptionParser
import os.path

import adolphus as A


def interpolate_points(points):
    split = [numpy.array([p[i] for p in points]) for i in range(3)]
    tm = [float(i) for i in range(len(points))]
    f = [interp1d(tm, p, kind='cubic') for p in split]
    return lambda t: A.Point([f[i](t) for i in range(3)])


def create_error_model(model, terror, rerror):
    emodel = A.Model(task_params=model._task_params)
    for camera in set(model.cameras):
        emodel[camera] = A.Camera(camera, model[camera]._params,
            pose=model[camera]._pose, mount=model[camera].mount)
        emodel[camera].set_absolute_pose(\
            A.random_pose_error(model[camera].pose, terror, rerror))
    for sceneobj in model.keys():
        if not sceneobj in model.cameras:
            emodel[sceneobj] = model[sceneobj]
    return emodel


def best_view(model, relevance, ocular=1, current=None, threshold=0,
              candidates=None):
    if candidates:
        scores = dict.fromkeys(candidates)
    else:
        scores = dict.fromkeys(model.views(ocular=ocular))
    for view in scores:
        scores[view] = model.performance(relevance, subset=view)
    if current:
        scores[current] += threshold
    best = sorted(scores.keys(), key=scores.__getitem__)[-1]
    return best, scores[best] - (best == current and threshold or 0)


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-c', '--conf', dest='conf', default=None,
        help='custom configuration file to load')
    parser.add_option('-t', '--threshold', dest='threshold', action='store',
        type='float', default=0.0, help='hysteresis threshold')
    parser.add_option('-j', '--jitter', dest='jitter', action='store',
        type='int', default=0, help='jitter threshold in frames')
    parser.add_option('-v', '--visualize', dest='visualize', default=False,
        action='store_true', help='enable visualization')
    parser.add_option('-z', '--zoom', dest='zoom', default=False,
        action='store_true', help='disable camera view and use visual zoom')
    parser.add_option('-g', '--graph', dest='graph', action='store',
        default=None, help='pickle of vision graph')
    parser.add_option('-C', '--camera-error', dest='cerror', action='store',
        nargs=2, type='float', default=None, help='camera calibration error')
    parser.add_option('-T', '--target-error', dest='terror', action='store',
        nargs=2, type='float', default=None, help='target pose error')
    opts, args = parser.parse_args()
    points = []
    for line in open(args[1], 'r'):
        points.append(A.Point([float(s) for s in line.rstrip().split(',')]))
    f = interpolate_points(points)
    experiment = A.Experiment(zoom=opts.zoom)
    experiment.add_display()
    experiment.execute('loadmodel %s' % args[0])
    experiment.execute('loadconfig %s' % opts.conf)
    if opts.graph and os.path.exists(opts.graph):
        vision_graph = pickle.load(open(opts.graph, 'r'))
    else:
        try:
            vision_graph = experiment.model.coverage_hypergraph(\
                experiment.relevance_models[args[4]], K=2)
            pickle.dump(vision_graph, open(opts.graph, 'w'))
        except IndexError:
            vision_graph = None
    if opts.cerror:
        emodel = create_error_model(experiment.model, opts.cerror[0],
            opts.cerror[1])
    else:
        emodel = None
    if opts.terror:
        etarget = A.RelevanceModel(experiment.relevance_models[args[3]].original)
    else:
        etarget = None
    if opts.visualize:
        experiment.start()
    best = None
    score = 0.0
    current_frames = 0
    performance = 0.0
    if opts.visualize:
        experiment.event.wait()
    for t in range(1, 100 * (len(points) - 1)):
        if experiment.exit:
            break
        normal = (f(t / 100.0) - f((t - 1) / 100.0)).normal
        angle = A.Point((0, -1, 0)).angle(normal)
        axis = A.Point((0, -1, 0)) ** normal
        R = A.Rotation.from_axis_angle(angle, axis)
        experiment.model[args[2]].set_absolute_pose(\
            A.Pose(T=f(t / 100.0), R=R))
        if opts.visualize:
            experiment.model[args[2]].update_visualization()
        current = best
        if etarget:
            etarget.set_absolute_pose(A.random_pose_error(experiment.\
                relevance_models[args[3]].pose, opts.terror[0], opts.terror[1]))
        best, score = best_view(emodel or experiment.model, etarget or \
            experiment.relevance_models[args[3]], current=(current and \
            frozenset([current]) or None), threshold=opts.threshold,
            candidates=((vision_graph and score) and [frozenset(c) for c in \
            vision_graph.neighbors(current) | set([current])] or None))
        if emodel or etarget:
            score = experiment.model.performance(\
                experiment.relevance_models[args[3]], subset=best)
        best = set(best).pop()
        current_frames += 1
        if current_frames > opts.jitter:
            performance += score
        if current != best:
            if opts.visualize:
                experiment.execute('select %s' % best)
                experiment.altdisplays[0].camera_view(experiment.model[best])
                try:
                    experiment.execute('fov %s' % current)
                except:
                    pass
                experiment.execute('fov %s' % best)
            current_frames = 0
    print('Performance (j = %d, t = %g, C = %s, T = %s): %f' % (opts.jitter,
        opts.threshold, opts.cerror, opts.terror, (100 * performance / t)))
