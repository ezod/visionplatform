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
import sys

import adolphus as A


def interpolate_points(points):
    split = [numpy.array([p[i] for p in points]) for i in range(3)]
    tm = [float(i) for i in range(len(points))]
    f = [interp1d(tm, p, kind='cubic') for p in split]
    return lambda t: A.Point([f[i](t) for i in range(3)])


def create_error_model(model, vectors=None, terror=0.0, rerror=0.0):
    emodel = A.Model(task_params=model._task_params)
    convert = {}
    if not vectors:
        vectors = {}
    for camera in model.cameras:
        emodel[camera] = A.Camera(camera, model[camera]._params,
            pose=model[camera]._pose, mount=model[camera].mount)
        if not camera in vectors:
            vectors[camera] = (A.random_unit_vector(), A.random_unit_vector())
        emodel[camera].set_absolute_pose(\
            A.pose_error(model[camera].pose, vectors[camera][0], terror,
                vectors[camera][1], rerror))
        convert[camera] = -model[camera].pose + emodel[camera].pose
    for sceneobj in model.keys():
        if not sceneobj in model.cameras:
            emodel[sceneobj] = model[sceneobj]
    return emodel, convert, vectors


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-c', '--conf', dest='conf', default=None,
        help='custom configuration file to load')
    parser.add_option('-i', '--interpolate', dest='interpolate',
        action='store', type='int', default=100, help='interpolation pitch')
    parser.add_option('-t', '--threshold', dest='threshold', action='store',
        type='float', default=0.0, help='hysteresis threshold')
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
    parser.add_option('-V', '--error-vectors', dest='vectors', action='store',
        default=None, help='pickle of error model vectors')
    parser.add_option('-p', '--pose-tracking', dest='posetrack', default=False,
        action='store_true', help='target poses are obtained from cameras')
    opts, args = parser.parse_args()
    modelfile, pathfile, targetobj, targetrm = args[:4]
    # load path waypoints
    points = []
    for line in open(pathfile, 'r'):
        points.append(A.Point([float(s) for s in line.rstrip().split(',')]))
    f = interpolate_points(points)
    try:
        optperf = float(open(pathfile + '.%d.opt' % opts.interpolate,
            'r').readline())
        optcached = True
    except IOError:
        optperf = 0.0
        optcached = False
    # load model
    experiment = A.Experiment(zoom=opts.zoom)
    experiment.add_display()
    experiment.execute('loadmodel %s' % modelfile)
    experiment.execute('loadconfig %s' % opts.conf)
    # compute vision graph
    if opts.graph and os.path.exists(opts.graph):
        vision_graph = pickle.load(open(opts.graph, 'r'))
    else:
        try:
            vision_graph = experiment.model.coverage_hypergraph(\
                experiment.relevance_models[args[4]], K=2)
            pickle.dump(vision_graph, open(opts.graph, 'w'))
        except IndexError:
            vision_graph = None
    # build camera error model
    if opts.cerror:
        if opts.vectors:
            try:
                vectors = pickle.load(open(opts.vectors, 'r'))
            except IOError:
                vectors = None
        emodel, convert, vectors = create_error_model(experiment.model,
            vectors=vectors, terror=opts.cerror[0], rerror=opts.cerror[1])
        if opts.vectors:
            pickle.dump(vectors, open(opts.vectors, 'w'))
    else:
        emodel = None
    # set up target error model
    if opts.terror or (opts.posetrack and emodel):
        etarget = A.RelevanceModel(experiment.relevance_models[targetrm].original)
    else:
        etarget = None
    # start
    if opts.visualize:
        experiment.start()
    best = None
    score = 0.0
    current_frames = 0
    perf = 0.0
    perf_delta = 0.0
    if opts.visualize:
        experiment.event.wait()
    header = 't = %g, C = %s, T = %s' % \
        (opts.threshold, opts.cerror, opts.terror)
    print('#' * (len(header) + 2))
    print('# ' + header)
    print('#' * (len(header) + 2))
    for t in range(1, opts.interpolate * (len(points) - 1)):
        if experiment.exit:
            break
        current_frames += 1
        normal = (f(t / float(opts.interpolate)) - \
            f((t - 1) / float(opts.interpolate))).normal
        angle = A.Point((0, -1, 0)).angle(normal)
        axis = A.Point((0, -1, 0)) ** normal
        R = A.Rotation.from_axis_angle(angle, axis)
        experiment.model[targetobj].set_absolute_pose(\
            A.Pose(T=f(t / float(opts.interpolate)), R=R))
        if opts.visualize:
            experiment.model[targetobj].update_visualization()
        current = best
        if opts.terror:
            etarget.set_absolute_pose(A.pose_error(experiment.relevance_models[\
                targetrm].pose, A.random_unit_vector(), opts.terror[0],
                A.random_unit_vector(), opts.terror[1]))
        elif opts.posetrack and emodel:
            etarget.set_absolute_pose(experiment.relevance_models[targetrm].pose)
        if current and score and opts.posetrack and emodel:
            etarget.set_absolute_pose(etarget.pose + convert[current])
        best, score = (emodel or experiment.model).best_view(etarget or \
            experiment.relevance_models[targetrm], current=(current and \
            frozenset([current]) or None), threshold=opts.threshold,
            candidates=((vision_graph and score) and [frozenset(c) for c in \
            vision_graph.neighbors(current) | set([current])] or None))
        if emodel or etarget:
            score = experiment.model.performance(\
                experiment.relevance_models[targetrm], subset=best)
        if not optcached:
            optperf += experiment.model.best_view(\
                experiment.relevance_models[targetrm])[1]
        best = set(best).pop()
        if current != best:
            if opts.visualize:
                experiment.execute('select %s' % best)
                experiment.altdisplays[0].camera_view(experiment.model[best])
                try:
                    experiment.execute('fov %s' % current)
                except:
                    pass
                experiment.execute('fov %s' % best)
            if current:
                print('%s,%d,%f' % (current, current_frames, perf_delta))
            current_frames = 0
            perf_delta = 0.0
        perf_delta += score
    print('%s,%d,%f' % (current, current_frames, perf_delta))
    print('OPTIMAL=%f' % optperf)
    sys.stdout.flush()
    if not optcached:
        open(pathfile + '.%d.opt' % opts.interpolate, 'w').write('%f' % optperf)
