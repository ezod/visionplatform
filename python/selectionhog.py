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
    parser.add_option('-i', '--interpolate', dest='interpolate',
        action='store', type='int', default=100, help='interpolation pitch')
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
    # load model
    experiment = A.Experiment()
    experiment.execute('loadmodel %s' % modelfile)
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
    print('# C = %s, T = %s' % (opts.cerror, opts.terror))
    for t in range(1, opts.interpolate * (len(points) - 1)):
        if experiment.exit:
            break
        normal = (f(t / float(opts.interpolate)) - \
            f((t - 1) / float(opts.interpolate))).normal
        angle = A.Point((0, -1, 0)).angle(normal)
        axis = A.Point((0, -1, 0)) ** normal
        R = A.Rotation.from_axis_angle(angle, axis)
        experiment.model[targetobj].set_absolute_pose(\
            A.Pose(T=f(t / float(opts.interpolate)), R=R))
        if opts.terror:
            etarget.set_absolute_pose(A.pose_error(experiment.relevance_models[\
                targetrm].pose, A.random_unit_vector(), opts.terror[0],
                A.random_unit_vector(), opts.terror[1]))
        elif opts.posetrack and emodel:
            etarget.set_absolute_pose(experiment.relevance_models[targetrm].pose)
        scores = dict.fromkeys((emodel or experiment.model).views())
        for view in scores:
            if opts.posetrack and emodel:
                etarget.set_absolute_pose(etarget.pose + convert[set(view).pop()])
            scores[view] = (emodel or experiment.model).performance(\
                etarget or experiment.relevance_models[targetrm], subset=view)
        print(','.join(['%f' % scores[view] for view in sorted(scores.keys())]))
    sys.stdout.flush()
