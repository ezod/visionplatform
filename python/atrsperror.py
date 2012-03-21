import argparse
from random import gauss

try:
    import cPickle as pickle
except ImportError:
    import pickle

import warnings
with warnings.catch_warnings():
    warnings.filterwarnings('ignore', category=UserWarning)
    from adolphus.laser import RangeModel
    from adolphus.geometry import Point, Rotation, Angle, Pose, gaussian_pose_error

from atrsp import load_model, modify_camera


def gaussian_yz_pose_error(pose, tsigma, rsigma):
    T, R = pose.T, pose.R
    T = Point(gauss(T.x, tsigma), gauss(T.y, tsigma), gauss(T.z, tsigma))
    R += Rotation.from_axis_angle(Angle(gauss(0, rsigma)), Point(1, 0, 0))
    return Pose(T=T, R=R)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-C', '--cerror', dest='cerror', type=float,
        nargs=2, default=(0.0, 0.0))
    parser.add_argument('-T', '--terror', dest='terror', type=float,
        nargs=2, default=(0.0, 0.0))
    parser.add_argument('-n', '--number', dest='number', type=int, default=1)
    parser.add_argument('datafile')
    args = parser.parse_args()

    result = pickle.load(open(args.datafile, 'r'))
    ex, lut = load_model(result['args'].modelfile, result['args'].cameras, result['args'].fnumber)

    for c, camera in enumerate(result['args'].cameras):
        modify_camera(ex.model, camera[0], lut[c], *(result['best'][4 * c: 4 * (c + 1)]))

    original_pose = {}
    for obj in [ex.tasks[result['args'].task].mount.name] + [c[0] for c in result['args'].cameras]:
        original_pose[obj] = ex.model[obj].pose
    perf = []
    for i in range(args.number):
        for obj in original_pose:
            if obj in ex.model.cameras:
                ex.model[obj].set_absolute_pose(gaussian_yz_pose_error(original_pose[obj], *args.cerror))
            else:
                ex.model[obj].set_absolute_pose(gaussian_pose_error(original_pose[obj], *args.terror))
        coverage = ex.model.range_coverage(ex.tasks[result['args'].task], RangeModel.LinearTargetTransport)
        perf.append(ex.model.performance(ex.tasks[result['args'].task], coverage=coverage))
    print('Nominal Performance: %g' % result['F'][-1])
    print('Average Performance: %g' % (sum(perf) / float(len(perf))))
