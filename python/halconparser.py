"""\
HALCON parameter parser module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import sys
import os.path
from glob import glob
from math import pi

from adolphus.geometry import Pose, Point, Rotation

from composition import absolute_poses


def parse_internal(filename):
    """\
    Parse internal calibration parameters from a HALCON internal calibration
    output file.

    @param filename: The calibration output file to load from.
    @type filename: C{str}
    @return: Tuple containing the internal parameters f, s, o, and dim.
    @rtype: C{tuple}
    """
    f, k, s, o, dim = None, None, [None, None], [None, None], [None, None]
    with open(filename, 'r') as icfile:
        for line in icfile:
            line = line.rstrip()
            if not line or line.startswith('#') or line.startswith('\t') \
            or line.startswith('ParGroup'):
                continue
            value = float(line.split(':')[2][1:-1])
            if line.startswith('Focus'):
                f = 1e3 * value
            elif line.startswith('Kappa'):
                k = value
            elif line.startswith('Sx'):
                s[0] = 1e3 * value
            elif line.startswith('Sy'):
                s[1] = 1e3 * value
            elif line.startswith('Cx'):
                o[0] = value
            elif line.startswith('Cy'):
                o[1] = value
            elif line.startswith('ImageWidth'):
                dim[0] = int(value)
            elif line.startswith('ImageHeight'):
                dim[1] = int(value)
    return f, k, s, o, dim


def parse_external(filename):
    """\
    Parse external calibration parameters from a HALCON external calibration
    output file.
    
    @param filename: The calibration output file to load from.
    @type filename: C{str}
    """
    with open(filename, 'r') as ecfile:
        for line in ecfile:
            line = line.rstrip()
            if line.startswith('t'):
                T = Point([1e3 * float(value) for value in line.split(' ')[1:]])
            elif line.startswith('r'):
                R = Rotation.from_euler('zyx', [(360. - float(value)) * pi \
                    / 180.0 for value in line.split(' ')[1:]])
    return Pose(T, R)


def parse_pose_string(pose_string):
    """\
    Parse a pose string from HALCON (comma-delimited concantenation of a
    homogeneous 3D transformation matrix).
    
    @param pose_string: The pose string from HALCON.
    @type pose_string: C{str}
    @return: The parsed pose.
    @rtype: L{Pose}
    """
    pose = pose_string.split(',')
    T = Point([1e3 * float(s) for s in [pose[4 * i + 3] for i in range(3)]])
    rot = [pose[4 * i:4 * i + 3] for i in range(3)]
    for i in range(3):
        for j in range(3):
            rot[i][j] = float(rot[i][j])
    R = Rotation.from_rotation_matrix(rot)
    return Pose(T, R)


def parse_calibration_files(directory):
    """\
    Parse a set of calibration pose files output by HALCON into a dict suitable
    for multi-camera calibration.

    @param directory: The directory to scan for files.
    @type directory: C{str}
    @return: A dict of relative poses.
    @rtype: C{dict} of L{Pose}, C{float}
    """
    calfiles = glob(os.path.join(directory, 'pose_*_*'))
    poses = {}
    for cf in calfiles:
        with open(cf, 'r') as data:
            pose = parse_pose_string(data.readline().rstrip())
            error = float(data.readline().rstrip())
            poses[tuple(os.path.basename(cf).split('_')[1:])] = (pose, error)
    return poses


if __name__ == '__main__':
    try:
        rp = absolute_poses(parse_calibration_files(sys.argv[2]), sys.argv[3])
        print('    cameras:')
        for camera in rp:
            f, s, o, dim = parse_internal(os.path.join(sys.argv[2], 'internal_%s.cal' % camera))
            print('        - name:         %s' % camera)
            print('          sprites:      [cameras/prosilicaec1350.yaml, lenses/computarm3z1228cmp.yaml]')
            print('          A:            %f' % float(sys.argv[4]))
            print('          f:            %f' % f)
            if abs(s[0] - s[1]) < 1e-8:
                print('          s:            %f' % s[0])
            else:
                print('          s:            [%f, %f]' % tuple(s))
            print('          o:            [%f, %f]' % tuple(o))
            print('          dim:          [%d, %d]' % tuple(dim))
            print('          zS:           %f' % float(sys.argv[5]))
            print('          pose:')
            print('              T:            %s' % list(rp[camera][0].T))
            print('              R:            %s' % [rp[camera][0].R.Q.a, list(rp[camera][0].R.Q.v)])
            print('              Rformat:      quaternion')
    except IndexError:
        print('       %s <directory> <reference> <A> <zS>' % sys.argv[0])
