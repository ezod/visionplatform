"""\
Camera selection application.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

try:
    import cPickle as pickle
except ImportError:
    import pickle

import socket
import serial
import sys
import os.path
from optparse import OptionParser
from math import pi

import adolphus as A

from halconparser import parse_pose_string
from melfaparser import parse_positions
from robotjpos import query_position


def parse_from_halcon(hstring):
    """\
    Convert tuple data in string format from HALCON into the camera ID and
    target pose.

    @param hstring: The string data from HALCON.
    @type hstring: C{str}
    @return: Camera ID and target pose.
    @rtype: C{str}, L{Pose}
    @raise ValueError: String is malformed.
    """
    hstring = hstring.split(',')
    camera = hstring.pop(0)
    pnum = int(hstring.pop(0))
    try:
        pose = A.Pose(R=A.Rotation.from_axis_angle(pi, A.Point((1, 0, 0)))) + parse_pose_string(','.join(hstring))
    except:
        pose = None
    return camera, pnum, pose


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-p', '--port', dest='port', action='store',
        type='int', default=5678, help='network port to listen on')
    parser.add_option('-s', '--serialport', dest='serialport', action='store',
        default=None, help='serial port for robot')
    parser.add_option('-t', '--threshold', dest='threshold', action='store',
        type='float', default=0.0, help='hysteresis threshold')
    parser.add_option('-J', '--jitter', dest='jitter', action='store',
        type='int', default=0, help='jitter threshold in frames')
    parser.add_option('-c', '--conf', dest='conf', default=None,
        help='custom configuration file to load')
    parser.add_option('-z', '--zoom', dest='zoom', default=False,
        action='store_true', help='disable camera view and use visual zoom')
    parser.add_option('-g', '--graph', dest='graph', action='store',
        default=None, help='pickle of vision graph')
    parser.add_option('-r', '--robotpath', dest='robotpath', action='store',
        default=None, help='robot positions file')
    parser.add_option('-w', '--wtolerance', dest='w', action='store',
        type='float', default=0.1, help='jitter tolerance')
    opts, args = parser.parse_args()
    modelfile, targetobj, targetrm = args[:3]
    # load model
    experiment = A.Experiment(zoom=opts.zoom)
    #experiment.add_display()
    experiment.execute('loadmodel %s' % modelfile)
    experiment.execute('loadconfig %s' % opts.conf)
    # compute vision graph
    if opts.graph and os.path.exists(opts.graph):
        vision_graph = pickle.load(open(opts.graph, 'r'))
    else:
        try:
            vision_graph = experiment.model.coverage_hypergraph(\
                experiment.relevance_models[args[3]], K=2)
        except IndexError:
            vision_graph = None
        if opts.graph:
            pickle.dump(vision_graph, open(opts.graph, 'w'))
    # set up network socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('localhost', opts.port))
    sock.listen(20)
    # set up serial communication
    if opts.serialport:
        port = serial.Serial(port=opts.serialport, baudrate=19200)
    else:
        port = None
    # load robot positions
    if opts.robotpath:
        positions = parse_positions(opts.robotpath)
    else:
        positions = None
    # start
    best = None
    score = 0.0
    frames = 0
    current_frames = 0
    Mbest = 0.0
    M = 0.0
    U = 0
    experiment.start()
    channel, details = sock.accept()
    channel.settimeout(0.1)
    try:
        while True:
            frames += 1
            current_frames += 1
            current = best
            hstring = ''
            while not hstring.endswith('#'):
                if experiment.exit:
                    break
                try:
                    hstring += channel.recv(65536)
                except:
                    pass
            camera, pnum, pose = parse_from_halcon(hstring.strip('#'))
            if pose:
                pose = pose + experiment.model[camera].pose
                experiment.model[targetobj].set_absolute_pose(pose)
            elif port:
                experiment.model[targetobj].config = \
                    query_position(port) + [8.0]
            elif positions:
                experiment.model[targetobj].config = \
                    positions['J%d' % pnum] + [8.0]
            experiment.model[targetobj].update_visualization()
            best, score = experiment.model.best_view(\
                experiment.relevance_models[targetrm], threshold=(current_frames < opts.jitter and opts.threshold or 0.00001),
                current=frozenset([camera]),
                candidates=((vision_graph and score) and [frozenset(c) for c \
                in vision_graph.neighbors(current) | set([current])] or None))
            Mbest += experiment.model.best_view(\
                experiment.relevance_models[targetrm])[1]
            best = set(best).pop()
            if current != best:
                experiment.execute('select %s' % best)
                #experiment.altdisplays[0].camera_view(experiment.model[best])
                try:
                    experiment.execute('fov %s' % current)
                except:
                    pass
                experiment.execute('fov %s' % best)
                if current_frames < opts.jitter:
                    U += 1
                current_frames = 0
            M += score
            channel.sendall(best)
            if positions and not 'J%d' % (pnum + 1) in positions:
                break
    finally:
        try:
            print('A: %.4f, M: %.4f, U: %d' % ((opts.w * (M / Mbest)) / (float(U) / float(frames) + opts.w), (M / Mbest), U))
            sys.stdout.flush()
        except ZeroDivisionError:
            pass
        if port:
            port.close()
        channel.close()
        sock.close()
        experiment.execute('exit')
        experiment.join()
