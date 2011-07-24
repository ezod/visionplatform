"""\
Camera selection application.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import socket
#import serial
from optparse import OptionParser

from adolphus.yamlparser import YAMLParser

from halconparser import parse_pose_string


class CameraSelector(object):
    """\
    Camera selector class.
    """
    def __init__(self, model_file):
        """\
        Constructor.

        @param model_file: The YAML file for the model.
        @type model_file: C{str}
        """
        self.model, relevance = YAMLParser(model_file).experiment
        try:
            self.target = relevance['target']
            #self.robot = self.model.scene['robot']
            self.vision_graph = \
                self.model.coverage_hypergraph(relevance['cell'], K=[2])
        except KeyError:
            raise KeyError('incorrect experiment format')

    def best_view(self, current, target_pose, robot_config, threshold=0.0):
        """\
        TODO

        @param current: The current active camera ID.
        @type current: C{str}
        @param target_pose: The current pose of the target.
        @type target_pose: L{Pose}
        @param robot_config: The current state of the robotic arm.
        @type robot_config: C{list} of C{float}
        @param threshold: Hysteresis threshold.
        @type threshold: C{float}
        @return: The next camera to make active.
        @rtype: C{str}
        """
        self.target.set_relative_pose(target_pose)
        self.target.mount = self.model[current]
        print('Received pose %s from camera %s.' % (self.target.pose, current))
        #self.robot.config = robot_config
        candidates = dict.fromkeys(self.vision_graph.neighbors(current) | \
            set([current]))
        for camera in candidates:
            candidates[camera] = self.model[camera].performance(self.target)
        print(candidates)
        candidates[current] += threshold
        return sorted(candidates.keys(), key=candidates.__getitem__)[-1]


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
    pose = parse_pose_string(','.join(hstring))
    return camera, pose


if __name__ == '__main__':
    parser = OptionParser()
    parser.add_option('-p', '--port', dest='port', action='store',
        type='int', default=5678)
    parser.add_option('-c', '--comport', dest='comport', action='store',
        default='COM1')
    parser.add_option('-t', '--threshold', dest='threshold', action='store',
        type='float', default=0.0)
    opts, args = parser.parse_args()
    selector = CameraSelector(args[0])
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(('localhost', opts.port))
    sock.listen(20)
    #port = serial.Serial(port=opts.comport, baudrate=19200)
    print('Ready.')
    channel, details = sock.accept()
    try:
        while True:
            hstring = ''
            while True:
                hstring += channel.recv(65536)
                try:
                    camera, pose = parse_from_halcon(hstring)
                    break
                except ValueError:
                    continue
            # TODO: get robot config
            #port.write('PRNS\r')
            config = []
            bestview = selector.best_view(camera, pose, config, opts.threshold)
            print('Best view is camera %s.' % bestview)
            channel.sendall(bestview)
    finally:
        #port.close()
        channel.close()
        sock.close()
