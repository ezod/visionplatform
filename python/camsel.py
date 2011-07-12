"""\
Camera selection application.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import sys
import socket
#import serial
from math import pi

from adolphus.yamlparser import YAMLParser
from adolphus.geometry import Pose, Point, Rotation


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

    def best_view(self, current, target_pose, robot_config):
        """\
        TODO

        @param current: The current active camera ID.
        @type current: C{str}
        @param target_pose: The current pose of the target.
        @type target_pose: L{Pose}
        @param robot_config: The current state of the robotic arm.
        @type robot_config: C{list} of C{float}
        @return: The next camera to make active.
        @rtype: C{str}
        """
        self.target.pose = target_pose
        self.target.mount = self.model[current]
        print('Received pose %s from camera %s.' % (self.target.pose, current))
        #self.robot.config = robot_config
        candidates = dict.fromkeys(self.vision_graph.neighbors(current) | \
            set([current]))
        for camera in candidates:
            candidates[camera] = self.model[camera].performance(self.target)
        print(candidates)
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
    T = Point([1e3 * float(s) for s in hstring[0:3]])
    R = Rotation.from_euler('zyx', [(360.0 - float(s)) * pi / 180.0 \
        for s in hstring[3:6]])
    return camera, Pose(T, R)


if __name__ == '__main__':
    selector = CameraSelector(sys.argv[1])
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        port = int(sys.argv[2])
    except IndexError:
        port = 5678
    sock.bind(('localhost', port))
    sock.listen(20)
    channel, details = sock.accept()
    #port = serial.Serial(port=sys.argv[1], baudrate=19200)
    print('Ready.')
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
            bestview = selector.best_view(camera, pose, config)
            print('Best view is camera %s.' % bestview)
            channel.sendall(bestview)
    finally:
        #port.close()
        channel.close()
        sock.close()
