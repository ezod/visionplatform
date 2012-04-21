"""\
Module for composing multiple relative camera calibrations.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from adolphus.geometry import Pose
from hypergraph.core import Graph, Edge
from hypergraph.path import dijkstra
from hypergraph.connectivity import connected


def absolute_poses(poses, reference):
    """\
    Provided a set of relative poses, return a set of poses of all objects with
    respect to the specified reference frame object.

    The relative poses should be supplied as a dict, with tuple keys, where the
    key indicates the source and destination frame respectively, and the value
    contains the pose of the source in the coordinate system of the destination
    as well as the mean error, in the form {(<str>, <str>): (<Pose>, <float>)}.
    
    @param poses: The set of relative poses.
    @type poses: C{dict} of L{Pose}, C{float}
    @param reference: The final reference frame.
    @type reference: C{str}
    @return: C{dict} of L{Pose}, C{float}
    """
    cameras = set([key[0] for key in poses])
    frames = set([key[i] for key in poses for i in range(2)])
    try:
        assert(reference in frames)
    except AssertionError:
        raise ValueError('reference frame %s does not exist' % reference)
    cg = Graph(vertices=frames)
    for pair in poses:
        cg.add_edge(Edge(pair), weight=poses[pair][1])
    try:
        assert(connected(cg))
    except AssertionError:
        raise AssertionError('calibration graph is not connected')
    prev = dijkstra(cg, reference)
    rposes = {}
    for camera in cameras:
        pose, error = Pose(), 0.0
        frame = camera
        while prev[frame]:
            try:
                pose += poses[(frame, prev[frame])][0]
                error += poses[(frame, prev[frame])][1]
            except KeyError:
                pose -= poses[(prev[frame], frame)][0]
                error += poses[(prev[frame], frame)][1]
            frame = prev[frame]
        rposes[camera] = (pose, error)
    return rposes
