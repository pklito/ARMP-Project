# This python code is meant to follow a path.
# Input: a list of (x,y) coordinates.
# Stream: updates of robot position or returns NaN if not updated.

# womp womp i use numpy :/
from math import fmod, modf
import numpy as np
from constants import *
def clamp(val, v_min, v_max):
    return min(v_max, max(v_min,val))

def getEdgeProjection(config, edge):
    p1 = np.asarray(edge[0])
    p2 = np.asarray(edge[1])
    point = np.asarray(config)

    # Get a vector of the given path edge
    edge_vector = p2 - p1
    edge_length = np.linalg.norm(edge_vector)
    if edge_length <= 0.001:
        return p2, 1, edge_length

    # Vector from path start to current point
    point_vector = point - p1

    # T is the fraction along the path the projection is on.
    t_distance = edge_vector.dot(point_vector)
    t = t_distance / edge_length

    projection = None
    if(t < 0):
        projection = p1
    elif (t>1):
        projection = p2
    else:
        projection = t*edge_vector + p1

    return projection, clamp(t, 0, 1), clamp(t_distance, 0, edge_length)

def getClampedLookahead(point, target, lookahead_distance):
    target_vector = np.asarray(target) - point
    target_distance = np.linalg.norm(target_vector)
    if target_distance <= 0.001:
        return target

    return point + lookahead_distance * target_vector / target_distance
class PathFollowStrict:
    path = None     #List of the path configurations
    path_edges = None       #List of edges I.E. pairs of points
    PATH_LOOKAHEAD = 0.3
    EDGE_CUTOFF = 0.3
    current_edge = 0
    def __init__(self, path, path_lookahead = TASK_PATH_LOOKAHEAD, EDGE_CUTOFF = TASK_EDGE_CUTOFF):
        self.path = path
        self.path_edges = [edge for edge in zip(self.path, self.path[1:])]
        self.PATH_LOOKAHEAD = path_lookahead
        self.EDGE_CUTOFF = EDGE_CUTOFF

    def getLookaheadConfig(self, config, lookahead_distance = None):
        if lookahead_distance == None:
            lookahead_distance = self.PATH_LOOKAHEAD

        if self.current_edge >= len(self.path)-1:
            return np.asarray(self.path[-1])

        point = np.asarray(config)
        edge = self.path_edges[self.current_edge]

        proj, t, t_dist = getEdgeProjection(config, edge)
        edge_length = np.linalg.norm(edge[1] - np.array(edge[0]))

        target = None
        if edge_length <= 0.001:
            target = edge[1]
        else:
            target = self.getPointFromT(self.current_edge, clamp(t + lookahead_distance / edge_length, 0, 1))

        return getClampedLookahead(point, target, lookahead_distance)

    def updateCurrentEdge(self, config, cutoff_radius = None):
        if cutoff_radius == None:
            cutoff_radius = self.EDGE_CUTOFF
        
        if self.current_edge >= len(self.path)-1:
            return
        if np.linalg.norm(np.asarray(config) - self.path[self.current_edge + 1]) < cutoff_radius:
            self.current_edge += 1

    def getPointFromT(self,edge_num, t):
        if edge_num >= len(self.path_edges):
            return np.array(self.path_edges[-1][1])
        p1 = np.asarray(self.path_edges[edge_num][0])
        p2 = np.asarray(self.path_edges[edge_num][1])

        return (1-t) * p1 + t * p2

