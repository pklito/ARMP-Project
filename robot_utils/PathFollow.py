# This python code is meant to follow a path.
# Input: a list of (x,y) coordinates.
# Stream: updates of robot position or returns NaN if not updated.

# womp womp i use numpy :/
import numpy as np
from constants import *

def getEdgeProjection(config, edge):
    p1 = np.asarray(edge[0])
    p2 = np.asarray(edge[1])
    point = np.asarray(config)

    edge_vector = p2 - p1
    edge_length = np.linalg.norm(edge_vector)

    point_vector = point - p1

    t_distance = edge_vector.dot(point_vector)
    t = t_distance / edge_length

    projection = None
    if(t < 0):
        projection = p1
    elif (t>1):
        projection = p2
    else:
        projection = t*edge_vector + p1

    return projection, t, t_distance

class PathFollowStrict:
    path = None
    path_edges = None
    PATH_LOOKAHEAD = 0
    EDGE_CUTOFF = 0
    current_edge = 0
    def __init__(self, path, path_lookahead = TASK_PATH_LOOKAHEAD, EDGE_CUTOFF = TASK_EDGE_CUTOFF):
        self.path = path
        self.path_edges = zip(self.path, self.path[1:])
        self.PATH_LOOKAHEAD = path_lookahead

    def getLookaheadConfig(self, config, lookahead_distance = PATH_LOOKAHEAD):
        if self.current_edge >= len(self.path):
            return self.path[-1]
        
        point = np.asarray(config)
        edge = self.path_edges[self.current_edge]

        projection, t, t_distance = getEdgeProjection(config, edge)
        distance = np.linalg.norm(projection-point)

        if distance > lookahead_distance:
            return point + lookahead_distance*((projection-point)/distance)
        else:
            edge_vector = np.asarray(edge[1]) - edge[0]
            edge_length = np.linalg.norm(edge_vector)
            edge_unit_vector = edge_vector/edge_length
            return edge[0] + (edge_unit_vector * max(t_distance + lookahead_distance - distance, edge_length))

    def updateCurrentEdge(self, config, cutoff_radius = EDGE_CUTOFF):
        if current_edge >= len(self.path):
            return
        if np.linalg.norm(np.asarray(config) - self.path[self.current_edge + 1]) < cutoff_radius:
            current_edge += 1



# Stores function follow()
# this  is its own class so i can store
class PathFollow:
    path = None
    PATH_LOOKAHEAD = 0
    EDGE_CUTOFF = 0
    current_edge = 0
    def __init__(self, path, path_lookahead = TASK_PATH_LOOKAHEAD, EDGE_CUTOFF = TASK_EDGE_CUTOFF):
        self.path = path
        self.PATH_LOOKAHEAD = path_lookahead

    # Return:
    # best edge index (bounded)
    # best distance
    # projection from point to that edge and it's t value
    def getClosestEdge(self, config):
        best_distance = 999999
        best_index = -1
        # iterate through the path in segments (p0, p1, 0), (p1,p2,1), ...., (pn-1, pn, n-1)
        for i, j, index in zip(self.path[:self.current_edge+2], self.path[1: self.current_edge + 2], range(len(self.path)-1)):
            projection, t, t_distance = getEdgeProjection(config, (i,j))
            distance = np.linalg.norm(np.asarray(config)-projection)

            if( np.linalg.norm(np.asarray(i)-j) - t_distance < self.EDGE_CUTOFF and index >= self.current_edge):
                self.current_edge += 1


            # If better than the current best, update it
            if distance <= best_distance:
                best_distance = distance
                best_index = index
                best_projection = (projection, t)
        #print(best_distance,best_index,self.current_edge,[(a,b,c) for a,b,c in zip(self.path[:self.current_edge+2], self.path[1: self.current_edge + 2], range(len(self.path)-1))],self.path[:self.current_edge+2])
        return best_index, best_distance, best_projection


    def getLookaheadEdge(self, config):
        # Get current lookahead number
        lookahead_left = self.PATH_LOOKAHEAD
        # Get projection of config onto the path
        current_path_index, distance, projection = self.getClosestEdge(config)
        # Closest edge and the ones after it
        remaining_path_edges = [(p1,p2) for p1, p2 in zip(self.path[current_path_index:], self.path[current_path_index+1:])]

        # Extract the point from the (point, float[0-1]) pair
        point = np.array(projection[0])
        while(lookahead_left > 0 and len(remaining_path_edges)>0):
            curr_edge_remainder = np.linalg.norm(np.array(point)-remaining_path_edges[0][1])
            if(curr_edge_remainder <= lookahead_left):
                lookahead_left -= curr_edge_remainder
                point = np.array(remaining_path_edges[0][1])
                remaining_path_edges = remaining_path_edges[1:]
            else:
                edge = np.array(remaining_path_edges[0][1]) - remaining_path_edges[0][0]
                #point = [float(point[0]), float(point[1])]
                point += (lookahead_left*(edge))/np.linalg.norm(edge)
                lookahead_left = 0

        return point, current_path_index

