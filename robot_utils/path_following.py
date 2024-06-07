# This python code is meant to follow a path.
# Input: a list of (x,y) coordinates.
# Stream: updates of robot position or returns NaN if not updated.

# womp womp i use numpy :/
import numpy as np
from constants import TASK_PATH_LOOKAHEAD

# Stores function follow()
# this  is its own class so i can store
class PathFollow:
    path = None
    path_lookahead = 0
    current_edge = 0
    def __init__(self, path, path_lookahead = TASK_PATH_LOOKAHEAD):
        self.path = path
        self.path_lookahead = path_lookahead

    # Return:
    # best edge index (bounded)
    # best distance
    # projection from point to that edge and it's t value
    def getClosestEdge(self, config):
        best_distance = 999999
        best_index = -1
        # iterate through the path in segments (p0, p1, 0), (p1,p2,1), ...., (pn-1, pn, n-1)
        for i, j, index in zip(self.path[:self.current_edge+2], self.path[1: self.current_edge + 2], range(len(self.path)-1)):
            p1 = np.asarray(i)
            p2 = np.asarray(j)
            point = np.asarray(config)

            # We assume the path starts at the origin, shift the odometry to match that space
            vecpath = p2 - p1
            pr = config - p1 #relative point to vector origin

            distance = None
            projection = None
            t = vecpath.dot(pr)/np.linalg.norm(vecpath) # Percentage on the current line
            # Create a bounded projection:
            # Projection is the closest point on the line to our robot.
            #  this differs from a regular projection because our line is finite.
            if(t < 0):
                projection = p1
            elif (t>1):
                projection = p2
            else:
                projection = t*vecpath + p1

            distance = np.linalg.norm(point-projection)

            if( t > 0.8 and index >= self.current_edge):
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
        lookahead_left = self.path_lookahead
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
                point = remaining_path_edges[0][1]
                remaining_path_edges = remaining_path_edges[1:]
            else:
                edge = np.array(remaining_path_edges[0][1]) - remaining_path_edges[0][0]
                #point = [float(point[0]), float(point[1])]
                point += (lookahead_left*(edge))/np.linalg.norm(edge)
                lookahead_left = 0

        return point, current_path_index

