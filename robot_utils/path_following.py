# This python code is meant to follow a path.
# Input: a list of (x,y) coordinates.
# Stream: updates of robot position or returns NaN if not updated.

# womp womp i use numpy :/
import numpy as np
from util.constants import PATH_LOOKAHEAD

def get_angle_to(odometry, point):
    return ((np.arctan2(point[1]-odometry.y, point[0] - odometry.x) - odometry.angle)+np.pi)%(2*np.pi)-np.pi


# These are classes which store data.
class Odometry:
    x = None
    y = None
    angle = None
    speed_l = None
    speed_r = None
    def __init__(self, x=None,y=None,angle=None,speed_l=None,speed_r=None):
        self.x = x
        self.y = y
        self.angle = angle
        self.speed_l = speed_l
        self.speed_r = speed_r

# This class has a function to retrieve data,
# get() ~ which returns None if nothing has changed since the last reading
class Stream:
    data = Odometry()
    is_data_new = False
    def __init__(self):
        pass

    def get_last(self):
        return self.data
    def get(self):
        # UPDATE THE DATA
        if not self.is_data_new:
            return None
        else:
            return self.data

# Stores function follow()
# this  is its own class so i can store
class PathFollow:
    path = None
    path_lookahead = 0
    current_edge = 0
    def __init__(self, path, path_lookahead = PATH_LOOKAHEAD):
        self.path = path
        self.path_lookahead = path_lookahead

    # Return:
    # best edge index (bounded)
    # best distance
    # projection from point to that edge
    def getClosestEdge(self, odometry):
        best_distance = 999999
        best_index = -1
        # iterate through the path in segments (p0, p1, 0), (p1,p2,1), ...., (pn-1, pn, n-1)
        for i, j, index in zip(self.path[:self.current_edge+2], self.path[1: self.current_edge + 2], range(len(self.path)-1)):
            p1 = np.asarray(i)
            p2 = np.asarray(j)
            point = np.asarray((odometry.x, odometry.y))

            # We assume the path starts at the origin, shift the odometry to match that space
            vecpath = p2 - p1
            pr = np.asarray((odometry.x - p1[0], odometry.y - p1[1])) #relative point to vector origin

            distance = None
            projection = None
            t = (pr[0] * vecpath[0] + pr[1] * vecpath[1])/(vecpath[1]*vecpath[1] + vecpath[0]*vecpath[0]) # Percentage on the current line
            # Create a bounded projection:
            # Projection is the closest point on the line to our robot.
            #  this differs from a regular projection because our line is finite.
            if(t < 0):
                projection = p1
            elif (t>1):
                projection = p2
            else:
                projection = (vecpath[0]*t + p1[0], vecpath[1]*t + p1[1])

            if( t > 0.8 and index >= self.current_edge):
                self.current_edge += 1


            distance = np.linalg.norm(point-projection)

            # If better than the current best, update it
            if distance <= best_distance:
                best_distance = distance
                best_index = index
                best_projection = (projection, t)
        #print(best_distance,best_index,self.current_edge,[(a,b,c) for a,b,c in zip(self.path[:self.current_edge+2], self.path[1: self.current_edge + 2], range(len(self.path)-1))],self.path[:self.current_edge+2])
        return best_index, best_distance, best_projection


    def getLookaheadEdge(self, odometry):
        lookahead_left = self.path_lookahead
        current_path_index, distance, projection = self.getClosestEdge(odometry)
        remaining_path_edges = [(p1,p2) for p1, p2 in zip(self.path[current_path_index:], self.path[current_path_index+1:])]

        point = np.array(projection[0])
        while(lookahead_left > 0 and len(remaining_path_edges)>0):
            curr_edge_remainder = np.linalg.norm(np.array(point)-remaining_path_edges[0][1])
            if(curr_edge_remainder <= lookahead_left):
                lookahead_left -= curr_edge_remainder
                point = remaining_path_edges[0][1]
                remaining_path_edges = remaining_path_edges[1:]
            else:
                path = np.array(remaining_path_edges[0][1]) - remaining_path_edges[0][0]
                point = [float(point[0]), float(point[1])]
                point += (lookahead_left*(path))/np.linalg.norm(path)
                lookahead_left = 0

        return point, current_path_index

