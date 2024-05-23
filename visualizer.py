import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D


class Visualize_UR(object):
    def __init__(self, ur_params, env, transform, bb):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.colors = ur_params.ur_links_color
        self.sphere_radius = ur_params.sphere_radius
        self.end_effector_pos = np.array([0,0,0,1])
        self.env = env
        self.transform = transform
        self.bb = bb
        plt.ion()
        plt.show()
    
    def plot_links(self,end_efctors):
        for link_edge in end_efctors:
            self.ax.scatter(link_edge[0], link_edge[1], link_edge[2])
        for i in range(len(end_efctors)-1):
            self.ax.plot([end_efctors[i][0],end_efctors[i+1][0]], [end_efctors[i][1],end_efctors[i+1][1]],[end_efctors[i][2],end_efctors[i+1][2]])
    
    def show(self, end_effctors=None):
        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')
        self.ax.set_zlabel('z')
        self.ax.set_xlim3d([-0.7, 0.7])
        self.ax.set_ylim3d([-0.7, 0.7])
        self.ax.set_zlim3d([0, 1.0])
        self.ax.plot([0,0.5],[0,0],[0,0],c='red')
        self.ax.plot([0,0],[0,0.5],[0,0],c='green')
        self.ax.plot([0,0],[0,0],[0,0.5],c='blue')
        self.draw_obstacles()
        self.fig.canvas.flush_events()

    def draw_obstacles(self):
        '''
        Draws the spheres constructing the obstacles
        '''
        u, v = np.mgrid[0:2 * np.pi:10j, 0:np.pi:10j]
        for sphere in self.env.obstacles:
            x = np.cos(u) * np.sin(v) * self.env.radius + sphere[0]
            y = np.sin(u) * np.sin(v) * self.env.radius + sphere[1]
            z = np.cos(v) * self.env.radius + sphere[2]
            self.ax.plot_surface(x, y, z, color='red',alpha=0.5)
            
        
    def draw_spheres(self, global_sphere_coords, track_end_effector=False):
        '''
        Draws the spheres constructing the manipulator
        '''
        u, v = np.mgrid[0:2 * np.pi:10j, 0:np.pi:10j]
        for frame in global_sphere_coords.keys():
            for sphere_coords in global_sphere_coords[frame]:
                x = np.cos(u) * np.sin(v) *self.sphere_radius[frame] + sphere_coords[0]
                y = np.sin(u) * np.sin(v) *self.sphere_radius[frame] + sphere_coords[1]
                z = np.cos(v) * self.sphere_radius[frame] + sphere_coords[2]
                self.ax.plot_surface(x, y, z, color=self.colors[frame], alpha=0.3) 
        if track_end_effector:
            self.end_effector_pos =np.vstack((self.end_effector_pos, global_sphere_coords['wrist_3_link'][-1]))
            self.ax.scatter(self.end_effector_pos[:,0], self.end_effector_pos[:,1], self.end_effector_pos[:,2])
    
    def show_path(self, path):
        '''
        Plots the path
        '''
        for conf in path:
            global_sphere_coords = self.transform.conf2sphere_coords(conf)
            self.draw_spheres(global_sphere_coords,  track_end_effector=True)
            self.show()
            time.sleep(0.3)
            self.ax.axes.clear()
    
    def show_conf(self, conf:np.array):
        '''
        Plots configuration
        '''
        global_sphere_coords = self.transform.conf2sphere_coords(conf)
        self.draw_spheres(global_sphere_coords)
        plt.ioff()
        self.show()
        plt.show()
        time.sleep(0.1)
