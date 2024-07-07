
import numpy as np

class Environment(object):
    '''
    Environment class implements the physical robot's environment
    '''
    def __init__(self, env_idx):
        self.radius = 0.05
        obstacles = []
        # env_idx = 0 is obstacle free
        if env_idx == 1:
            self.radius = 0.05
            self.wall_x_const(y_min=-0.58, y_max=0, z_min=0, z_max=0.45, x_const=-0.17, obstacles=obstacles)

        elif env_idx == 2:
            x_const = 0
            y_min, y_max = -0.65, -0.3
            z_min, z_max = 0 ,0.5
            self.wall_x_const(y_min, y_max, z_min,z_max, x_const, obstacles)

        elif env_idx == 3:
            self.radius=0.025
            self.box(x=-0.3, y=-0.3, dx=0.15, dy=0.30, dz=0.15, obstacles=obstacles, skip=['-x'])
            self.box(x=0, y=-0.6, dx=0.3, dy=0.15, dz=0.15, obstacles=obstacles, skip=['-y'])

        self.obstacles = np.array(obstacles)

    def sphere_num(self, min_coord, max_cord):
        '''
        Return the number of spheres based on the distance
        '''
        return int(np.ceil(abs(max_cord-min_coord) / (self.radius*2))+2)

    def wall_y_const(self, x_min, x_max, z_min, z_max, y_const, obstacles):
        '''
        Constructs a wall with constant y coord value
        '''
        num_x = self.sphere_num(x_min, x_max)
        num_z = self.sphere_num(z_min, z_max)
        for x in list(np.linspace(x_min, x_max,  num= num_x, endpoint=True)):
                for z in list(np.linspace(z_min, z_max, num= num_z, endpoint=True)):
                    obstacles.append([x, y_const, z])

    def wall_x_const(self, y_min, y_max, z_min, z_max, x_const, obstacles):
        '''
        Constructs a wall with constant x coord value
        '''
        num_y = self.sphere_num(y_min, y_max)
        num_z = self.sphere_num(z_min, z_max)
        for y in list(np.linspace(y_min, y_max,  num= num_y, endpoint=True)):
                for z in list(np.linspace(z_min, z_max , num= num_z, endpoint=True)):
                    obstacles.append([x_const, y, z])

    def wall_z_const(self, x_min, x_max, y_min, y_max, z_const, obstacles):
        '''
        Constructs a wall with constant z coord value
        '''
        num_y = self.sphere_num(y_min, y_max)
        num_x = self.sphere_num(x_min, x_max)
        for y in list(np.linspace(y_min, y_max,  num= num_y, endpoint=True)):
                for x in list(np.linspace(x_min, x_max , num= num_x, endpoint=True)):
                    obstacles.append([x, y, z_const])

    def box(self, x, y, dx, dy, dz, obstacles, skip =[]):
        '''
        Constructs a Box
        '''
        if '-x' not in skip:
            self.wall_x_const(y-dy/2, y+dy/2, 0, dz, x-dx/2, obstacles)
        if 'x' not in skip:
            self.wall_x_const(y-dy/2, y+dy/2, 0, dz, x+dx/2, obstacles)
        if '-y' not in skip:
            self.wall_y_const(x-dx/2, x+dx/2, 0, dz, y-dy/2, obstacles)
        if 'y' not in skip:
            self.wall_y_const(x-dx/2, x+dx/2, 0, dz, y+dy/2, obstacles)
        if 'z' not in skip:
            self.wall_z_const(x-dx/2, x+dx/2, y-dy/2, y+dy/2, dz, obstacles)
