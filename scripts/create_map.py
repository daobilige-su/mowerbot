import cv2
import numpy as np
import sys
import rospy
import rospkg
from transform_tools import *
import datetime
import yaml

class map_creator:
    def __init__(self):
        # load params
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('mowerbot') + '/'
        t_now = datetime.datetime.now()
        self.output_map_file_name = 'map_' + t_now.strftime("%Y-%m-%d-%H-%M-%S")

        # CHANGE THESE KEY PARAMS BELOW:
        self.ref_lla = np.array([40.0, 117.0, 0.0])
        self.obstacles_lla = None  # np.array([[]])  # 3x2N
        self.obstacles_xy = np.array([[0.0,     50.0,   0.0,    50.0,   0.0,    50.0],
                                      [-5.0,    -2.5,   5.0,    7.5,    15.0,   17.5]])  # 2x2N
        self.map_reso = 0.05
        self.map_edge_dist = 5.0


    def create_map(self):
        if self.obstacles_xy is None:
            self.compute_obstacles_xy_from_lla()

        x_min = np.min(self.obstacles_xy[0, :])
        x_max = np.max(self.obstacles_xy[0, :])
        y_min = np.min(self.obstacles_xy[1, :])
        y_max = np.max(self.obstacles_xy[1, :])

        map_bound = np.array([x_min-self.map_edge_dist, x_max+self.map_edge_dist, y_min-self.map_edge_dist, y_max+self.map_edge_dist])
        # The origin data in the YAML file is the position of the lower left corner of the occupancy map image with respect to the origin in Rviz.
        map_yaml_origin = np.array([x_min - self.map_edge_dist, y_min - self.map_edge_dist])

        # map: x is u (hor), y is v (ver)
        map_u_max = int(np.round((map_bound[1]-map_bound[0])/self.map_reso))
        map_v_max = int(np.round((map_bound[3]-map_bound[2])/self.map_reso))

        map = 255*np.ones((map_v_max, map_u_max))

        for n in range(int(self.obstacles_xy.shape[1]/2)):
            x1 = self.obstacles_xy[0, 2*n] - map_yaml_origin[0]
            y1 = self.obstacles_xy[1, 2*n] - map_yaml_origin[1]  # note: direction of the y in map coord and image v are opposite
            x2 = self.obstacles_xy[0, 2*n+1] - map_yaml_origin[0]
            y2 = self.obstacles_xy[1, 2*n+1] - map_yaml_origin[1]  # note: direction of the y in map coord and image v are opposite
            map = cv2.rectangle(map, (int(x1/self.map_reso), map_v_max - int(y1/self.map_reso)),
                                     (int(x2/self.map_reso), map_v_max - int(y2/self.map_reso)), 0, -1)  # draw rect

        cv2.imwrite(self.pkg_path + 'map/' + self.output_map_file_name+'.pgm', map)

        map_yaml = dict(image=self.output_map_file_name+'.pgm',
                        resolution=self.map_reso,
                        origin=[float(map_yaml_origin[0]), float(map_yaml_origin[1]), 0.0],
                        negate=0,
                        occupied_thresh=0.65,
                        free_thresh=0.2,
                        ref_lla=[float(self.ref_lla[0]), float(self.ref_lla[1]), float(self.ref_lla[2])])

        with open(self.pkg_path + 'map/' + self.output_map_file_name+'.yaml', 'w') as outfile:
            yaml.dump(map_yaml, outfile, default_flow_style=False, sort_keys=False)

    def compute_obstacles_xy_from_lla(self):
        pass


def main(args):
    mc = map_creator()
    mc.create_map()


if __name__ == '__main__':
    main(sys.argv)
