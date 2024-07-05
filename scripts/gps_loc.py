#! /usr/bin/env python

import rospy
import sys
import tf
from std_msgs.msg import Float64MultiArray
from gps_common.msg import GPSFix
import numpy as np
from geo_coord_transform import *
from transform_tools import *

gps_src = 0  # 0: sim, 1: fdi

class gps_localizer:
    def __init__(self):
        self.send_rate = rospy.Rate(10)  # send with 10 hz
        self.br = tf.TransformBroadcaster()

        self.lla_ori = np.array([40.0, 117.0, 0.0])
        self.sim_gps_sub = rospy.Subscriber('dgps_floatarray', Float64MultiArray, self.sim_gps_cb)
        self.fdi_gps_sub = rospy.Subscriber('/gnss_dual_ant/fix', GPSFix, self.fdi_gps_cb)

    def tf_broadcast(self, trans, quat, child_frame, parent_frame):
        self.br.sendTransform((trans[0], trans[1], trans[2]), (quat[0], quat[1], quat[2], quat[3]), rospy.Time.now(),
                              child_frame, parent_frame)

    def send_tf(self):
        self.tf_broadcast([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0],
                          'laser_link', 'base_link')

    def sim_gps_cb(self, msg):
        if gps_src == 0:
            data = np.array(msg.data)
            lla = data[0:3]
            track = data[3]
            theta = track+np.pi/2.0
            if theta > np.pi:
                theta = theta - 2.0*np.pi

            enu = geodetic_to_enu(lla[0], lla[1], lla[2], self.lla_ori[0], self.lla_ori[1], self.lla_ori[2])

            quat = ypr2quat(np.array([theta, 0, 0]))

            self.tf_broadcast(enu, quat, 'base_link', 'map')

    def fdi_gps_cb(self, msg):
        if gps_src == 1:
            lla = np.array([np.deg2rad(msg.latitude), np.deg2rad(msg.longitude), msg.altitude])
            track = np.deg2rad(msg.track)
            theta = track + np.pi / 2.0
            if theta > np.pi:
                theta = theta - 2.0*np.pi

            enu = geodetic_to_enu(lla[0], lla[1], lla[2], self.lla_ori[0], self.lla_ori[1], self.lla_ori[2])

            quat = ypr2quat(np.array([theta, 0, 0]))

            self.tf_broadcast(enu, quat, 'base_link', 'map')


def main(args):
    rospy.init_node('gps_loc_node', anonymous=True)
    gl = gps_localizer()

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)
