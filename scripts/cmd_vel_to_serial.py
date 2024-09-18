#! /usr/bin/env python

import rospy
import sys
import tf
from geometry_msgs.msg import Twist
import numpy as np
import yaml
import rospkg
import serial
from threading import Thread

class cmd_vel_sender:
    def __init__(self):
        self.debug_on = 1
        # locate ros pkg
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('mowerbot') + '/'
        # load key params
        # params_filename = rospy.get_param('param_file')  # self.pkg_path + 'cfg/' + 'param.yaml'
        params_filename = self.pkg_path + 'param/' + 'mowerbot_params.yaml'
        with open(params_filename, 'r') as file:
            self.param = yaml.safe_load(file)

        self.hw_action = self.param['motor']['hw_action']
        self.wheel_dist = self.param['motor']['wheel_dist']  # wheel dist
        self.max_vel_cmd = self.param['motor']['max_vel_cmd']
        self.serial_cmd_rate = rospy.Rate(10)

        self.lw_cmd = int(1580)
        self.rw_cmd = int(1580)

        # serial to weeder control
        if self.hw_action:
            self.ser = serial.Serial()
            self.ser.port = self.param['motor']['ser_port']
            self.ser.baudrate = self.param['motor']['ser_baudrate']
            # self.ser.bytesize = self.param['weeder']['bytesize']  # len
            # self.ser.stopbits = self.param['weeder']['stopbits']  # stop
            # self.ser.parity = self.param['weeder']['parity']  # check
            self.ser.open()

        self.sim_gps_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_cb)

        self.thr = Thread(target=self.print_serial)
        self.thr.start()

    def __del__(self):
        self.ser.close()
        self.thr.join()

    def cmd_vel_cb(self, msg):
        vx = msg.linear.x
        w = msg.angular.z

        lw_v = vx - w*(self.wheel_dist/2.0)
        rw_v = vx + w*(self.wheel_dist/2.0)

        # lw_cmd = lw_v*(self.max_vel_cmd/0.5)
        # rw_cmd = rw_v*(self.max_vel_cmd/0.5)
        lw_cmd = lw_v/0.5*30.0
        rw_cmd = rw_v/0.5*30.0


        max_vel = max(abs(lw_cmd), abs(rw_cmd))
        if max_vel>self.max_vel_cmd:
            lw_cmd = lw_cmd*(self.max_vel_cmd/max_vel)
            rw_cmd = rw_cmd*(self.max_vel_cmd/max_vel)

        if abs(lw_cmd)<0.05:
            self.lw_cmd = 1500
        else:
            if lw_cmd>0:
                self.lw_cmd = 1570+lw_cmd
            else:
                self.lw_cmd = 1430+lw_cmd
        if abs(rw_cmd)<0.05:
            self.rw_cmd = 1500
        else:
            if rw_cmd>0:
                self.rw_cmd = 1570+rw_cmd
            else:
                self.rw_cmd = 1430+rw_cmd

        # self.lw_cmd = int(lw_cmd)+1500
        # self.rw_cmd = int(rw_cmd)+1500

    def send_serial_cmd(self):
        ser_cmd_str = 's%04d%04de' % (self.lw_cmd, self.rw_cmd)
        rospy.loginfo('cmd_vel_to_serial: sending motor cmd: '+ser_cmd_str)

        if self.hw_action:
            # send weeder control
            self.ser.write(ser_cmd_str.encode())
            self.ser.flush()

    def print_serial(self):
        while not rospy.is_shutdown():
            if self.debug_on:
                in_num = self.ser.in_waiting
                if not in_num==0:
                    rospy.logwarn(self.ser.read(size=in_num))


def main(args):
    rospy.init_node('cmd_vel_to_serial_node', anonymous=True)
    cv_sender = cmd_vel_sender()

    while not rospy.is_shutdown():
        cv_sender.send_serial_cmd()
        cv_sender.serial_cmd_rate.sleep()


if __name__ == '__main__':
    main(sys.argv)
