#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import LaserScan
# from data_scaner_handler import ScanerHandler


class ScanerHandler(object):

    def __init__(self):
        self.laser_topic_name = 'robot/laser'
        self.new_data_state = False
        self.sample_num = 720
        self.min_angle = - 3.14159265359  # - pi/2
        self.max_angle = 3.14159265359   # pi/2
        self.sub = rospy.Subscriber(self.laser_topic_name, LaserScan, self.__get_new_data)
        self.laser_scan = LaserScan()

    def get_state(self):  # Returns if there is new laser data
        return self.new_data_state

    def get_data(self):
        self.new_data_state = False
        return self.laser_scan.ranges

    def __get_new_data(self, msg):
        self.laser_scan = msg
        # print(self.laser_scan)
        self.new_data_state = True


