#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
# from data_scaner_handler import ScanerHandler


class OdometryHandler(object):

    def __init__(self):
        self.laser_topic_name = '/odom'
        self.sub = rospy.Subscriber(self.laser_topic_name, Odometry, self.get_odometry_data)
        self.odomdata = Odometry()

        # rospy.init_node('data_handle')
        # laser_topic_name = 'robot/laser'
        # sub = rospy.Subscriber(laser_topic_name, LaserScan, get_laser_data)
        # rospy.spin()
        # # def check_data(self):

        # self.laser_topic_name = 'robot/laser'
        # self.check_data()
        # self.sub = rospy.Subscriber(self.laser_topic_name, LaserScan, self.get_laser_data)
        # self.rospy.spin()

    def get_odometry_data(self, msg):
        self.odomdata = msg
        print(self.odomdata)
