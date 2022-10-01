#!/usr/bin/python3

import rospy
from sensor_msgs.msg import LaserScan
# from data_scaner_handler import ScanerHandler


class ScanerHandler(object):

    def __init__(self):
        self.laser_topic_name = 'robot/laser'
        self.sub = rospy.Subscriber(self.laser_topic_name, LaserScan, self.get_laser_data)
        # rospy.init_node('data_handle')
        # laser_topic_name = 'robot/laser'
        # sub = rospy.Subscriber(laser_topic_name, LaserScan, get_laser_data)
        # rospy.spin()
        # # def check_data(self):

        # self.laser_topic_name = 'robot/laser'
        # self.check_data()
        # self.sub = rospy.Subscriber(self.laser_topic_name, LaserScan, self.get_laser_data)
        # self.rospy.spin()

    def get_laser_data(self, msg):
        print(msg)



