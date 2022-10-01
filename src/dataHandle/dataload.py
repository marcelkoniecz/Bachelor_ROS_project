#!/usr/bin/python3

import rospy
# from sensor_msgs.msg import LaserScan
from data_scaner_handler import ScanerHandler
from robot_odometry import OdometryHandler
# def get_laser_data(msg):
#     print(msg.ranges[1])
#
#
# rospy.init_node('data_handle')
# laser_topic_name = 'robot/laser'
# sub = rospy.Subscriber(laser_topic_name, LaserScan, get_laser_data)
# rospy.spin()
# # def check_data(self):


class Robot(object):

    def __init__(self):
        self.scaner_data_handler = ScanerHandler()
        self.odometry_handler = OdometryHandler()

    # def start_handle(self):


#
# def start_data_handle():
#
#     robot_han.start_handle()

if __name__ == "__main__":
    rospy.init_node('data_handle')
    robot_han = Robot()
    rospy.spin()
