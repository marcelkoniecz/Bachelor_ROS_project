#!/usr/bin/python2.7

import rospy
from sensor_msgs.msg import LaserScan
# from dataload import Robot
# from data_scaner_handler import ScanerHandler
import rospy
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point


def calculate_yaw(msg):
    orientation_q = msg.pose.pose.orientation
    orientatation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientatation_list)
    return yaw


class ScanerHandler(object):

    def __init__(self):
        self.laser_topic_name = 'robot/laser'
        self.new_data_state = False
        self.sample_num = 720
        self.min_angle = - 3.14159265359  # - pi/2
        self.max_angle = 3.14159265359   # pi/2
        self.sub = rospy.Subscriber(self.laser_topic_name, LaserScan, self.__get_new_laser_data)
        self.laser_scan = LaserScan()

        self.topic_name = '/odom'
        self.sub = rospy.Subscriber(self.topic_name, Odometry, self.__get_new_odom_data)
        self.odomdata = Odometry()
        self.rob_pos = Point()      # Robot centre position
        self.pos_during_laser = Point()
        self.euler_yaw = 0
        self.lock = False

    def get_state(self):  # Returns if there is new laser data
        return self.new_data_state

    def wait(self):
        while self.lock:
            i = 1

    def get_data(self):
        self.new_data_state = False
        return self.laser_scan.ranges

    def get_las_pos(self):
        self.wait()
        return self.pos_during_laser

    def get_yaw(self):
        self.wait()
        return self.euler_yaw

    def __get_new_laser_data(self, msg):
        self.lock = True
        self.laser_scan = msg
        self.pos_during_laser = self.rob_pos
        # self.callbackfunction()
        # print("AAA")
        self.new_data_state = True
        self.lock = False

    def __get_new_odom_data(self, msg):
        self.lock = False
        self.wait()
        self.odomdata = msg
        self.rob_pos = msg.pose.pose.position
        self.euler_yaw = calculate_yaw(msg)
        # self.las_pos.x = msg.pose.pose.position.x
        # self.las_pos.y = msg.pose.pose.position.y
        # x= msg.pose.pose.position.x
        # y= msg.pose.pose.position.y
        # inital laser position is (0.17,0) -> in equation we skip  multiplication by y
        # self.las_pos.x = msg.pose.pose.position.x + math.cos(self.euler_yaw) * self.las_dist
        # self.las_pos.y = msg.pose.pose.position.y + math.sin(self.euler_yaw) * self.las_dist
        self.lock = False

