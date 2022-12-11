#!/usr/bin/python2.7

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

class OdometryHandler(object):

    def __init__(self):
        self.laser_topic_name = '/odom'
        self.sub = rospy.Subscriber(self.laser_topic_name, Odometry, self.__get_new_data)
        self.new_data_state = False
        self.odomdata = Odometry()
        self.rob_pos = Point()      # Robot centre position
        self.las_pos = Point()      # Laser position
        self.saved_pos = Point()
        self.saved_yaw = 0
        self.euler_yaw = 0
        self.las_dist = 0    # Distance between laser and robot centre
        self.lock = False
        self.angular_speed = True
        self.lock_during_read = False

    def get_state(self):
        return self.new_data_state

    def is_angular_vel(self):
        # return False
        return self.angular_speed

    def save_pos(self):
        self.saved_pos = self.las_pos
        self.saved_yaw = self.euler_yaw

    def lock_write(self):
        self.lock_during_read = True

    def unlock(self):
        self.lock_during_read = False

    def get_data(self):
        self.new_data_state = False
        # laser_distance = 0.17
        # orientation_q = self.odomdata.pose.pose.orientation
        # orientatation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # (roll, pitch, yaw) = euler_from_quaternion(orientatation_list)
        # print("Robot x " + str(self.odomdata.pose.pose.position.x) + "y " + str(self.odomdata.pose.pose.position.y))
        # laser_x = self.odomdata.pose.pose.position.x + math.sin(yaw) * laser_distance
        # laser_y = self.odomdata.pose.pose.position.y + math.cos(yaw) * laser_distance
        # print("Laser x " + str(laser_x) + "y " + str(laser_y))
        # print(str(roll)+" "+str(pitch)+" "+str(yaw))
        # return self.odomdata

    def wait(self):
        while self.lock:
            i = 1

    def get_robot_pos(self):
        self.wait()
        return self.rob_pos

    def get_odometry(self):
        self.wait()
        return self.odomdata

    def get_yaw(self):
        self.wait()
        # return self.euler_yaw
        return self.saved_yaw

    def get_las_pos(self):
        self.wait()
        # return self.las_pos
        return self.saved_pos

    def __get_new_data(self, msg):
        if self.lock_during_read:
            return
        self.lock = True
        self.new_data_state = True
        self.odomdata = msg
        self.rob_pos = msg.pose.pose.position
        self.euler_yaw = calculate_yaw(msg)
        # if abs(msg.twist.twist.angular.z) > 0.01:
        #     self.angular_speed = True
        # else:
        #     self.angular_speed = False
        print(abs(self.euler_yaw))
        self.las_pos.x = msg.pose.pose.position.x
        self.las_pos.y = msg.pose.pose.position.y
        print(abs(msg.twist.twist.linear.x))
        # if abs(msg.twist.twist.angular.z) < 0.01 and abs(msg.twist.twist.linear.x) < 0.01:
        #     self.angular_speed = False
        #     print("nie")
        # else:
        #     self.angular_speed = True
        # x= msg.pose.pose.position.x
        # y= msg.pose.pose.position.y
        # inital laser position is (0.17,0) -> in equation we skip  multiplication by y
        # self.las_pos.x = msg.pose.pose.position.x + math.cos(self.euler_yaw) * self.las_dist
        # self.las_pos.y = msg.pose.pose.position.y + math.sin(self.euler_yaw) * self.las_dist
        self.lock = False
        # print("rob x "+str(x)+" y "+str(y))
        # print("las x "+str(self.las_pos.x) + " y " + str(self.las_pos.y))
         # print(self.odomdata)
        # print(str(self.rob_pos))
        # print(str(self.las_pos))
        # print(str(self.euler_yaw))