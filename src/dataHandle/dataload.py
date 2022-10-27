#!/usr/bin/python2.7

import rospy
import math
# from sensor_msgs.msg import LaserScan
from data_scaner_handler import ScanerHandler
from robot_odometry import OdometryHandler
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

class Robot(object):

    def __init__(self):
        self.odometry = OdometryHandler()
        self.scaner = ScanerHandler()
        self.robot_pos = Point()
        self.pub = rospy.Publisher('points', PointStamped, queue_size=0)

    def check_new_data(self):
        if self.scaner.get_state():
            self.handle_laser_data(self.scaner.get_data())
            # print()
        # if self.odometry.get_state():
            self.odometry.get_data()

    def handle_laser_data(self, laser_data):
        sample = self.scaner.sample_num
        min_ang = self.scaner.min_angle
        max_ang = self.scaner.max_angle
        cur_ang = min_ang
        resolution = math.pi*2/sample
        las_pos = self.odometry.get_las_pos()
        yaw = self.odometry.get_yaw()
        point_list = []
        # cur_ang = min_ang - yaw
        for cur_point in laser_data:
            las_pos = self.odometry.get_las_pos()
            yaw = self.odometry.get_yaw()
            # x = math.cos(cur_ang)*cur_point/1
            # y = -math.sin(cur_ang)*cur_point/1

            # x = math.sin(yaw)*(math.sin(cur_ang)*cur_point + las_pos.x)
            # y = math.sin(yaw)*(math.cos(cur_ang)*cur_point + las_pos.y)

            #     te sa dobre
            # x = -math.sin(cur_ang)*cur_point + las_pos.x
            # y = math.cos(cur_ang)*cur_point + las_pos.y

            x1 = math.cos(cur_ang) * cur_point
            y1 = math.sin(cur_ang) * cur_point

            # x = x1 + las_pos.x
            # y = y1 + las_pos.y

            x = math.cos(yaw) * x1 - math.sin(yaw) * y1 + las_pos.x
            y = math.sin(yaw) * x1 + math.cos(yaw) * y1 + las_pos.y

            point = Point()
            # point.header.stamp = rospy.Time.now()
            # point.header.frame_id = "/dummyLink"
            point.x = x
            point.y = y
            # point.point.z = 0
            # self.pub.publish(point)
            point_list.append(point)
            # rospy.loginfo(str(point))
            # x = x * math.cos(yaw - math.pi/2) - y * math.sin(yaw-math.pi/2)
            # y = x * math.sin(yaw-math.pi/2) + y * math.cos(yaw-math.pi/2)
            # print(yaw)
            cur_ang += resolution
            # print(str(x)+";"+str(y)+";"+str(las_pos.x)+";"+str(las_pos.y)+";"+str(cur_point))
            # print(str(resolution)+";"+str(cur_ang))
            # print("x = " + str(x)+" y = " + str(y))
            # print("cur ang " + str(cur_ang))
            # print(str(cur_point))


    def start_simulation(self):

        while not rospy.is_shutdown():
            self.check_new_data()


def start_data_handle():
    rospy.init_node('data_handle')
    robot_sim = Robot()
    robot_sim.start_simulation()


if __name__ == "__main__":
    start_data_handle()

