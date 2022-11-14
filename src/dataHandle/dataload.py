#!/usr/bin/python2.7

import rospy
import math
# from sensor_msgs.msg import LaserScan
from data_scaner_handler import ScanerHandler
from robot_odometry import OdometryHandler
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from matplotlib.patches import Ellipse
import matplotlib.pyplot as plt
import numpy as np
# from dataclasses import dataclass


class GridCell:
    def __init__(self, grid_pos, point_pos):
        self.grid_pos = grid_pos
        self.points = [point_pos]
        # [0] - x, [1] - y
        self.avg = [0]*2    # Average position
        self.std_dev = [0]*2   # Standard deviation
        self.angle = 0
        self.angle_deg =0
        self.first_calculation = True
        self.point_num = 0

    def get_angle(self):
        return self.angle_deg

    def get_grid_pos(self):
        return self.grid_pos

    def get_points(self):
        return self.points

    def set_new_point(self, new_point):
        self.points.append(new_point)

    def print_cell(self):
        print(self.grid_pos)
        print(self.points)

    def get_average_position(self):
        return self.avg

    def get_standard_deviation(self):
        return self.std_dev

    def process_data(self):
        tab = [0]*2
        sumx = 0
        sumy = 0
        self.point_num += self.points.__len__()
        # print(self.points.__len__())
        if self.points.__len__() == 0 or self.points.__len__() == 1:
            return False

        # self.angle = 90
        if self.first_calculation:
            s = np.polyfit([x.x for x in self.points], [y.y for y in self.points], 1)
            self.angle = math.atan(s[0])

        self.avg[0] = 0
        self.avg[1] = 0
        for tmp in self.points:
            self.avg[0] += tmp.x
            self.avg[1] += tmp.y

        self.avg[0] = self.avg[0] / self.points.__len__()
        self.avg[1] = self.avg[1] / self.points.__len__()

        self.first_calculation = True
        if self.first_calculation:
            # self.angle = math.atan(s[0])
            if self.angle > 0.01 or self.angle < -0.01:
                for tmp1 in self.points:
                    x = tmp1.x - self.avg[0]
                    y = tmp1.y - self.avg[1]
                    tmp1.x = math.cos(-self.angle) * x - math.sin(-self.angle) * y + self.avg[0]
                    tmp1.y = math.sin(-self.angle) * x + math.cos(-self.angle) * y + self.avg[1]

            for tmp in self.points:
                sumx += pow((tmp.x - self.avg[0]), 2)
                sumy += pow((tmp.y - self.avg[1]), 2)


            # print("y = " +str(s[0])+"x +" +str(s[1]))

            self.std_dev[0] = (math.sqrt(sumx/(self.points.__len__()-1)) + self.std_dev[0]) / 2
            self.std_dev[1] = (math.sqrt(sumy/(self.points.__len__()-1)) + self.std_dev[1]) / 2

            self.angle_deg = self.angle * 180 / math.pi

        # self.first_calculation = False
        del self.points[:]
        return True
        # print(str(self.avg) + " " + str(self.std_dev))


class Robot(object):

    def __init__(self):
        # self.odometry = OdometryHandler()
        self.scaner = ScanerHandler()
        self.robot_pos = Point()
        # self.pub = rospy.Publisher('points', PointStamped, queue_size=0)
        self.cells_list = []
        self.grid_size = 0.5  # in m
        self.yaw = 0

    # def callback_save_pos(self):
    #     self.yaw = self.odometry.get_yaw()
    #     self.robot_pos = self.odometry.get_las_pos()
    #     print("call")

    def check_new_data(self):
        if self.scaner.get_state():
            self.handle_laser_data(self.scaner.get_data())

        #     self.odometry.get_data()

    def handle_laser_data(self, laser_data):
        self.robot_pos = self.scaner.get_las_pos()
        self.yaw = self.scaner.get_yaw()

        sample = self.scaner.sample_num
        min_ang = self.scaner.min_angle
        max_ang = self.scaner.max_angle
        # print(yaw)
        cur_ang = min_ang
        resolution = math.pi*2/sample

        point_list = []
        cells_list = []
        in_list = False

        # cur_ang = min_ang - yaw
        for cur_point in laser_data:

            x1 = math.cos(cur_ang) * cur_point
            y1 = math.sin(cur_ang) * cur_point
            x = math.cos(self.yaw) * x1 - math.sin(self.yaw) * y1 + self.robot_pos.x
            y = math.sin(self.yaw) * x1 + math.cos(self.yaw) * y1 + self.robot_pos.y

            # x = math.cos(yaw) * x1 - math.sin(yaw) * y1 + las_pos.x
            # y = math.sin(yaw) * x1 + math.cos(yaw) * y1 + las_pos.y
            point = Point()
            point.x = x
            point.y = y
            # if x == 'inf' or x == '-inf' or y == 'inf' or y =='-inf':
            #     print("aa")
            #     continue
            point_list.append(point)
            cur_ang += resolution
            # print(point)
            try:
                pos_grid = self.calculate_mod(point)

            except:
                print("Blad")
                continue
            in_list = False
            for m in self.cells_list:
                if m.grid_pos == pos_grid:
                    m.set_new_point(point)
                    in_list = True

            if not in_list:
                new_cell = GridCell(pos_grid, point)
                self.cells_list.append(new_cell)

        # print(str(self.cells_list.__len__()))
        self.handle_data()
        self.plot_data()
        #
        # try:
        #     self.cells_list[1].print_cell()
        # except:
        #     print("no data")
        # del self.cells_list[:]

        # print(str(self.cells_list.__len__()))

    def plot_data(self):
        plt.ion()
        plt.clf()
        fig = plt.figure(0)
        ax = fig.add_subplot(111)
        ax.set_xlim(-20, 20)
        ax.set_ylim(-20, 20)

        for cur_cell in self.cells_list:

            pos = cur_cell.get_average_position()
            dev = cur_cell.get_standard_deviation()
            # print(dev[0])
            ell = Ellipse(pos, ((dev[0])*6), ((dev[1])*6), angle=cur_cell.get_angle(), color='red')
            # print(ell)

            # ax = fig.add_subplot(111, aspect = 'equal')
            # ax.add_artist(ell)
            ax.add_artist(ell)

        ell = Ellipse([self.robot_pos.x,self.robot_pos.y], 0.8, 0.5, angle=(self.yaw* 180 / math.pi), color='blue')
        ax.add_artist(ell)
        plt.show()
        plt.pause(0.00001)
        print("tak")
        # plt.plot(ell)
        # plt.show()
        # plt.plot([1,2,3,4])
        # plt.show()
        return True

    def handle_data(self):
        for cur_cell in self.cells_list:
            cur_cell.process_data()

    def calculate_mod(self, point):
        ret_point = Point()
        if point.x >= 0:
            ret_point.x = int(point.x / self.grid_size + 1)
        elif point.x < 0:
            ret_point.x = int(point.x / self.grid_size - 1)
        if point.y >= 0:
            ret_point.y = int(point.y / self.grid_size + 1)
        elif point.y < 0:
            ret_point.y = int(point.y / self.grid_size - 1)
        return ret_point

    def start_simulation(self):

        while not rospy.is_shutdown():
            self.check_new_data()

# def callback(x):
#     x.callback_save_pos()


def start_data_handle():
    rospy.init_node('data_handle')
    robot_sim = Robot()
    robot_sim.start_simulation()


if __name__ == "__main__":
    start_data_handle()

