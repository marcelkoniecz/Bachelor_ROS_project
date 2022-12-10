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
        if  self.points.__len__() < 3:
            return False

        # self.angle = 90

        # if self.first_calculation:
        # ang = (self.points[self.points.__len__()-1].y-self.points[0].y) / \
        #         (self.points[self.points.__len__()-1].x-self.points[0].x)
        # self.angle = math.atan(ang)
        # print("ang 1 = " + str(ang))
        # s = np.polyfit([x.x for x in self.points], [y.y for y in self.points], 1)
        # self.angle = math.atan(s[0])
        # print("ang = " + str(ang)+" s= " + str(s[0]))
        # print("polifit =" + str(s[0]))
        x_arr = np.matrix([np.ones(self.points.__len__()),[x.x for x in self.points]])
        # x_arr_trans = x_arr.transpose()
        y_arr = np.matrix([y.y for y in self.points])
        print("ilos" + str(self.points.__len__()) + "2: " + str(x_arr.size))
        # print()
        # print(np.linalg.det(np.outer( x_arr, x_arr)))
        # w_lr =np.linalg.inv((np.outer( x_arr, x_arr)))
        # w_lr=np.linalg.inv(np.dot(x_arr_trans, x_arr))
        # try:
        w_lr = np.dot(np.dot(np.linalg.inv(np.dot(x_arr, x_arr.transpose())),x_arr), y_arr.transpose())
        self.angle = math.atan(w_lr[1])
        # print("ang 2 = " + str(self.angle))
        # w_lr = np.dot(np.dot(np.linalg.inv(np.outer(x_arr, x_arr)),x_arr)
        # print("ang 2 = " + str(w_lr[1]))

        # except:
        #     print("error")
        # print(str(w_lr))

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
            if self.angle > 0.05 or self.angle < -0.05:
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
        self.odometry = OdometryHandler()
        self.scaner = ScanerHandler(self.odometry)
        self.robot_pos = Point()
        self.pub = rospy.Publisher('points', PointStamped, queue_size=0)
        self.cells_list = []
        self.grid_size = 0.5  # in m
        self.size = 20
        self.angular_vel_counter = 1

    def check_new_data(self):
        if self.scaner.get_state() and not self.odometry.is_angular_vel():
            # self.odometry.lock_write()
            self.handle_laser_data(self.scaner.get_data())

        #     self.odometry.get_data()

    def handle_laser_data(self, laser_data):
        # if self.odometry.is_angular_vel():
        #     # self.odometry.unlock()
        #     return True

        yaw = self.odometry.get_yaw()
        las_pos = self.odometry.get_las_pos()
        # print(str(self.odometry.las_pos) + " " + str(las_pos))
        # self.odometry.unlock()

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
            x = math.cos(yaw) * x1 - math.sin(yaw) * y1 + las_pos.x
            y = math.sin(yaw) * x1 + math.cos(yaw) * y1 + las_pos.y
            point = Point()
            point.x = x
            point.y = y

            # tmpx = math.fabs(x)
            # tmpy = math.fabs(y)

            # print(point)
            point_list.append(point)
            cur_ang += resolution
            try:
                pos_grid = self.calculate_mod(point)

            except:
                # print("Blad")
                continue

            self.size = max(self.size, math.fabs(x)+0.5, math.fabs(y)+0.5)
            # print(self.size)
            # if tmpx > self.size or tmpy > self.size:
            #     if tmpx > tmpy:
            #         self.size = tmpx
            #     else:
            #         self.size = y

            # if x == 'inf' or x == '-inf' or y == 'inf' or y =='-inf':
            #     print("aa")
            #     continue

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
        try:
            self.plot_data()

        except:
            return
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
        # ax.set_xlim(-10, 10)
        # ax.set_ylim( -10,  10)
        ax.set_xlabel('Polozenie [m]')
        ax.set_ylabel('Polozenie [m]')
        ax.set_title('Mapa otoczenia')
        #
        # ax.set_xlim(-self.size, self.size)
        # ax.set_ylim(-self.size, self.size)

        ax.set_xlim(-10, 10)
        ax.set_ylim(-10,10)

        for cur_cell in self.cells_list:

            pos = cur_cell.get_average_position()
            dev = cur_cell.get_standard_deviation()
            # print(dev[0])
            ell = Ellipse(pos, ((dev[0])*3), ((dev[1])*3), angle=cur_cell.get_angle(), color='red')
            # print(ell)

            # ax = fig.add_subplot(111, aspect = 'equal')
            # ax.add_artist(ell)
            ax.add_artist(ell)

        # ell = Ellipse([self.robot_pos.x,self.robot_pos.y], 0.8, 0.5, angle=(self.odometry.get_yaw()* 180 / math.pi), color='blue')
        ax.add_artist(ell)
        plt.show()
        plt.pause(0.00001)
        # print("tak")
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


def start_data_handle():
    rospy.init_node('data_handle')
    robot_sim = Robot()
    robot_sim.start_simulation()


if __name__ == "__main__":
    start_data_handle()

