#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import QuaternionStamped
import matplotlib.pyplot as plt

import csv
from pyproj import Transformer
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray

from Status import Carstatus
from PID import PID

transformer = Transformer.from_crs('epsg:4326', 'epsg:5178')
class AutonomousDriver():
    DRIVING_MODE_MANUAL           = -1
    DRIVING_MODE_GNSS             =  0
    # 120000/11pi
    def __init__(self) -> None:
        rospy.init_node('icelab')
        rospy.Subscriber('/filter/positionlla', Vector3Stamped, self.position_callback)
        rospy.Subscriber('/filter/quaternion', QuaternionStamped, self.quat_callback)
        rospy.Subscriber('icejoy', Float32MultiArray, self.cb_receiver)
        #rospy.Subscriber('/sensors/core', VescStateStamped, self.cb_vesc)
        #rospy.Subscriber('/commands/motor/speed', Float64, self.speed_callback)
        self.motor_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
        self.steering_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=10) # 0(좌회전) ~ 1(우회전)
        self.log_steering_pub = rospy.Publisher('/icelab/steer', Float64, queue_size=10) # 0(좌회전) ~ 1(우회전)

        self.loop_hz = 50


        self.my_car = Carstatus()
        self.volt = 0
        self.stanley_k = 0.5
        self.minimum_idx = 0
        self.slope_heading = 0
        self.stanley_error = 0
        self.x_list, self.y_list = [], []
        self.len_list = 0
        self.target_x = 0
        self.target_y = 0
        self.load_path('/home/jun/catkin_ws/src/icecar/src/utm-k_path.csv')
        self.LOOK_AHEAD_DIST = 0.1
        self.pid_steer = PID(1,0.1,0.005,1/self.loop_hz,0.5)
        self.manual_angle, self.manual_speed = 0, 0

    def position_callback(self, data):
        self.my_car.x, self.my_car.y = transformer.transform(data.vector.x, data.vector.y)

    def quat_callback(self, data):
        quaternion = (
            data.quaternion.x,
            data.quaternion.y,
            data.quaternion.z,
            data.quaternion.w
        )
        euler_angles = euler_from_quaternion(quaternion)
        self.my_car.heading = self.nomarlize_pi(- euler_angles[2] + np.pi/2) # radian
        
    
    def cb_vesc(self,data):
        self.volt = data.state.voltage_input

    def cb_receiver(self,data):
        self.manual_angle = np.interp(data.data[0], [990,2000], [0,1])
        self.manual_speed = np.interp(data.data[1],[998,1995], [-30000,30000] )

    def motor_publish(self, angle: float, speed: float) -> None:
        self.steering_pub.publish(angle)
        self.motor_pub.publish(speed)
        self.my_car.v = speed/120000*(11*np.pi)

    def main_loop(self) -> None:
        rate = rospy.Rate(self.loop_hz)
        self.my_car.angle, self.my_car.speed = 0.5, 0
        self.driving_mode = AutonomousDriver.DRIVING_MODE_MANUAL

        # self.gnss.load_path()
        while not rospy.is_shutdown():
            if self.driving_mode == AutonomousDriver.DRIVING_MODE_MANUAL:
                angle = self.stanley_control()
                self.log_steering_pub.publish(angle)
                
                self.motor_publish(angle, self.manual_speed)
                rospy.loginfo(f"e_x: {self.my_car.x - self.target_x}, e_y: {self.my_car.y - self.target_y}")
                #self.update_plot()
                #rospy.loginfo(f'speed: {self.manual_speed/120000*(11*np.pi):5.2f}m/s, angle: {angle:5.2f}, c_angle={angle:5.2f}, volt: {self.volt:5.1f}, stanley: {self.stanley_error:2.3f}')
            rate.sleep()


    def load_path(self, file_name):
        with open(file_name, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader, None)
            for row in csvreader:                
                self.x_list.append(float(row[0]))
                self.y_list.append(float(row[1]))
        self.len_list = len(self.x_list) - 1
        rospy.loginfo("-----Path loaded----------")
        plt.figure()
        plt.ion()
        plt.show()
    
    def update_plot(self):

        plt.clf()
        plt.scatter(self.x_list, self.y_list,s=0.01,label='map', color='blue')

        plt.scatter(self.my_car.x, self.my_car.y, label='Current Position')
        arrow_length = 1
        arrow_end_x = self.my_car.x + arrow_length * np.cos(self.my_car.heading)
        arrow_end_y = self.my_car.y + arrow_length * np.sin(self.my_car.heading)
        plt.arrow(self.my_car.x, self.my_car.y, arrow_end_x-self.my_car.x, arrow_end_y-self.my_car.y, head_width = 0.5, head_length=0.5, fc='red', ec='red')
        plt.scatter(self.target_x, self.target_y, c='red', label='target Point')
        slope_x = self.target_x + (arrow_length + 0.2) * np.cos(self.slope_heading)
        slope_y = self.target_y + (arrow_length + 0.2) * np.sin(self.slope_heading)
        plt.arrow(self.target_x, self.target_y, slope_x - self.target_x, slope_y - self.target_y, head_width = 0.3, head_length = 0.6, fc='green', ec='green')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.legend()
        plt.title('Position Plot')
        plt.grid(True)
        plt.pause(0.01)

    #----- Path tracking -------#
    def calc_ahead_point(self):
        global LOOK_AHEAD_DIST
        dx = self.LOOK_AHEAD_DIST * np.cos(self.my_car.heading)
        dy = self.LOOK_AHEAD_DIST * np.sin(self.my_car.heading)

        ahead_x = self.my_car.x + dx
        ahead_y = self.my_car.y + dy

        return ahead_x, ahead_y

    def calc_nearest_point(self, ahead_x, ahead_y):
        self.minimum_idx = 0
        minimum_dist = 1e10
        for i, (rx, ry) in enumerate(zip(self.x_list, self.y_list)):
            dist = math.dist((ahead_x, ahead_y), (rx, ry))            
            if (dist < minimum_dist):
                minimum_dist = dist
                self.minimum_idx = i
        return (self.x_list[self.minimum_idx], self.y_list[self.minimum_idx])
    #----- Path tracking -------#

    def nomarlize_pi(self, data):
        if data > np.pi:
            data = -2*np.pi + data
        elif data < -np.pi:
            data = 2*np.pi + data
        return data
    def control(self):
        ahead_x, ahead_y = self.calc_ahead_point()
        target_x, target_y = self.calc_nearest_point(ahead_x, ahead_y)

        dx, dy = target_x - self.my_car.x, target_y - self.my_car.y
        target_yaw = math.atan2(dy, dx)

        yaw_error = self.nomarlize_pi(self.my_car.heading - target_yaw)
        
        
        steer = 0.5 + self.pid_steer.do(yaw_error)
        return steer

    def stanley_control(self):
        self.target_x, self.target_y = self.calc_nearest_point(self.my_car.x, self.my_car.y)
        dx, dy = self.my_car.x - self.target_x , self.my_car.y - self.target_y

        self.Calc_slopeofpath()

        cte = - np.dot([dx, dy], [np.cos(self.my_car.heading + np.pi/2), np.sin(self.my_car.heading + np.pi/2)])
        
        cross_track_steering = np.arctan(self.stanley_k * cte / (self.my_car.v + 1e-6))

        heading_error = self.nomarlize_pi(self.slope_heading - self.my_car.heading)
        
        self.stanley_error = self.nomarlize_pi(cross_track_steering + heading_error)
        
        steer = 0.5 + self.pid_steer.do(self.stanley_error)
        #steer = 0.5 + self.stanley_error
        #rospy.loginfo((f'stanley_st:{steer:5.2f}, path:{self.slope_heading:5.2f}, myheading:{self.my_car.heading:5.2f}, h_err:{heading_error:5.2f},cte:{cte:5.2f}, cts:{cross_track_steering:5.2f} '))
       
        return steer
    


    # Calculate slope of current path
    def Calc_slopeofpath(self):
        idx_1 = self.minimum_idx
        if (self.minimum_idx + 1) > self.len_list:
            idx_2 = 0
        else:
            idx_2 = self.minimum_idx + 1
        
        x_1, y_1 = self.x_list[idx_1], self.y_list[idx_1]
        x_2, y_2 = self.x_list[idx_2], self.y_list[idx_2]

        dx = x_1 - x_2
        dy = y_1 - y_2

        self.slope_heading = self.nomarlize_pi(- np.arctan2(dx , dy) - np.pi/2)
        

    
        
if __name__ == '__main__':    
    ads = AutonomousDriver()
    ads.main_loop()