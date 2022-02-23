#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
    

    def __init__(self):
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 20)
        self.sub = rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.callback)

        self.viz = rospy.Publisher("/wall", Marker, queue_size = 1)

        self.avg_sum = None
        self.avg_num = 0

    # TODO:
    # Write your callback functions here.
    
    def slice_scan(self, angle_min, angle_max, scan):
        inc = scan.angle_increment
        iL = int(round((angle_min - scan.angle_min)/inc))
        iH = int(round((angle_max - scan.angle_min)/inc))

        start = scan.angle_min + iL*inc
        stop = scan.angle_min + (iH + 1)*inc
        angles = np.arange(start, stop, inc)
        ranges = scan.ranges[iL:iH+1]
        return angles, ranges

    def find_wall(self, angles, ranges):
        x = np.cos(angles)*ranges
        y = np.sin(angles)*ranges

        if x.size == 0:
           return np.array([0]), np.array([0, self.DESIRED_DISTANCE]) 
        
        dists = np.sqrt(np.square(y) + np.square(x))


        if self.avg_sum is None:
            self.avg_sum = np.average(dists)
            self.avg_num = 1

        else:
            self.avg_sum = self.avg_sum*self.avg_num + np.average(dists)
            self.avg_num = self.avg_num + 1
            self.avg_sum = self.avg_sum/self.avg_num

        print(self.avg_sum)
        indys = np.where((dists/self.avg_sum) > 1.3)[0]
        y_filt = np.delete(y, indys)
        x_filt = np.delete(x, indys)
        
        
        if x_filt.size > 0:
            for i in indys:
                idx = (np.abs(x_filt - x[i])).argmin()
                y[i] = y_filt[idx]
                x[i] = x_filt[idx]


        return x, np.polyfit(x, y, 1)

    def control(self, coeff):
        Kd = 4.2
        Kp = 6.9
        err = -self.SIDE*self.DESIRED_DISTANCE + coeff[1]
        return (Kp*err + Kd*self.VELOCITY*coeff[0])


    def callback(self, scan):
        slice_angle_o = -np.pi/2 if self.SIDE == -1 else 0
        slice_angle_f = 0 if self.SIDE == -1 else np.pi/2
        angles, ranges = self.slice_scan(slice_angle_o, slice_angle_f, scan)

        x, coeff = self.find_wall(angles, ranges)

        VisualizationTools.plot_line(x, coeff[0]*x + coeff[1], self.viz, frame = "/laser")
        inpt = self.control(coeff)

        levi = AckermannDriveStamped()
        levi.drive.speed = self.VELOCITY
        levi.drive.steering_angle = inpt
        levi.drive.acceleration = 0
        levi.drive.steering_angle_velocity = 0
        levi.drive.jerk = 0
        levi.header.stamp = rospy.Time.now()
        self.pub.publish(levi)


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
