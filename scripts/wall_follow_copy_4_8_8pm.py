#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = #TODO
kd = #TODO
ki = 0.0003
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav' # think this is /drive

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)
        
        # Topic that publishes speed of vehicle
        # self.drive_subscriber = rospy.Subscriber("/drive", AckermannDriveStamped, self.drive_callback)
        # self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.drive_pub = #TODO: Publish to drive
        
        
        # Constants set by '/scan' topic 
        self.lidar_constants_set = False



    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        return 0.0

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        angle = 0.0
        #TODO: Use kp, ki & kd to implement a PID controller for 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement

        

        return 0.0 

    def lidar_callback(self, lidar_data):
        """ 
        """
        
        if (self.lidar_constants_set == False):
            self.laser_angle_min = lidar_data.angle_min
            self.laser_angle_max = lidar_data.angle_max
            self.laser_angle_increment = lidar_data.angle_increment
            self.laser_ranges_len = len(lidar_data.ranges)
            
            self.laser_constants_set = True        
            
        
        # Get the distance directly to the left of the car 
        '''
            | < 
            |  \
            |   \   a
            |    \
            |     \  
            |      \     
            |       \
            |        \
            | <-------[ ^ ]
            |   b     [car]      
        '''  
        
        # Determnining the lidar_data.ranges[] index that contains the lidar
        # distance reading at an angle of 90 degrees to the left of the axis pointing directly
        # infront of the vehicle
        directly_forward_lidar_index = (self.laser_ranges_len  / 2 ) - 1 # 539
        directly_left_radian_angle = 1.5708 # 90 degrees
        # Produces value of 270
        num_increments_for_left_angle = math.ceil(directly_left_radian_angle /self.laser_angle_increment)
        self.directly_left_ranges_index = directly_forward_lidar_index - num_increments_for_left_angle # 269
        
        distance_a = lidar_data.ranges[self.directly_left_ranges_index]
        
        # Determining the lidar_data.ranges[] index that contains the lidar distance
        # reading at an angle of 30 degrees to the left of the axis pointing directly 
        # in front of the vehicle (this is index 539 of lidar ranges (angle 0)).
        radian_angle_distance_b = 0.523599 # 30 degrees
        num_increments_for_left_angle_b = math.ceil(radian_angle_distance_b / self.laser_angle_increment)
        self.distance_b_ranges_index = directly_forward_lidar_index - num_increments_for_left_angle_b #
        
        distance_b = lidar_data.ranges[self.distance_b_ranges_index]
        
        angle_between_a_b = directly_left_radian_angle - radian_angle_distance_b # about 60 degrees
        
        alpha = math.atan((distance_a*math.cos(angle_between_a_b) - distance_b) / (distance_a*math.sin(angle_between_a_b)))
        
        # Calculating distance to the left wall at time t
        D_t = distance_b * math.cos(alpha)
        
        # Calculating the estimated future distance of the vehicle:
        # The approximated distance forward the vehicle has traveled:
        time_between_callbacks = 0 #POPULATE ME WITH CONSTANT?? OR FIND RATE HZ (about 60 maybe)??
        projected_forward_distance = VELOCITY * time_between_callbacks
        D_t_plus_1 = D_t + (projected_forward_distance) * math.sin(alpha)
        
        
        error = DESIRED_DISTANCE_LEFT - D_t_plus_1
        
        error = 0.0 #TODO: replace with error returned by followLeft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
