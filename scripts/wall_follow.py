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
# kp = #TODO
# kd = #TODO
kp = 0
kd = 0
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
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped)
        # self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # self.drive_pub = #TODO: Publish to drive
        
        
        # Constants set by '/scan' topic with default values
        self.lidar_constants_set = False
        self.laser_angle_min = 0
        self.laser_angle_max = 0
        self.laser_angle_increment = 0
        self.laser_ranges_len = 0
        # 0 degree index of ranges array from lidar readings 
        self.directly_forward_lidar_index = 0



    def getRange(self, lidar_data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.

        # CAN HANGLE INVALID DATA BY USING A LIDAR DATA READING +-X INDEXES FROM DESIRED INDEX
        #TODO: implement


        # Produces value of 270
        num_increments_for_left_angle = math.ceil(angle /lidar_data.angle_increment)
        ranges_index_at_angle = int(self.directly_forward_lidar_index - num_increments_for_left_angle) # 269
        # print(f"index, angle = {ranges_index_at_angle} {angle}" )
        print("index, angle = {} {}".format(ranges_index_at_angle, angle) )
        # Ensure that this is a valid index:
        distance = lidar_data.ranges[ranges_index_at_angle]

        return distance

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

        return angle

    def followLeft(self, lidar_data, leftDist):
        #Follow left wall as per the algorithm 
        
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
        directly_left_radian_angle = 1.5708 # 90 degrees
        distance_b = self.getRange(lidar_data, directly_left_radian_angle)
        print("distance_a", distance_b)

        # Determining the lidar_data.ranges[] index that contains the lidar distance
        # reading at an angle of 30 degrees to the left of the axis pointing directly 
        # in front of the vehicle (this is index 539 of lidar ranges (angle 0)).
        radian_angle_distance_a = 0.523599 # 30 degrees
        distance_a = self.getRange(lidar_data, radian_angle_distance_a)
        print("distance_b", distance_a)
        
        angle_between_a_b = directly_left_radian_angle - radian_angle_distance_a # about 60 degrees
        print("angle_between_a_b", angle_between_a_b)
        
        alpha = math.atan((distance_a*math.cos(angle_between_a_b) - distance_b) / (distance_a*math.sin(angle_between_a_b)))
        print("alpha = ", alpha)
        # Calculating distance to the left wall at current time t
        D_t = distance_b * math.cos(alpha)
        print("D_t = ", D_t)
        
        # Calculating the estimated future distance of the vehicle:
        # The approximated distance forward the vehicle has traveled:
        time_between_callbacks = 0.1 #POPULATE ME WITH CONSTANT?? OR FIND RATE HZ (about 60 maybe)??
        projected_forward_distance = VELOCITY * time_between_callbacks
        print("projected_forward_distance = ", projected_forward_distance)
        D_t_plus_1 = D_t + (projected_forward_distance * math.sin(alpha))
        
        print("D_t_plus_1 = ", D_t_plus_1)

        error = leftDist - D_t_plus_1

        return error

    def lidar_callback(self, lidar_data):
        """ 
        """
        if (self.lidar_constants_set == False):
            self.laser_angle_min = lidar_data.angle_min
            self.laser_angle_max = lidar_data.angle_max
            self.laser_angle_increment = lidar_data.angle_increment
            self.laser_ranges_len = len(lidar_data.ranges)
            self.directly_forward_lidar_index = (self.laser_ranges_len  / 2 ) - 1 # 539
            
            self.laser_constants_set = True        
        
        error = self.followLeft(lidar_data, DESIRED_DISTANCE_LEFT)
        print("Value of 'error' = ", error)
        
        print("=========== lidar loop ===============")
        # self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
