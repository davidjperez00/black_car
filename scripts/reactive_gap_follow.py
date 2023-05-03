#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

kp = 0.5
kd = 0.2

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        self.lidar_sub = rospy.Subscriber(lidarscan_topic,LaserScan,self.lidar_callback) 
        self.drive_pub = rospy.Publisher(drive_topic,AckermannDriveStamped,queue_size=5)
        
        self.prev_error = 0
        self.angle_increment = 0
    	

    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        # Remove instances that are greater than +- 100 degrees from center
        proc_ranges = np.array(ranges[239:839])
        
        # Convert instances that have a distance greater than 2m to NaN
        proc_ranges[proc_ranges > 2.0] = np.NaN
        
        # Convert infinite values to NaN's
        proc_ranges[proc_ranges == np.Inf] = np.NaN

        return proc_ranges

    def find_max_gap(self, proc_ranges, closest_index, bubble_radius):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        start_index = end_index = 0
        if (len(proc_ranges[0:closest_index-bubble_radius]) > len(proc_ranges[closest_index+bubble_radius:len(proc_ranges) - 1])):
            start_index = 0
            end_index = closest_index - bubble_radius
        else:
            start_index = closest_index + bubble_radius
            end_index = len(proc_ranges) - 1
            
        return start_index, end_index
    
    def find_best_point(self, start_i, end_i, proc_ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """

        steering_angle_index = np.nanargmax(proc_ranges[start_i:end_i]) + start_i

        return steering_angle_index
    
    def bubble_lidar_closest_distance(self, bubble_radius,  proc_ranges, closest_index):
        ''' Settings lidar readings to zero that are within radius of car
            at the closest lidar reading index
        '''
        
        proc_len = len(proc_ranges)
        for i in range(1, bubble_radius +1):
            if ((closest_index - i) >= 0):
                proc_ranges[closest_index - i] = 0.0 
            if ((closest_index + i) <= proc_len - 1):
                proc_ranges[closest_index + i] = 0.0
                
    def drive_pid_control(self, steering_index, closest_distance, proc_ranges):
        '''
            Determine the speed and steering angle of the vehicle
        '''
        global kp
        global kd
        
        center_index = math.floor(len(proc_ranges) / 2)
        index_difference = steering_index - center_index
        steering_index_angle_from_center = index_difference * self.angle_increment

        if (self.prev_error == 0.0):
            self.prev_error = steering_index_angle_from_center
        
        steering_angle = kp *(steering_index_angle_from_center) + kd*(steering_index_angle_from_center - self.prev_error)

        if (closest_distance < 0.1):
            vehicle_speed = 0.0
        
        vehicle_speed = 1.75
         
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = vehicle_speed
        self.drive_pub.publish(drive_msg)
        
        # Update previous error for next iteration
        self.prev_error = steering_index_angle_from_center
        
        
    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        
        ranges = data.ranges
        self.angle_increment = data.angle_increment

        # Setting max distance and handling invalid data
        proc_ranges = self.preprocess_lidar(ranges)

        # Find closest point to LiDAR
        closest_index = np.nanargmin(proc_ranges)
        closest_distance = proc_ranges[closest_index]
        
        # Eliminate all points inside 'bubble' (set them to zero)
        bubble_radius = 10
        self.bubble_lidar_closest_distance(bubble_radius, proc_ranges, closest_index)

        # Find the index of the largest gap on nonzero values
        start_index, end_index = self.find_max_gap(proc_ranges, closest_index, bubble_radius)
        
        # Determine the index that contains the largest distance
        # from lidar reading
        steering_angle_index = self.find_best_point(start_index, end_index, proc_ranges)
        
        # Determine the speed and angle to drive the vehicle
        self.drive_pid_control(steering_angle_index, closest_distance, proc_ranges)
        

def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)