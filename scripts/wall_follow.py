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
kp = 3
kd = 0.4
prev_error = 0.0 
velocity = 0.0

#WALL FOLLOW PARAMS
DESIRED_DISTANCE_LEFT = 0.55

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        # Subscriber to the '/scan' that publishes lidar scan data
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        
        # Publisher to set the speed of the vehicle
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
        
        # Constants set by '/scan' topic with default values
        self.lidar_constants_set = False
        # 0 degree index of ranges array from lidar readings 
        self.directly_forward_lidar_index = 0 # populated with value of 539


    def pid_control(self, error):
        global prev_error
        global kp
        global kd
        global velocity
        angle = 0.0
        
        if (prev_error == 0.0):
            prev_error = error
            return 
        
        # Negative angle is used to divert the vehicle to trail left wall
        # since equations for right wall following were used
        angle = -((kp * error) + (kd * (error - prev_error)))
        
        # Speed control of vehicle for better performance on turns
        if (((angle > 0.0) and (angle <= 0.174533)) or ((angle < 0.00) and (angle >= -0.174533))):
            # in degrees: (0 < angle <= 10) or (0 > angle >= -10)
            velocity = 1.5
        elif (((angle > 0.174533) and (angle <= 0.349066)) or ((angle < -0.174533) and (angle >= -0.349066))): 
            # in degrees: (10 < angle <= 20) or (1-0 > angle >= -20)
            velocity = 1.0 
        else:
            velocity = 0.5
                        
        print("angle = ", angle)
        print("velocity = ", velocity)
        print("=============== one loop ================")
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        

    def getRange(self, lidar_data, angle):
        '''
            Retrieving the distance of a lidar scan based on the input angle.
            Note: 'lidar_data.ranges' is index where the 0 index represents the 
                  the lidar readings to the rightmost of the car, and vice versa for
                  the last index (1079).
            lidar_data: single message from topic /scan
            angle: The angle to the left of center forward of car.
            Output: Distance in meters of 'lidar_data' at 'angle'
        '''

        num_increments_for_left_angle = math.ceil(angle / lidar_data.angle_increment) 
        ranges_index_at_angle = int(self.directly_forward_lidar_index + num_increments_for_left_angle)

        distance = lidar_data.ranges[ranges_index_at_angle]
        
        if (math.isnan(distance)):
            distance = lidar_data.ranges[ranges_index_at_angle + 1]

        return distance


    def followLeft(self, lidar_data, leftDist):
        '''
            Calculating the error in the desired distance to follow
            the left wall in relation to the projected (and actual)
            distance from the left wall.
            
            lidar_data: Single message from topic /scan.
            leftDist: The desired distance to maintain from left wall
            Output: The error between the desired and actual distance from wall.
            
            Example of lidar readings for wall following algorithm:
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
        global velocity

        # Determnining the lidar_data.ranges[] index that contains the lidar
        # distance reading at an angle of 90 degrees to the left of the axis pointing directly
        # infront of the vehicle/
        directly_left_radian_angle = 1.5708 # 90 degrees in radians
        distance_b = self.getRange(lidar_data, directly_left_radian_angle)

        # Determining the lidar_data.ranges[] index that contains the lidar distance
        # reading at an angle of 30 degrees to the left of the axis pointing directly 
        # in front of the vehicle (this is index 539 of lidar ranges (angle 0)).
        radian_angle_distance_a = 0.523599 # 30 degrees  in radians
        distance_a = self.getRange(lidar_data, radian_angle_distance_a)
        
        angle_between_a_b = directly_left_radian_angle - radian_angle_distance_a # about 60 degrees
        
        # Angle from the center forward of vehicle
        alpha = math.atan((distance_a*math.cos(angle_between_a_b) - distance_b) / (distance_a*math.sin(angle_between_a_b)))
        # Calculating distance to the left wall at current time t
        D_t = distance_b * math.cos(alpha)
        
        # Calculating the estimated future distance of the vehicle:
        time_between_callbacks= 0.0015 # average time between callbacks in seconds
        projected_forward_distance = velocity * time_between_callbacks
        # Tacking on our projected distance to our current derived distance from left wall:
        D_t_plus_1 = D_t + (projected_forward_distance * math.sin(alpha))
        
        # Determining difference of our projected distance of wall
        # and our desired distance.
        error = leftDist - D_t_plus_1

        return error

    def lidar_callback(self, lidar_data):
        """ 
            Calculates the error between the vehicles projected and actual distance from
            the left wall in relation to the desired distance to maintain from the left wall.
            This then fees that error to a PD controller that determines an appropriate
            angle and speed to drive the vehicle at.
            
            lidar_data: Single message from topic /scan.
        """
        if (self.lidar_constants_set == False):
            self.directly_forward_lidar_index = (len(lidar_data.ranges)  / 2 ) - 1 # 539
            
            self.laser_constants_set = True    
                    
        error = self.followLeft(lidar_data, DESIRED_DISTANCE_LEFT)
        
        self.pid_control(error)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
