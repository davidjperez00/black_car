#!/usr/bin/env python
# File Name: tf_bd_listener_node.py
# File Brief: Listens to the tf transformation between the 'base_link'
#     'laser' frame and computes the minimum TTC.
# Date: 3/31/2023
# Author: David Perez

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64
from black_car.msg import TTC_Data
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry


class BroadCasterTTCNode(object):
    """
    Tf Broadcaster listener that gets the tf transformation from the 'laser' frame
    to the 'base_link' frame and calculates the miminum Time to Collision (TTC) based
    on the miminum disance of the lidar's '/scan' topic
    """
    def __init__(self):
        # Constants used for scan_callback of "/scan" topic for calculating TTC   
        self.laser_constants_set = False
        self.laser_angle_min = 0
        self.laser_angle_max = 0
        self.laser_angle_increment = 0
        self.laser_ranges_len = 0
        
        '''     Subscibers     '''
        # This code block continues to break our main node thread for 1 second looking
        # to get transformations but also causes errors when no 'timeout' is added
        # Hence why establishing the result of the transformation is used as
        # a constant instead:
        self.laser_base_link_x_trans = 0.275
        '''
        # Get's transformation between 'laser' frame and 'base_link' frame
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        # Getting the x-difference between the 'base_link' frame and 'laser' frame
        # this produces a value of 0.275
        self.laser_base_link_tf_listener = tfBuffer.lookup_transform('base_link', 'laser', rospy.Time(), timeout=rospy.Duration(1.0))
        self.laser_base_link_x_trans = self.laser_base_link_tf_listener.transform.translation.x
        '''
        
        # Topic used to retrieve lidar information
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Topic that publishes speed of vehicle
        # self.drive_subscriber = rospy.Subscriber("/drive", AckermannDriveStamped, self.drive_callback)
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self._vehicle_speed = 0 # Class level storage of current vehicle speed

        '''     Publishers     '''
        # Topic that publishes the position of TTC_min in 'laser' frame 
        self.pose_laser_frame_publisher = rospy.Publisher("/min_laser_pose", PoseStamped, queue_size=10)
        self.pose_laser_msg = PoseStamped()
        
        # Topic that publishes the position of TTC_min in 'base_link' frame
        self.pose_base_link_frame_publisher = rospy.Publisher("/min_base_link_pose", PoseStamped, queue_size=10)
        self.pose_base_link_msg = PoseStamped()
        
        self.ttc_data_publisher = rospy.Publisher("/ttc_data", TTC_Data, queue_size=10)
        self.ttc_data_msg = TTC_Data()

    def scan_callback(self, scan_msg):
        # Get constants of scan topic once
        if (self.laser_constants_set == False):
            self.laser_angle_min = scan_msg.angle_min
            self.laser_angle_max = scan_msg.angle_max
            self.laser_angle_increment = scan_msg.angle_increment
            self.laser_ranges_len = len(scan_msg.ranges)
            
            self.laser_constants_set = True
        
        # When vehicle is not moving TTC is infinity
        if (self._vehicle_speed == 0 or self._vehicle_speed == -0):
            return
        
        min_laser_distance = 0
        min_base_link_distance = 0
        current_angle = self.laser_angle_min # First index of lidar reading is at lidar's min angle
        min_angle = 0
        temp_TTC = 0
        min_ttc = 10000
        # Find the smallest lidar distance and it's corresponding angle
        for i in range(0, self.laser_ranges_len):
            # Getting velocity with respect to min lidar angled reading
            velocity_cos_x = self._vehicle_speed * (math.cos(current_angle))
            
            if (velocity_cos_x != 0 or velocity_cos_x != -0 ): 
                # Calculate min TTC for smallest lidar reading from 'laser' frame to'base_link' frame
                # First convert laser distance from laser frame to base_link frame:
                base_link_x = (self.laser_base_link_x_trans) + (scan_msg.ranges[i] * math.cos(min_angle))
                base_link_y = (scan_msg.ranges[i] * math.sin(min_angle))
                base_link_distance = math.sqrt((base_link_x ** 2) + (base_link_y ** 2))
                
                # If vehicle speed gets updated to zero we no longer
                # want to try to produce a ttc.
                try:
                    temp_TTC = base_link_distance / velocity_cos_x
                except:
                    continue
                
                if (temp_TTC > 0 and temp_TTC < min_ttc):
                    min_ttc = temp_TTC
                    min_angle = current_angle
                    # Used to generate PoseStamped for 'laser' frame
                    # and 'base_link' frame for shortest lidar distance
                    min_laser_distance = scan_msg.ranges[i]
                    min_base_link_distance = base_link_distance
    
            # increment angle for next iteration
            current_angle += self.laser_angle_increment
            
        # Publish ttc data for '/ttc_data' topic
        self.ttc_data_msg.ttc_min = min_ttc
        self.ttc_data_msg.ttc_min_angle = min_angle
        self.ttc_data_publisher.publish(self.ttc_data_msg)

        # Populating and Publishing the pose in 'laser' frame
        self.pose_laser_msg.header.stamp  = rospy.Time.now()
        self.pose_laser_msg.header.frame_id = 'laser'
        self.pose_laser_msg.pose.position.x = (min_laser_distance * math.cos(min_angle))
        self.pose_laser_msg.pose.position.y = (min_laser_distance * math.sin(min_angle))
        self.pose_laser_msg.pose.position.z = 0.0
        quaternion_laser_pose = tf_conversions.transformations.quaternion_from_euler(0, 0, min_angle)
        self.pose_laser_msg.pose.orientation.x = quaternion_laser_pose[0]
        self.pose_laser_msg.pose.orientation.y = quaternion_laser_pose[1]
        self.pose_laser_msg.pose.orientation.z = quaternion_laser_pose[2]
        self.pose_laser_msg.pose.orientation.w = quaternion_laser_pose[3]
        self.pose_laser_frame_publisher.publish(self.pose_laser_msg)
        
        # Populating and  Publishing the pose in 'base_link' frame
        self.pose_base_link_msg.header.stamp  = rospy.Time.now()
        self.pose_base_link_msg.header.frame_id = 'base_link'
        self.pose_base_link_msg.pose.position.x = (self.laser_base_link_x_trans) + (min_base_link_distance * math.cos(min_angle))
        self.pose_base_link_msg.pose.position.y = (min_base_link_distance * math.sin(min_angle))
        self.pose_base_link_msg.pose.position.z = 0.0
        quaternion_base_link_pose = tf_conversions.transformations.quaternion_from_euler(0, 0, min_angle)
        self.pose_base_link_msg.pose.orientation.x = quaternion_base_link_pose[0]
        self.pose_base_link_msg.pose.orientation.y = quaternion_base_link_pose[1]
        self.pose_base_link_msg.pose.orientation.z = quaternion_base_link_pose[2]
        self.pose_base_link_msg.pose.orientation.w = quaternion_base_link_pose[3]
        self.pose_base_link_frame_publisher.publish(self.pose_base_link_msg)
        
    
    # Callback for updating the vehicle speed
    def odom_callback(self, odom_msg):
        self._vehicle_speed = odom_msg.twist.twist.linear.x

            
            
def main():
    rospy.init_node('broadcaster_min_tcc_node')
    bd = BroadCasterTTCNode()
    rospy.spin()

if __name__ == '__main__':
    main()
    
