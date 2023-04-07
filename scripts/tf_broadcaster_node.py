#!/usr/bin/env python
# File Name: tf_broadcaster_node.py
# File Brief: Subscribe to laser scans and broadcast a static
#     transform the pose p from the laser frame to the 
#     base_link frame.
# Date: 04/26/2023
# Author: David Perez

import rospy
import math
from sensor_msgs.msg import LaserScan

# imports for tf transformations
import geometry_msgs.msg
import tf_conversions
import tf2_ros

class TfLaserBroadcaster(object):
    """
    This class generates a 'PoseStamped' message that converts
    the mininum laser scan distance and uses a tf transform to converts
    from the laser frame to the base_link frame.
    """
    def __init__(self):
        '''  Subscribers  '''
        # Topic used to retrieve lidar information
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Constants populated by '/scan' topic  
        self._laser_constants_set = False
        self._laser_angle_min = 0
        self._laser_angle_max = 0
        self._laser_angle_increment = 0
        self._laser_ranges_len = 0
        
        # Constant generated from command 'rosrun tf tf_echo laser base_link'
        self._x_laser_base_link_offset = 0.275

    def scan_callback(self, scan_msg):
        if (self._laser_constants_set == False):
            self._laser_angle_min = scan_msg.angle_min
            self._laser_angle_max = scan_msg.angle_max
            self._laser_angle_increment = scan_msg.angle_increment
            self._laser_ranges_len = len(scan_msg.ranges)
            self._laser_constants_set = True
           
        # Determine the smallest distance read from lidar
        # published on '/scan' topic 
        current_angle = self._laser_angle_min # min angle from '/scan'
        min_distance_angle = 0
        temp_min_distance = float('inf')
        for i in range(0, self._laser_ranges_len):
            if(scan_msg.ranges[i] < temp_min_distance):
                temp_min_distance = scan_msg.ranges[i]
                min_distance_angle = current_angle
                
            # increment angle for next iteration
            current_angle += self.laser_angle_increment
        
        '''  Creating and publishing transformation of minimum laser reading'''
        broadcaster = tf2_ros.TransformBroadcaster()
        transformer = geometry_msgs.msg.TransformStamped()
        
        transformer.header.stamp = rospy.Time.now()
        transformer.header.frame_id = "laser"
        transformer.child_frame_id = "base_link_laser"
        transformer.transform.translation.x = self._x_laser_base_link_offset + (temp_min_distance * math.cos(min_distance_angle))
        transformer.transform.translation.y = temp_min_distance * math.sin(min_distance_angle)
        transformer.transform.translation.z = 0.0
        
        broadcaster.sendTransform(transformer)
        

def main():
    rospy.init_node('TfLaserBroadcaster')
    sn = TfLaserBroadcaster()
    rospy.spin()

if __name__ == '__main__':
    main()