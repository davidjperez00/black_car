#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

class Safety(object):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        # This constant value is used as the velocity for calculating TTC
        # but also for setting the speed of the vehicle
        self.vehicle_speed = 1.25

        # Topic used to retrieve libar information
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # immediatley initiate a nonzero velocity once node is launched
        self.f1tenth_driving_publisher = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0", AckermannDriveStamped, queue_size=10)
        self.f1tenth_drive_msg = AckermannDriveStamped()
        self.f1tenth_drive_msg.drive.speed = self.vehicle_speed # set speed to x meters per second
        self.f1tenth_driving_publisher.publish(self.f1tenth_drive_msg)

        # Data to be retrieved for bag files
        self.ttc_vals_publisher = rospy.Publisher("/ttc_vals", Float64, queue_size=10)
        self.ttc_vals_msg = Float64()
        self.ttc_threshold_publisher = rospy.Publisher("/ttc_threshold", Float64, queue_size=10)
        self.ttc_threshold_msg = Float64()

        # Constants used for scan_callback of "/scan" topic for calculating TTC   
        self.laser_constants_set = False
        self.laser_angle_min = 0
        self.laser_angle_max = 0
        self.laser_angle_increment = 0
        self.laser_ranges_len = 0


    def scan_callback(self, scan_msg):
        # Continuously set vehicle speed
        self.f1tenth_driving_publisher.publish(self.f1tenth_drive_msg)
        # Continuously publish ttc vals
        self.ttc_threshold_publisher.publish(self.ttc_threshold_msg)

        # When vehicle is not moving TTC is infinity
        if (self.f1tenth_drive_msg.drive.speed == 0.0):
            return

        # Get constants of scan topic once
        if (self.laser_constants_set == False):
            self.laser_angle_min = scan_msg.angle_min
            self.laser_angle_max = scan_msg.angle_max
            self.laser_angle_increment = scan_msg.angle_increment
            self.laser_ranges_len = len(scan_msg.ranges)
            self.laser_constants_set = True

            # Setting TTC threshold value
            self.ttc_threshold_msg.data = 0.77

        current_angle = self.laser_angle_min # min angle of LaserScan

        # loop for number or items in scan_msg.ranges
        min_ttc = 10000 # for keeping track of temp ttc, arbitrary initial val
        temp_TTC = 0
        for i in range(0, self.laser_ranges_len):
            velocity_cos_x = self.vehicle_speed * (math.cos(current_angle))
            
            # increment angle for next iteration
            current_angle += self.laser_angle_increment
                        
            # When the cars not moving the time to collision is infinity
            if (velocity_cos_x != 0 or velocity_cos_x != -0 ): 
                temp_TTC = scan_msg.ranges[i] / velocity_cos_x


                if (temp_TTC > 0 and temp_TTC < min_ttc):
                    min_ttc = temp_TTC

                # Threshold for stoping car based on TTC
                # Negative TTC means car is moving away from object
                if (temp_TTC > 0 and temp_TTC < 0.77):
                    # Set the vehicle speed to zero
                    self.f1tenth_drive_msg.drive.speed = 0.0
                    self.f1tenth_driving_publisher.publish(self.f1tenth_drive_msg)

                    # publish TTC on last iteration
                    self.ttc_vals_msg.data = temp_TTC
                    self.ttc_vals_publisher.publish(self.ttc_vals_msg)
                    
                    return

        # Publishing min TTC value for bag file
        self.ttc_vals_msg.data = min_ttc
        self.ttc_vals_publisher.publish(self.ttc_vals_msg)
        

def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()
    
