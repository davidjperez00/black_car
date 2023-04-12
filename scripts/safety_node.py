#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool

# Custom message
from black_car.msg import TTC_Data

class Safety(object):
    """
    The class that handles emergency braking. This nodes gets ttc data from
    the '/ttc_data' topic and uses a threshold to determine when to stop the vehicle.
    """
    def __init__(self):
        
        # Class constants
        self.TTC_FRONT_THRESHOLD = 0.8
        self.TTC_SIDE_THRESHOLD = 0.6
        
        '''     Subscibers     '''
        self.scan_subscriber = rospy.Subscriber("/ttc_data", TTC_Data, self.tcc_data_callback)
        
        
        '''     Publishers     '''
        self.drive_subscriber = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        
        self.brake_subscriber = rospy.Publisher("/brake", AckermannDriveStamped, queue_size=10)
        self.brake_msg = AckermannDriveStamped()
        
        self.brake_bool_subscriber = rospy.Publisher("/brake_bool", Bool, queue_size=10)
        self.brake_bool_msg = Bool()
        

    def tcc_data_callback(self, ttc_data_msg):
        # In case there is an instance where the ttc could be zero.
        # We also want to neglect negative ttc values since this means the vehicle 
        # is moving away from the nearest object.
        if ((ttc_data_msg.ttc_min != 0) or (ttc_data_msg.ttc_min != -0)):
            
            if (ttc_data_msg.ttc_min > 0):
                # Extra credit implementation. If the ttc angle is between is in said
                # range (i.e. its on the side of the vehicle) then don't stop the car.
                # This ensures that if the vehicle is driving next to a wall a different threshold
                # will be used to stop the vehicle which will be smaller than that of 
                # the ttc for the front of the vehicle. The cut-off used is if the ttc angle 
                # is greater than +- 90 degrees (or 1.5708 radians)
                if ((ttc_data_msg.ttc_min_angle > 1.5708) or (ttc_data_msg.ttc_min_angle < -1.5708)):
                    
                    if (ttc_data_msg.ttc_min < self.TTC_SIDE_THRESHOLD):        
                        self.drive_msg.drive.speed = 0 # stop the vehicle
                        self.drive_subscriber.publish(self.drive_msg)
                        
                        self.brake_msg.drive.speed = 0 # stop the vehicle
                        self.brake_subscriber.publish(self.brake_msg)
                        
                        self.brake_bool_msg.data = True # stop the vehicle
                        self.brake_bool_subscriber.publish(self.brake_bool_msg)
                        
                        return 
                
                if (ttc_data_msg.ttc_min < self.TTC_FRONT_THRESHOLD):
                    self.brake_msg.drive.speed = 0 # stop the vehicle
                    self.brake_subscriber.publish(self.brake_msg)
                    
                    self.brake_bool_msg.data = True # stop the vehicle
                    self.brake_bool_subscriber.publish(self.brake_bool_msg)
                    
                    self.drive_msg.drive.speed = 0 # stop the vehicle
                    self.drive_subscriber.publish(self.drive_msg)
                    
                    return 


def main():
    rospy.init_node('safety_node')
    sn = Safety()
    rospy.spin()

if __name__ == '__main__':
    main()
    
