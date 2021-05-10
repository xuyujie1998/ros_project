#! /usr/bin/env python

import sys
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        print("================================= CREATED ObstacleAvoidance CLASS =================================")
        self.D_MAX = 1.0     # MAX_OBSTACLE_DISTANCE detection
        self.LV_MAX = 2.6     # MAX_LINEAR_VELOCITY
        self.AV_MAX = 1.82     # MAX_ANGULAR_VELOCITY

        self.DEFAULT_LV = 0.5    # DEFAULT_LINEAR_VELOCITY
        self.DEFAULT_AV = 0.0    # DEFAULT_ANGULAR_VELOCITY
        
        self.sub = rospy.Subscriber('scan', LaserScan, self.laserScanCallback)  # define subscriber with name "scan"
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        self.move = Twist()
        
        self.run()
        
    def run(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)   # initialize node
        self.printDefaultValues()   # print default parameter values
        rospy.spin()

    def printDefaultValues(self):   # print default parameter values
        print('D_MAX = ' + str(self.D_MAX) + '\t# MAX_OBSTACLE_DISTANCE detection')
        print('LV_MAX = ' + str(self.LV_MAX) + '\t# MAX_LINEAR_VELOCITY')
        print('AV_MAX = ' + str(self.AV_MAX) + '\t# MAX_ANGULAR_VELOCITY')
        print('DEFAULT_LV = ' + str(self.DEFAULT_LV) + '\t# DEFAULT_LINEAR_VELOCITY')
        print('DEFAULT_AV = ' + str(self.DEFAULT_AV) + '\t# DEFAULT_ANGULAR_VELOCITY')


    # Input:
    #   msg: Data returned from callback function
    # Output:
    #   AV: New bot angular velocity
    def laserScanCallback(self, msg):
        self.move.linear.x = self.DEFAULT_LV
        self.move.angular.z = self.DEFAULT_AV

        sys.stdout.write('.') # write dots
        sys.stdout.flush()

        cloest_obtacle_distance = self.D_MAX
        cloest_obtacle_angle = 0

        # All beams used for detecting obstacles with difference of 5 degree (except the back one)
        for angle in range(-175, 175, 5):
            distance = round(msg.ranges[angle], 4)    # Distance to any obstacle along the current beam. Infinit if no obstacle was found
            if distance < cloest_obtacle_distance:     # If found any obstacle
                cloest_obtacle_distance = distance
                cloest_obtacle_angle = angle

        print('cloest_obtacle_distance = ' + str(cloest_obtacle_distance))
        print('cloest_obtacle_angle = ' + str(cloest_obtacle_angle))

        self.pub.publish(self.move)
    
def main():
    try:
        odom = ObstacleAvoidance()
    except KeyboardInterrupt:
        sys.exit()

if __name__ == '__main__':
    main()