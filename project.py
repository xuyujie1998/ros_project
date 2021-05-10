#! /usr/bin/env python

import sys
import rospy

import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        # print("================================= CREATED ObstacleAvoidance CLASS =================================")
        self.D_MAX = 1.0     # MAX_OBSTACLE_DISTANCE detection

        self.LV_MIN = 0.2     # MIN_LINEAR_VELOCITY
        self.LV_MAX = 2.6     # MAX_LINEAR_VELOCITY

        self.AV_MIN = 0.3     # MAX_ANGULAR_VELOCITY
        self.AV_MAX = 1.82     # MAX_ANGULAR_VELOCITY
        self.AV_FACTOR = 0.3    # adjuest performance

        self.DEFAULT_LV = 0.5    # DEFAULT_LINEAR_VELOCITY
        self.DEFAULT_AV = 0.0    # DEFAULT_ANGULAR_VELOCITY

        
        
        self.sub = rospy.Subscriber('scan', LaserScan, self.laserScanCallback)  # define subscriber with name "scan"
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)   # define publisher with name "cmd_vel" # queue: cache of message
        
        self.move = Twist() # get velocity info
        self.run()
        
    def run(self):
        rospy.init_node('obstacle_avoidance', anonymous=True)   # initialize node
        self.printDefaultValues()   # print default parameter values
        rospy.spin()

    def printDefaultValues(self):   # print default parameter values
        print('D_MAX = ' + str(self.D_MAX) + '\t# MAX_OBSTACLE_DISTANCE detection')
        print('LV_MAX = ' + str(self.LV_MAX) + '\t# MAX_LINEAR_VELOCITY')
        print('LV_MIN = ' + str(self.LV_MIN) + '\t# MIN_LINEAR_VELOCITY')
        print('AV_MAX = ' + str(self.AV_MAX) + '\t# MAX_ANGULAR_VELOCITY')
        print('AV_MIN = ' + str(self.AV_MIN) + '\t# MIN_ANGULAR_VELOCITY')
        print('DEFAULT_LV = ' + str(self.DEFAULT_LV) + '\t# DEFAULT_LINEAR_VELOCITY')
        print('DEFAULT_AV = ' + str(self.DEFAULT_AV) + '\t# DEFAULT_ANGULAR_VELOCITY')
        
    # Input:
    #   d: The distance from bot to obstacle
    #   LV: Current bot linear velocity
    #   angle: angle between this beam (obstacle) and X axis
    # Output:
    #   LV: New bot linear velocity
    def getLinearVelocity(self, d, LV, angle):
        
        LV = self.LV_MIN + (LV - self.LV_MIN) * d * math.sin(math.radians(angle)) / self.D_MAX
        print('\tLV=' + str(LV))
        return LV

    # Input:
    #   d: The distance from bot to obstacle
    #   angle: angle between this beam (obstacle) and X axis
    # Output:
    #   AV: New bot angular velocity
    def getAngularVelocity(self, d, angle):
        
        AV = self.AV_FACTOR * (self.AV_MIN + (self.AV_MAX - self.AV_MIN) * (1 - d * math.sin(math.radians(angle)) / self.D_MAX))
        print('\tAV=' + str(AV))
        return AV

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

        if cloest_obtacle_distance < self.D_MAX:     # If found any obstacle
            LV = self.DEFAULT_LV
            AV = self.DEFAULT_AV
            d = cloest_obtacle_distance
            angle = cloest_obtacle_angle
            print()
            print('FOUND CLOSET OBSTACLE!! Distance:' + str(d) + ', Angle:' + str(angle) + ', Linear Velocity:' + str(LV) + ', Angular Velocity:' + str(AV))

            if angle >= 0 and angle < 90:
                LV = self.getLinearVelocity(d, LV, angle)
                AV = 0 - self.getAngularVelocity(d, angle)
            elif angle >= 90 and angle < 180:
                LV = self.getLinearVelocity(d, LV, 180-angle)
                AV = self.getAngularVelocity(d, 180-angle)
            elif angle >= -90 and angle < 0:
                LV = self.getLinearVelocity(d, LV, 0-angle)
                AV = self.getAngularVelocity(d, 0-angle)
            elif angle >= -180 and angle < -90:
                LV = self.getLinearVelocity(d, LV, 180+angle)
                AV = 0 - self.getAngularVelocity(d, 180+angle)
            self.move.linear.x = LV
            self.move.angular.z = AV
            print('ACTION!! LV:' + str(LV) + ', AV:' + str(AV))

        self.pub.publish(self.move)
    
def main():
    try:
        odom = ObstacleAvoidance()
    except KeyboardInterrupt:
        sys.exit()

if __name__ == '__main__':
    main()