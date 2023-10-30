#!/usr/bin/env python3
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

TIME_STEP = 0.001

class Hover:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.twist = Twist()
        
        self.controle = self.PID(kp = 0.1, ki = 50, kd = 0.00075, target = 320)
        self.velang_z = 0
        

    
    class PID(object):
        def __init__(self, kp, ki, kd, target):
            self.kp = kp
            self.kd = kd
            self.ki = ki
            self.setpoint = target
            self.error = 0
            self.integral_error = 0
            self.error_last = 0
            self.derivate_error = 0
            self.output = 0
        def compute(self, pos):
            self.error = -(self.setpoint - pos)/20
            print()
            self.integral_error += self.error * TIME_STEP
            self.derivate_error = (self.error - self.error_last)/TIME_STEP
            self.error_last = self.error
            self.output = self.kp * self.error + self.kd * self.derivate_error + self.ki * self.integral_error
            return self.output

rospy.init_node('hover')
hover = Hover()
rospy.spin()
