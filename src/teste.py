
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from pid import PID

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Inicialize os parâmetros do PID
        self.pid = PID(0.1, 0.01, 0.05)
        self.pid.setpoint = 320  # Ajuste este valor de acordo com a posição da linha na imagem
        self.pid.output_limits = (-1, 1)  # Limites da velocidade angular

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Converta a imagem em escala de cinza e aplique um filtro para detectar a linha preta
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY)

        # Encontre os contornos na imagem binária
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Encontre o contorno mais longo (a linha preta)
            longest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(longest_contour)

            # Calcule o centro da linha preta
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Calcule o erro (diferença entre o centro da linha e o ponto de referência)
                error = cx - self.pid.setpoint

                # Aplique o PID para calcular a velocidade angular
                angular_vel = self.pid(error)

                # Crie a mensagem Twist e publique
                twist = Twist()
                twist.angular.z = angular_vel
                self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = LineFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass








    #!/usr/bin/env python3

import rospy
import cv2 as cv
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist


bridge = CvBridge()

class Sensores:

    def __init__(self):

        self.sensores = np.array([-1, -1, -1, -1, -1])
        self.publish_time = 0.05

        self.img_sensor_full_left = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imgFullLeftCallback, queue_size = 10)
        self.sensor_pub = rospy.Publisher("/dados", Int32MultiArray, queue_size=10)
        self.timer_pub = rospy.Timer(rospy.Duration(self.publish_time), self.timerCallback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        rospy.logwarn("Rodando") 
        self.cmd_vel_msg = Twist()


    def imgFullLeftCallback (self, ros_img):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
        # change below lines to map the color you wanted robot to follow
        lower_yellow = numpy.array([ 10,  10,  10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # CONTROL starts
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        # CONTROL ends
        cv2.imshow("mask",mask)
        cv2.imshow("output", image)
        cv2.waitKey(3)

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def timerCallback(self, event):
        msg = Int32MultiArray()
        msg.data = self.sensores
        self.sensor_pub.publish(msg)

if __name__ == '__main__':

    try:
        rospy.init_node("sensores")
        Sensores()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass