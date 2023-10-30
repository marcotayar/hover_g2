#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError

class SET_POINT:
    def __init__(self):
        # Inicialização do nó ROS
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.image_data_pub = rospy.Publisher('image_data', Float64, queue_size=1)
        self.image_publisher = rospy.Publisher('image_with_masks', Image, queue_size=1)  # Tópico para publicar a imagem com as máscaras
        # Definição das faixas de cores em formato HSV
        self.lower_green = np.array([40, 100, 100])   # HSV (verde)
        self.upper_green = np.array([80, 255, 255])  # HSV (verde)
        self.lower_red = np.array([0, 100, 100])    # HSV (vermelho)
        self.upper_red = np.array([20, 255, 255])   # HSV (vermelho)

    def image_callback(self, msg):
        # Conversão da mensagem da imagem para o formato OpenCV
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detecção da cor verde
        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        green_mask_largest = np.zeros_like(green_mask)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if green_contours:
            green_largest_contour = max(green_contours, key=cv2.contourArea)
            cv2.drawContours(green_mask_largest, [green_largest_contour], 0, 255, -1)

        # Detecção da cor vermelha
        red_mask = cv2.inRange(hsv, self.lower_red, self.upper_red)
        red_mask_largest = np.zeros_like(green_mask)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if red_contours:
            red_largest_contour = max(red_contours, key=cv2.contourArea)
            cv2.drawContours(red_mask_largest, [red_largest_contour], 0, 255, -1)

        # Criação da máscara combinada usando a operação lógica OR
        combined_mask = cv2.bitwise_or(green_mask_largest, red_mask_largest)

        # Cálculo da posição horizontal da máscara combinada
        M = cv2.moments(combined_mask)
        if M['m00'] > 0:
            combined_mask_x = int(M['m10'] / M['m00'])
        else:
            combined_mask_x = 0.0
        # Desenhe um círculo preenchido no centro da imagem combinada
        cv2.circle(combined_mask, ((combined_mask_x, combined_mask.shape[0] // 2)), 5, 255, -1)
    
        

        # Exibição das máscaras e da máscara combinada
        #cv2.imshow("green_mask", green_mask)
        #cv2.imshow("green_mask_largest", green_mask_largest)
        #cv2.imshow("red_mask", red_mask)
        #cv2.imshow("red_mask_largest", red_mask_largest)
        #cv2.imshow("Result Image", combined_mask)
        cv2.waitKey(3)
        image_with_masks_msg = self.bridge.cv2_to_imgmsg(combined_mask, "mono8")
        self.image_publisher.publish(image_with_masks_msg)

        # Publicação da posição horizontal da máscara combinada
        image_data_msg = Float64(data=combined_mask_x)
        self.image_data_pub.publish(image_data_msg)


rospy.init_node('Set_Point')
Processa = SET_POINT()
rospy.spin()
