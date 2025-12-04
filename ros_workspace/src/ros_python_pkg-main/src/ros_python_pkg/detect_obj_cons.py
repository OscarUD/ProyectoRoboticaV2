#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import numpy as np
import cv2
from copy import deepcopy

class objetosNode:
    
    def __init__(self):
        rospy.init_node("gestos", anonymous=True)
        #Subscripciones
        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber("/camara1/usb_cam/image_raw", Image, self.callback_image)
        rospy.wait_for_message("/camara1/usb_cam/image_raw", Image)
        rospy.sleep(1)
        #publicaciones
        self.pub_juego = rospy.Publisher("/juego", Int16, queue_size=1)
        self.pub_ganador = rospy.Publisher("/ganador", Int16, queue_size=1)
        
        
    def procesar_frame(self, imagen:Image) -> tuple:
        cv2.imshow("Prueba", imagen)
        cv2.waitKey(1)
        
        return 1,2

    def callback_image(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Simulación: generar posiciones aleatorias        

    def start(self):
        while not rospy.is_shutdown():
            imagen = deepcopy(self.img)
            num1, num2 = self.procesar_frame(imagen)
            self.pub_ganador.publish(Int16(data=num1))
            self.pub_juego.publish(Int16(data=num2))

if __name__ == "__main__":
    node = GestosNode()
    node.start()
