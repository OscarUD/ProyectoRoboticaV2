#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge
import numpy as np
import cv2
import mediapipe as mp
from copy import deepcopy
from gestos import get_hand_rois, detect_selection_gesture, process_round, detect_gameselector



class GestosNode:
    
    
    def __init__(self):
        rospy.init_node("gestos", anonymous=True)
        #Subscripciones
        self.bridge = CvBridge()
        self.estado = "VAQUEROS"
        self.sub_image = rospy.Subscriber("/camara1/usb_cam/image_raw", Image, self.callback_image)
        rospy.wait_for_message("/camara1/usb_cam/image_raw", Image)
        rospy.sleep(1)
        #publicaciones
        self.pub_juego = rospy.Publisher("/juego", Int16, queue_size=1)
        self.pub_ganador = rospy.Publisher("/ganador", Int16, queue_size=1)
        
        #Cosas de los juegos
        self.bullets_p1 = 0 #azul
        self.bullets_p2 = 0 # rojo
        
        
        
        self.score_p1 = 0 #azul
        self.score_p2 = 0 #rojo
        
        
    def procesar_frame(self, imagen:Image) -> tuple:
        cv2.imshow("Prueba", imagen)
        cv2.waitKey(1)
        
        return 1,2
    
    def procesar_juego(self, imagen: Image) -> str:
        gesto = ""
        mask = None
        # Copia de la imagen para no modificar la original
        
        # Detectar manos
        hand_rois = get_hand_rois(imagen, max_hands=1)
        
        for hand in hand_rois:
            roi = hand["roi"]  # ESTE es un array válido de OpenCV
            gesto, mask, max_contour, hull_points = detect_gameselector(roi)  
            #print("Gesto detectado:", gesto)
        
        # Texto estático y dinámico
        texto_info = "Puño=Salir | 1 dedo=PPTLS | 2 dedos=Vaqueros"
        texto_gesto = f"Gesto actual: {gesto}"
        
        # Propiedades del texto
        font = cv2.FONT_HERSHEY_SIMPLEX
        escala = 0.6
        grosor = 2

        # Pintar texto estático (arriba derecha)
        (ancho_texto, alto_texto), _ = cv2.getTextSize(texto_info, font, escala, grosor)
        x_info = imagen.shape[1] - ancho_texto - 10
        y_info = 30
        cv2.putText(imagen, texto_info, (x_info, y_info), font, escala, (0, 255, 0), grosor, cv2.LINE_AA)
        
        # Pintar gesto dinámico (debajo del texto estático)
        (ancho_gesto, alto_gesto), _ = cv2.getTextSize(texto_gesto, font, escala, grosor)
        x_gesto = imagen.shape[1] - ancho_gesto - 10
        y_gesto = y_info + alto_texto + 10
        cv2.putText(imagen, texto_gesto, (x_gesto, y_gesto), font, escala, (0, 0, 255), grosor, cv2.LINE_AA)
        
        # Mostrar imagen
        #if mask is not None:
        #    cv2.imshow("Mascara", mask)
        
        cv2.waitKey(1)
        
        return gesto

    def procesar_gesto_inicio_ronda(self, imagen:Image) -> str:
        
        gesto = None
        #detectar gesto para empezar la ronda
        hand_roi = get_hand_rois(imagen, max_hands=1)
        for hand in hand_roi:
            roi = hand["roi"]  # ESTE es un array válido de OpenCV
            gesto = detect_selection_gesture(roi)  
            print("Gesto detectado:", gesto)
           
        return gesto

    def callback_image(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Simulación: generar posiciones aleatorias        

    def start(self):
        
        
        
        while not rospy.is_shutdown():
            
            imagen = deepcopy(self.img)
            
            if self.estado == "gameselector":
                imagenGS = imagen
                
                gesto_actual = self.procesar_juego(imagenGS)

                
                texto_info = "Puno=Salir | 1 dedo=PPTLS | 2 dedos=Vaqueros"
                texto_gesto = f"Gesto actual: {gesto_actual}"
                
                # Propiedades del texto
                font = cv2.FONT_HERSHEY_SIMPLEX
                escala = 0.6
                grosor = 2

                # Pintar texto estático (arriba derecha)
                (ancho_texto, alto_texto), _ = cv2.getTextSize(texto_info, font, escala, grosor)
                x_info = imagenGS.shape[1] - ancho_texto - 10
                y_info = 30
                cv2.putText(imagenGS, texto_info, (x_info, y_info), font, escala, (0, 255, 0), grosor, cv2.LINE_AA)
                
                # Pintar gesto dinámico (debajo del texto estático)
                (ancho_gesto, alto_gesto), _ = cv2.getTextSize(texto_gesto, font, escala, grosor)
                x_gesto = imagenGS.shape[1] - ancho_gesto - 10
                y_gesto = y_info + alto_texto + 10
                cv2.putText(imagenGS, texto_gesto, (x_gesto, y_gesto), font, escala, (0, 0, 255), grosor, cv2.LINE_AA)
                
                cv2.imshow("GameSelector", imagenGS)
                cv2.waitKey(1)
                # Detectar gesto del frame actual

                # Inicializar variables de control si no existen
                if not hasattr(self, "gesto_anterior"):
                    self.gesto_anterior = gesto_actual
                    self.contador_gesto = 1
                    self.gesto_confirmado = ""

                else:
                    # Comprobar si el gesto es el mismo que el anterior
                    if gesto_actual == self.gesto_anterior:
                        self.contador_gesto += 1
                    else:
                        self.gesto_anterior = gesto_actual
                        self.contador_gesto = 1

                # Confirmar gesto tras 30 frames consecutivos
                if self.contador_gesto >= 60:
                    if self.gesto_confirmado != gesto_actual:  # Solo publicar si cambia
                        self.gesto_confirmado = gesto_actual
                        self.pub_juego.publish(Int16(data=gesto_actual))
                        print("JUEGO SELECCIONADO CONFIRMADO:", gesto_actual)
                        
                        cv2.destroyWindow("GameSelector")
                        self.estado = gesto_actual  # actualizar estado
                        
            elif self.estado == "COUNTDOWN":
                imagenV = imagen
                
                score1 = self.score_p1
                score2 = self.score_p2
                
                bullets1 = self.bullets_p1
                bullets2 = self.bullets_p2
    

                # Texto de información de los jugadores
                texto_jug1 = f"Jugador 1 | Balas: {bullets1} | Puntuacion: {score1}"
                texto_jug2 = f"Jugador 2 | Balas: {bullets2} | Puntuacion: {score2}"

                # Propiedades del texto
                font = cv2.FONT_HERSHEY_SIMPLEX
                escala = 0.6
                grosor = 2

                # Posición Jugador 1 (arriba izquierda)
                x_jug1 = 10
                y_jug1 = 30
                
                cv2.putText(imagenV, texto_jug1, (x_jug1, y_jug1), font, escala, (255, 0, 0), grosor, cv2.LINE_AA)

                # Posición Jugador 2 (arriba derecha)
                
                (ancho_jug2, alto_jug2), _ = cv2.getTextSize(texto_jug2, font, escala, grosor)
                
                x_jug2 = imagenV.shape[1] - ancho_jug2 - 10
                y_jug2 = 30
                cv2.putText(imagenV, texto_jug2, (x_jug2, y_jug2), font, escala, (0, 0, 255), grosor, cv2.LINE_AA)
                
                cv2.imshow("Vaqueros", imagenV)
                #cv2.resizeWindow("Vaqueros", 1200, 800)
                cv2.waitKey(1)
                
                gesto_inicio = self.procesar_gesto_inicio_ronda(imagenV)
                

            if self.estado == "VAQUEROS":
                pass
            


            if self.estado == "PPTLS":
                
                #num1, num2 = self.procesar_frame(imagen)
                #self.pub_ganador.publish(Int16(data=num1))
                #self.pub_juego.publish(Int16(data=num2))
                pass

if __name__ == "__main__":
    node = GestosNode()
    node.start()


