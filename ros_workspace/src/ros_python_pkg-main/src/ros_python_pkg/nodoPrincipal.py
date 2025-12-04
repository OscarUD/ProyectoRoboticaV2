#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32, Float32MultiArray

class PrincipalNode:
    def __init__(self):
        rospy.init_node("principal", anonymous=True)

        # Suscriptores
        rospy.Subscriber("/juego", Int32, self.callback_juego)
        rospy.Subscriber("/ganador", Int32, self.callback_ganador)
        rospy.Subscriber("/posicioncubos", Float32MultiArray, self.callback_cubos)
        rospy.Subscriber("/posicionfichas", Float32MultiArray, self.callback_fichas)

        # Estado interno
        self.juego_actual = None
        self.ganador_actual = None
        self.lista_cubos = []
        self.lista_fichas = []

    # ----------------------------------------
    # FUNCIONES QUE RECIBEN Y PROCESAN DATOS
    # ----------------------------------------

    def recibir_juego(self, juego):
        """Procesa el juego recibido desde gestos_node"""
        self.juego_actual = juego
        rospy.loginfo(f"[PRINCIPAL] Juego recibido: {self.juego_actual}")
        # Aquí puedes añadir más lógica: decidir turno, actualizar UI, etc.

    def recibir_ganador(self, ganador):
        """Procesa el ganador recibido desde gestos_node"""
        self.ganador_actual = ganador
        rospy.loginfo(f"[PRINCIPAL] Ganador recibido: {self.ganador_actual}")
        # Aquí puedes hacer acciones: reiniciar partida, mostrar resultado, etc.

    def recibir_cubos(self, lista_cubos):
        """Procesa la lista de posiciones de cubos recibida desde detector"""
        self.lista_cubos = lista_cubos
        rospy.loginfo(f"[PRINCIPAL] Posiciones de cubos: {self.lista_cubos}")
        # Aquí puedes hacer cálculos: buscar colisiones, planificar movimiento, etc.

    def recibir_fichas(self, lista_fichas):
        """Procesa la lista de posiciones de fichas recibida desde detector"""
        self.lista_fichas = lista_fichas
        rospy.loginfo(f"[PRINCIPAL] Posiciones de fichas: {self.lista_fichas}")
        # Aquí puedes hacer cálculos: comprobar ocupación de casillas, etc.

    # ----------------------------------------
    # CALLBACKS que conectan ROS con las funciones
    # ----------------------------------------
    def callback_juego(self, msg):
        self.recibir_juego(msg.data)

    def callback_ganador(self, msg):
        self.recibir_ganador(msg.data)

    def callback_cubos(self, msg):
        self.recibir_cubos(list(msg.data))

    def callback_fichas(self, msg):
        self.recibir_fichas(list(msg.data))

if __name__ == "__main__":
    node = PrincipalNode()
    rospy.spin()
