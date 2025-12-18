#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16  # ✅ Cambiar a Int16 (igual que gestos)
from geometry_msgs.msg import Twist  # ✅ Para mover robot

class PrincipalNode:
    def __init__(self):
        rospy.init_node('principal_node', anonymous=True)
        
        # Suscribirse a /juego (del nodo gestos)
        rospy.Subscriber('/juego', Int16, self.callback_juego)
        
        # Suscribirse a /ganador (del nodo gestos)
        rospy.Subscriber('/ganador', Int16, self.callback_ganador)
        
        # Publicador para mover robot
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Lista de objetos detectados (rojo=jugador1, azul=jugador2)
        self.objetos_rojos = []  # Fichas jugador 1
        self.objetos_azules = [] # Fichas jugador 2
        
        rospy.loginfo("Nodo Principal iniciado")
        rospy.spin()

    def callback_juego(self, msg):
        """Recibe juego seleccionado"""
        juego_id = msg.data
        if juego_id == 1:
            rospy.loginfo("Juego: Piedra, Papel o Tijeras")
        elif juego_id == 2:
            rospy.loginfo("Juego: Pistoleros (Vaqueros)")

    def callback_ganador(self, msg):
        """✅ Recibe ganador de ronda (1=jugador1, 2=jugador2)"""
        ganador = msg.data
        
        if ganador == 1:
            rospy.loginfo("¡JUGADOR 1 GANA RONDA! → Mover ficha ROJA")
            self.mover_ficha_jugador1()  # ✅ Mueve ficha roja
        elif ganador == 2:
            rospy.loginfo("¡JUGADOR 2 GANA RONDA! → Mover ficha AZUL") 
            self.mover_ficha_jugador2()  # ✅ Mueve ficha azul
        else:
            rospy.loginfo("Empate - Sin movimiento")

    def mover_ficha_jugador1(self):
        """✅ Mueve robot hacia la PRIMER ficha ROJA más cercana"""
        if self.objetos_rojos:
            # Tomar la primera ficha roja (más cercana)
            target_x, target_y = self.objetos_rojos[0]  
            
            rospy.loginfo(f"Moviendo hacia ficha roja: ({target_x:.1f}, {target_y:.1f}) cm")
            
            # Crear comando Twist para mover robot
            twist = Twist()
            twist.linear.x = 0.2  # Velocidad hacia adelante
            twist.angular.z = 0.0 # Sin rotación (ajustar según posición)
            
            self.pub_cmd_vel.publish(twist)
            rospy.sleep(1)  # Mover 1 segundo
            twist.linear.x = 0.0
            self.pub_cmd_vel.publish(twist)  # Parar

    def mover_ficha_jugador2(self):
        """✅ Mueve robot hacia la PRIMER ficha AZUL más cercana"""
        if self.objetos_azules:
            target_x, target_y = self.objetos_azules[0]
            
            rospy.loginfo(f"Moviendo hacia ficha azul: ({target_x:.1f}, {target_y:.1f}) cm")
            
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            
            self.pub_cmd_vel.publish(twist)
            rospy.sleep(1)
            twist.linear.x = 0.0
            self.pub_cmd_vel.publish(twist)

    def actualizar_objetos(self, rojos_cm, azules_cm):
        """Actualizar lista de objetos desde nodo de detección"""
        self.objetos_rojos = rojos_cm or []
        self.objetos_azules = azules_cm or []

if __name__ == '__main__':
    try:
        nodo = PrincipalNode()
        nodo.start()
    except rospy.ROSInterruptException:
        pass



