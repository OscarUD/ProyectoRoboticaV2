#!/usr/bin/python3

import sys
import copy
import rospy
import numpy as np
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, PoseArray
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandResult
from actionlib import SimpleActionClient
from tf.transformations import quaternion_from_euler
from typing import List
from std_msgs.msg import Int16
import yaml
from math import pi
import copy
import math


class ControlRobot:
    def __init__(self) -> None:
        # Inicializar ROS y MoveIt
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name) 
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        
        self.red_pose_array = []   # Para guardar el PoseArray de las rojas
        self.blue_pose_array = []
        self.green_pose_array = []
        
        self.contadorV = 0
        self.contadorR = 0
        self.contadorA = 0
        
        self.recibidas_rojas = False
        self.recibidas_azules = False
        self.recibidas_verdes = False
        
        self.ganador = -1
        self.juego = -1
        
        rospy.Subscriber('/juego', Int16, self.callback_juego)
        
        rospy.Subscriber('/ganador', Int16, self.callback_ganador)        
        
        self.sub_rojas = rospy.Subscriber(
            '/coordenadasRojas', PoseArray, self.callback_rojas
        )
        
        # Subscribir a las coordenadas azules
        self.sub_azules = rospy.Subscriber(
            '/coordenadasAzules', PoseArray, self.callback_azules
        )
        
        self.sub_verdes = rospy.Subscriber(
            '/coordenadasVerdes', PoseArray, self.callback_verdes
        )
        
        self.añadir_suelo()
        
        with open("./src/ros_python_pkg-main/src/ros_python_pkg/posiciones.yaml", "r") as file:
            data = yaml.safe_load(file)
        
        self.arts_aruco =  [0.9992761611938477,-1.0477239054492493 ,0.9035361448871058,-1.41451849163089 ,-1.5649026075946253, -0.707092587147848]
        self.arts_torre_azul =  [ 2.0560805797576904,1.471076452439167,1.4115451017962855,-1.5421768252602597,-1.5916088263141077,0.3771408796310425]
            
        pose_aruco = data[0]["pose_aruco"]
        pose_caja1 = data[1]["pose_caja1"]
        pose_caja2 = data[2]["pose_caja2"]
        
        pose_aruco_dict = pose_aruco["pose"]
        pose_caja1_dict = pose_caja1["pose"]
        pose_caja2_dict = pose_caja2["pose"]
        
        pos1 = self.dict_a_pose(pose_caja1_dict)
        pos2 = self.dict_a_pose(pose_caja2_dict)
        
        self.añadir_caja_a_escena_de_planificacion(pos1, "caja1", (2,.1,.4))
        self.añadir_caja_a_escena_de_planificacion(pos2, "caja2", (.05,.08,.8))
        
        self.posAruco = self.dict_a_pose(pose_aruco_dict)
        
    def callback_rojas(self, msg: PoseArray):
        if not self.recibidas_rojas:
            self.red_pose_array = msg.poses
            self.recibidas_rojas = True

    def callback_azules(self, msg: PoseArray):
        if not self.recibidas_azules:
            self.blue_pose_array = msg.poses
            self.recibidas_azules = True
    
    def callback_verdes(self, msg: PoseArray):
        if not self.recibidas_verdes:
            self.green_pose_array = msg.poses
            self.recibidas_verdes = True
    
    def callback_juego(self, msg):
        """ Recibe juego del selector (0=VAQUEROS, 1=PPTLS)"""
        self.juego = msg.data
        
        if self.juego == 0:
            self.movPinza = 0.072
            rospy.loginfo("JUEGO VAQUEROS SELECCIONADO")
        elif self.juego == 1:
            self.movPinza = 0.06
            rospy.loginfo("JUEGO PPTLS SELECCIONADO") 

    def callback_ganador(self, msg):
        self.ganador = msg.data
        if self.ganador in [1,2]:
            rospy.loginfo(f"GANADOR RONDA: Jugador {self.ganador}")
        else:
            rospy.loginfo("Empate - Sin movimiento")
        
    def mover_ficha(self):
        
        if self.juego == -1:
            return
        if self.ganador == -1:
            return
        if not (self.recibidas_rojas and self.recibidas_azules):
            return
        
        self.mover_articulaciones(self.arts_aruco)
        
        arts_abaco_rojo = [1.3183988332748413,-0.8618126076510926,0.49755889574159795,-1.208068923359253,-1.5649502913104456,-0.19018108049501592]
        arts_abaco_azul = [1.5276987552642822,-0.8243203920177002,0.5101397673236292,-1.258183316593506,-1.5649545828448694,0.01942265033721924]
        
        pose_torre_azul = data[5]["torre_azul"]
        pose_torre_azul_dict = pose_torre_azul["pose"]
        pose_torre_verde = data[6]["torre_verde"]
        pose_torre_verde_dict = pose_torre_verde["pose"]
        pos_tAzul = control.dict_a_pose(pose_torre_azul_dict)
        pos_tVerde = control.dict_a_pose(pose_torre_verde_dict)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.recibidas_rojas and self.recibidas_azules:
                break
            rate.sleep()
        
        print("\n==============================")
        print("📍 Coordenadas recibidas")
        print("==============================")

        if self.ganador == 1 and self.juego == 0:
            print("🔴 Fichas rojas:")
            pose = self.red_pose_array[0]
            print(f"  Roja {self.contadorR+1}: x={pose.position.x:}, y={pose.position.y:}")
            self.red_pose_array.pop(0)
            rospy.sleep(1)
            roja = copy.deepcopy(self.posAruco)
            roja.position.x -= pose.position.x 
            print(pose.position.x )
            # Corrección Y proporcional a la distancia hacia el QR 7 (máx 1 cm)
            bx, by = 0.38, 0.155  # baseline 23->7 en metros (38 cm, 15.5 cm)
            L = math.hypot(bx, by)
            ux, uy = bx / L, by / L
            proj = ux * pose.position.x + uy * pose.position.y
            t = max(0.0, min(1.0, proj / L))
            bias = -0.018
            y_corr = pose.position.y + bias * t
            roja.position.y += y_corr
            print(y_corr)
            self.mover_trayectoria([roja])
            rospy.sleep(1)
            self.mover_pinza(60, 10)
            rospy.sleep(1)
            roja.position.z -= self.movPinza
            self.mover_trayectoria([roja])
            rospy.sleep(1)
            self.mover_pinza(5, 10)
            rospy.sleep(1)
            roja.position.z += self.movPinza + 0.08
            self.mover_trayectoria([roja])
            rospy.sleep(1)
            
            if self.juego == 0:
                self.mover_articulaciones(arts_abaco_rojo)
                rospy.sleep(1)
                pos = self.pose_actual()
                pos.position.z -= 0.034
                self.mover_trayectoria([pos])
                rospy.sleep(1)
                
            self.mover_pinza(60, 10)
            rospy.sleep(1)
            self.contadorR += 1
        
        if self.ganador == 2:
            print("🔵 Fichas azules:")
            
            pose = self.blue_pose_array[0]
            print(f"  Azul {self.contadorA+1}: x={pose.position.x:}, y={pose.position.y:}")
            self.blue_pose_array.pop(0)
            azul = copy.deepcopy(self.posAruco)
            azul.position.x -= pose.position.x 
            # Corrección Y proporcional a la distancia hacia el QR 7 (máx 1 cm)
            bx, by = 0.38, 0.155  # baseline 23->7 en metros (38 cm, 15.5 cm)
            L = math.hypot(bx, by)
            ux, uy = bx / L, by / L
            proj = ux * pose.position.x + uy * pose.position.y
            t = max(0.0, min(1.0, proj / L))
            bias = -0.018
            y_corr = pose.position.y + bias * t
            azul.position.y += y_corr 
            self.mover_trayectoria([azul])
            rospy.sleep(1)
            self.mover_pinza(60, 10)
            rospy.sleep(1)
            azul.position.z -= self.movPinza
            self.mover_trayectoria([azul])
            rospy.sleep(1)
            self.mover_pinza(5, 10)
            rospy.sleep(2)
            azul.position.z += self.movPinza
            self.mover_trayectoria([azul])
            rospy.sleep(1)
            if self.juego == 1:
                self.mover_trayectoria([pos_tAzul])
                rospy.sleep(1)
                pos = self.pose_actual()
                pos.position.z -= 0.077 - self.contadorA*0.025
                self.mover_trayectoria([pos])
                rospy.sleep(1)
            else:
                self.mover_articulaciones(arts_abaco_azul)
                rospy.sleep(1)
                pos = self.pose_actual()
                pos.position.z -= 0.017
                self.mover_trayectoria([pos])
                rospy.sleep(1)
            self.mover_pinza(60, 10)
            rospy.sleep(1)
            self.contadorA += 1

        if self.ganador == 1 and self.juego == 1:
            print("🟢 Fichas verdes:")
            
            pose = self.green_pose_array[0]
            print(f"  Verde {self.contadorV+1}: x={pose.position.x:}, y={pose.position.y:}")      
            self.green_pose_array.pop(0)
            
            verde = copy.deepcopy(self.posAruco)
            verde.position.x -= pose.position.x 
            # Corrección Y proporcional a la distancia hacia el QR 7 (máx 1 cm)
            bx, by = 0.38, 0.155  # baseline 23->7 en metros (38 cm, 15.5 cm)
            L = math.hypot(bx, by)
            ux, uy = bx / L, by / L
            proj = ux * pose.position.x + uy * pose.position.y
            t = max(0.0, min(1.0, proj / L))
            bias = -0.018
            y_corr = pose.position.y + bias * t
            verde.position.y += y_corr 
            self.mover_trayectoria([verde])
            rospy.sleep(1)
            self.mover_pinza(60, 10)
            rospy.sleep(1)
            verde.position.z -= self.movPinza
            self.mover_trayectoria([verde])
            rospy.sleep(1)
            self.mover_pinza(5, 10)
            rospy.sleep(1)
            verde.position.z += self.movPinza
            self.mover_trayectoria([verde])
            rospy.sleep(1)
            if self.juego == 1:
                pos_tVerde.position.z += 0.05
                self.mover_trayectoria([pos_tVerde])
                rospy.sleep(1)
                pos = self.pose_actual()
                pos.position.z -= 0.095 - self.contadorV*0.025
                self.mover_trayectoria([pos])
                rospy.sleep(1)
            self.mover_pinza(60, 10)
            rospy.sleep(1)
            self.contadorV += 1
        
        self.ganador = -1
        print(self.ganador)
        print("==============================\n")

    def articulaciones_actuales(self) -> list:
        return self.move_group.get_current_joint_values()
    
    def mover_articulaciones(self, joint_goal: List[float], wait: bool= True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)
    
    def pose_actual(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def mover_a_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    def mover_trayectoria(self, poses: List[Pose], wait: bool = True) -> bool:
        # Copiar la lista de poses y añadir la pose actual al inicio
        poses_aux = copy.deepcopy(poses)
        poses_aux.insert(0, self.pose_actual())
        
        # Calcular trayectoria cartesiana en línea recta entre las poses
        (plan, fraction) = self.move_group.compute_cartesian_path(poses_aux, 0.01)
        
        if fraction != 1.0:
            print("Advertencia: trayectoria incompleta")
            return False
        
        # Ejecutar el plan
        return self.move_group.execute(plan, wait=wait)

    def añadir_caja_a_escena_de_planificacion(self, pose_caja: Pose, name: str,
                                              tamaño: tuple = (.1,.1,.1)) -> None:
        # Convertir a PoseStamped
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        # Añadir la caja a la escena
        self.scene.add_box(name, box_pose, size=tamaño)

    def añadir_suelo(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.04
        self.añadir_caja_a_escena_de_planificacion(pose_suelo,"suelo",(2,2,.05))

    def dict_a_pose(self, pose_dict):
        pose = Pose()

        pose.position.x = pose_dict["position"]["x"]
        pose.position.y = pose_dict["position"]["y"]
        pose.position.z = pose_dict["position"]["z"]

        pose.orientation.x = pose_dict["orientation"]["x"]
        pose.orientation.y = pose_dict["orientation"]["y"]
        pose.orientation.z = pose_dict["orientation"]["z"]
        pose.orientation.w = pose_dict["orientation"]["w"]

        return pose

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        
        return result.reached_goal
    
if __name__ == '__main__':
    control = ControlRobot()
    
    with open("./src/ros_python_pkg-main/src/ros_python_pkg/posiciones.yaml", "r") as file:
        data = yaml.safe_load(file)
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        control.mover_ficha()
        if control.contadorR >= 3 or control.contadorA >= 3 or control.contadorV >= 3:
            control.mover_articulaciones(control.arts_aruco)
            rospy.sleep(2)
            rospy.signal_shutdown("Fin de la partida")
        rate.sleep()

    
    '''
    with open("./src/ros_python_pkg-main/src/ros_python_pkg/posiciones.yaml", "r") as file:
        data = yaml.safe_load(file)
    
    fueraAruco = copy.deepcopy(posAruco)
    fueraAruco.position.x -= 0.05
    fueraAruco.position.y -= 0.05
    control.mover_trayectoria([fueraAruco])
    
    pose_aruco = data[0]["pose_aruco"]
    pose_caja1 = data[1]["pose_caja1"]
    pose_caja2 = data[2]["pose_caja2"]
    pose_abaco_azul = data[3]["pose_abaco_azul"]
    pose_abaco_rojo = data[4]["pose_abaco_rojo"]
     
    pose_aruco_dict = pose_aruco["pose"]
    pose_caja1_dict = pose_caja1["pose"]
    pose_caja2_dict = pose_caja2["pose"]
    pose_abaco_azul_dict = pose_abaco_azul["pose"]
    pose_abaco_rojo_dict = pose_abaco_rojo["pose"]
    
    posAruco = control.dict_a_pose(pose_aruco_dict)
    pos1 = control.dict_a_pose(pose_caja1_dict)
    pos2 = control.dict_a_pose(pose_caja2_dict)
    pos_azul = control.dict_a_pose(pose_abaco_azul_dict)
    pos_int_azul = control.dict_a_pose(pose_abaco_azul_dict)
    pos_rojo = control.dict_a_pose(pose_abaco_rojo_dict)
    pos_int_rojo = control.dict_a_pose(pose_abaco_rojo_dict)
    
    pos1.position.z += 0.09
    pos1.position.y += 0.06
    pos2.position.z += 0.16
    pos2.position.y += 0.08
    pos_int_azul.position.z += 0.025
    pos_int_rojo.position.z += 0.025
    
    arts_abaco_azul= [ 1.7125924825668335, -1.0432720941356202, 0.5994351545916956, -1.128568874006607, -1.5650060812579554, -0.09494001070131475]
    arts_aruco =  [1.0049527883529663, -1.0491498869708558, 0.8802011648761194, -1.3895794463208695, -1.5648420492755335, -0.7018120924579065]
    arts_abaco_rojo = [1.4389657974243164, -1.0563444060138245, 0.6231582800494593, -1.1390932959369202, -1.5650342146502894 , -0.6984780470477503]
    
    control.añadir_caja_a_escena_de_planificacion(pos1, "caja1", (2,.1,.4))
    control.añadir_caja_a_escena_de_planificacion(pos2, "caja2", (.35,.08,.8))    

    control.mover_pinza(0.468, 10)
    control.mover_articulaciones(arts_aruco)
    actual = control.pose_actual()
    actual.position.z += 0.065
    control.mover_trayectoria([actual])
    
    control.mover_trayectoria([pos_int_rojo])
    control.mover_pinza(0.6, 10)
    control.mover_articulaciones(arts_abaco_rojo)
    control.mover_trayectoria([pos_int_rojo])
    
    control.mover_pinza(0.468, 10)
    control.mover_trayectoria([actual])
    
    control.mover_trayectoria([pos_int_azul])
    control.mover_pinza(0.6, 10)
    control.mover_articulaciones(arts_abaco_azul)
    control.mover_trayectoria([pos_int_azul])
    
    control.mover_pinza(0.468, 10)
    control.mover_trayectoria([actual])
    
    
    
    
    pass
    '''
    
    
    # try:
       
        
    #     if control.red_pose_array:
    #         print("Coordenadas rojas guardadas:", control.red_pose_array)
    #     if control.blue_pose_array:
    #         print("Coordenadas azules guardadas:", control.blue_pose_array)
    #     with open("./src/ros_python_pkg-main/src/ros_python_pkg/posiciones.yaml", "r") as file:
    #         data = yaml.safe_load(file)

    #         # Obtener pose y articulaciones
    #         pose_yaml = data[0]["pose_aruco"]["pose"]
    #         articulaciones = data[0]["pose_aruco"]["articulaciones"]

    #         # Reconstruir Pose de ROS
            
    #         pose_aruco = Pose()
    #         pose_aruco.position = Point(**pose_yaml["position"])
    #         pose_aruco.orientation = Quaternion(**pose_yaml["orientation"])
            
    #         pose_aruco_list: List[Pose] = []
            
            
    #         pose_aruco.position.z += 0.1
    #         pose_aruco_list.append(pose_aruco)
            
    #         control.mover_trayectoria(pose_aruco_list)
            
    #         pose_aruco_list.remove(pose_aruco)
    #         pose_aruco.position.y += 0.1
            
    #         pose_aruco_list.append(pose_aruco)
            
    #         rospy.sleep(5)
    #     while not rospy.is_shutdown():
           
    #         # Leer YAML
            

    #         # Mover robot exactamente a esa pose
    #         control.mover_a_pose(pose_aruco)
            
    #         pose = pose_aruco
            
            

    #         print(control.red_pose_array)
 
    #         for e in control.red_pose_array:
    #             poses: List[Pose] = []
    #             pose_copy = copy.deepcopy(pose_aruco)
    #             pose_copy.position.x += e.position.x / 100
    #             pose_copy.position.y += e.position.y / 100
    #             poses.append(pose_copy)
    #             control.mover_trayectoria(poses)
                
    #             control.mover_trayectoria(pose_aruco_list)
                
            

    # except rospy.ROSInterruptException:
    #     pass
