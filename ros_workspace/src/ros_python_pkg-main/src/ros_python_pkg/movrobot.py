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
        
        self.recibidas_rojas = False
        self.recibidas_azules = False

        
        self.sub_rojas = rospy.Subscriber(
            '/coordenadasRojas', PoseArray, self.callback_rojas
        )
        
        # Subscribir a las coordenadas azules
        self.sub_azules = rospy.Subscriber(
            '/coordenadasAzules', PoseArray, self.callback_azules
        )
        
        ##self.ganador = rospy.Subscriber('/ganador', Int16, self.callback_ganador)
        
 
        self.añadir_suelo()
        
        
    def callback_rojas(self, msg: PoseArray):
        self.red_pose_array = msg.poses
        self.recibidas_rojas = True

    def callback_azules(self, msg: PoseArray):
        self.blue_pose_array = msg.poses
        self.recibidas_azules = True
        
    def callback_ganador(self, msg):
        """ Recibe ganador de ronda (1=jugador1, 2=jugador2)"""
        ganador = msg.data
        
        if ganador == 1:
            rospy.loginfo("¡JUGADOR 1 GANA RONDA! → Mover ficha ROJA")
            #self.mover_ficha_jugador1()  #  Mueve ficha roja
        elif ganador == 2:
            rospy.loginfo("¡JUGADOR 2 GANA RONDA! → Mover ficha AZUL") 
            #self.mover_ficha_jugador2()  #  Mueve ficha azul
        else:
            rospy.loginfo("Empate - Sin movimiento")
        
    def esperar_y_mostrar_coordenadas(self, posAruco: Pose):
        rospy.loginfo("Esperando coordenadas de fichas...")
        juego = "cubos"
        contadorR = 0
        contadorA = 0
        arts_aruco =  [0.9992761611938477,-1.0477239054492493,0.9035361448871058,-1.41451849163089,-1.5649026075946253,-0.707092587147848]
        arts_torre_azul =  [ 2.0560805797576904,1.471076452439167,1.4115451017962855,-1.5421768252602597,-1.5916088263141077,0.3771408796310425]
        
        pose_torre_azul = data[5]["torre_azul"]
        pose_torre_azul_dict = pose_torre_azul["pose"]
        pos_tAzul = control.dict_a_pose(pose_torre_azul_dict)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.recibidas_rojas and self.recibidas_azules:
                break
            rate.sleep()

        if juego == "cubos":
            movPinza = 0.06
        else:
            movPinza = 0.072
        
        print("\n==============================")
        print("📍 Coordenadas recibidas")
        print("==============================")

        if self.red_pose_array:
            print("🔴 Fichas rojas:")
            for i, pose in enumerate(self.red_pose_array):
                control.mover_articulaciones(arts_aruco)
                roja = copy.deepcopy(posAruco)
                roja.position.x -= pose.position.x 
                print(pose.position.x )
                # Corrección Y proporcional a la distancia hacia el QR 7 (máx 1 cm)
                bx, by = 0.38, 0.155  # baseline 23->7 en metros (38 cm, 15.5 cm)
                L = math.hypot(bx, by)
                ux, uy = bx / L, by / L
                proj = ux * pose.position.x + uy * pose.position.y
                t = max(0.0, min(1.0, proj / L))
                bias = -0.052
                y_corr = pose.position.y + bias * t
                roja.position.y += y_corr
                print(y_corr)
                control.mover_trayectoria([roja])
                control.mover_pinza(60, 10)
                roja.position.z -= movPinza
                control.mover_trayectoria([roja])
                control.mover_pinza(5, 10)
                roja.position.z += movPinza
                control.mover_trayectoria([roja])
                
                if juego == "cubos":
                    control.mover_articulaciones(arts_aruco)
                else:
                    control.mover_articulaciones(arts_aruco)
                control.mover_pinza(60, 10)
                contadorR += 1
                print(f"  Roja {i+1}: x={pose.position.x:}, y={pose.position.y:}")
        
        if self.blue_pose_array:
            print("🔵 Fichas azules:")
            for i, pose in enumerate(self.blue_pose_array):
                print(f"  Azul {i+1}: x={pose.position.x:}, y={pose.position.y:}")
                
            for i, pose in enumerate(self.blue_pose_array):
                azul = copy.deepcopy(posAruco)
                azul.position.x -= pose.position.x 
                # Corrección Y proporcional a la distancia hacia el QR 7 (máx 1 cm)
                bx, by = 0.38, 0.155  # baseline 23->7 en metros (38 cm, 15.5 cm)
                L = math.hypot(bx, by)
                ux, uy = bx / L, by / L
                proj = ux * pose.position.x + uy * pose.position.y
                t = max(0.0, min(1.0, proj / L))
                bias = -0.052
                y_corr = pose.position.y + bias * t
                azul.position.y += y_corr 
                control.mover_trayectoria([azul])
                rospy.sleep(1)
                control.mover_pinza(60, 10)
                rospy.sleep(1)
                azul.position.z -= movPinza
                control.mover_trayectoria([azul])
                rospy.sleep(1)
                control.mover_pinza(5, 10)
                rospy.sleep(1)
                azul.position.z += movPinza
                control.mover_trayectoria([azul])
                rospy.sleep(1)
                if juego == "cubos":
                    control.mover_trayectoria([pos_tAzul])
                    rospy.sleep(1)
                    pos = self.pose_actual()
                    pos.position.z -= 0.077 - contadorA*0.025
                    control.mover_trayectoria([pos])
                    rospy.sleep(1)
                else:
                    control.mover_articulaciones(arts_aruco)
                    rospy.sleep(1)
                control.mover_pinza(60, 10)
                rospy.sleep(1)
                contadorA += 1
                print(f"  Azul {i+1}: x={pose.position.x:}, y={pose.position.y:}")

        print("==============================\n")

    
    import math

    def desplazamiento_corregido(x, y, destino_x, destino_y):
        """
        Calcula el desplazamiento en x e y considerando el ángulo entre X y Y.
        
        Args:
            x, y: Coordenadas del punto actual (X)
            destino_x, destino_y: Coordenadas del punto objetivo (Y)
        
        Returns:
            dx, dy: Desplazamiento corregido en x e y
            distancia: distancia lineal entre X y Y
            angulo: ángulo entre X->Y en radianes
        """
        # Vector de X a Y
        vx = destino_x - x
        vy = destino_y - y
        
        # Distancia lineal
        distancia = math.sqrt(vx**2 + vy**2)
        
        # Ángulo del vector
        angulo = math.atan2(vy, vx)  # devuelve en radianes
        
        # Desplazamiento corregido en dirección del ángulo
        dx = distancia * math.cos(angulo)
        dy = distancia * math.sin(angulo)
        
        return dx, dy, distancia, angulo



    def articulaciones_actuales(self) -> list:
        return self.move_group.get_current_joint_values()
    
    def mover_articulaciones(self, joint_goal: List[float], wait: bool= True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)
    
    def pose_actual(self) -> Pose:
        return self.move_group.get_current_pose().pose
    
    def mover_a_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)
    
    def generar_trayectoria_lineal(self, pose_inicio: Pose, pose_fin: Pose, num_puntos: int = 20) -> list:
        """
        Genera una lista de poses linealmente interpoladas entre pose_inicio y pose_fin.

        Args:
            pose_inicio (Pose): Pose inicial.
            pose_fin (Pose): Pose final.
            num_puntos (int): Número de poses intermedias (incluyendo inicio y fin).

        Returns:
            List[Pose]: Lista de poses interpoladas.
        """
        # Convertir posiciones a arrays
        pos_ini = np.array([pose_inicio.position.x, pose_inicio.position.y, pose_inicio.position.z])
        pos_fin = np.array([pose_fin.position.x, pose_fin.position.y, pose_fin.position.z])

        # Convertir orientaciones a arrays
        ori_ini = np.array([pose_inicio.orientation.x, pose_inicio.orientation.y,
                            pose_inicio.orientation.z, pose_inicio.orientation.w])
        ori_fin = np.array([pose_fin.orientation.x, pose_fin.orientation.y,
                            pose_fin.orientation.z, pose_fin.orientation.w])

        # Interpolación lineal de posiciones
        posiciones = np.linspace(pos_ini, pos_fin, num_puntos)

        # Interpolación lineal de orientaciones (simple LERP, para pequeñas rotaciones sirve)
        orientaciones = np.linspace(ori_ini, ori_fin, num_puntos)

        # Generar lista de Pose
        lista_poses = []
        for i in range(num_puntos):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = posiciones[i]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientaciones[i]
            lista_poses.append(copy.deepcopy(pose))

        control.mover_trayectoria(lista_poses)


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
        pose_suelo.position.z = -0.026
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
    
    arts_aruco =  [0.9992761611938477,-1.0477239054492493 ,0.9035361448871058,-1.41451849163089 ,-1.5649026075946253, -0.707092587147848]
        
    pose_aruco = data[0]["pose_aruco"]
    pose_caja1 = data[1]["pose_caja1"]
    pose_caja2 = data[2]["pose_caja2"]
    
    pose_aruco_dict = pose_aruco["pose"]
    pose_caja1_dict = pose_caja1["pose"]
    pose_caja2_dict = pose_caja2["pose"]
    
    pos1 = control.dict_a_pose(pose_caja1_dict)
    pos2 = control.dict_a_pose(pose_caja2_dict)
    
    control.añadir_caja_a_escena_de_planificacion(pos1, "caja1", (2,.1,.4))
    control.añadir_caja_a_escena_de_planificacion(pos2, "caja2", (.05,.08,.8))
    
    posAruco = control.dict_a_pose(pose_aruco_dict)
   
    #control.mover_pinza(60, 10)
    #control.mover_pinza(0, 10)
    #control.mover_articulaciones(arts_aruco)
    
    #control.mover_pinza(20, 20)
    control.esperar_y_mostrar_coordenadas(posAruco)
    
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
