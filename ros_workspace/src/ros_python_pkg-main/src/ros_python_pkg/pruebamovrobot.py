#!/usr/bin/python3

import sys
import copy
import rospy
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
from geometry_msgs.msg import Pose, Point, PoseStamped
from tf.transformations import quaternion_from_euler
from typing import List
import yaml
from math import pi
import copy

class ControlRobot:
    def __init__(self) -> None:
        # Inicializar ROS y MoveIt
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.añadir_suelo()

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
        pose_suelo.position.z = -0.026
        self.añadir_caja_a_escena_de_planificacion(pose_suelo,"suelo",(2,2,.05))


if __name__ == '__main__':
    control = ControlRobot()
    
    # Mover el robot a posición inicial con articulaciones
   
    control.mover_articulaciones([0, -pi/2, -pi/2, -pi/2, pi/2, 0])
    
    # Pose inicial del efector
    pose_inicio = control.pose_actual()
    articulaciones = control.articulaciones_actuales()
    
  
    nombre = "Pose ARUCO"
    with open("./src/ros_python_pkg-main/src/ros_python_pkg/posiciones.yaml", "+a") as file:
        # yaml.dump(pose_dict, file)
        yaml.dump([{nombre:{"pose":pose_inicio,"articulaciones":articulaciones}}], file)

    print("Posición inicial guardada en posiciones.yaml")
    
    # Pose final (movimiento en línea recta)
    pose_final = copy.deepcopy(pose_inicio)
    pose_final.position.x += 0.2  # 20 cm adelante
    pose_final.position.y += 0.1  # 10 cm a la derecha

    # Crear lista de poses intermedias para suavizar trayectoria
    num_puntos = 10
    poses = []
    for i in range(1, num_puntos + 1):
        pose_intermedia = copy.deepcopy(pose_inicio)
        pose_intermedia.position.x += (pose_final.position.x - pose_inicio.position.x) * i / num_puntos
        pose_intermedia.position.y += (pose_final.position.y - pose_inicio.position.y) * i / num_puntos
        pose_intermedia.position.z += (pose_final.position.z - pose_inicio.position.z) * i / num_puntos
        poses.append(pose_intermedia)
    
    # Mover el robot siguiendo la línea
    control.mover_trayectoria(poses)
    

    
    print("Movimiento completado")