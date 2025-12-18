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
    
    # Pose inicial del efector
    pose_aruco = control.pose_actual()
    articulaciones = control.articulaciones_actuales()

    # Convertir Pose a diccionario
    pose_dict = {
        "position": {
            "x": pose_aruco.position.x,
            "y": pose_aruco.position.y,
            "z": pose_aruco.position.z
        },
        "orientation": {
            "x": pose_aruco.orientation.x,
            "y": pose_aruco.orientation.y,
            "z": pose_aruco.orientation.z,
            "w": pose_aruco.orientation.w
        }
    }

    # Guardar articulaciones y pose como diccionario
    nombre = "pose_abaco_azul"
    with open("./src/ros_python_pkg-main/src/ros_python_pkg/posiciones.yaml", "a") as file:
        yaml.dump([{nombre: {"pose": pose_dict, "articulaciones": articulaciones}}], file)
