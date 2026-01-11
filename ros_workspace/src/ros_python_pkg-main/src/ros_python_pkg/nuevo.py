class ControlRobot:
    
    self.ganador = -1
    self.juego = -1
    
    rospy.Subscriber('/juego', Int16, self.callback_juego)
    
    rospy.Subscriber('/ganador', Int16, self.callback_ganador)

    

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
            
            contadorR = 0
            contadorA = 0
            
            pose_torre_azul = data[5]["torre_azul"]
            pose_torre_azul_dict = pose_torre_azul["pose"]
            pos_tAzul = control.dict_a_pose(pose_torre_azul_dict)
            
            print("\n==============================")
            print("📍 Coordenadas recibidas")
            print("==============================")
            
            if self.ganador == 2:                 
                for i, pose in enumerate(self.blue_pose_array):
                    if i == contadorA:
                        print(f"🔵  Azul {i+1}: x={pose.position.x:}, y={pose.position.y:}")
                        
                        azul = copy.deepcopy(self.posAruco)
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
                        azul.position.z -= self.movPinza
                        control.mover_trayectoria([azul])
                        rospy.sleep(1)
                        control.mover_pinza(5, 10)
                        rospy.sleep(1)
                        azul.position.z += self.movPinza
                        control.mover_trayectoria([azul])
                        rospy.sleep(1)
                        
                        if self.juego == 0:
                            control.mover_articulaciones(self.arts_aruco)
                            rospy.sleep(1)
                        elif self.juego == 1:
                            control.mover_trayectoria([pos_tAzul])
                            rospy.sleep(1)
                            pos = self.pose_actual()
                            pos.position.z -= 0.077 - contadorA*0.025
                            control.mover_trayectoria([pos])
                            rospy.sleep(1)
                        control.mover_pinza(60, 10)
                        rospy.sleep(1)
                        contadorA += 1
                        print(f"  Azul {i+1}: x={pose.position.x:}, y={pose.position.y:}")

            self.ganador = -1
            print("==============================\n")

if __name__ == '__main__':
    control = ControlRobot()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        control.mover_ficha()
        if control.contadorR >= 3 or control.contadorA >= 3:
            control.mover_articulaciones(control.arts_aruco)
            rospy.sleep(2)
            rospy.signal_shutdown("Fin de la partida")
        rate.sleep()




### GESTOS NODE UNIFICADO.PY ###

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError

## LINEA 23

rospy.init_node("gestos", anonymous=True)
#Subscripciones
self.bridge = CvBridge()
self.estado = "gameselector"
self.sub_image = rospy.Subscriber("/camara1/usb_cam/image_raw", Image, self.callback_image)
rospy.sleep(1)
#publicaciones
self.pub_juego = rospy.Publisher("/juego", Int16, queue_size=1)
self.pub_ganador = rospy.Publisher("/ganador", Int16, queue_size=1)


## BORRAR LINEAS 51 a 54: cv2.VideoCapture(0)


## LINEA 58

def callback_image(self, msg):
    try:
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)


## LINEA 731

while True:
    imagen_copy = deepcopy(self.img)


## LINEA 756

if gesto_actual == "VAQUEROS":
    self.pub_juego.publish(Int(data=0))  # Enviar a nodo principal que juego es el que se ha escogido
elif gesto_actual == "PPTLS":
    self.pub_juego.publish(Int(data=1))  # Enviar a nodo principal que juego es el que se ha escogido


## LINEA 785

if puntuador == "1":
    self.pub_ganador.publish(Int(data=1))  # Enviar a nodo principal que jugador 1 ha ganado la ronda
    print("Punto para Jugador 1")
elif puntuador == "2":
    self.pub_ganador.publish(Int(data=2))  # Enviar a nodo principal que jugador 2 ha ganado la ronda
    print("Punto para Jugador 2")
elif puntuador is None:
    print("Nadie puntuó en esta ronda.")