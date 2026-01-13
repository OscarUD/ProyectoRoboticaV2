# -*- coding: utf-8 -*-
import cv2
import numpy as np
import mediapipe as mp
from copy import deepcopy
from funcionesAuxiliaresObjetos import detectar_marcador_sin_aruco, detectar_distancia_objetos, dibujar_sistema_coordenadas
import time
import rospy
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image  # IMPORTAR CAMARA DE ROS
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError



camera_matrix = [
    [675.637434, 0.000000, 337.275136],
    [0.000000, 676.471546, 233.844278],
    [0.0, 0.0, 1.0 ]
]

# [narrow_stereo] distortion
# -0.404441 0.942400 0.001988 -0.002131 0.000000
dist_coeffs = [-0.446356, 0.217270, -0.000749, -0.000805, 0.000000]

# Tamaño esperado de la imagen
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
 

class GestosNode:

    #INIT
    def __init__(self):

        rospy.init_node("objetos", anonymous=True)
        #Subscripciones
        self.bridge = CvBridge()
        self.sub_image = rospy.Subscriber("/camara1/usb_cam/image_raw", Image, self.callback_image )  #cambiar topi al de la segunda camara
        #rospy.wait_for_message("/camara1/usb_cam/image_raw", Image)
        self.sub_juego = rospy.Subscriber("/juego", Int16, self.callback_juego)
        #rospy.wait_for_message("/juego", Int16)
        rospy.sleep(1)
         #publicaciones
        self.pub_coordenadasRojas = rospy.Publisher("/coordenadasRojas", PoseArray, queue_size=10)
        self.pub_coordenadasAzules = rospy.Publisher("/coordenadasAzules", PoseArray, queue_size=10)
        self.pub_coordenadasVerdes = rospy.Publisher("/coordenadasVerdes", PoseArray, queue_size=10)


        self.last_stable_centers = []
        self.estable_count = 0
        self.printed_stable_flag = False
        self.TOLERANCIA_CM = 0.2
        self.MAX_ERROR_CM = 2 * self.TOLERANCIA_CM
        self.numFramesEstables = 60  
        self.enviar1vez = True

        # Corcción lineal de Y hacia el ArUco 7
        try:
            bx = rospy.get_param("~baseline_x_cm", 38.0)
            by = rospy.get_param("~baseline_y_cm", 15.5)
            y_bias = rospy.get_param("~y_bias_at_id7_cm", 1.0)  # +1 cm por defecto; ajusta signo según error
            enable_corr = rospy.get_param("~enable_y_correction", True)
        except Exception:
            bx, by, y_bias, enable_corr = 38.0, 15.5, 1.0, True

        self.baseline_cm = np.array([bx, by], dtype=np.float32)
        self.y_bias_at_id7_cm = float(y_bias)
        self.enable_y_correction = bool(enable_corr)


        # Inicializar cámara
        #self.cap = cv2.VideoCapture(0)
        #if not self.cap.isOpened():
            #raise RuntimeError("No se pudo abrir la cámara")

    def callback_image(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def callback_juego(self, msg):
        self.juego = msg.data

    # ====================================================================
    # FUNCIONES AUXILIARES PARA PROCESAMIENTO DE OBJETOS
    # ====================================================================

    def _calcular_tolerancia_px(self, escala):
        """
        Convierte la tolerancia de CM a píxeles.
        
        Args:
            escala: Factor de conversión píxel-a-cm. None si no hay ArUco detectado.
            
        Returns:
            Tupla (TOLERANCIA_PX, tiene_escala)
        """
        if escala is not None:
            TOLERANCIA_PX = self.MAX_ERROR_CM / escala
            return TOLERANCIA_PX, True
        else:
            self.estable_count = 0
            return 0.0, False

    def _generar_lista_centros(self, red_centers, blue_centers, green_centers):
        """
        Agrupa centros rojos y azules en una lista única y ordenada.
        
        Args:
            red_centers: Lista de tuplas (x, y) de centros rojos o None
            blue_centers: Lista de tuplas (x, y) de centros azules o None
            
        Returns:
            Lista de tuplas ('R'|'B', x, y) ordenada por color, x, y
        """
        current_centers = []
        if red_centers is not None:
            for center in red_centers:
                current_centers.append(('R', center[0], center[1]))
        if blue_centers is not None:
            for center in blue_centers:
                current_centers.append(('B', center[0], center[1]))
        if green_centers is not None:
            for center in green_centers:
                current_centers.append(('G', center[0], center[1]))
        current_centers.sort()
        return current_centers

    def _comprobar_estabilidad(self, current_centers, TOLERANCIA_PX):
        """
        Verifica si los centros actuales son estables comparados con los últimos conocidos.
        
        Args:
            current_centers: Lista de centros actuales (color, x, y)
            TOLERANCIA_PX: Tolerancia en píxeles para la comparación
            
        Returns:
            True si son estables, False en caso contrario
        """
        es_estable = False
        
        # Condición 1: El número de objetos debe ser el mismo
        if len(current_centers) == len(self.last_stable_centers):
            coincidencias = 0
            
            # Condición 2: Distancia euclidiana dentro de tolerancia
            for i in range(len(current_centers)):
                color_actual, x_actual, y_actual = current_centers[i]
                color_estable, x_estable, y_estable = self.last_stable_centers[i]
                
                # El color debe coincidir
                if color_actual != color_estable:
                    break
                
                # Calcular la distancia euclidiana
                distancia_px = np.sqrt((x_actual - x_estable)**2 + (y_actual - y_estable)**2)
                
                if distancia_px <= TOLERANCIA_PX:
                    coincidencias += 1
                else:
                    break
            
            if coincidencias == len(current_centers):
                es_estable = True
        
        return es_estable

    def _actualizar_estabilidad(self, es_estable, current_centers):

        if es_estable:
            # Solo contar hasta numFramesEstables
            if self.estable_count < self.numFramesEstables:
                self.estable_count += 1

                # Print cada 15 frames
                if self.estable_count % 15 == 0:
                    porcentaje = int((self.estable_count / self.numFramesEstables) * 100)

                    bloques_llenos = int((self.estable_count / self.numFramesEstables) * 30)
                    barra = "█" * bloques_llenos + "░" * (30 - bloques_llenos)

                    print(
                        f"⏳ Verificando estabilidad: "
                        f"[{barra}] {porcentaje}% "
                        f"({self.estable_count}/{self.numFramesEstables})"
                    )

        else:
            # Si NO es estable → reinicio total
            self.estable_count = 0
            self.printed_stable_flag = False
            self.enviar1vez = True
            self.last_stable_centers = current_centers

        # Guardar referencia SOLO al empezar una nueva verificación
        if self.estable_count == 1:
            self.last_stable_centers = current_centers


    def _formato_coordenadas_cm(self, centers, color_label, escala, aruco_center):
        """
        Formatea una lista de centros en string con coordenadas en CM.
        
        Args:
            centers: Lista de tuplas (x, y) en píxeles o None
            color_label: String para identificar el color ("Rojo" o "Azul")
            escala: Factor de conversión píxel-a-cm
            aruco_center: Centro del ArUco en píxeles
            
        Returns:
            String formateado con las coordenadas en CM
        """
        coords_str = ""
        
        if centers and escala is not None and aruco_center is not None:
            for i, center_px in enumerate(centers):
                # Calcular offset relativo desde el centro del ArUco
                dx_px = center_px[0] - aruco_center[0]
                dy_px = center_px[1] - aruco_center[1]
                # Convertir a CM
                x_cm = dx_px * escala
                y_cm = dy_px * escala
                coords_str += f"{color_label}{i+1} X={x_cm:.2f}cm Y={y_cm:.2f}cm, "
        
        return coords_str

    def _imprimir_coordenadas_estables(self, red_centers, blue_centers, green_centers, escala, aruco_center):
        """
        Imprime las coordenadas estables en formato legible.
        Se llama cuando se alcanzan 60 frames con centros estables (dentro de tolerancia).
        
        Args:
            red_centers: Lista de centros rojos en píxeles o None
            blue_centers: Lista de centros azules en píxeles o None
            escala: Factor de conversión píxel-a-cm
            aruco_center: Centro del ArUco en píxeles
        """
        red_coords_str = self._formato_coordenadas_cm(red_centers, "Rojo", escala, aruco_center)
        blue_coords_str = self._formato_coordenadas_cm(blue_centers, "Azul", escala, aruco_center)
        green_coords_str = self._formato_coordenadas_cm(green_centers, "Verde", escala, aruco_center)
        
        
        #print("\n" + "="*60)
        #print("COORDENADAS VERIFICADAS Y ESTABLES")
        #print(f"   Confirmadas después de {self.numFramesEstables} frames de verificación")
        #print("="*60)
        #print(f"Coordenadas Rojas:  {red_coords_str}".rstrip(', '))
        #print(f"Coordenadas Azules: {blue_coords_str}".rstrip(', '))
        #print(f"Centro Aruco: {aruco_center}.rstrip(', ')")
        #print("="*60 + "\n")

    def _obtener_coordenadas_cm(self, centers, escala, aruco_center):
        """
        Convierte una lista de centros en píxeles a coordenadas en CM relativas al ArUco 23.
        
        Args:
            centers: Lista de tuplas (x, y) en píxeles o None
            escala: Factor de conversión píxel-a-cm
            aruco_center: Centro del ArUco 23 en píxeles
            
        Returns:
            Array Nx2 con coordenadas [X, Y] en CM, o None si no hay centros válidos
        """
        if centers is None or escala is None or aruco_center is None:
            return None
        
        coords_cm = []
        for center_px in centers:
            # Calcular offset relativo desde el centro del ArUco 23
            dx_px = center_px[0] - aruco_center[0]
            dy_px = center_px[1] - aruco_center[1]
            # Convertir a CM
            x_cm = dx_px * escala
            y_cm = dy_px * escala
            coords_cm.append([x_cm, y_cm])
        
        return np.array(coords_cm) if coords_cm else None

    def _procesar_objetos(self, undistorted, red_centers, blue_centers, green_centers, escala, aruco_center):
        """
        Procesa la lógica completa de detección, estabilidad e impresión de objetos.
        
        Args:
            undistorted: Imagen corregida
            red_centers: Lista de centros rojos
            blue_centers: Lista de centros azules
            green_centers: Lista de centros verdes
            escala: Factor de conversión píxel-a-cm
            aruco_center: Centro del ArUco detectado
            
        Returns:
            Tupla (undistorted, red_coords_cm, blue_coords_cm) donde:
            - undistorted: La imagen sin cambios
            - red_coords_cm: Array Nx2 con coordenadas [X, Y] en CM de fichas rojas, o None
            - blue_coords_cm: Array Nx2 con coordenadas [X, Y] en CM de fichas azules, o None
        """
        # 1. Calcular tolerancia en píxeles
        TOLERANCIA_PX, tiene_escala = self._calcular_tolerancia_px(escala)
        
        # 2. Generar lista de centros actual
        current_centers = self._generar_lista_centros(red_centers, blue_centers, green_centers)
        
        # 3. Comprobar estabilidad
        es_estable = self._comprobar_estabilidad(current_centers, TOLERANCIA_PX)
        
        # 4. Actualizar contador de estabilidad
        self._actualizar_estabilidad(es_estable, current_centers)
        
        # 5. Imprimir coordenadas cuando sean estables
        if self.estable_count >= self.numFramesEstables and not self.printed_stable_flag:
            self._imprimir_coordenadas_estables(red_centers, blue_centers, green_centers, escala, aruco_center)
            self.printed_stable_flag = True
        
        # 6. Obtener arrays de coordenadas en CM
        red_coords_cm = self._obtener_coordenadas_cm(red_centers, escala, aruco_center)
        blue_coords_cm = self._obtener_coordenadas_cm(blue_centers, escala, aruco_center)
        green_coords_cm = self._obtener_coordenadas_cm(green_centers, escala, aruco_center)
        
        return undistorted, red_coords_cm, blue_coords_cm, green_coords_cm

    def _aplicar_correccion_lineal_y(self, coords_cm):
        """
        Corrige Y linealmente: 0 cm en ArUco 23, y_bias_at_id7_cm en ArUco 7.
        Proyección del punto sobre la recta 23->7 en cm para escalar la corrección.
        """
        if coords_cm is None or len(coords_cm) == 0 or not self.enable_y_correction:
            return coords_cm

        b = self.baseline_cm.astype(np.float32)
        L = float(np.linalg.norm(b))
        if L < 1e-6:
            return coords_cm

        u = b / L  # unitario
        out = []
        for x_cm, y_cm in coords_cm:
            proj_len = x_cm * u[0] + y_cm * u[1]  # cm
            t = max(0.0, min(1.0, proj_len / L))  # fracción 0..1
            y_corr = y_cm + t * self.y_bias_at_id7_cm
            out.append([x_cm, y_corr])

        return np.array(out, dtype=np.float32)

    def array_to_pose_array(self, coords):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"

        if coords is None or len(coords) == 0:
            return pose_array  # devuelve vacío, pero válido

        for x, y in coords:
            pose = Pose()
            pose.position.x = x / 100.0
            pose.position.y = y / 100.0
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        return pose_array



    def start(self):

        # Convertir listas a matrices de OpenCV
        K = np.array(camera_matrix, dtype=np.float32)
        D = np.array(dist_coeffs, dtype=np.float32)

        # Obtener newCameraMatrix para undistort
        new_K, roi = cv2.getOptimalNewCameraMatrix(
            K, D, (IMAGE_WIDTH, IMAGE_HEIGHT), 1, (IMAGE_WIDTH, IMAGE_HEIGHT)
        )

        while not rospy.is_shutdown():

            # Esperar a recibir al menos una imagen de ROS
            if not hasattr(self, "img"):
                continue

            # Copiar la imagen que llega desde callback_image
            imagen_copy = deepcopy(self.img)

            # Redimensionar y corregir distorsión
            imagen_copy = cv2.resize(imagen_copy, (IMAGE_WIDTH, IMAGE_HEIGHT))
            undistorted = cv2.undistort(imagen_copy, K, D, None, new_K)

            # Estado de juego
            #juego = getattr(self, "juego", 1)  # Si no existe, usar 1
            juego = 1
            
            if juego == 1:
                undistorted, all_dist_red_cm, all_dist_blue_cm, all_dist_green_cm, red_centers, blue_centers, green_centers, escala, aruco_center = detectar_distancia_objetos(
                    undistorted, escala_cm=5.0, distancia_real_entre_arucos_cm=41.04
                )

                if aruco_center is not None:
                    dibujar_sistema_coordenadas(undistorted, aruco_center, escala, eje_length_cm=5)

                undistorted, red_coords_cm, blue_coords_cm, green_coords_cm = self._procesar_objetos(
                    undistorted, red_centers, blue_centers, green_centers, escala, aruco_center
                )

                # Corrección lineal de Y proporcional a la distancia 23->7
                red_coords_cm = self._aplicar_correccion_lineal_y(red_coords_cm)
                blue_coords_cm = self._aplicar_correccion_lineal_y(blue_coords_cm)
                green_coords_cm = self._aplicar_correccion_lineal_y(green_coords_cm)
                
                # Imprimir coordenadas solo cuando sean estables (después de 10 frames)
                if self.estable_count >= 10 and self.enviar1vez:
                    print("Red Coords (cm, Y corregido):", red_coords_cm)
                    print("Blue Coords (cm, Y corregido):", blue_coords_cm)
                    print("Green Coords (cm, Y corregido):", green_coords_cm)
                    # Convertir y publicar coordenadas rojas
                    msgR = self.array_to_pose_array(red_coords_cm)
                    self.pub_coordenadasRojas.publish(msgR)

                    # # Convertir y publicar coordenadas azules
                    msgA = self.array_to_pose_array(blue_coords_cm)
                    self.pub_coordenadasAzules.publish(msgA)
                    
                    msgV = self.array_to_pose_array(green_coords_cm)
                    self.pub_coordenadasVerdes.publish(msgV)
                    
                    print("coordenadas enviadas")

                    # En el nodo principal recibirás dos mensajes:

                    # /coordenadasRojas → PoseArray con 4 puntos

                    # /coordenadasAzules → PoseArray con 4 puntos

                    # Cada punto contiene:

                    # pose.position.x
                    # pose.position.y

                    #IGUAL HAY QUE MIRAR QUE SI YA LOS A BERIFICADO CON QUE LOS ENVIE UNA SOLA VEZ YA BASTA 
                    self.enviar1vez = False

            # Mostrar imagen
            cv2.imshow("Corregida (undistort) con ArUco y objetos", undistorted)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = GestosNode()
    node.start()