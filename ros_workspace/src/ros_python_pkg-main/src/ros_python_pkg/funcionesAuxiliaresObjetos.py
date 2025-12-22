import cv2
import numpy as np
import os

def detectar_objeto(mask):
    """
    Encuentra los centros de TODOS los contornos válidos en la máscara.
    """
    centros = []
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 50: # Usamos un filtro bajo, por ejemplo 50
            M = cv2.moments(cnt)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centros.append(np.array([cX, cY]))
                
    return centros

def detectar_marcador_sin_aruco(img, marcador_real_cm=5.0):
    img_gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # DICCIONARIO CORRECTO DETECTADO: 6x6
    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    parametros = cv2.aruco.DetectorParameters()
    parametros.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 

    detector = cv2.aruco.ArucoDetector(diccionario, parametros)
    corners, ids, _ = detector.detectMarkers(img_gris)

    if ids is None or 23 not in ids.flatten():
        return None, None, None

    # Obtener datos del ID 42
    idx = list(ids.flatten()).index(23)
    pts = corners[idx][0].astype(np.float32)

    # Ordenar esquinas y calcular escala
    s = pts.sum(axis=1)
    tl, br = pts[np.argmin(s)], pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    tr, bl = pts[np.argmin(diff)], pts[np.argmax(diff)]
    
    ancho = np.linalg.norm(tr - tl)
    alto = np.linalg.norm(bl - tl)
    lado_px = (ancho + alto) / 2
    escala = marcador_real_cm / lado_px 

    return np.array([tl, tr, br, bl], dtype=np.int32), escala, tl


def detectar_distancia_objetos(undistorted, escala_cm=5.0):
    """
    Detecta la distancia del ArUco a objetos rojo y azul en cm.
    Devuelve la imagen con anotaciones y las distancias.
    """
    # ==== 1️⃣ Detectar ArUco ====
    corners, escala, _ = detectar_marcador_sin_aruco(undistorted, marcador_real_cm=escala_cm)
    if corners is None:
        return undistorted, None, None, None, None, None, None

    # Calcular centro del ArUco
    cx, cy = np.mean(corners, axis=0).astype(int)
    aruco_center = np.array([cx, cy])

    # Dibujar cuadrado amarillo alrededor del ArUco
    x, y, w, h = cv2.boundingRect(corners)
    cv2.rectangle(undistorted, (x, y), (x + w, y + h), (0, 255, 255), 2)

    # ==== 2️⃣ Crear máscaras de color ====
    hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

    # Rojo (puede estar en dos rangos)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    # Azul
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # ==== 3️⃣ Encontrar contorno más grande de cada color ====
    
    red_center = detectar_objeto(mask_red)
    blue_center = detectar_objeto(mask_blue)

    # Inicializamos las listas de distancias (ya no es un solo valor)
    all_dist_red_cm = []
    all_dist_blue_cm = []

    if corners is None:
        # 7 valores siempre
        return undistorted, None, None, None, None, None, None

    # --- Objetos Rojos ---
    if red_center:
        for i, center in enumerate(red_center):
            # 1. Convertir el array de NumPy de 2 elementos a una tupla de enteros (x, y)
            center_tuple = tuple(center.astype(int)) 
            
            # 2. Dibujar todos los puntos rojos detectados (esto ya lo hacía)
            cv2.circle(undistorted, center_tuple, 5, (0, 0, 255), -1) 
            
            # 3. Dibujar la línea desde el ArUco a CADA objeto rojo
            cv2.line(undistorted, tuple(aruco_center), center_tuple, (0, 0, 255), 1)
            
            # 4. Calcular la distancia de CADA objeto
            dist_px = np.linalg.norm(center - aruco_center)
            #print("escala:",escala )
            dist_cm = dist_px * escala # convertir px a cm
            
            # 5. Almacenar la distancia
            all_dist_red_cm.append(dist_cm)

            # Opcional: Escribir la distancia junto al objeto en la imagen
            text = f"{dist_cm:.1f}cm"
            cv2.putText(undistorted, text, center_tuple, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)


    # --- Objetos Azules ---
    if blue_center:
        for i, center in enumerate(blue_center):
            # 1. Convertir el array de NumPy de 2 elementos a una tupla de enteros (x, y)
            center_tuple = tuple(center.astype(int))
            
            # 2. Dibujar todos los puntos azules detectados (esto ya lo hacía)
            cv2.circle(undistorted, center_tuple, 5, (255, 0, 0), -1) 
            
            # 3. Dibujar la línea desde el ArUco a CADA objeto azul
            cv2.line(undistorted, tuple(aruco_center), center_tuple, (255, 0, 0), 1)
            
            # 4. Calcular la distancia de CADA objeto
            dist_px = np.linalg.norm(center - aruco_center)
            dist_cm = dist_px * escala
            
            # 5. Almacenar la distancia
            all_dist_blue_cm.append(dist_cm)

            # Opcional: Escribir la distancia junto al objeto en la imagen
            text = f"{dist_cm:.1f}cm"
            cv2.putText(undistorted, text, center_tuple, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1, cv2.LINE_AA)

    return undistorted, all_dist_red_cm, all_dist_blue_cm, red_center, blue_center, escala, aruco_center

def dibujar_sistema_coordenadas(img, centro_aruco, escala, eje_length_cm=5):
    """
    Dibuja un sistema de coordenadas (ejes X e Y) desde el centro del ArUco.
    - centro_aruco: tupla (x, y) del centro del ArUco en píxeles
    - escala: factor de conversión píxeles a cm
    - eje_length_cm: longitud de los ejes en cm
    """
    if escala is None or centro_aruco is None:
        return
    
    # Convertir la longitud del eje de cm a píxeles
    eje_length_px = int(eje_length_cm / escala)
    
    # Punto de origen (centro del ArUco)
    origin = tuple(centro_aruco.astype(int))
    
    # Eje X (rojo) - hacia la derecha
    x_end = (origin[0] + eje_length_px, origin[1])
    cv2.arrowedLine(img, origin, x_end, (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(img, f"X({eje_length_cm}cm)", (x_end[0]+5, x_end[1]), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    # Eje Y (azul) - hacia abajo
    y_end = (origin[0], origin[1] + eje_length_px)
    cv2.arrowedLine(img, origin, y_end, (255, 0, 0), 2, tipLength=0.3)
    cv2.putText(img, f"Y({eje_length_cm}cm)", (y_end[0]+5, y_end[1]), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    
    # Círculo verde en el origen (0, 0)
    cv2.circle(img, origin, 5, (0, 255, 0), -1)
    cv2.putText(img, "O(0,0)", (origin[0]-40, origin[1]-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)