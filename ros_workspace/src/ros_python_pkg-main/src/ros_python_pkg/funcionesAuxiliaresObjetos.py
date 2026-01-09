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

def detectar_marcador_sin_aruco_2(img, marcador_real_cm=5.0):
    img_gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # DICCIONARIO CORRECTO DETECTADO: 6x6
    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    parametros = cv2.aruco.DetectorParameters()
    parametros.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX 

    detector = cv2.aruco.ArucoDetector(diccionario, parametros)
    corners, ids, _ = detector.detectMarkers(img_gris)

    if ids is None or 42 not in ids.flatten():
        return None, None, None

    # Obtener datos del ID 42
    idx = list(ids.flatten()).index(42)
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
def detectar_dos_marcadores(img, marcador_real_cm=5.0, distancia_real_entre_arucos_cm=None):
    """
    Detecta los dos ArUcos (7 y 23) y calcula la escala.
    
    Args:
        img: Imagen de entrada
        marcador_real_cm: Tamaño real de cada ArUco en cm (5 cm)
        distancia_real_entre_arucos_cm: Distancia real entre centros (opcional, para calibración)
    """
    img_gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    diccionario = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
    parametros = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(diccionario, parametros)

    corners, ids, _ = detector.detectMarkers(img_gris)

    # DEBUG: Ver qué IDs detecta
    if ids is not None:
        print(f"[DEBUG] ArUcos detectados: {ids.flatten().tolist()}")
    else:
        print("[DEBUG] No se detectaron ArUcos")

    if ids is None:
        return None

    marcadores = {}

    for i, marker_id in enumerate(ids.flatten()):
        if marker_id in [7, 23]:   # ⬅️ IDs correctos: 7 y 23
            pts = corners[i][0].astype(np.float32)

            # Ordenar esquinas
            s = pts.sum(axis=1)
            tl, br = pts[np.argmin(s)], pts[np.argmax(s)]
            diff = np.diff(pts, axis=1)
            tr, bl = pts[np.argmin(diff)], pts[np.argmax(diff)]

            lado_px = (np.linalg.norm(tr - tl) + np.linalg.norm(bl - tl)) / 2
            escala = marcador_real_cm / lado_px
            centro = np.mean([tl, tr, br, bl], axis=0)

            marcadores[marker_id] = {
                "corners": np.array([tl, tr, br, bl], dtype=np.int32),
                "center": centro,
                "scale": escala,
                "lado_px": lado_px
            }

    if 7 not in marcadores or 23 not in marcadores:
        return None

    # Usar escala promedio de ambos marcadores (más estable)
    escala_23 = marcadores[23]["scale"]
    escala_7 = marcadores[7]["scale"]
    escala_promedio = (escala_23 + escala_7) / 2
    
    print(f"[DEBUG] Escala ArUco 23: {escala_23:.6f} cm/px")
    print(f"[DEBUG] Escala ArUco 7: {escala_7:.6f} cm/px")
    print(f"[DEBUG] Escala promedio: {escala_promedio:.6f} cm/px")
    
    if distancia_real_entre_arucos_cm is not None:
        dist_px_entre_arucos = np.linalg.norm(marcadores[23]["center"] - marcadores[7]["center"])
        escala_desde_distancia = distancia_real_entre_arucos_cm / dist_px_entre_arucos
        print(f"[DEBUG] Distancia entre ArUcos (px): {dist_px_entre_arucos:.2f}")
        print(f"[DEBUG] Distancia entre ArUcos (cm): {distancia_real_entre_arucos_cm:.2f} (esperada)")
        print(f"[DEBUG] Escala desde distancia: {escala_desde_distancia:.6f} cm/px")
        
        # Usar la escala calculada desde la distancia real (más precisa que individual)
        marcadores[23]["scale"] = escala_desde_distancia
        marcadores[7]["scale"] = escala_desde_distancia

    return marcadores

def calcular_origen_y_escala(marcadores):
    A = marcadores[23]  # ArUco 23 como referencia principal
    B = marcadores[7]   # ArUco 7 como segundo marcador

    # Origen = punto medio
    origen = ((A["center"] + B["center"]) / 2).astype(int)

    # Escala media
    escala = (A["scale"] + B["scale"]) / 2

    # Vector X real
    eje_x = B["center"] - A["center"]
    angulo = np.degrees(np.arctan2(eje_x[1], eje_x[0]))

    return origen, escala, angulo

# ============================================================
# 2. DETECCIÓN DE CUBOS (COLOR + FORMA + EROSIÓN)
# ============================================================
def detectar_cubos(img, escala, origen):
    import math  # Necesario para calcular la línea de dirección visual
    
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, w = img.shape[:2]

    # Rangos de color HSV
    rangos = {
        "azul":  ([85, 40, 20], [135, 255, 255]), 
        "verde": ([35, 60, 40], [85, 255, 255]) 
    }

    print("\n Buscando cubos...")
    
    resultados = []

    for color, (low, high) in rangos.items():
        mask = cv2.inRange(hsv, np.array(low), np.array(high))
        
        if color == "azul":
            kernel_erosion = np.ones((5,5), np.uint8)
            mask = cv2.erode(mask, kernel_erosion, iterations=2)

        kernel_ruido = np.ones((3,3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_ruido)
        
        contornos, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        count = 0
        for c in contornos:
            area = cv2.contourArea(c)
            if area < 30: 
                continue

            rect = cv2.minAreaRect(c)
            (cx, cy), (w_box, h_box), angle = rect
            
            relacion_aspecto = float(w_box) / h_box if h_box > 0 else 0
            if not (0.5 < relacion_aspecto < 2.0):
                continue 

            cx, cy = int(cx), int(cy)

            if w_box < h_box: 
                angle += 90
            
            ang_fmt = round(angle, 2)
            
            dx_cm = round((cx - origen[0]) * escala, 2)
            dy_cm = round((origen[1] - cy) * escala, 2)

            resultados.append(f"{color.upper()} -> Pos:({dx_cm}, {dy_cm}) cm | Ángulo: {ang_fmt}°")

            box = np.intp(cv2.boxPoints(rect))
            color_bgr = (255, 0, 0) if color == "azul" else (0, 255, 0)
            cv2.drawContours(img, [box], 0, color_bgr, 2)
            
            cv2.circle(img, (cx, cy), 4, (0,0,255), -1)

            texto = f"({dx_cm},{dy_cm}) {ang_fmt}deg"
            cv2.putText(img, texto, (cx-40, cy-25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)
            
            rad = math.radians(angle)
            end_x = int(cx + 20 * math.cos(rad))
            end_y = int(cy + 20 * math.sin(rad))
            cv2.line(img, (cx, cy), (end_x, end_y), (0, 0, 0), 2)
            
            count += 1
        
        print(f"   -> Encontrados {count} objetos color {color}.")

    # Dibujar ejes centrados en el QR
    cv2.line(img, (origen[0], 0), (origen[0], h), (0, 255, 255), 1)
    cv2.line(img, (0, origen[1]), (w, origen[1]), (0, 255, 255), 1)
    cv2.putText(img, "Centro (0,0)", (origen[0]+5, origen[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
    cv2.circle(img, origen, 5, (0,0,255), -1)

    return resultados
def detectar_distancia_objetos(undistorted, escala_cm=5.0, distancia_real_entre_arucos_cm=None):
    """
    Detecta la distancia del sistema de referencia (ArUco 23)
    a objetos rojos y azules en cm.
    Usa 2 ArUcos para calcular una escala más estable.
    
    Args:
        undistorted: Imagen sin distorsión
        escala_cm: Tamaño de cada ArUco en cm (5 cm)
        distancia_real_entre_arucos_cm: Distancia real entre centros de ArUcos (opcional)
    """

    # ==== 1️⃣ Detectar DOS ArUcos ====
    marcadores = detectar_dos_marcadores(undistorted, marcador_real_cm=escala_cm, 
                                         distancia_real_entre_arucos_cm=distancia_real_entre_arucos_cm)
    if marcadores is None:
        return undistorted, None, None, None, None, None, None

    # Escala promedio de ambos marcadores (más estable)
    escala = (marcadores[23]["scale"] + marcadores[7]["scale"]) / 2
    
    # Usar el centro del ArUco 23 como origen del sistema de coordenadas
    aruco_center = marcadores[23]["center"]

    # Dibujar ambos ArUcos
    for marker_id, m in marcadores.items():
        color = (0, 255, 0) if marker_id == 23 else (0, 255, 255)  # Verde para 23, amarillo para 7
        cv2.polylines(undistorted, [m["corners"]], True, color, 2)
        c = m["center"].astype(int)
        cv2.circle(undistorted, tuple(c), 4, color, -1)
        cv2.putText(undistorted, f"ID:{marker_id}", (c[0]-20, c[1]-15), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # ==== 2️⃣ Crear máscaras de color ====
    hsv = cv2.cvtColor(undistorted, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)

    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

    # ==== 3️⃣ Detectar centros ====
    red_center = detectar_objeto(mask_red)
    blue_center = detectar_objeto(mask_blue)

    all_dist_red_cm = []
    all_dist_blue_cm = []

    # --- Objetos Rojos ---
    if red_center:
        for center in red_center:
            center_tuple = tuple(center.astype(int))
            cv2.circle(undistorted, center_tuple, 5, (0, 0, 255), -1)
            cv2.line(undistorted, tuple(aruco_center.astype(int)), center_tuple, (0, 0, 255), 1)

            # Calcular distancias X e Y desde ArUco 23
            dx_px = center[0] - aruco_center[0]
            dy_px = center[1] - aruco_center[1]
            dx_cm = dx_px * escala
            dy_cm = dy_px * escala
            dist_cm = np.linalg.norm(center - aruco_center) * escala
            all_dist_red_cm.append({'x': dx_cm, 'y': dy_cm, 'dist': dist_cm})

            cv2.putText(
                undistorted, f"X:{dx_cm:.1f} Y:{dy_cm:.1f}cm",
                (center_tuple[0], center_tuple[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (0, 0, 255), 1
            )

    # --- Objetos Azules ---
    if blue_center:
        for center in blue_center:
            center_tuple = tuple(center.astype(int))
            cv2.circle(undistorted, center_tuple, 5, (255, 0, 0), -1)
            cv2.line(undistorted, tuple(aruco_center.astype(int)), center_tuple, (255, 0, 0), 1)

            # Calcular distancias X e Y desde ArUco 23
            dx_px = center[0] - aruco_center[0]
            dy_px = center[1] - aruco_center[1]
            dx_cm = dx_px * escala
            dy_cm = dy_px * escala
            dist_cm = np.linalg.norm(center - aruco_center) * escala
            all_dist_blue_cm.append({'x': dx_cm, 'y': dy_cm, 'dist': dist_cm})

            cv2.putText(
                undistorted, f"X:{dx_cm:.1f} Y:{dy_cm:.1f}cm",
                (center_tuple[0], center_tuple[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (255, 0, 0), 1
            )

    return undistorted, all_dist_red_cm, all_dist_blue_cm, red_center, blue_center, escala, aruco_center


def dibujar_sistema_coordenadas(img, centro_aruco, escala, eje_length_cm=5, rotacion_deg=-85):#-100.5):
    """
    Dibuja un sistema de coordenadas rotado desde el centro del ArUco.
    - centro_aruco: tupla (x, y) del centro del ArUco en píxeles
    - escala: factor de conversión píxeles a cm
    - eje_length_cm: longitud de los ejes en cm
    - rotacion_deg: rotación en grados del sistema respecto al original
    """
    if escala is None or centro_aruco is None:
        return
    
    # Convertir longitud de eje de cm a píxeles
    eje_length_px = int(eje_length_cm / escala)
    
    # Convertir grados a radianes
    theta = np.radians(rotacion_deg)
    
    # Punto de origen
    origin = np.array(centro_aruco.astype(int))
    
    # Vector original de ejes (X y Y)
    eje_x = np.array([eje_length_px, 0])
    eje_y = np.array([0, eje_length_px])
    
    # Matriz de rotación 2D
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta),  np.cos(theta)]])
    
    # Rotar los ejes
    x_end = origin + R @ eje_x
    y_end = origin + R @ eje_y
    
    # Dibujar eje X (rojo)
    cv2.arrowedLine(img, tuple(origin), tuple(x_end.astype(int)), (0, 0, 255), 2, tipLength=0.3)
    cv2.putText(img, f"X({eje_length_cm}cm)", tuple(x_end.astype(int) + np.array([5,0])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    
    # Dibujar eje Y (azul)
    cv2.arrowedLine(img, tuple(origin), tuple(y_end.astype(int)), (255, 0, 0), 2, tipLength=0.3)
    cv2.putText(img, f"Y({eje_length_cm}cm)", tuple(y_end.astype(int) + np.array([5,0])),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    
    # Círculo verde en el origen
    cv2.circle(img, tuple(origin), 5, (0, 255, 0), -1)
    cv2.putText(img, "O(0,0)", (origin[0]-40, origin[1]-10), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
