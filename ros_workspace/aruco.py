import numpy as np
import cv2 as cv
import sys

# --- 1. Inicialización de la Detección ArUco ---
# Usamos el mismo diccionario que se vio en el video: DICT_4X4_100
aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_100)

# Configuramos los parámetros del detector
detector_params = cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(aruco_dict, detector_params)


# --- 2. Inicialización de la Cámara y Ventana ---
cap = cv.VideoCapture(0)

# Comprobación de que la cámara se ha abierto correctamente
if not cap.isOpened():
    print("Error: No se puede abrir la cámara (VideoCapture(0)).")
    sys.exit()

# CREAMOS LA VENTANA PERSISTENTE (SOLUCIÓN AL PROBLEMA)
window_name = 'Detección ArUco'
cv.namedWindow(window_name, cv.WINDOW_AUTOSIZE)

print("Cámara iniciada. Presiona 'q' para salir.")

# ----------------------------------------------------

# --- 3. Bucle Principal de Captura y Detección ---
while True:
    # Capturar frame por frame
    ret, frame = cap.read()
    
    # Si 'ret' es False, significa que no se pudo leer el frame (error de cámara o fin de stream)
    if not ret:
        print("Error: No se pudo recibir frame.")
        break
    
    # Convertir el frame a escala de grises para un procesamiento más rápido
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Detección de los marcadores ArUco
    corners, ids, rejected = detector.detectMarkers(gray)

    
    # Dibujar la esquina y el contorno del marcador
    if ids is not None:
        
        # Iterar sobre todos los marcadores detectados
        for i, c in enumerate(corners):
            # c[0, 0] es la esquina superior izquierda (índice 0)
            corner_point = tuple(c[0][0].astype(int))
            
            # Dibujar el contorno del marcador
            cv.aruco.drawDetectedMarkers(frame, [c])
            
            # Dibujar un círculo grande y rojo en la primera esquina (superior izquierda)
            cv.circle(frame, corner_point, 10, (0, 0, 255), -1) # Rojo y relleno
            
            # Mostrar el ID detectado cerca del marcador
            text = f"ID: {ids[i][0]}"
            cv.putText(frame, text, (corner_point[0], corner_point[1] - 20), 
                       cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


    # Mostrar el frame resultante usando el MISMO nombre de ventana
    cv.imshow(window_name, frame)
    
    # Salir del bucle si se presiona la tecla 'q'
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# --- 4. Limpieza ---
cap.release()
cv.destroyAllWindows()