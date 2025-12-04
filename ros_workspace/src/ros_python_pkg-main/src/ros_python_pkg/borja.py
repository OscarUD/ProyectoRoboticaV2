#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image  # IMPORTANTE: Mensaje de imagen
from cv_bridge import CvBridge, CvBridgeError # IMPORTANTE: Para convertir ROS -> OpenCV
import cv2
import mediapipe as mp
import numpy as np
import math
import time
from collections import deque
mp_hands = mp.solutions.hands
hands_detector = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5)
# ==============================================================================
#                      CLASE PARA GESTIONAR LA CÁMARA ROS
# ==============================================================================

class FuenteCamaraROS:
    """
    Esta clase simula el comportamiento de cv2.VideoCapture pero
    obteniendo las imágenes desde un tópico de ROS.
    """
    def __init__(self, topic_name="/usb_cam/image_raw"):
        self.bridge = CvBridge()
        self.current_frame = None
        self.image_received = False
        
        # AQUÍ ESTÁ EL SUSCRIPTOR QUE FALTABA
        self.sub = rospy.Subscriber(topic_name, Image, self.callback_imagen)
        rospy.loginfo(f"Suscrito al topico de camara: {topic_name}")
        rospy.loginfo("Esperando primera imagen...")

    def callback_imagen(self, data):
        try:
            # Convertimos el mensaje ROS a imagen OpenCV (BGR)
            self.current_frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(e)

    def read(self):
        """
        Simula la función .read() de OpenCV.
        Devuelve (ret, frame).
        """
        if self.image_received and self.current_frame is not None:
            return True, self.current_frame.copy()
        else:
            return False, None

    def isOpened(self):
        return True # Asumimos abierto si el nodo corre

    def release(self):
        self.sub.unregister()

# ==============================================================================
#                               CONFIGURACIÓN Y UTILIDADES
# ==============================================================================

mp_hands = mp.solutions.hands

def inicializar_mediapipe(max_manos=2, conf_deteccion=0.5, conf_seguimiento=0.5):
    return mp_hands.Hands(
        max_num_hands=max_manos,
        min_detection_confidence=conf_deteccion,
        min_tracking_confidence=conf_seguimiento
    )

def detectar_mano(hands, frame):
    # Convertimos a RGB para MediaPipe
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(frame_rgb)
    return results

def generar_mascara_opencv(frame, bbox):
    h, w, _ = frame.shape
    x_min, y_min, x_max, y_max = bbox
    x_min, y_min = max(0, x_min), max(0, y_min)
    x_max, y_max = min(w, x_max), min(h, y_max)

    roi = frame[y_min:y_max, x_min:x_max]
    if roi.size == 0: return np.zeros((h, w), dtype=np.uint8)
    
    roi_ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    lower = np.array([0, 133, 77], dtype=np.uint8)
    upper = np.array([255, 173, 127], dtype=np.uint8)
    
    mask_roi = cv2.inRange(roi_ycrcb, lower, upper)
    mask_roi = cv2.GaussianBlur(mask_roi, (7, 7), 0)
    
    mask_global = np.zeros((h, w), dtype=np.uint8)
    mask_global[y_min:y_max, x_min:x_max] = mask_roi
    return mask_global

def luminosidad(roi):
    if roi is None or roi.size == 0:
        return np.array([0, 133, 77], dtype=np.uint8), np.array([255, 173, 127], dtype=np.uint8)
    ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    Y_channel = ycrcb[:, :, 0]
    lum_mean = np.mean(Y_channel)
    if lum_mean < 80: lower_Y, upper_Y = 0, 200
    elif lum_mean > 180: lower_Y, upper_Y = 40, 255
    else: lower_Y, upper_Y = 0, 255
    return np.array([lower_Y, 133, 77], dtype=np.uint8), np.array([upper_Y, 173, 127], dtype=np.uint8)

# ==============================================================================
#                         LÓGICA JUEGO 1: RPSLS
# ==============================================================================

def detectar_gesto_ppt(mask):
    kernel = np.ones((5,5), np.uint8)
    mask_proc = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask_proc = cv2.medianBlur(mask_proc, 5)

    contornos, _ = cv2.findContours(mask_proc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contornos: return "Sin mano"

    contorno = max(contornos, key=cv2.contourArea)
    if cv2.contourArea(contorno) < 1500: return "Sin mano"

    hull_idx = cv2.convexHull(contorno, returnPoints=False)
    defects = cv2.convexityDefects(contorno, hull_idx) if hull_idx is not None and len(hull_idx) > 2 else None

    defectos_validos = 0
    angulos_defectos = []

    if defects is not None:
        x, y, bw, bh = cv2.boundingRect(contorno)
        tamaño_mano = max(bw, bh)
        for i in range(defects.shape[0]):
            s, e, f, d = defects[i,0]
            start = tuple(contorno[s][0])
            end   = tuple(contorno[e][0])
            far   = tuple(contorno[f][0])
            x1,y1 = start; x2,y2 = end; x0,y0 = far
            denom = math.hypot(x2-x1, y2-y1)
            if denom == 0: continue
            depth_px = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1) / denom
            depth_norm = depth_px / (tamaño_mano + 1e-6)
            a = np.array(start); b = np.array(far); c = np.array(end)
            ba = a - b; bc = c - b
            norm_ba = np.linalg.norm(ba); norm_bc = np.linalg.norm(bc)
            if norm_ba == 0 or norm_bc == 0: continue
            cos_angle = np.dot(ba, bc) / (norm_ba * norm_bc)
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle_deg = math.degrees(math.acos(cos_angle))
            if depth_norm > 0.03 and angle_deg < 100:
                defectos_validos += 1
                angulos_defectos.append(angle_deg)

    if defectos_validos == 0: return "Piedra"
    elif defectos_validos == 4: return "Papel"
    elif defectos_validos == 2: return "Spock"
    elif defectos_validos == 1:
        if len(angulos_defectos) > 0 and angulos_defectos[0] > 60: return "Lagarto"
        else: return "Tijera"
    return "Desconocido"

class GestorJuegoRPSLS:
    def __init__(self, publisher):
        self.pub = publisher
        self.puntos_izq = 0; self.puntos_der = 0
        self.ronda_actual = 1; self.ganador_partida = None
        self.estado = "ESPERANDO"
        self.t_inicio_countdown = 0; self.duracion_countdown = 3
        self.t_inicio_resultado = 0; self.duracion_resultado = 4
        self.mensaje_resultado = ""; self.gestos_bloqueados = {}
        self.history_len = 20; self.stable_threshold = 10
        self.histories = {'Izquierda': deque(maxlen=self.history_len), 'Derecha': deque(maxlen=self.history_len)}
        self.candidate = {'Izquierda': None, 'Derecha': None}
        self.cheating = {'Izquierda': False, 'Derecha': False}
        self.cheating_reason = {'Izquierda': None, 'Derecha': None}
        self.reveal_stable_frames = {'Izquierda': 0, 'Derecha': 0}
        self.reveal_timeout = 2.0; self.t_reveal_start = 0; self.change_detection_threshold = 8
        self.reglas = {"Piedra": ["Tijera", "Lagarto"], "Papel": ["Piedra", "Spock"], "Tijera": ["Papel", "Lagarto"], "Lagarto": ["Papel", "Spock"], "Spock": ["Tijera", "Piedra"]}

    def actualizar(self, info_manos, frame):
        h, w, _ = frame.shape
        gL = next((m['gesto'] for m in info_manos if m['label'] == "Izquierda"), None)
        gR = next((m['gesto'] for m in info_manos if m['label'] == "Derecha"), None)

        self.pub.publish(f"RPSLS | Estado: {self.estado} | Ronda: {self.ronda_actual}")

        if self.estado == "ESPERANDO":
            if gL == "Piedra" and gR == "Piedra":
                self.estado = "CUENTA_ATRAS"
                self.t_inicio_countdown = time.time()
                for k in self.histories.keys(): self.histories[k].clear(); self.candidate[k] = None; self.cheating[k] = False
            cv2.putText(frame, "Ambos PIEDRA para empezar", (w//2 - 200, h - 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        elif self.estado == "CUENTA_ATRAS":
            for label, gesto in (("Izquierda", gL), ("Derecha", gR)):
                val = gesto if gesto else "Sin mano"
                self.histories[label].append(val)
                if self.candidate[label] is None:
                    if len(self.histories[label]) >= self.stable_threshold:
                        last_vals = list(self.histories[label])[-self.stable_threshold:]
                        if all(v == last_vals[0] for v in last_vals) and last_vals[0] != "Sin mano": self.candidate[label] = last_vals[0]
            elapsed = time.time() - self.t_inicio_countdown
            countdown = self.duracion_countdown - int(elapsed)
            if countdown > 0: cv2.putText(frame, str(countdown), (w//2 - 40, h//2), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 255), 8)
            else:
                self.estado = "REVEAL"; self.t_reveal_start = time.time()
                for k in self.cheating_reason.keys(): self.cheating_reason[k] = None; self.reveal_stable_frames[k] = 0
        elif self.estado == "REVEAL":
            elapsed_reveal = time.time() - self.t_reveal_start
            currL = gL if gL else "Sin mano"; currR = gR if gR else "Sin mano"
            for label, curr in (("Izquierda", currL), ("Derecha", currR)):
                cand = self.candidate[label]
                if cand is not None and self.cheating_reason[label] != 'change':
                    if curr == cand: self.reveal_stable_frames[label] += 1
                    else:
                        if self.reveal_stable_frames[label] >= self.change_detection_threshold: self.cheating[label] = True; self.cheating_reason[label] = 'change'
                        else: self.reveal_stable_frames[label] = 0
            if elapsed_reveal >= self.reveal_timeout:
                finalL = self.candidate['Izquierda'] if self.candidate['Izquierda'] else (gL if gL else "Sin mano")
                finalR = self.candidate['Derecha'] if self.candidate['Derecha'] else (gR if gR else "Sin mano")
                for label, final, cand in (("Izquierda", finalL, self.candidate['Izquierda']), ("Derecha", finalR, self.candidate['Derecha'])):
                    if not self.cheating[label]:
                        if cand is None and final == "Sin mano": self.cheating[label] = True; self.cheating_reason[label] = 'late'
                        elif cand is not None and final == "Sin mano": self.cheating[label] = True; self.cheating_reason[label] = 'late'
                self.gestos_bloqueados = {"Izquierda": finalL, "Derecha": finalR}
                reasons = []
                for label in ('Izquierda', 'Derecha'):
                    if self.cheating[label]:
                        r = self.cheating_reason[label] or ('change' if self.cheating[label] else 'late')
                        reasons.append(f"{label}: {('Cambio' if r=='change' else 'Retraso')}")
                if reasons: self.mensaje_resultado = "Trampa: " + ", ".join(reasons); self.ronda_actual += 1; self.verificar_ganador_partida()
                else: self.evaluar_ronda()
                self.estado = "RESULTADO"; self.t_inicio_resultado = time.time()
        elif self.estado == "RESULTADO":
            self.mostrar_resultado_ronda(frame)
            if time.time() - self.t_inicio_resultado > self.duracion_resultado:
                if self.ganador_partida: self.estado = "FIN_PARTIDA"
                else: self.estado = "ESPERANDO"; self.mensaje_resultado = ""; self.gestos_bloqueados = {}
        elif self.estado == "FIN_PARTIDA":
            cv2.putText(frame, f"GANADOR: {self.ganador_partida}", (w//2 - 300, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 215, 255), 4)
            cv2.putText(frame, "PUÑO para Salir / PIEDRA reiniciar", (w//2 - 250, h//2 + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            if gL == "Piedra" and gR == "Piedra": self.reiniciar_partida()
        self.dibujar_marcador(frame)

    def evaluar_ronda(self):
        gL = self.gestos_bloqueados["Izquierda"]; gR = self.gestos_bloqueados["Derecha"]
        if gL == "Sin mano" or gR == "Sin mano": self.mensaje_resultado = "Nula"
        elif gL == gR: self.mensaje_resultado = f"Empate ({gL})"
        elif gR in self.reglas.get(gL, []): self.puntos_izq += 1; self.mensaje_resultado = f"Punto Izq ({gL} vence {gR})"
        elif gL in self.reglas.get(gR, []): self.puntos_der += 1; self.mensaje_resultado = f"Punto Der ({gR} vence {gL})"
        else: self.mensaje_resultado = "Desconocido"
        self.ronda_actual += 1; self.verificar_ganador_partida()

    def verificar_ganador_partida(self):
        if self.puntos_izq >= 3: self.ganador_partida = "IZQUIERDA"
        elif self.puntos_der >= 3: self.ganador_partida = "DERECHA"

    def reiniciar_partida(self):
        self.puntos_izq = 0; self.puntos_der = 0; self.ronda_actual = 1; self.ganador_partida = None; self.estado = "ESPERANDO"

    def mostrar_resultado_ronda(self, frame):
        cv2.putText(frame, self.mensaje_resultado, (50, frame.shape[0] - 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def dibujar_marcador(self, frame):
        cv2.rectangle(frame, (0,0), (frame.shape[1], 60), (50, 50, 50), -1)
        cv2.putText(frame, f"IZQ: {self.puntos_izq}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
        cv2.putText(frame, f"DER: {self.puntos_der}", (frame.shape[1] - 180, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 128, 255), 2)
        cv2.putText(frame, f"Ronda: {self.ronda_actual}", (frame.shape[1]//2 - 70, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    def run(self, cap):
        hands = inicializar_mediapipe()
        print("Iniciando RPSLS...")
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret: 
                # Si no hay imagen de ROS aún, esperamos un poco y continuamos
                rospy.sleep(0.1)
                continue
            
            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            info_manos = []
            results = detectar_mano(hands, frame)
            if results.multi_hand_landmarks:
                tmp_manos = []
                for hand_landmarks in results.multi_hand_landmarks:
                    x_coords = [lm.x for lm in hand_landmarks.landmark]
                    y_coords = [lm.y for lm in hand_landmarks.landmark]
                    x_min, x_max = int(min(x_coords) * w), int(max(x_coords) * w)
                    y_min, y_max = int(min(y_coords) * h), int(max(y_coords) * h)
                    bbox = (x_min - 20, y_min - 20, x_max + 20, y_max + 20)
                    mask = generar_mascara_opencv(frame, bbox)
                    gesto = detectar_gesto_ppt(mask)
                    x_center = (x_min + x_max) // 2
                    tmp_manos.append({'bbox': bbox, 'mask': mask, 'gesto': gesto, 'x_center': x_center})
                tmp_manos.sort(key=lambda m: m['x_center'])
                if len(tmp_manos) >= 1: info_manos.append({'label': 'Izquierda', 'gesto': tmp_manos[0]['gesto'], 'mask': tmp_manos[0]['mask']})
                if len(tmp_manos) >= 2: info_manos.append({'label': 'Derecha', 'gesto': tmp_manos[-1]['gesto'], 'mask': tmp_manos[-1]['mask']})

            for m in info_manos:
                label = m['label']
                cv2.putText(frame, f"{label}: {m['gesto']}", (10 if label=='Izquierda' else w-200, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

            self.actualizar(info_manos, frame)
            cv2.imshow("Juego RPSLS (q para salir)", frame)
            if cv2.waitKey(5) & 0xFF == ord('q'): break
        cv2.destroyAllWindows()

# ==============================================================================
#                         LÓGICA JUEGO 2: VAQUEROS
# ==============================================================================
def get_hand_rois(frame, max_hands=1):
    """
    Detecta manos en un frame y devuelve una lista de ROIs válidos.
    
    Devuelve lista de diccionarios:
        {
            "roi": imagen recortada,
            "bbox": (x_min, y_min, x_max, y_max),
            "landmarks": hand_landmarks
        }
    """

    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands_detector.process(rgb)

    hand_rois = []

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks[:max_hands]:

            # Bounding box en coordenadas de píxeles
            x_coords = [lm.x for lm in hand_landmarks.landmark]
            y_coords = [lm.y for lm in hand_landmarks.landmark]

            x_min = int(min(x_coords) * w)
            x_max = int(max(x_coords) * w)
            y_min = int(min(y_coords) * h)
            y_max = int(max(y_coords) * h)

            # Padding seguro
            pad = 30
            x_min = max(0, x_min - pad)
            y_min = max(0, y_min - pad)
            x_max = min(w, x_max + pad)
            y_max = min(h, y_max + pad)

            # Validar ROI
            if x_max <= x_min or y_max <= y_min:
                continue  # salto si ROI inválido

            roi = frame[y_min:y_max, x_min:x_max]

            # Evitar ROIs demasiado pequeños
            if roi.shape[0] < 20 or roi.shape[1] < 20:
                continue

            hand_rois.append({
                "roi": roi,
                "bbox": (x_min, y_min, x_max, y_max),
                "landmarks": hand_landmarks
            })

    return hand_rois
def detect_gesture_vaqueros(roi):
    
    if roi is None:
        return "DESCONOCIDO"

    ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    #lower_skin = np.array([0, 133, 77], dtype=np.uint8)
    #upper_skin = np.array([255, 173, 127], dtype=np.uint8)
    lower_skin, upper_skin = luminosidad(roi) 
    mask = cv2.inRange(ycrcb, lower_skin, upper_skin)

    mask = cv2.GaussianBlur(mask, (5, 5), 0)


    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

    #mask = cv2.medianBlur(mask, 5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "DESCONOCIDO"

    max_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_contour)
    if area < 3000:
        return "DESCONOCIDO"

    hull = cv2.convexHull(max_contour, returnPoints=False)
    finger_count = 0
    hull_pts = cv2.convexHull(max_contour)

    if len(hull) > 3:
        defects = cv2.convexityDefects(max_contour, hull)
        if defects is not None:
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                start = tuple(max_contour[s][0])
                end = tuple(max_contour[e][0])
                far = tuple(max_contour[f][0])
                a = np.linalg.norm(np.array(start) - np.array(far))
                b = np.linalg.norm(np.array(end) - np.array(far))
                c = np.linalg.norm(np.array(start) - np.array(end))
                angle = np.arccos(np.clip((a**2 + b**2 - c**2) / (2 * a * b), -1.0, 1.0))
                if angle <= np.pi / 2 and d > 10000:
                    finger_count += 1

    x, y, w, h = cv2.boundingRect(max_contour)
    rect_area = w * h
    extent = float(area) / rect_area if rect_area > 0 else 0
    hull_area = cv2.contourArea(hull_pts)
    solidity = float(area) / hull_area if hull_area > 0 else 0
    aspect_ratio = float(w) / h if h > 0 else 0

    if finger_count <= 1 and solidity > 0.80 and w > h * 1.7:
        return "RECARGAR"
    elif solidity > 0.85 and finger_count <= 1 and extent > 0.5 and aspect_ratio > 0.5:
        return "PROTEGERSE"
    elif finger_count == 2 and solidity > 0.6 and aspect_ratio > 1:
        return "BOMBA"
    elif (aspect_ratio > 1.3 or (finger_count >= 1 and finger_count <= 3 and solidity > 0.6)) and h < w * 1.5:
        return "DISPARAR"
    
    return "DESCONOCIDO"


class VaquerosGame:
    def __init__(self, publisher):
        self.pub = publisher
        self.bullets_p1 = 1; self.bullets_p2 = 1
        self.score_p1 = 0; self.score_p2 = 0

    def process_round(self, g1, g2):
        res = ""; go = False
        if g1 == "RECARGAR": self.bullets_p1 += 1; res += "J1 Recarga. "
        if g2 == "RECARGAR": self.bullets_p2 += 1; res += "J2 Recarga. "
        if g1 == "BOMBA" and g2 == "BOMBA":
            if self.bullets_p1 >= 3 and self.bullets_p2 >= 3: self.bullets_p1 -= 3; self.bullets_p2 -= 3; res += "Doble Bomba (Empate). "
            else: res += "Intento de bomba fallido. "
        elif g1 == "DISPARAR":
            if self.bullets_p1 > 0:
                self.bullets_p1 -= 1
                if g2 == "PROTEGERSE": res += "J1 Dispara, J2 Bloquea. "
                elif g2 == "RECARGAR" or g2 == "BOMBA": self.score_p1 += 1; res += "J1 Gana Punto. "
                elif g2 == "DISPARAR":
                    if self.bullets_p2 > 0: self.bullets_p2 -= 1; res += "Cruce de disparos. "
                    else: self.score_p1 += 1; res += "J1 Gana (J2 disparó sin balas). "
                else: res += "J1 Dispara. "
            else: res += "J1 Click (Sin balas). "
        if g2 == "DISPARAR" and "J1 Gana" not in res and "Cruce" not in res:
             if self.bullets_p2 > 0:
                self.bullets_p2 -= 1
                if g1 == "PROTEGERSE": res += "J2 Dispara, J1 Bloquea. "
                elif g1 == "RECARGAR" or g1 == "BOMBA": self.score_p2 += 1; res += "J2 Gana Punto. "
                elif g1 == "DISPARAR" and self.bullets_p1 == 0: self.score_p2 += 1; res += "J2 Gana (J1 sin balas). "
        if self.score_p1 >= 3: res += "VICTORIA J1"; go = True
        elif self.score_p2 >= 3: res += "VICTORIA J2"; go = True
        return res, go

    def run(self, cap):
        print("Iniciando Vaqueros...")
        mp_hands_local = mp.solutions.hands.Hands(max_num_hands=2, min_detection_confidence=0.5)
        state = "CUENTA_ATRAS"; timer_start = time.time(); msg_resultado = ""
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.sleep(0.1)
                continue
            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = mp_hands_local.process(rgb)
            gestos_detectados = ["NINGUNO", "NINGUNO"]
            print(manos)
            if results.multi_hand_landmarks:
                manos = get_hand_rois(frame,2)
                if len(manos) >= 1: gestos_detectados[0] = detect_gesture_vaqueros(manos[0][1])
                if len(manos) >= 2: gestos_detectados[1] = detect_gesture_vaqueros(manos[1][1])
            cv2.putText(frame, f"J1: {self.score_p1} (B:{self.bullets_p1}) | J2: {self.score_p2} (B:{self.bullets_p2})", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
            if state == "CUENTA_ATRAS":
                t_restante = 4 - (time.time() - timer_start)
                if t_restante > 1: cv2.putText(frame, str(int(t_restante)), (w//2, h//2), cv2.FONT_HERSHEY_SIMPLEX, 4, (0,255,255), 5)
                elif t_restante > 0: cv2.putText(frame, "YA!", (w//2-100, h//2), cv2.FONT_HERSHEY_SIMPLEX, 4, (0,255,0), 5)
                else: state = "PROCESAR"
            elif state == "PROCESAR":
                res, game_over = self.process_round(gestos_detectados[0], gestos_detectados[1])
                msg_resultado = res
                self.pub.publish(f"Vaqueros: {res}")
                state = "RESULTADO"; timer_start = time.time()
                if game_over: state = "FIN"
            elif state == "RESULTADO":
                cv2.putText(frame, f"J1:{gestos_detectados[0]} vs J2:{gestos_detectados[1]}", (50, h-100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200,200,200), 2)
                cv2.putText(frame, msg_resultado, (50, h-50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                if time.time() - timer_start > 3: state = "CUENTA_ATRAS"; timer_start = time.time()
            elif state == "FIN": cv2.putText(frame, "JUEGO TERMINADO. 'q' para salir", (w//2-200, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 3)
            cv2.imshow("Juego Vaqueros", frame)
            if cv2.waitKey(5) & 0xFF == ord('q'): break
        cv2.destroyAllWindows()

# ==============================================================================
#                         SELECTOR DE JUEGO (Menú Principal)
# ==============================================================================

def detect_gameselector(roi):
    if roi is None or roi.size == 0: return "NINGUNO"
    ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    lower, upper = luminosidad(roi)
    mask = cv2.inRange(ycrcb, lower, upper)
    mask = cv2.GaussianBlur(mask, (5,5), 0)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours: return "NINGUNO"
    max_c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(max_c) < 3000: return "NINGUNO"
    hull = cv2.convexHull(max_c, returnPoints=False)
    if hull is None or len(hull) < 3: return "NINGUNO"
    defects = cv2.convexityDefects(max_c, hull)
    cnt = 0
    if defects is not None:
        for i in range(defects.shape[0]):
            s,e,f,d = defects[i,0]
            if d > 12000: cnt += 1 
    if cnt == 0: return "PPT" 
    elif cnt == 1: return "VAQUEROS"
    elif cnt >= 4: return "SALIR" 
    return "PPT"

class GameSelector:
    def run(self, cap, pub):
        mp_hands_sel = mp.solutions.hands.Hands(max_num_hands=1)
        stable_gesture = None; stable_count = 0
        print("=== SELECCION DE JUEGO ===")
        print("1 Dedo: RPSLS | 2 Dedos: VAQUEROS | Mano Abierta: SALIR")
        
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                rospy.sleep(0.1)
                continue

            frame = cv2.flip(frame, 1)
            h, w, _ = frame.shape
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = mp_hands_sel.process(rgb)
            gesture = "NINGUNO"
            if results.multi_hand_landmarks:
                hl = results.multi_hand_landmarks[0]
                xs = [lm.x for lm in hl.landmark]; ys = [lm.y for lm in hl.landmark]
                roi = frame[int(min(ys)*h):int(max(ys)*h), int(min(xs)*w):int(max(xs)*w)]
                gesture = detect_gameselector(roi)
                if gesture == stable_gesture: stable_count += 1
                else: stable_gesture = gesture; stable_count = 0
            cv2.putText(frame, "MENU PRINCIPAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Gesto: {gesture} ({stable_count}/30)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, "1 Dedo -> RPSLS", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
            cv2.putText(frame, "2 Dedos -> VAQUEROS", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
            cv2.imshow("Selector de Juego", frame)
            pub.publish(f"Selector: {gesture}")
            if stable_count > 30: return stable_gesture
            if cv2.waitKey(5) & 0xFF == ord('q'): return "SALIR"
        return "SALIR"

# ==============================================================================
#                               MAIN ROS NODE
# ==============================================================================

if __name__ == "__main__":
    rospy.init_node('nodo_juegos_gestuales')
    pub = rospy.Publisher('/juego/estado', String, queue_size=10)
    
    # IMPORTANTE: Definir el tópico de cámara correcto.
    # Habitualmente es '/usb_cam/image_raw' o '/camera/image_raw'
    TOPIC_CAMARA = "/camara1/usb_cam/image_raw"
    
    # En lugar de cv2.VideoCapture(0), usamos nuestra clase de ROS
    cap = FuenteCamaraROS(topic_name=TOPIC_CAMARA)
    
    selector = GameSelector()
    
    try:
        while not rospy.is_shutdown():
            opcion = selector.run(cap, pub)
            
            if opcion == "PPT":
                juego = GestorJuegoRPSLS(pub)
                juego.run(cap)
            elif opcion == "VAQUEROS":
                juego = VaquerosGame(pub)
                juego.run(cap)
            elif opcion == "SALIR":
                break
            
            time.sleep(1)
            
    except rospy.ROSInterruptException:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()