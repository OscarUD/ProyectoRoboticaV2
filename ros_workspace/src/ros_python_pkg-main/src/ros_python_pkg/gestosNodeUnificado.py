# -*- coding: utf-8 -*-
import cv2
import numpy as np
import mediapipe as mp
from copy import deepcopy
from funcionesAuxiliaresVaqueros import get_hand_rois, detect_selection_gesture, detect_gameselector, detect_gesture_juego, process_round
import time
from collections import deque
import math
import rospy
from sensor_msgs.msg import Image  # IMPORTAR CAMARA DE ROS
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError


    # --- CONFIGURACIÓN MEDIAPIPE ---
mp_hands = mp.solutions.hands
detector_manos = None

# --- ESTADO GLOBAL (Sustituye a la clase) ---
ESTADO_JUEGO = None

class GestosNode:

    #INIT
    def __init__(self):

        rospy.init_node("gestos", anonymous=True)
        #Subscripciones
        self.bridge = CvBridge()
        self.estado = "gameselector"
        self.sub_image = rospy.Subscriber("/camara2/usb_cam/image_raw", Image, self.callback_image)
        rospy.sleep(1)
        #publicaciones
        self.pub_juego = rospy.Publisher("/juego", Int16, queue_size=1)
        self.pub_ganador = rospy.Publisher("/ganador", Int16, queue_size=1)


        # Variables de estado
        self.estado = "gameselector"
        self.bullets_p1 = 0
        self.bullets_p2 = 0
        self.score_p1 = 0
        self.score_p2 = 0
        self.color_p1 = (255, 0, 0)  # Azul
        self.color_p2 = (0, 0, 255)  # Rojo
        
        # Variables para countdown
        self.countdown_active = False
        self.countdown_start_time = None
        self.countdown_duration = 3
        self.countdown_just_finished = False  

        # Inicializar cámara
        #self.cap = cv2.VideoCapture(0)
        #if not self.cap.isOpened():
            #raise RuntimeError("No se pudo abrir la cámara")

    
    
    def callback_image(self, msg):
        """ PROCESA CADA FRAME"""
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Error CvBridge: {e}")

    # Procesar frame (simulación de callback)
    def procesar_frame(self, imagen):
        cv2.imshow("Prueba", imagen)
        cv2.waitKey(1)
        return 1, 2

    # Procesar juego
    def procesar_juego(self, imagen):
        gesto = ""
        hand_rois = get_hand_rois(imagen, max_hands=1)
        for hand in hand_rois:
            roi = hand["roi"]
            gesto, mask, max_contour, hull_points = detect_gameselector(roi)
            # Dibujar hull en la ROI
            # if hull_points is not None:
            #     cv2.drawContours(mask, [hull_points], 0, (0, 255, 0), 2)
            cv2.imshow("Mask GameSelector", mask)
            cv2.waitKey(1)

        # Textos
        titulo1 = "Bienvenido a JuegosConGestos"
        titulo2 = "Seleccione el juego que quiera jugar"
        texto_info = "Puno = Salir | 1 dedo = PPTLS | 2 dedos = Vaqueros"
        texto_gesto = f"Gesto actual: {gesto}"

        font = cv2.FONT_HERSHEY_SIMPLEX
        escala = 0.55      # 🔹 Más pequeño
        grosor = 1         # 🔹 Más fino
        padding = 10

        # Tamaños de texto
        (w_t1, h_t1), _ = cv2.getTextSize(titulo1, font, escala, grosor)
        (w_t2, h_t2), _ = cv2.getTextSize(titulo2, font, escala, grosor)
        (w1, h1), _ = cv2.getTextSize(texto_info, font, escala, grosor)
        (w2, h2), _ = cv2.getTextSize(texto_gesto, font, escala, grosor)

        ancho_caja = max(w_t1, w_t2, w1, w2) + padding * 2
        alto_caja = h_t1 + h_t2 + h1 + h2 + padding * 5

        # Caja centrada arriba
        x = (imagen.shape[1] - ancho_caja) // 2
        y = 10

        # Fondo semitransparente
        overlay = imagen.copy()
        cv2.rectangle(overlay, (x, y), (x + ancho_caja, y + alto_caja), (25, 25, 25), -1)
        cv2.addWeighted(overlay, 0.40, imagen, 0.60, 0, imagen)

        # Colores mejorados
        color_titulo1 = (160, 180, 255)  # azul suave
        color_titulo2 = (170, 255, 170)  # verde suave
        color_texto = (230, 230, 230)    # gris claro elegante
        sombra = (0, 0, 0)

        # --- Título 1 ---
        x_t1 = x + (ancho_caja - w_t1) // 2
        y_t1 = y + h_t1 + padding

        for dx, dy, col in [(1, 1, sombra), (0, 0, color_titulo1)]:
            cv2.putText(imagen, titulo1, (x_t1 + dx, y_t1 + dy), font, escala, col, grosor, cv2.LINE_AA)

        # --- Título 2 ---
        x_t2 = x + (ancho_caja - w_t2) // 2
        y_t2 = y_t1 + h_t2 + padding

        for dx, dy, col in [(1, 1, sombra), (0, 0, color_titulo2)]:
            cv2.putText(imagen, titulo2, (x_t2 + dx, y_t2 + dy), font, escala, col, grosor, cv2.LINE_AA)

        # --- Info ---
        x_info = x + padding
        y_info = y_t2 + h1 + padding
        for dx, dy, col in [(1, 1, sombra), (0, 0, color_texto)]:
            cv2.putText(imagen, texto_info, (x_info + dx, y_info + dy), font, escala, col, grosor, cv2.LINE_AA)

        # --- Gesto actual ---
        x_gesto = x + (ancho_caja - w2) // 2
        y_gesto = y_info + h2 + padding
        for dx, dy, col in [(1, 1, sombra), (0, 0, color_texto)]:
            cv2.putText(imagen, texto_gesto, (x_gesto + dx, y_gesto + dy), font, escala, col, grosor, cv2.LINE_AA)

        cv2.imshow("GameSelector", imagen)
        cv2.waitKey(1)

        return gesto

    # Procesar gesto de inicio de ronda
    def procesar_gesto_inicio_ronda(self, imagen):
        gesto = None
        hand_roi = get_hand_rois(imagen, max_hands=1)
        for hand in hand_roi:
            roi = hand["roi"]
            gesto = detect_selection_gesture(roi)
            #print("Gesto detectado:", gesto)
        return gesto

    def start_countdown(self):
        """Iniciar la cuenta atrás"""
        self.countdown_active = True
        self.countdown_start_time = time.time()
    
    def get_countdown_text(self):
        """Obtener el texto y estado de la cuenta atrás"""
        if not self.countdown_active:
            return None, False
        
        elapsed = time.time() - self.countdown_start_time
        remaining = int(self.countdown_duration - elapsed) + 1
        
        if remaining > 0:
            self.countdown_just_finished = False
            return str(remaining), False
        elif elapsed < self.countdown_duration + 0.5:
            self.countdown_just_finished = False
            return "YA!", False
        else:
            # Countdown terminado
            if not self.countdown_just_finished:
                self.countdown_just_finished = True
                self.countdown_active = False
                return None, True
            else:
                return None, False

    def dibujar_countdown(self, imagen_copy, countdown_text):
        """
        Dibuja un countdown bonito centrado en pantalla.
        countdown_text: str, puede ser '3', '2', '1' o 'YA!'
        """

        if not countdown_text:
            return

        # Fuente y escalado
        font_cd = cv2.FONT_HERSHEY_SIMPLEX
        escala = 5          # tamaño grande
        grosor = 10         # grosor del texto

        # Colores
        color_texto = (0, 255, 255) if countdown_text != "YA!" else (0, 255, 0)  # amarillo o verde
        sombra = (0, 0, 0)

        # Obtener tamaño del texto para centrar
        (w_text, h_text), baseline = cv2.getTextSize(countdown_text, font_cd, escala, grosor)

        x = (imagen_copy.shape[1] - w_text) // 2
        y = (imagen_copy.shape[0] + h_text) // 2  # centrado verticalmente

        # Dibujar sombra
        cv2.putText(imagen_copy, countdown_text, (x + 5, y + 5), font_cd, escala, sombra, grosor, cv2.LINE_AA)
        # Dibujar texto principal
        cv2.putText(imagen_copy, countdown_text, (x, y), font_cd, escala, color_texto, grosor, cv2.LINE_AA)

    def render_vaqueros_hud(self, imagen_copy, countdown_text=None):
       
        font = cv2.FONT_HERSHEY_SIMPLEX
        escala_texto = 0.7
        grosor_texto = 2
        padding = 15

        # --- HUD superior ---
        hud_altura = 80
        overlay = imagen_copy.copy()
        cv2.rectangle(overlay, (0, 0), (imagen_copy.shape[1], hud_altura), (50, 50, 50), -1)
        cv2.addWeighted(overlay, 0.5, imagen_copy, 0.5, 0, imagen_copy)

        # --- Jugador 1 (izquierda) ---
        texto_p1 = [f"Jugador 1", f"Balas: {self.bullets_p2}"]
        x_p1 = padding
        y_offset = 30
        for txt in texto_p1:
            cv2.putText(imagen_copy, txt, (x_p1+2, y_offset+2), font, escala_texto, (0,0,0), grosor_texto+1, cv2.LINE_AA)
            cv2.putText(imagen_copy, txt, (x_p1, y_offset), font, escala_texto, self.color_p1, grosor_texto, cv2.LINE_AA)
            y_offset += int(escala_texto*30) + 5

        # --- Jugador 2 (derecha) ---
        texto_p2 = [f"Jugador 2", f"Balas: {self.bullets_p1}"]
        y_offset = 30
        for txt in texto_p2:
            (text_width, _), _ = cv2.getTextSize(txt, font, escala_texto, grosor_texto)
            x_p2 = imagen_copy.shape[1] - text_width - padding
            cv2.putText(imagen_copy, txt, (x_p2+2, y_offset+2), font, escala_texto, (0,0,0), grosor_texto+1, cv2.LINE_AA)
            cv2.putText(imagen_copy, txt, (x_p2, y_offset), font, escala_texto, self.color_p2, grosor_texto, cv2.LINE_AA)
            y_offset += int(escala_texto*30) + 5

        # --- Puntuaciones en el centro ---
        escala_punt = 1.5
        grosor_punt = 3
        texto_score_p1 = str(self.score_p2)
        texto_score_p2 = str(self.score_p1)

        centro = imagen_copy.shape[1] // 2
        offset = 60

        (w1, h1), _ = cv2.getTextSize(texto_score_p1, font, escala_punt, grosor_punt)
        x1 = centro - offset - w1
        y1 = hud_altura // 2 + h1 // 2
        cv2.putText(imagen_copy, texto_score_p1, (x1+2, y1+2), font, escala_punt, (0,0,0), grosor_punt+1, cv2.LINE_AA)
        cv2.putText(imagen_copy, texto_score_p1, (x1, y1), font, escala_punt, self.color_p1, grosor_punt, cv2.LINE_AA)

        (w2, h2), _ = cv2.getTextSize(texto_score_p2, font, escala_punt, grosor_punt)
        x2 = centro + offset
        y2 = hud_altura // 2 + h2 // 2
        cv2.putText(imagen_copy, texto_score_p2, (x2+2, y2+2), font, escala_punt, (0,0,0), grosor_punt+1, cv2.LINE_AA)
        cv2.putText(imagen_copy, texto_score_p2, (x2, y2), font, escala_punt, self.color_p2, grosor_punt, cv2.LINE_AA)

        # --- Countdown ---
        self.dibujar_countdown(imagen_copy, countdown_text)


        # --- Leyenda de gestos (abajo derecha, vertical con lista) ---
        gestos = ["       ", "Gestos:", "- Disparar", "- Recargar", "- Protegerse", "- Bomba"]
        escala_leyenda = 0.5
        grosor_leyenda = 1
        line_spacing = 25  # espaciado vertical

        # Calcular ancho máximo correctamente (incluyendo "Gestos:")
       # Obtener tamaño de cada texto correctamente
        text_sizes = [cv2.getTextSize(g, cv2.FONT_HERSHEY_SIMPLEX, escala_leyenda, grosor_leyenda)[0] for g in gestos]
        max_width = max([w for w, h in text_sizes])
        total_height = len(gestos) * line_spacing


        # Posición rectángulo
        x_g = imagen_copy.shape[1] - max_width - 15
        y_start = imagen_copy.shape[0] - total_height - 15

        # Fondo semitransparente
        overlay_g = imagen_copy.copy()
        cv2.rectangle(
            overlay_g,
            (x_g - 5, y_start - 5),
            (x_g + max_width + 5, y_start + total_height + 5),
            (25, 25, 25),
            -1
        )
        cv2.addWeighted(overlay_g, 0.6, imagen_copy, 0.4, 0, imagen_copy)

        # Dibujar cada línea de texto, alineado a la izquierda dentro del rectángulo
        y_offset = y_start
        for g in gestos:
            # Sombra
            cv2.putText(imagen_copy, g, (x_g + 1, y_offset + 1), cv2.FONT_HERSHEY_SIMPLEX, escala_leyenda, (0, 0, 0), grosor_leyenda+1, cv2.LINE_AA)
            # Texto principal
            cv2.putText(imagen_copy, g, (x_g, y_offset), cv2.FONT_HERSHEY_SIMPLEX, escala_leyenda, (255, 255, 255), grosor_leyenda, cv2.LINE_AA)
            y_offset += line_spacing


        # --- Mostrar ventana ---
        cv2.imshow("VAQUEROS", imagen_copy)
        cv2.waitKey(1)

    def procesar_ronda_vaqueros(self, imagen):
        gesto_j1 = None
        gesto_j2 = None

        hand_rois = get_hand_rois(imagen, max_hands=2)
        for idx, hand in enumerate(hand_rois):
            roi = hand["roi"]
            gesto, mask, max_contour, hull_pts = detect_gesture_juego(roi)
            if idx == 0:
                gesto_j2 = gesto
            elif idx == 1:
                gesto_j1 = gesto
            
            # Dibujar hull en la ROI
            if hull_pts is not None:
                cv2.drawContours(mask, [hull_pts], 0, (0, 255, 0), 2)
            
            # Mostrar máscara y ROI con hull
            window_mask = f"Mask P{idx+1}"
            #window_roi = f"Hull P{idx+1}"
            cv2.imshow(window_mask, mask)
            cv2.waitKey(1)
            #cv2.imshow(window_roi, roi)
            #cv2.waitKey(1)


        bullets_p1, bullets_p2, score_p1, score_p2, result, puntuador, game_over = process_round(
            self.bullets_p1, self.bullets_p2, self.score_p1, self.score_p2, gesto_j1, gesto_j2)

        print("RESULTADO RONDA:\n", result)
        print(f"Gesto Jugador 2:, {gesto_j2}, Gesto Jugador 1: {gesto_j1}")
        print(f"Jugador 2 - Balas: {bullets_p2}, Puntuacion: {score_p2}")
        print(f"Jugador 1 - Balas: {bullets_p1}, Puntuacion: {score_p1}")

        # Actualizar estado
        self.bullets_p1 = bullets_p1
        self.bullets_p2 = bullets_p2
        self.score_p1 = score_p1
        self.score_p2 = score_p2

        return puntuador, game_over



    def inicializar_estado(self):
        """Inicializa el diccionario que mantiene todo el estado del juego"""
        history_len = 20
        return {
            "puntos_izq": 0,
            "puntos_der": 0,
            "ronda_actual": 1,
            "ganador_partida": None,
            "estado": "ESPERANDO",
            "t_inicio_countdown": 0,
            "duracion_countdown": 3,
            "t_inicio_resultado": 0,
            "duracion_resultado": 4,
            "mensaje_resultado": "",
            "gestos_bloqueados": {},
            "stable_threshold": 10,
            "histories": {
                'Izquierda': deque(maxlen=history_len), 
                'Derecha': deque(maxlen=history_len)
            },
            "candidate": {'Izquierda': None, 'Derecha': None},
            "cheating": {'Izquierda': False, 'Derecha': False},
            "cheating_reason": {'Izquierda': None, 'Derecha': None},
            "reveal_stable_frames": {'Izquierda': 0, 'Derecha': 0},
            "reveal_timeout": 1.5,
            "t_reveal_start": 0,
            "change_detection_threshold": 8,
            "reglas": {
                "Piedra": ["Tijera", "Lagarto"],
                "Papel": ["Piedra", "Spock"],
                "Tijera": ["Papel", "Lagarto"],
                "Lagarto": ["Papel", "Spock"],
                "Spock": ["Tijera", "Piedra"]
            }
        }

    def inicializar_mediapipe(self, max_manos=2, conf_deteccion=0.5, conf_seguimiento=0.5):
        return mp_hands.Hands(
            max_num_hands=max_manos,
            min_detection_confidence=conf_deteccion,
            min_tracking_confidence=conf_seguimiento
        )

    def detectar_mano(self, hands, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(frame_rgb)
        return results

    def generar_mascara_opencv(self, frame, bbox):
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

    def detectar_gesto(self, mask):
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
                angle_deg = math.degrees(math.acos(np.clip(cos_angle, -1.0, 1.0)))

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

    # --- LÓGICA DEL JUEGO (Funciones puras o que modifican el estado) ---

    def reiniciar_partida(self, estado):
        estado["puntos_izq"] = 0
        estado["puntos_der"] = 0
        estado["ronda_actual"] = 1
        estado["ganador_partida"] = None
        estado["estado"] = "ESPERANDO"

    def verificar_ganador_partida(self, estado):
        if estado["puntos_izq"] >= 3:
            estado["ganador_partida"] = "IZQUIERDA"
        elif estado["puntos_der"] >= 3:
            estado["ganador_partida"] = "DERECHA"

    def evaluar_ronda(self, estado):
        gL = estado["gestos_bloqueados"]["Izquierda"]
        gR = estado["gestos_bloqueados"]["Derecha"]
        
        if gL == "Sin mano" or gR == "Sin mano":
            estado["mensaje_resultado"] = "Nula"
            return
        
        if gL == gR:
            estado["mensaje_resultado"] = f"Empate ({gL})"
            return
        
        reglas = estado["reglas"]
        if gR in reglas.get(gL, []):
            estado["puntos_izq"] += 1
            estado["mensaje_resultado"] = f"Punto Izq! ({gL} gana)"
            self.pub_ganador.publish(Int16(data=1))
        elif gL in reglas.get(gR, []):
            estado["puntos_der"] += 1
            estado["mensaje_resultado"] = f"Punto Der! ({gR} gana)"
            self.pub_ganador.publish(Int16(data=2))
        else:
            estado["mensaje_resultado"] = "Error"
        
        estado["ronda_actual"] += 1
        self.verificar_ganador_partida(estado)

    def dibujar_marcador(self,frame, estado):
        h, w, _ = frame.shape
        cv2.rectangle(frame, (0,0), (w, 60), (50, 50, 50), -1)
        cv2.putText(frame, f"IZQ: {estado['puntos_izq']}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 200, 0), 2)
        cv2.putText(frame, f"DER: {estado['puntos_der']}", (w - 180, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 128, 255), 2)
        cv2.putText(frame, f"Ronda: {estado['ronda_actual']}", (w//2 - 70, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    def mostrar_resultado_ronda(self, frame, estado):
        h, w, _ = frame.shape
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h-150), (w, h), (0,0,0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)
        cv2.putText(frame, estado["mensaje_resultado"], (50, h - 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def actualizar_logica_juego(self, info_manos, frame, estado):
        h, w, _ = frame.shape
        gL = next((m['gesto'] for m in info_manos if m['label'] == "Izquierda"), None)
        gR = next((m['gesto'] for m in info_manos if m['label'] == "Derecha"), None)
        
        curr_state = estado["estado"]

        if curr_state == "ESPERANDO":
            if gL == "Piedra" and gR == "Piedra":
                estado["estado"] = "CUENTA_ATRAS"
                estado["t_inicio_countdown"] = time.time()
                # Resetear historiales
                for k in estado["histories"]:
                    estado["histories"][k].clear()
                    estado["candidate"][k] = None
                    estado["cheating"][k] = False
            cv2.putText(frame, "Ambos PIEDRA para empezar", (w//2 - 200, h - 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        elif curr_state == "CUENTA_ATRAS":
            # Recolectar historia para candidato inicial
            for label, gesto in (("Izquierda", gL), ("Derecha", gR)):
                val = gesto if gesto else "Sin mano"
                estado["histories"][label].append(val)
                if estado["candidate"][label] is None:
                    if len(estado["histories"][label]) >= estado["stable_threshold"]:
                        last_vals = list(estado["histories"][label])[-estado["stable_threshold"]:]
                        if all(v == last_vals[0] for v in last_vals) and last_vals[0] != "Sin mano":
                            estado["candidate"][label] = last_vals[0]
            
            elapsed = time.time() - estado["t_inicio_countdown"]
            countdown = estado["duracion_countdown"] - int(elapsed)
            if countdown > 0:
                cv2.putText(frame, str(countdown), (w//2 - 40, h//2), cv2.FONT_HERSHEY_SIMPLEX, 4, (0, 255, 255), 8)
            else:
                # Transición a REVEAL
                estado["estado"] = "REVEAL"
                estado["t_reveal_start"] = time.time()
                # Limpiar historiales para detección de trampas
                for k in estado["histories"]:
                    estado["histories"][k].clear()
                    estado["candidate"][k] = None
                    estado["cheating"][k] = False
                for k in estado["cheating_reason"]:
                    estado["cheating_reason"][k] = None
                    estado["reveal_stable_frames"][k] = 0

        elif curr_state == "RESULTADO":
            self.mostrar_resultado_ronda(frame, estado)
            if time.time() - estado["t_inicio_resultado"] > estado["duracion_resultado"]:
                if estado["ganador_partida"]:
                    estado["estado"] = "FIN_PARTIDA"
                else:
                    estado["estado"] = "ESPERANDO"
                    estado["mensaje_resultado"] = ""
                    estado["gestos_bloqueados"] = {}

        elif curr_state == "FIN_PARTIDA":
            cv2.putText(frame, f"GANADOR: {estado['ganador_partida']}", (w//2 - 200, h//2), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 215, 255), 4)
            cv2.putText(frame, "Piedra para reiniciar", (w//2 - 150, h//2 + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
            if gL == "Piedra" and gR == "Piedra":
                self.reiniciar_partida(estado)

        elif curr_state == "REVEAL":
            elapsed_reveal = time.time() - estado["t_reveal_start"]
            currL = gL if gL else "Sin mano"
            currR = gR if gR else "Sin mano"

            # Seguimiento de candidatos
            for label, gesto in (("Izquierda", gL), ("Derecha", gR)):
                val = gesto if gesto else "Sin mano"
                estado["histories"][label].append(val)
                if estado["candidate"][label] is None:
                    if len(estado["histories"][label]) >= estado["stable_threshold"]:
                        last_vals = list(estado["histories"][label])[-estado["stable_threshold"]:]
                        if all(v == last_vals[0] for v in last_vals) and last_vals[0] != "Sin mano":
                            estado["candidate"][label] = last_vals[0]

            # Detección de cambios (trampas)
            for label, curr in (("Izquierda", currL), ("Derecha", currR)):
                cand = estado["candidate"][label]
                if cand is not None and estado["cheating_reason"][label] != 'change':
                    if curr == cand:
                        estado["reveal_stable_frames"][label] += 1
                    else:
                        if estado["reveal_stable_frames"][label] >= estado["change_detection_threshold"]:
                            estado["cheating"][label] = True
                            estado["cheating_reason"][label] = 'change'
                        else:
                            estado["reveal_stable_frames"][label] = 0

            # Fin del tiempo de revelación
            if elapsed_reveal >= estado["reveal_timeout"]:
                finalL = estado["candidate"]['Izquierda'] if estado["candidate"]['Izquierda'] else currL
                finalR = estado["candidate"]['Derecha'] if estado["candidate"]['Derecha'] else currR
                
                # Verificar manos tardías
                for label, final, cand in (("Izquierda", finalL, estado["candidate"]['Izquierda']), 
                                        ("Derecha", finalR, estado["candidate"]['Derecha'])):
                    if estado["cheating_reason"][label] != 'change':
                        if cand is None and final == "Sin mano":
                            estado["cheating"][label] = True; estado["cheating_reason"][label] = 'late'
                        elif cand is not None and final == "Sin mano":
                            estado["cheating"][label] = True; estado["cheating_reason"][label] = 'late'

                estado["gestos_bloqueados"] = {"Izquierda": finalL, "Derecha": finalR}
                reasons = []
                for label in ('Izquierda', 'Derecha'):
                    if estado["cheating"][label]:
                        r = estado["cheating_reason"][label] or ('change' if estado["cheating"][label] else 'late')
                        reasons.append(f"{label}: {('Cambio' if r=='change' else 'Retraso')}")

                if reasons:
                    estado["mensaje_resultado"] = "Trampa: " + ", ".join(reasons)
                    estado["ronda_actual"] += 1
                    self.verificar_ganador_partida(estado)
                else:
                    self.evaluar_ronda(estado)
                
                estado["estado"] = "RESULTADO"
                estado["t_inicio_resultado"] = time.time()

        self.dibujar_marcador(frame, estado)

    def juego_piedra_tijera_papel(self, frame):
        global ESTADO_JUEGO, detector_manos
        
        if frame is None or frame.size == 0: return
        if np.all(frame == 0): return
        
        # Inicialización de globales si es la primera vez
        if ESTADO_JUEGO is None:
            ESTADO_JUEGO = self.inicializar_estado()
        if detector_manos is None:
            detector_manos = self.inicializar_mediapipe()

        # Espejo y tamaños
        frame = cv2.flip(frame, 1) 
        h, w, _ = frame.shape
        
        # Detección de manos
        info_manos = []
        results = self.detectar_mano(detector_manos, frame)
        
        if results.multi_hand_landmarks:
            tmp_manos = []
            for hand_landmarks in results.multi_hand_landmarks:
                x_coords = [lm.x for lm in hand_landmarks.landmark]
                y_coords = [lm.y for lm in hand_landmarks.landmark]
                x_min, x_max = int(min(x_coords) * w), int(max(x_coords) * w)
                y_min, y_max = int(min(y_coords) * h), int(max(y_coords) * h)
                bbox = (x_min - 20, y_min - 20, x_max + 20, y_max + 20)
                
                mask = self.generar_mascara_opencv(frame, bbox)
                gesto = self.detectar_gesto(mask)
                x_center = (x_min + x_max) // 2
                
                tmp_manos.append({'bbox': bbox, 'mask': mask, 'gesto': gesto, 'x_center': x_center})
                
            tmp_manos.sort(key=lambda m: m['x_center'])
            if len(tmp_manos) >= 1:
                info_manos.append({'label': 'Izquierda', 'gesto': tmp_manos[0]['gesto'], 'mask': tmp_manos[0]['mask'], 'bbox': tmp_manos[0]['bbox']})
            if len(tmp_manos) >= 2:
                info_manos.append({'label': 'Derecha', 'gesto': tmp_manos[-1]['gesto'], 'mask': tmp_manos[-1]['mask'], 'bbox': tmp_manos[-1]['bbox']})

        # Dibujar cajas visuales
        for mano in info_manos:
            x1, y1, x2, y2 = mano['bbox']
            color = (0, 200, 0) if 'Izquierda' in mano['label'] else (0, 128, 255)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{mano['label']}: {mano['gesto']}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Actualizar lógica usando funciones y el diccionario
        self.actualizar_logica_juego(info_manos, frame, ESTADO_JUEGO)

        # --- Construcción del Canvas (Debug) ---
        debug_width, debug_height = 200, 250
        main_h, main_w = frame.shape[:2]
        
        total_w = debug_width * 2 + main_w
        total_h = max(debug_height, main_h)
        
        canvas = np.zeros((total_h, total_w, 3), dtype=np.uint8)
        
        y_off = (total_h - main_h) // 2
        canvas[y_off:y_off+main_h, debug_width:debug_width+main_w] = frame
        
        if len(info_manos) >= 1 and info_manos[0]['label'] == 'Izquierda':
            m = cv2.resize(info_manos[0]['mask'], (debug_width, debug_height))
            canvas[(total_h-debug_height)//2:(total_h+debug_height)//2, 0:debug_width] = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            
        if len(info_manos) >= 2 and info_manos[-1]['label'] == 'Derecha':
            m = cv2.resize(info_manos[-1]['mask'], (debug_width, debug_height))
            canvas[(total_h-debug_height)//2:(total_h+debug_height)//2, total_w-debug_width:] = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)

        cv2.imshow("Juego Activo", canvas)


    # Bucle principal
    def start(self):
        gesto_anterior_G = None
        gesto_anterior_R = None
        contador_gesto = 0
        gesto_confirmado = ""

        while not rospy.is_shutdown():

            imagen_copy = deepcopy(self.img)

            if self.estado == "gameselector":
                gesto_actual = self.procesar_juego(imagen_copy)

                if gesto_anterior_G is None:
                    gesto_anterior_G = gesto_actual
                    contador_gesto = 1
                else:
                    if gesto_actual == gesto_anterior_G:
                        contador_gesto += 1
                    else:
                        gesto_anterior_G = gesto_actual
                        contador_gesto = 1

                if contador_gesto >= 60:
                    if gesto_confirmado != gesto_actual:
                        gesto_confirmado = gesto_actual
                        print("JUEGO SELECCIONADO CONFIRMADO:", gesto_actual)
                        self.estado = gesto_actual
                        if gesto_actual == "VAQUEROS":
                            self.pub_juego.publish(Int16(data=0))  # Enviar a nodo principal que juego es el que se ha escogido
                        elif gesto_actual == "PPTLS":
                            self.pub_juego.publish(Int16(data=1))  # Enviar a nodo principal que juego es el que se ha escogido
                        cv2.destroyWindow("GameSelector")
                        cv2.destroyWindow("Mask GameSelector")


            elif self.estado == "VAQUEROS":
                countdown_text, countdown_finished = self.get_countdown_text()
                self.render_vaqueros_hud(imagen_copy, countdown_text)

                if self.score_p1 >= 3 or self.score_p2 >= 3:
                    print("Juego terminado. Ganador:", "Jugador 2" if self.score_p1 >= 3 else "Jugador 2")
                    self.estado = "gameselector"
                    self.bullets_p1 = 0
                    self.bullets_p2 = 0
                    self.score_p1 = 0
                    self.score_p2 = 0
                    cv2.destroyWindow("VAQUEROS")
                    continue
                
                # Detectar si countdown acaba de terminar
                if self.countdown_just_finished:
                    print("RONDA INICIADA!")
                    self.countdown_just_finished = False
                    # Aquí puedes agregar lógica después de que termine el countdown
                    puntuador, game_over = self.procesar_ronda_vaqueros(imagen_copy)

                    #DESCOMENTAR PARA PUBLICAR 
                    if puntuador == "1":
                        self.pub_ganador.publish(Int16(data=1))  # Enviar a nodo principal que jugador 1 ha ganado la ronda
                        print("Punto para Jugador 1")
                    elif puntuador == "2":
                        self.pub_ganador.publish(Int16(data=2))  # Enviar a nodo principal que jugador 2 ha ganado la ronda
                        print("Punto para Jugador 2")
                    elif puntuador is None:
                        print("Nadie puntuó en esta ronda.")
                
                # Solo detectar gestos si no hay countdown activo
                if not self.countdown_active:
                    gesto = self.procesar_gesto_inicio_ronda(imagen_copy)
                    
                    if gesto_anterior_R is None:
                        gesto_anterior_R = gesto
                        contador_gesto = 1
                    elif gesto == gesto_anterior_R:
                        contador_gesto += 1
                    else:
                        gesto_anterior_R = gesto
                        contador_gesto = 1

                    if contador_gesto >= 45:
                        self.start_countdown()
                        gesto_anterior_R = None
                        contador_gesto = 0                 
                
            elif self.estado == "PPTLS":
                self.juego_piedra_tijera_papel(imagen_copy)

            # Salir con tecla q
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    node = GestosNode()
    node.start()