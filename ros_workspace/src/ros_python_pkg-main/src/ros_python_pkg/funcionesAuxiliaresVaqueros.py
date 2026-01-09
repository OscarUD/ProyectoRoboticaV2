import cv2
import numpy as np
import mediapipe as mp

mp_hands = mp.solutions.hands
hands_detector = mp_hands.Hands(max_num_hands=2, min_detection_confidence=0.5)

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



def luminosidad(roi):
    """
    Ajusta los umbrales de la máscara YCrCb según la luminosidad del ROI.
    ROI: imagen BGR de la mano.
    Devuelve: lower_skin, upper_skin (np.array)
    """
    if roi is None:
        # Valores por defecto
        lower_skin = np.array([0, 133, 77], dtype=np.uint8)
        upper_skin = np.array([255, 173, 127], dtype=np.uint8)
        return lower_skin, upper_skin

    # Convertir a YCrCb
    ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    Y_channel = ycrcb[:, :, 0]  # Canal de luminancia

    # Media de luminosidad
    lum_mean = np.mean(Y_channel)

    # Ajustar rango de Y (luminancia) según luminosidad
    if lum_mean < 80:        # muy oscuro
        lower_Y = 0
        upper_Y = 200
    elif lum_mean > 180:     # muy brillante
        lower_Y = 40
        upper_Y = 255
    else:                    # luminosidad media
        lower_Y = 0
        upper_Y = 255

    # Puedes ajustar los límites de Cr y Cb también si quieres
    lower_skin = np.array([lower_Y, 133, 77], dtype=np.uint8)
    upper_skin = np.array([upper_Y, 173, 127], dtype=np.uint8)

    return lower_skin, upper_skin


def detect_selection_gesture(roi):
    """
    Detecta gestos de 0 o 1 dedos (PPT o VAQUEROS) y PROTEGERSE usando máscara de piel.
    Devuelve: gesture, mask, contour, hull_points
    """


    # ==== Máscara de piel ====
    ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    lower_skin = np.array([0, 133, 77], dtype=np.uint8)
    upper_skin = np.array([255, 173, 127], dtype=np.uint8)
    mask = cv2.inRange(ycrcb, lower_skin, upper_skin)
    mask = cv2.GaussianBlur(mask, (7, 7), 0)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

    # ==== Contorno más grande ====
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "NINGUNO", mask, None, None

    max_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_contour)
    if area < 3000:
        return "NINGUNO", mask, None, None

    # Crear máscara limpia
    mask_clean = np.zeros_like(mask)
    cv2.drawContours(mask_clean, [max_contour], -1, 255, thickness=cv2.FILLED)
    mask = mask_clean

    # ==== Propiedades del contorno ====
    x, y, w, h = cv2.boundingRect(max_contour)
    aspect_ratio = w / float(h)
    rect_area = w * h
    extent = float(area) / rect_area
    hull_pts = cv2.convexHull(max_contour)
    hull_area = cv2.contourArea(hull_pts)
    solidity = float(area) / hull_area if hull_area != 0 else 0

    # ==== Hull defects ====
    hull = cv2.convexHull(max_contour, returnPoints=False)
    finger_count = 0
    if hull is not None and len(hull) >= 3:
        defects = cv2.convexityDefects(max_contour, hull)
        if defects is not None:
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                start = np.array(max_contour[s][0])
                end = np.array(max_contour[e][0])
                far = np.array(max_contour[f][0])
                a = np.linalg.norm(end - start)
                b = np.linalg.norm(far - start)
                c = np.linalg.norm(end - far)
                angle = np.arccos((b**2 + c**2 - a**2) / (2*b*c + 1e-6)) * 180 / np.pi
                if d > 5000 and angle < 90:
                    finger_count += 1

    # ==== Clasificación de gestos ====
    gesture = ""
    # PROTEGERSE
    if solidity > 0.85 and finger_count <= 1 and extent > 0.5 and aspect_ratio > 0.5:
        gesture = "PUNO"

    # 0 dedos → PPT, 1 dedo → VAQUEROS
    elif finger_count == 0:
        gesture = "DEDO"
    

    return gesture


def detect_gesture_juego(roi):
    
    if roi is None:
        return "DESCONOCIDO", None, None, None

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
        return "DESCONOCIDO", mask, None, None

    max_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_contour)
    if area < 3000:
        return "DESCONOCIDO", mask, None, None

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
        return "RECARGAR", mask, max_contour, hull_pts
    elif solidity > 0.85 and finger_count <= 1 and extent > 0.5 and aspect_ratio > 0.5:
        return "PROTEGERSE", mask, max_contour, hull_pts
    elif finger_count == 2 and solidity > 0.6 and aspect_ratio > 1:
        return "BOMBA", mask, max_contour, hull_pts
    elif (aspect_ratio > 1.3 or (finger_count >= 1 and finger_count <= 3 and solidity > 0.6)) and h < w * 1.5:
        return "DISPARAR", mask, max_contour, hull_pts
    
    return "DESCONOCIDO", mask, None, None



def process_round(bullets_p1, bullets_p2, score_p1, score_p2, gesture_p1, gesture_p2):
    """
    Procesa la ronda y devuelve:
    bullets_p1, bullets_p2, score_p1, score_p2, result, game_over
    """
    result = ""
    game_over = False
    puntuador = None

    # RECARGAR
    if gesture_p1 == "RECARGAR":
        bullets_p1 += 1
        result += "🔵 Jugador 1 RECARGA (+1 bala)\n"
    if gesture_p2 == "RECARGAR":
        bullets_p2 += 1
        result += "🔴 Jugador 2 RECARGA (+1 bala)\n"

    # BOMBA
    if gesture_p1 == "BOMBA" and gesture_p2 == "BOMBA":
        if bullets_p1 >= 3 and bullets_p2 >= 3:
            bullets_p1 -= 3
            bullets_p2 -= 3
            result += "⚔️ AMBOS HACEN BOMBA! EMPATE\n"
        elif bullets_p1 < 3 and bullets_p2 >= 3:
            bullets_p2 -= 3
            score_p2 += 1
            result += "💥 P1 no tiene balas para BOMBA! Punto para P2\n"
            puntuador = "2"
        elif bullets_p1 >= 3 and bullets_p2 < 3:
            bullets_p1 -= 3
            score_p1 += 1
            result += "💥 P2 no tiene balas para BOMBA! Punto para P1\n"
            puntuador = "1"
        else:
            result += "⚠️ Ninguno tiene balas suficientes para BOMBA! Empate\n"
    elif gesture_p1 == "BOMBA":
        if bullets_p1 >= 3:
            bullets_p1 -= 3
            score_p1 += 1
            result += "💣🔵 Jugador 1 HACE BOMBA! Punto automático\n"
            puntuador = "1"
        else:
            score_p2 += 1
            result += "💥🔴 Jugador 1 quería BOMBA pero no tiene balas! Punto para Jugador 2\n"
            puntuador = "2"
    elif gesture_p2 == "BOMBA":
        if bullets_p2 >= 3:
            bullets_p2 -= 3
            score_p2 += 1
            result += "💣🔴 Jugador 2 HACE BOMBA! Punto automático\n"
            puntuador = "2"
        else:
            score_p1 += 1
            result += "💥🔵 Jugador 2 quería BOMBA pero no tiene balas! Punto para Jugador 1\n"
            puntuador = "1"

    # DISPARAR
    if gesture_p1 == "DISPARAR" and gesture_p2 == "DISPARAR":
        if bullets_p1 > 0 and bullets_p2 > 0:
            bullets_p1 -= 1
            bullets_p2 -= 1
            result += "⚔️ EMPATE! Ambos disparan\n"
        elif bullets_p1 > 0 and bullets_p2 == 0:
            bullets_p1 -= 1
            score_p1 += 1
            result += "🔵 GANA JUGADOR 1! (J2 sin balas)\n"
            puntuador = "1"
        elif bullets_p1 == 0 and bullets_p2 > 0:
            bullets_p2 -= 1
            score_p2 += 1
            result += "🔴 GANA JUGADOR 2! (J1 sin balas)\n"
            puntuador = "2"
        else:
            result += "⚔️ EMPATE! Ambos sin balas\n"
    elif gesture_p1 == "DISPARAR":
        if bullets_p1 > 0:
            bullets_p1 -= 1
            if gesture_p2 == "PROTEGERSE":
                result += "🛡️ Jugador 2 BLOQUEA el disparo\n"
            elif gesture_p2 == "RECARGAR":
                score_p1 += 1
                result += "🔵 GANA JUGADOR 1! Disparo certero\n"
                puntuador = "1"
            else:
                result += "🔵 Jugador 1 dispara (J2 sin gesto válido)\n"
        else:
            score_p2 += 1
            result += "🔴 GANA JUGADOR 2! (J1 disparó sin balas)\n"
            puntuador = "2"
    elif gesture_p2 == "DISPARAR":
        if bullets_p2 > 0:
            bullets_p2 -= 1
            if gesture_p1 == "PROTEGERSE":
                result += "🛡️ Jugador 1 BLOQUEA el disparo\n"
            elif gesture_p1 == "RECARGAR":
                score_p2 += 1
                result += "🔴 GANA JUGADOR 2! Disparo certero\n"
                puntuador = "2"
            else:
                result += "🔴 Jugador 2 dispara (J1 sin gesto válido)\n"
        else:
            score_p1 += 1
            result += "🔵 GANA JUGADOR 1! (J2 disparó sin balas)\n"
            puntuador = "1"

    # PROTEGERSE
    elif gesture_p1 == "PROTEGERSE" and gesture_p2 == "PROTEGERSE":
        result += "🛡️🛡️ Ambos se protegen\n"
    else:
        if gesture_p1 != "DESCONOCIDO" or gesture_p2 != "DESCONOCIDO":
            result += "⚠️ Ronda sin acción significativa\n"

    # Fin de juego
    if score_p1 == 3:
        result += "Game Over! Ganador: Jugador 1"
        game_over = True
    elif score_p2 == 3:
        result += "Game Over! Ganador: Jugador 2"
        game_over = True

    
    return bullets_p1, bullets_p2, score_p1, score_p2, result, puntuador, game_over

    #result = comentarista
    #puntuador quien puntua en la ronda
    #game_over True si alguien ha ganado el juego


def detect_gameselector(roi):
    """
    Detecta gestos de 0 o 1 dedos (PPT o VAQUEROS) y PROTEGERSE usando máscara de piel.
    Devuelve: gesture, mask, contour, hull_points
    """


    # ==== Máscara de piel ====
    ycrcb = cv2.cvtColor(roi, cv2.COLOR_BGR2YCrCb)
    lower_skin = np.array([0, 133, 77], dtype=np.uint8)
    upper_skin = np.array([255, 173, 127], dtype=np.uint8)
    mask = cv2.inRange(ycrcb, lower_skin, upper_skin)
    mask = cv2.GaussianBlur(mask, (7, 7), 0)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

    # ==== Contorno más grande ====
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return "NINGUNO", mask, None, None

    max_contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(max_contour)
    if area < 3000:
        return "NINGUNO", mask, None, None

    # Crear máscara limpia
    mask_clean = np.zeros_like(mask)
    cv2.drawContours(mask_clean, [max_contour], -1, 255, thickness=cv2.FILLED)
    mask = mask_clean

    # ==== Propiedades del contorno ====
    x, y, w, h = cv2.boundingRect(max_contour)
    aspect_ratio = w / float(h)
    rect_area = w * h
    extent = float(area) / rect_area
    hull_pts = cv2.convexHull(max_contour)
    hull_area = cv2.contourArea(hull_pts)
    solidity = float(area) / hull_area if hull_area != 0 else 0

    # ==== Hull defects ====
    hull = cv2.convexHull(max_contour, returnPoints=False)
    finger_count = 0
    if hull is not None and len(hull) >= 3:
        defects = cv2.convexityDefects(max_contour, hull)
        if defects is not None:
            for i in range(defects.shape[0]):
                s, e, f, d = defects[i, 0]
                start = np.array(max_contour[s][0])
                end = np.array(max_contour[e][0])
                far = np.array(max_contour[f][0])
                a = np.linalg.norm(end - start)
                b = np.linalg.norm(far - start)
                c = np.linalg.norm(end - far)
                angle = np.arccos((b**2 + c**2 - a**2) / (2*b*c + 1e-6)) * 180 / np.pi
                if d > 5000 and angle < 90:
                    finger_count += 1

    # ==== Clasificación de gestos ====
    # PROTEGERSE
    if solidity > 0.85 and finger_count <= 1 and extent > 0.5 and aspect_ratio > 0.5:
        return "SALIR", mask, max_contour, hull_pts

    # 0 dedos → PPT, 1 dedo → VAQUEROS
    if finger_count == 0:
        gesture = "PPTLS"
    elif finger_count == 1:
        gesture = "VAQUEROS"
    else:
        gesture = "NINGUNO"

    return gesture, mask, max_contour, hull_pts