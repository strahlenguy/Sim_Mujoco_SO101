"""Rastrea tu mano con la camara y publica su posicion por UDP.

    uv run python vision/mano.py                 # con ventana de video
    uv run python vision/mano.py --sin-video     # sin ventana
    uv run python vision/mano.py --cam 1

Corre con `python` normal, NO con mjpython. En otra terminal:

    uv run mjpython vision/brazo.py

MediaPipe HandLandmarker encuentra los 21 puntos de la mano en cada fotograma
(en CPU, sin GPU ni nada raro). De ahi salen cuatro numeros -- centro de la
palma, ancho de nudillos y pellizco -- que van por UDP al otro proceso. Este
script no sabe nada del robot: solo mira y publica.

La imagen se voltea en espejo, como un selfie, para que mover la mano a tu
derecha mueva el objetivo a tu derecha.

Teclas en la ventana de video: q o Esc para salir.
"""

import argparse
import sys
import time
import urllib.request
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import (
    HandLandmarker, HandLandmarkerOptions, RunningMode,
)

sys.path.insert(0, str(Path(__file__).parent))
import enlace  # noqa: E402

MODELO = Path(__file__).parent.parent / "models" / "mediapipe" / "hand_landmarker.task"
URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
       "hand_landmarker/float16/1/hand_landmarker.task")
SUAVIZADO = 0.5   # 1 = crudo (tiembla), 0.2 = muy suave (va con retraso)

ap = argparse.ArgumentParser()
ap.add_argument("--cam", type=int, default=0, help="indice de camara")
ap.add_argument("--puerto", type=int, default=enlace.PUERTO)
ap.add_argument("--sin-video", action="store_true", help="no abrir ventana")
args = ap.parse_args()

# --- modelo: se descarga solo la primera vez (~8 MB) --------------------
if not MODELO.exists():
    MODELO.parent.mkdir(parents=True, exist_ok=True)
    print(f"Descargando el modelo de manos -> {MODELO} ...")
    urllib.request.urlretrieve(URL, MODELO)

detector = HandLandmarker.create_from_options(HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=str(MODELO),
                             delegate=BaseOptions.Delegate.CPU),
    running_mode=RunningMode.IMAGE,
    num_hands=1,
    min_hand_detection_confidence=0.5,
    min_tracking_confidence=0.5,
))

camara = cv2.VideoCapture(args.cam)
if not camara.isOpened():
    sys.exit(f"No pude abrir la camara {args.cam}.\n"
             "En macOS: Ajustes > Privacidad y seguridad > Camara, y da permiso\n"
             "a la terminal. Con otra camara conectada, prueba --cam 1.")
camara.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
camara.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

emisor = enlace.Emisor(puerto=args.puerto)
print(f"Publicando en {enlace.HOST}:{args.puerto}.  q/Esc para salir.")
print("Arranca en otra terminal:  uv run mjpython vision/brazo.py\n")


def dibujar(img, lm, s, w, h):
    """Esqueleto de la mano, linea del pellizco y centro de la palma."""
    for i, j in enlace.HUESOS:
        cv2.line(img, (int(lm[i].x * w), int(lm[i].y * h)),
                 (int(lm[j].x * w), int(lm[j].y * h)), (0, 200, 0), 2)
    for p in lm:
        cv2.circle(img, (int(p.x * w), int(p.y * h)), 3, (0, 255, 255), -1)
    a, b = lm[enlace.PULGAR], lm[enlace.INDICE]
    cv2.line(img, (int(a.x * w), int(a.y * h)), (int(b.x * w), int(b.y * h)),
             (255, 0, 255), 2)
    cv2.circle(img, (int(s["px"] * w), int(s["py"] * h)), 8, (255, 128, 0), -1)
    cv2.putText(img, f"pellizco {s['pellizco']:.2f}   ancho {s['ancho']:.3f}",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)


suave = None            # senales filtradas; None = ahora mismo no hay mano
t_informe = 0.0
con_mano = total = 0

while True:
    ok, img = camara.read()
    if not ok:
        continue
    img = cv2.flip(img, 1)                    # espejo
    h, w = img.shape[:2]
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    res = detector.detect(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb))
    total += 1

    if res.hand_landmarks:
        con_mano += 1
        lm = res.hand_landmarks[0]
        s = enlace.senales(lm)
        # Filtro exponencial: la deteccion salta un par de pixeles por
        # fotograma y ese ruido, amplificado por la IK, hace vibrar al brazo.
        suave = s if suave is None else {
            k: SUAVIZADO * s[k] + (1 - SUAVIZADO) * suave[k] for k in s
        }
        emisor.enviar(True, suave)
        if not args.sin_video:
            dibujar(img, lm, suave, w, h)
    else:
        suave = None        # al volver la mano se empieza el filtro de cero
        emisor.enviar(False)
        if not args.sin_video:
            cv2.putText(img, "sin mano", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 0, 255), 2)

    ahora = time.time()
    if ahora - t_informe > 1.0:
        t_informe = ahora
        pct = 100 * con_mano / max(1, total)
        estado = "" if suave is None else (
            f"palma=({suave['px']:.2f},{suave['py']:.2f}) "
            f"ancho={suave['ancho']:.3f} pellizco={suave['pellizco']:.2f}")
        print(f"\rmano detectada {pct:5.1f}%   {estado}   ", end="", flush=True)
        con_mano = total = 0

    if not args.sin_video:
        cv2.imshow("vision/mano.py   (q o Esc para salir)", img)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

camara.release()
cv2.destroyAllWindows()
print()
