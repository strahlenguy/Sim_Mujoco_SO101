"""Enlace UDP entre la camara (mano.py) y el visor (brazo.py).

Son dos procesos y no dos hilos por una razon concreta de macOS: solo el hilo
principal puede abrir ventanas, y aqui hay dos que lo quieren. El visor de
MuJoCo necesita mjpython y su bucle de Cocoa; OpenCV/MediaPipe necesitan el
suyo. Separados no se pelean, y se hablan por datagramas JSON en localhost.

UDP y no TCP a proposito: si un paquete se pierde da igual, el siguiente llega
en 30 ms con la posicion nueva. Lo que no queremos es que el visor se quede
esperando a la camara.
"""

import json
import socket

HOST = "127.0.0.1"
PUERTO = 9101

# Indices de los 21 puntos que devuelve MediaPipe Hands.
MUNECA = 0
PULGAR = 4          # punta del pulgar
INDICE = 8          # punta del indice
PALMA = (0, 5, 9, 13, 17)   # muneca + los 4 nudillos

# Que puntos unir al dibujar la mano en la ventana de video.
HUESOS = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (9, 10), (10, 11), (11, 12),
    (13, 14), (14, 15), (15, 16),
    (0, 17), (17, 18), (18, 19), (19, 20),
    (5, 9), (9, 13), (13, 17),
]


def senales(lm) -> dict:
    """De los 21 puntos de la mano a las 4 senales de control.

    px, py    centro de la palma, normalizado 0..1 (origen arriba-izquierda)
    ancho     separacion entre nudillos 5 y 17; crece al acercar la mano a la
              camara, asi que sirve de profundidad sin necesitar una camara 3D
    pellizco  distancia pulgar-indice DIVIDIDA por el ancho: al normalizar,
              vale lo mismo con la mano cerca que lejos (~0 cerrado, ~1 abierto)
    """
    px = sum(lm[i].x for i in PALMA) / len(PALMA)
    py = sum(lm[i].y for i in PALMA) / len(PALMA)
    ancho = _dist(lm[5], lm[17])
    pellizco = _dist(lm[PULGAR], lm[INDICE]) / max(ancho, 1e-6)
    return {"px": px, "py": py, "ancho": ancho, "pellizco": pellizco}


def _dist(a, b) -> float:
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5


class Emisor:
    """Lado camara: manda un datagrama por fotograma."""

    def __init__(self, host=HOST, puerto=PUERTO):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destino = (host, puerto)
        self.n = 0

    def enviar(self, hay_mano: bool, s: dict | None = None) -> None:
        self.n += 1
        msg = {"n": self.n, "ok": hay_mano}
        if s:
            msg.update(s)
        try:
            self.sock.sendto(json.dumps(msg).encode(), self.destino)
        except OSError:
            pass        # si nadie escucha, seguimos; no es un error


class Receptor:
    """Lado visor: lee sin bloquear y se queda con el paquete mas reciente."""

    def __init__(self, host=HOST, puerto=PUERTO):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self.sock.bind((host, puerto))
        except OSError as e:
            raise SystemExit(
                f"No pude abrir el puerto UDP {puerto} ({e}).\n"
                f"Probablemente hay otra copia de brazo.py corriendo:\n"
                f"    pkill -f vision/brazo.py"
            )
        self.sock.setblocking(False)

    def ultimo(self) -> dict | None:
        """El datagrama mas nuevo que haya llegado, o None si no hay ninguno.

        Vacia la cola en cada llamada: los paquetes atrasados se tiran. Con
        posiciones, el ultimo es el unico que importa.
        """
        out = None
        while True:
            try:
                datos, _ = self.sock.recvfrom(1024)
            except (BlockingIOError, OSError):
                return out
            try:
                out = json.loads(datos)
            except ValueError:
                pass
