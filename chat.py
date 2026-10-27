"""Mueve el SO-101 hablandole: un modelo local (Ollama) traduce tus frases a acciones.

    ollama serve                          # si no esta corriendo ya (brew services)
    ollama pull qwen3:8b                  # una sola vez

    uv run mjpython chat.py               # macOS
    uv run python   chat.py               # Linux / Windows
    uv run python   chat.py --sin-visor   # solo terminal, sin ventana
    uv run mjpython chat.py --modelo qwen3:4b

Escribes en la terminal, por ejemplo:

    > sube 5 centimetros
    > ve a la izquierda y abre la pinza
    > recoge el cubo verde y dejalo delante del robot
    > donde esta el cubo?
    > hola!                     (saluda con la mano; ver GESTOS para anadir mas)

El modelo no mueve juntas: llama a herramientas (mover_a, desplazar, pinza,
recoger_cubo, ...) con coordenadas en centimetros, y cada herramienta mueve
el objetivo de la IK -- la misma de wasd.py -- y espera a que el brazo llegue.
Asi el modelo nunca puede mandar el brazo fuera de la caja de trabajo.

Ejes, en el marco del robot:  X adelante  ·  Y izquierda  ·  Z arriba.
"""

import argparse
import json
import re
import threading
import time
import urllib.request
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

HOME = np.array([0.0, -1.0, 0.56, 0.97, 0.0, 1.2])   # 5 juntas + pinza
PINZA_ABIERTA, PINZA_CERRADA = 1.2, -0.1
VEL = 0.10        # m/s a los que viaja el objetivo
VEL_PINZA = 2.0   # rad/s
VEL_GESTO = 3.0   # rad/s de cada junta durante un gesto

# Gestos: secuencias de poses articulares [pan, lift, codo, muneca, giro, pinza]
# (rad). None en la pinza = dejarla como este. Al terminar el brazo vuelve a
# donde estaba. Para inventar uno nuevo basta con anadirlo aqui: el nombre
# aparece solo en la herramienta 'gesto'.
_ARRIBA = [0.0, 0.0, -1.5, 0.0, 0.0, None]          # antebrazo vertical, "mano alzada"
_H = list(HOME[:5]) + [None]


def _con(base, **cambios):
    i = {"pan": 0, "lift": 1, "codo": 2, "muneca": 3, "giro": 4, "pinza": 5}
    q = list(base)
    for k, v in cambios.items():
        q[i[k]] = v
    return q


GESTOS = {
    "saludar":  [_ARRIBA] + [_con(_ARRIBA, muneca=s) for s in (0.6, -0.6) * 3] + [_ARRIBA],
    "si":       [_con(_H, muneca=0.97 + s) for s in (0.45, -0.3) * 3],
    "no":       [_con(_H, pan=s) for s in (0.35, -0.35) * 3] + [_H],
    "festejar": [_con(_ARRIBA, pinza=p) for p in (1.5, -0.1) * 3],
    "bailar":   [_con(_H, pan=s, lift=-1.0 + abs(s), giro=2 * s) for s in (0.4, -0.4) * 3] + [_H],
    "reverencia": [_con(_H, lift=-0.2, codo=0.3, muneca=1.4), _H],   # punta a ~6 cm de la mesa
}

# Caja de trabajo del objetivo, en metros. Todo lo que pida el modelo se
# recorta aqui antes de llegar a la IK.
CAJA_LO = np.array([0.10, -0.25, 0.015])
CAJA_HI = np.array([0.32, 0.25, 0.28])

SISTEMA = """Eres el controlador de un brazo robotico SO-101 simulado. Hablas espanol.
Coordenadas en CENTIMETROS, marco del robot: x hacia adelante, y hacia la
IZQUIERDA del robot (y negativa = derecha), z hacia arriba (z=0 es la mesa).
Alcance aproximado: x de 10 a 32, y de -25 a 25, z de 1.5 a 28.
Para mover el brazo SIEMPRE usa las herramientas; nunca inventes que se movio.
Eres simpatico y expresivo, y lo demuestras con la herramienta gesto: si te
saludan o se despiden usala con nombre "saludar"; si te felicitan, "festejar";
para responder que si, "si"; para responder que no, "no".
Signos: subir = dz POSITIVO, bajar = dz NEGATIVO; adelante = dx positivo,
atras = dx negativo; izquierda = dy POSITIVO, derecha = dy NEGATIVO.
Si no indican cuanto, usa 5 cm.
Si piden varias cosas ("baja y luego avanza"), haz TODAS las llamadas, en orden.
Para mover el cubo a algun sitio usa dejar_cubo (lo recoge solo si hace falta);
nunca intentes moverlo con mover_a. Si no sabes donde esta algo, llama a estado.
Nunca digas que hiciste algo que las herramientas no confirmaron.
Tras actuar, responde en una o dos frases cortas que hiciste."""

HERRAMIENTAS = [
    {"type": "function", "function": {
        "name": "mover_a",
        "description": "Lleva la punta de la pinza a una posicion absoluta (cm).",
        "parameters": {"type": "object", "properties": {
            "x": {"type": "number", "description": "adelante, cm"},
            "y": {"type": "number", "description": "izquierda, cm"},
            "z": {"type": "number", "description": "altura sobre la mesa, cm"}},
            "required": ["x", "y", "z"]}}},
    {"type": "function", "function": {
        "name": "desplazar",
        "description": "Mueve la punta de la pinza relativo a donde esta (cm). "
                       "Ej: subir 5 -> dz=5; bajar 4 -> dz=-4; derecha 3 -> dy=-3; "
                       "izquierda 3 -> dy=3; atras 2 -> dx=-2. Varios movimientos "
                       "van en UNA llamada: 'baja 4 y avanza 5' -> dx=5, dz=-4.",
        "parameters": {"type": "object", "properties": {
            "dx": {"type": "number"}, "dy": {"type": "number"},
            "dz": {"type": "number"}}}}},
    {"type": "function", "function": {
        "name": "pinza",
        "description": "Abre o cierra la pinza. apertura: 0 = cerrada, 100 = abierta.",
        "parameters": {"type": "object", "properties": {
            "apertura": {"type": "number"}}, "required": ["apertura"]}}},
    {"type": "function", "function": {
        "name": "casa",
        "description": "Vuelve a la pose inicial con la pinza abierta.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "estado",
        "description": "Devuelve la posicion de la pinza, su apertura y donde esta el cubo verde.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "gesto",
        "description": "Hace un gesto expresivo con el brazo: saludar (hola/adios), "
                       "si (asentir), no (negar), festejar, bailar o reverencia.",
        "parameters": {"type": "object", "properties": {
            "nombre": {"type": "string", "enum": list(GESTOS)}},
            "required": ["nombre"]}}},
    {"type": "function", "function": {
        "name": "recoger_cubo",
        "description": "Busca el cubo verde, baja, lo agarra y lo levanta.",
        "parameters": {"type": "object", "properties": {}}}},
    {"type": "function", "function": {
        "name": "dejar_cubo",
        "description": "Pone el cubo verde en (x, y) de la mesa, en cm: si no lo "
                       "tiene en la pinza primero lo recoge. Izquierda = y positiva.",
        "parameters": {"type": "object", "properties": {
            "x": {"type": "number"}, "y": {"type": "number"}},
            "required": ["x", "y"]}}},
]


class Brazo:
    """Simulacion + IK. Un hilo llama a paso() en bucle; las herramientas
    cambian el objetivo y esperan a que el brazo llegue."""

    def __init__(self, sin_visor: bool):
        ruta = Path(__file__).parent / "models" / "so101" / "scene_ik.xml"
        self.m = mujoco.MjModel.from_xml_path(str(ruta))
        self.d = mujoco.MjData(self.m)
        self.ik = mujoco.MjData(self.m)   # copia cinematica, ver wasd.py
        self.tcp = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
        self.mocap = self.m.body_mocapid[
            mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "target")]
        self.cubo = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "cube")
        self.lo, self.hi = self.m.actuator_ctrlrange.T
        self.sin_visor = sin_visor
        self.lock = threading.Lock()
        self.jac = np.zeros((3, self.m.nv))
        self.jacr = np.zeros((3, self.m.nv))

        self.q = HOME.copy()
        self.d.qpos[:6] = self.q
        self.d.ctrl = self.q
        mujoco.mj_forward(self.m, self.d)
        self.inicio = self.d.site_xpos[self.tcp].copy()
        self.d.mocap_pos[self.mocap] = self.inicio
        self.meta = self.inicio.copy()       # adonde viaja el objetivo
        self.meta_pinza = self.q[5]
        self.rot = None                      # orientacion deseada del TCP (3x3) o None
        self.articular = None                # pose de gesto (5 juntas): pausa la IK

    # --- bucle de simulacion ------------------------------------------------
    def paso(self) -> None:
        with self.lock:
            dt = self.m.opt.timestep
            obj = self.d.mocap_pos[self.mocap]
            falta = self.meta - obj
            dist = np.linalg.norm(falta)
            obj += falta if dist < VEL * dt else falta / dist * VEL * dt
            dp = self.meta_pinza - self.q[5]
            self.q[5] += np.clip(dp, -VEL_PINZA * dt, VEL_PINZA * dt)

            if self.articular is not None:
                paso = VEL_GESTO * dt
                self.q[:5] += np.clip(self.articular - self.q[:5], -paso, paso)
                self.d.ctrl = self.q
                mujoco.mj_step(self.m, self.d)
                return

            self.ik.qpos[:6] = self.q
            mujoco.mj_kinematics(self.m, self.ik)
            mujoco.mj_comPos(self.m, self.ik)
            mujoco.mj_jacSite(self.m, self.ik, self.jac, self.jacr, self.tcp)
            e_pos = obj - self.ik.site_xpos[self.tcp]
            e_rot = None
            if self.rot is not None:
                R = self.ik.site_xmat[self.tcp].reshape(3, 3)
                e_rot = 0.5 * sum(np.cross(R[:, i], self.rot[:, i]) for i in range(3))
            # Una junta en su tope que el paso empuja mas alla se saca del
            # jacobiano y se resuelve con las demas; si no, el recorte congela
            # todo el brazo, posicion incluida.
            libre = np.ones(5, bool)
            for _ in range(3):
                dq = self._paso_ik(e_pos, e_rot, libre)
                nuevo = self.q[:5] + dq
                topa = ((nuevo > self.hi[:5]) | (nuevo < self.lo[:5])) & libre
                if not topa.any():
                    break
                libre &= ~topa
            self.q[:5] = np.clip(self.q[:5] + np.clip(dq, -0.05, 0.05),
                                 self.lo[:5], self.hi[:5])
            self.d.ctrl = self.q
            mujoco.mj_step(self.m, self.d)

    def _paso_ik(self, e_pos, e_rot, libre) -> np.ndarray:
        """Minimos cuadrados amortiguados para la posicion y, si hay
        orientacion pedida, la orientacion como tarea secundaria en el espacio
        nulo de la posicion: si no se puede cumplir, la posicion no se resiente."""
        Jp = self.jac[:, :5] * libre
        Jp_inv = Jp.T @ np.linalg.inv(Jp @ Jp.T + 1e-4 * np.eye(3))
        dq = Jp_inv @ e_pos
        if e_rot is not None:
            Jr = self.jacr[:, :5] * libre
            N = np.eye(5) - Jp_inv @ Jp
            JN = Jr @ N
            dq += N @ JN.T @ np.linalg.solve(JN @ JN.T + 1e-3 * np.eye(3),
                                             0.5 * e_rot - Jr @ dq)
        return dq * libre

    def esperar(self, maximo: float = 8.0) -> bool:
        """Espera a que objetivo y pinza lleguen y el brazo se asiente."""
        t0 = self.d.time
        while self.d.time - t0 < maximo:
            if self.sin_visor:
                for _ in range(10):
                    self.paso()
            else:
                time.sleep(0.02)
            with self.lock:
                if self.articular is not None:
                    # En un gesto basta con llegar a la consigna: no se espera
                    # a que el brazo se asiente, para que el movimiento fluya.
                    llego = (np.abs(self.articular - self.q[:5]).max() < 1e-3
                             and abs(self.meta_pinza - self.q[5]) < 1e-3)
                else:
                    llego = (np.linalg.norm(self.meta - self.d.mocap_pos[self.mocap]) < 1e-4
                             and abs(self.meta_pinza - self.q[5]) < 1e-3
                             and np.linalg.norm(self.d.qvel[:6]) < 0.05
                             and self.d.time - t0 > 0.3)
            if llego:
                return True
        return False

    def gesto(self, nombre):
        if nombre not in GESTOS:
            return {"error": f"gesto desconocido; hay: {', '.join(GESTOS)}"}
        antes, pinza_antes = self.q[:5].copy(), self.meta_pinza
        for pose in GESTOS[nombre]:
            if pose[5] is not None:
                self.meta_pinza = pose[5]
            self.articular = np.array(pose[:5], float)
            self.esperar(4.0)
        # De vuelta a donde estaba; la IK retoma sin salto porque la consigna
        # vuelve a ser la misma que antes del gesto.
        self.meta_pinza = pinza_antes
        self.articular = antes
        self.esperar(4.0)
        self.articular = None
        self.esperar(2.0)
        r = self.estado()
        r["resultado"] = f"gesto '{nombre}' hecho"
        return r

    # --- herramientas -------------------------------------------------------
    def _ir(self, p: np.ndarray) -> dict:
        pedido = np.asarray(p, float)
        self.meta = np.clip(pedido, CAJA_LO, CAJA_HI)
        self.esperar()
        r = self.estado()
        if not np.allclose(pedido, self.meta):
            r["aviso"] = "fuera del alcance; se recorto al limite de la caja de trabajo"
        return r

    def mover_a(self, x, y, z):
        return self._ir(np.array([x, y, z]) / 100)

    def desplazar(self, dx=0, dy=0, dz=0):
        return self._ir(self.meta + np.array([dx, dy, dz]) / 100)

    def pinza(self, apertura):
        a = np.clip(float(apertura), 0, 100) / 100
        self.meta_pinza = PINZA_CERRADA + a * (PINZA_ABIERTA - PINZA_CERRADA)
        self.esperar(3.0)
        return self.estado()

    def casa(self):
        self.rot = None
        self.meta_pinza = HOME[5]
        return self._ir(self.inicio)

    def estado(self):
        with self.lock:
            p = self.d.site_xpos[self.tcp] * 100
            c = self.d.xpos[self.cubo] * 100
            a = (self.q[5] - PINZA_CERRADA) / (PINZA_ABIERTA - PINZA_CERRADA) * 100
        return {"pinza_cm": [round(float(v), 1) for v in p],
                "apertura": round(float(np.clip(a, 0, 100))),
                "cubo_cm": [round(float(v), 1) for v in c],
                "sujeta_cubo": self.sujeta_cubo()}

    def sujeta_cubo(self) -> bool:
        c = self.d.xpos[self.cubo]
        return bool(c[2] > 0.035 and np.linalg.norm(c - self.d.site_xpos[self.tcp]) < 0.04)

    def _vertical(self, cierre: np.ndarray) -> np.ndarray:
        """Orientacion con la pinza mirando abajo (eje x del gripperframe) y
        las mordazas cerrando a lo largo de `cierre` (eje z del gripperframe)."""
        x = np.array([0.0, 0.0, -1.0])
        z = np.array([cierre[0], cierre[1], 0.0])
        z /= np.linalg.norm(z)
        return np.column_stack([x, np.cross(z, x), z])

    def recoger_cubo(self):
        c = self.d.xpos[self.cubo].copy()
        # Cara del cubo mas alineada con el cierre actual de la pinza, para no
        # tener que girar la muneca de mas.
        caras = self.d.xmat[self.cubo].reshape(3, 3)[:, :2].T
        z_ahora = self.d.site_xmat[self.tcp].reshape(3, 3)[:, 2]
        cierre = max([*caras, *-caras], key=lambda v: v[:2] @ z_ahora[:2])
        self.rot = self._vertical(cierre)
        cierre = self.rot[:, 2]
        # La mordaza fija queda a 2 cm del TCP (en -cierre) y el cubo mide 1.8
        # de medio lado: se corre el TCP 5 mm hacia ella para que baje por
        # fuera del cubo y no encima. La movil, abierta, queda lejos.
        agarre = c - 0.005 * cierre + [0, 0, -0.008]
        self.pinza(100)
        self._ir(agarre + [0, 0, 0.08])      # encima
        self._ir(agarre)                     # baja
        self.pinza(0)
        self._ir(agarre + [0, 0, 0.10])      # levanta
        r = self.estado()
        r["resultado"] = "cubo agarrado" if r["sujeta_cubo"] else "fallo: el cubo no quedo en la pinza"
        return r

    def dejar_cubo(self, x, y):
        if not self.sujeta_cubo():
            r = self.recoger_cubo()
            if not r["sujeta_cubo"]:
                return r
        self.rot = self._vertical(self.d.site_xmat[self.tcp].reshape(3, 3)[:, 2])
        # Se mueve el cubo, no el TCP: el cubo va desplazado respecto a la punta.
        desfase = self.d.site_xpos[self.tcp] - self.d.xpos[self.cubo]
        destino = np.array([x / 100, y / 100, 0.020]) + desfase
        self._ir(destino + [0, 0, 0.08])
        self._ir(destino)
        self.pinza(100)
        self._ir(destino + [0, 0, 0.08])
        self.rot = None
        r = self.estado()
        r["resultado"] = f"cubo dejado en x={r['cubo_cm'][0]}, y={r['cubo_cm'][1]}"
        return r

    def ejecutar(self, nombre: str, args: dict) -> dict:
        f = getattr(self, nombre, None)
        if nombre not in {h["function"]["name"] for h in HERRAMIENTAS} or f is None:
            return {"error": f"herramienta desconocida: {nombre}"}
        try:
            return f(**(args or {}))
        except (TypeError, ValueError) as e:
            return {"error": f"argumentos invalidos: {e}"}


# --- conversacion con Ollama ------------------------------------------------
PENSAR = False   # razonamiento de qwen3: acierta mas, tarda mas (--pensar)


def ollama(url: str, modelo: str, mensajes: list) -> dict:
    cuerpo = json.dumps({"model": modelo, "messages": mensajes, "tools": HERRAMIENTAS,
                         "stream": False, "think": PENSAR,
                         "options": {"temperature": 0}}).encode()
    pet = urllib.request.Request(f"{url}/api/chat", cuerpo,
                                 {"Content-Type": "application/json"})
    with urllib.request.urlopen(pet, timeout=300) as r:
        return json.load(r)["message"]


def llamada_escrita(texto: str) -> list:
    """Los modelos pequenos a veces escriben la llamada en vez de hacerla:
    'gesto(saludar)' o 'mover_a(x=20, y=5, z=10)'. Se reconoce y se ejecuta."""
    m = re.fullmatch(r"`*\s*(\w+)\((.*)\)\s*`*", texto.strip(), re.S)
    if not m:
        return []
    nombre, cuerpo = m.groups()
    esquema = next((h["function"]["parameters"] for h in HERRAMIENTAS
                    if h["function"]["name"] == nombre), None)
    if esquema is None:
        return []
    args = {}
    for i, trozo in enumerate(t.strip() for t in cuerpo.split(",") if t.strip()):
        clave, _, valor = trozo.rpartition("=")
        clave = clave.strip() or list(esquema["properties"])[i]
        valor = valor.strip().strip("'\"")
        try:
            args[clave] = float(valor)
        except ValueError:
            args[clave] = valor
    return [{"function": {"name": nombre, "arguments": args}}]


def turno(brazo: Brazo, url: str, modelo: str, mensajes: list, texto: str,
          eco=print) -> str:
    """Un turno del usuario: el modelo puede encadenar varias herramientas."""
    mensajes.append({"role": "user", "content": texto})
    for _ in range(10):
        msg = ollama(url, modelo, mensajes)
        mensajes.append(msg)
        llamadas = msg.get("tool_calls") or llamada_escrita(msg.get("content", ""))
        if not llamadas:
            return msg.get("content", "").strip()
        for ll in llamadas:
            nombre, args = ll["function"]["name"], ll["function"].get("arguments", {})
            eco(f"  -> {nombre}({', '.join(f'{k}={v}' for k, v in args.items())})")
            res = brazo.ejecutar(nombre, args)
            mensajes.append({"role": "tool", "tool_name": nombre,
                             "content": json.dumps(res, ensure_ascii=False)})
    return "(demasiadas acciones seguidas; paro aqui)"


def conversar(brazo: Brazo, url: str, modelo: str, fin: threading.Event) -> None:
    mensajes = [{"role": "system", "content": SISTEMA}]
    print(f"Modelo: {modelo}.  Escribe que hacer ('salir' para terminar).")
    try:
        while not fin.is_set():
            texto = input("\n> ").strip()
            if texto.lower() in ("salir", "exit", "quit"):
                break
            if texto:
                print(turno(brazo, url, modelo, mensajes, texto))
    except (EOFError, KeyboardInterrupt):
        pass
    except OSError as e:
        print(f"No pude hablar con Ollama en {url}: {e}\n"
              "Arrancalo con 'ollama serve' o 'brew services start ollama'.")
    fin.set()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--modelo", default="qwen3:8b")
    ap.add_argument("--ollama", default="http://localhost:11434")
    ap.add_argument("--sin-visor", action="store_true", help="solo terminal")
    ap.add_argument("--pensar", action="store_true",
                    help="razonamiento del modelo: algo mas listo, 10-50x mas lento")
    args = ap.parse_args()
    global PENSAR
    PENSAR = args.pensar

    brazo = Brazo(args.sin_visor)
    fin = threading.Event()
    if args.sin_visor:
        conversar(brazo, args.ollama, args.modelo, fin)
        return

    # El visor y la fisica van en este hilo; la conversacion en otro, porque
    # input() bloquea.
    threading.Thread(target=conversar, args=(brazo, args.ollama, args.modelo, fin),
                     daemon=True).start()
    with mujoco.viewer.launch_passive(brazo.m, brazo.d) as visor:
        visor.cam.azimuth, visor.cam.elevation, visor.cam.distance = 140, -20, 0.9
        visor.cam.lookat[:] = [0.15, 0.0, 0.12]
        while visor.is_running() and not fin.is_set():
            reloj = time.time()
            brazo.paso()
            with brazo.lock:
                visor.sync()
            time.sleep(max(0, brazo.m.opt.timestep - (time.time() - reloj)))
    fin.set()


if __name__ == "__main__":
    main()
