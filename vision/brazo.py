"""Mueve el SO-101 con la mano: tu palma arrastra el objetivo y la IK persigue.

    # Terminal 1 -- camara (python normal)
    uv run python vision/mano.py

    # Terminal 2 -- brazo
    uv run mjpython vision/brazo.py            # solo simulador  (macOS)
    uv run python   vision/brazo.py            # solo simulador  (Linux/Windows)
    uv run mjpython vision/brazo.py --real     # ademas mueve el brazo fisico

Que hace cada cosa de tu mano:

    izquierda / derecha        objetivo en Y
    arriba / abajo             objetivo en Z
    acercar / alejar de la cam objetivo en X   (por el ancho de nudillos)
    pellizco pulgar-indice     abre / cierra la pinza

ENGANCHE AUTOMATICO. Mientras la camara te ve la mano, el objetivo la sigue.
Al aparecer la mano se guarda donde esta ella y donde esta el objetivo, y a
partir de ahi el objetivo copia tus DESPLAZAMIENTOS, no tu posicion absoluta.
Sacas la mano del encuadre y el objetivo se congela; al volver a mostrarla se
toma una referencia nueva. Es exactamente levantar el raton para recolocarlo.

Teclas en el visor:  F o ESPACIO congelar/descongelar   ·   R objetivo a HOME

La IK son las mismas cuatro lineas de wasd.py: jacobiano del TCP y un paso de
minimos cuadrados amortiguados hacia el objetivo.
"""

import argparse
import glob
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import enlace  # noqa: E402

RAIZ = Path(__file__).parent.parent
HOME = np.array([0.0, -1.0, 0.56, 0.97, 0.0, 1.2])   # 5 juntas + pinza

# Caja de trabajo del objetivo, en metros y en el marco del robot. El brazo
# llega mas lejos, pero fuera de aqui la IK empieza a estirarse y a pegar
# tirones; es la red de seguridad contra un gesto brusco.
CAJA_LO = np.array([0.14, -0.18, 0.04])
CAJA_HI = np.array([0.30, 0.18, 0.24])

# Ganancias: cuantos metros de objetivo por unidad de mano. Subelas si tienes
# que mover mucho el brazo para mover poco el robot.
ESC_Y = 0.60     # px (0..1)  -> Y
ESC_Z = 0.55     # py (0..1)  -> Z   (se invierte: arriba en la imagen = +Z)
ESC_X = 1.30     # ancho      -> X   (mas ancho = mano mas cerca = +X)

# Pellizco -> apertura de la pinza (rad). Por debajo de CERRADO la pinza queda
# cerrada del todo, por encima de ABIERTO del todo abierta.
P_CERRADO, P_ABIERTO = 0.35, 1.20
PINZA_CERRADA, PINZA_ABIERTA = -0.1, 1.2

# --- brazo real (igual que wasd_real.py) --------------------------------
SIGNOS = np.array([1, 1, 1, 1, 1, 1])    # -1 en la junta que gire al reves
SLEW, HZ = 12, 50                        # pasos max por envio, envios/s
K = 4096 / (2 * math.pi)                 # pasos de servo por radian (STS3215, 1:1)
IDS = (1, 2, 3, 4, 5, 6)                 # del hombro a la pinza
MODE, TORQUE, ACC, GOAL_VEL, GOAL_POS, PRESENT_POS = 33, 40, 41, 46, 42, 56
CALIBRACION = RAIZ / "tareas" / "calibracion.json"

ap = argparse.ArgumentParser()
ap.add_argument("--real", action="store_true", help="ademas mueve el brazo fisico")
ap.add_argument("--puerto", type=int, default=enlace.PUERTO, help="puerto UDP de mano.py")
ap.add_argument("--serie", default=None, help="puerto serie del brazo (si no lo autodetecta)")
args = ap.parse_args()


def pellizco_a_pinza(p: float) -> float:
    t = (p - P_CERRADO) / (P_ABIERTO - P_CERRADO)
    t = min(1.0, max(0.0, t))
    return PINZA_CERRADA + t * (PINZA_ABIERTA - PINZA_CERRADA)


# --- servos -------------------------------------------------------------
def conectar_servos(nombre):
    """Abre el puerto, carga la calibracion y deja el brazo BLANDO."""
    from scservo_sdk import (
        GroupSyncWrite, PacketHandler, PortHandler, SCS_HIBYTE, SCS_LOBYTE,
    )

    if not CALIBRACION.exists():
        sys.exit("Falta tareas/calibracion.json. Corre primero:\n"
                 "    uv run python tareas/calibrar.py")
    c = json.loads(CALIBRACION.read_text())

    nombre = nombre or next(iter(sorted(
        glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*")
        + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))), "")
    try:
        port = PortHandler(nombre)
        abierto = bool(nombre) and port.openPort() and port.setBaudRate(1_000_000)
    except Exception as e:      # pyserial tira excepcion si la ruta no existe
        abierto, nombre = False, f"{nombre}: {e}"
    if not abierto:
        sys.exit(f"No pude abrir el puerto ({nombre or 'no encontrado'}).\n"
                 "Enchufa el brazo, o pasa la ruta con --serie.")
    pk = PacketHandler(0)                   # STS / SMS: protocol_end = 0
    for i in IDS:                           # arranca sin par
        pk.write1ByteTxRx(port, i, TORQUE, 0)

    return {
        "port": port, "pk": pk, "nombre": nombre,
        "sync": GroupSyncWrite(port, pk, GOAL_POS, 2),
        "lo_hi": (SCS_LOBYTE, SCS_HIBYTE),
        "smin": np.array(c["min"], float),
        "smax": np.array(c["max"], float),
        # pasos = offset + SIGNOS * K * q  (la recta que calibrar.py fija con HOME)
        "offset": np.array(c["home"], float) - SIGNOS * K * HOME,
        "goal": None,
    }


def leer_servos(r):
    return np.array([r["pk"].read2ByteTxRx(r["port"], i, PRESENT_POS)[0]
                     for i in IDS], float)


def escribir_servos(r, pasos, bruto=False):
    lobyte, hibyte = r["lo_hi"]
    v = pasos if bruto else np.clip(pasos, r["smin"], r["smax"])
    r["sync"].clearParam()
    for i, p in zip(IDS, np.asarray(v).astype(int)):
        r["sync"].addParam(i, [lobyte(int(p)), hibyte(int(p))])
    r["sync"].txPacket()


real = conectar_servos(args.serie) if args.real else None
if real:
    print(f"Puerto {real['nombre']}   |   calibracion: {CALIBRACION.name}")
    print("\n" + "!" * 60)
    print("  --real: el brazo se VA A MOVER solo en cuanto vea tu mano.")
    print("  Deja la zona despejada y ten a mano el corte de corriente.")
    print("!" * 60)
    if input('  Escribe "si" para continuar: ').strip().lower() != "si":
        sys.exit("Cancelado.")

# --- simulador (mismo montaje que wasd.py) ------------------------------
model = mujoco.MjModel.from_xml_path(str(RAIZ / "models" / "so101" / "scene_ik.xml"))
data = mujoco.MjData(model)
tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
mocap = model.body_mocapid[
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
]
lo, hi = model.actuator_ctrlrange.T

q = HOME.copy()                  # consigna que se manda a los actuadores

if real:
    # El simulador salta a la pose real ANTES de energizar: asi los dos
    # arrancan en el mismo sitio y el brazo no pega un tiron al activarse.
    pasos = leer_servos(real)
    q[:] = np.clip((pasos - real["offset"]) / (SIGNOS * K), lo, hi)

data.qpos[:6] = q
data.ctrl = q
mujoco.mj_forward(model, data)
inicio = data.site_xpos[tcp].copy()        # TCP de arranque = objetivo de R
objetivo = inicio.copy()
data.mocap_pos[mocap] = objetivo

if real:
    for i in IDS:
        real["pk"].write1ByteTxRx(real["port"], i, MODE, 0)      # modo posicion
        real["pk"].write1ByteTxRx(real["port"], i, ACC, 20)      # aceleracion suave
        real["pk"].write2ByteTxRx(real["port"], i, GOAL_VEL, 300)
    escribir_servos(real, pasos, bruto=True)   # objetivo = donde ya esta
    for i in IDS:
        real["pk"].write1ByteTxRx(real["port"], i, TORQUE, 1)
    real["goal"] = pasos
    print("Brazo activo y sincronizado con el visor.\n")

# Copia del estado para resolver la IK sin fisica: la consigna se calcula
# sobre donde el brazo *deberia* estar, no sobre donde esta (que va con
# retraso). Mezclar ambas cosas hace que la correccion se integre sin fin.
ik = mujoco.MjData(model)
rx = enlace.Receptor(puerto=args.puerto)
print(f"Escuchando la mano en {enlace.HOST}:{args.puerto}")

est = {"congelado": False, "ref": None, "aviso": ""}


def tecla(codigo):
    k = chr(codigo).upper() if 0 < codigo < 0x110000 else ""
    if k == "F" or codigo == 32:            # F o ESPACIO
        est["congelado"] = not est["congelado"]
        est["aviso"] = "CONGELADO" if est["congelado"] else "descongelado"
    elif k == "R":
        objetivo[:] = inicio
        data.mocap_pos[mocap] = inicio
        est["ref"] = None
        est["aviso"] = "objetivo al centro"


jac = np.zeros((3, model.nv))
mano, t_mano, t_envio, t_print, t_arranque, quejado = None, 0.0, 0.0, 0.0, time.time(), False

with mujoco.viewer.launch_passive(model, data, key_callback=tecla) as visor:
    visor.cam.azimuth, visor.cam.elevation, visor.cam.distance = 140, -20, 0.9
    visor.cam.lookat[:] = [0.15, 0.0, 0.12]
    print("Visor abierto. Pon la mano delante de la camara: el objetivo la sigue.\n")

    while visor.is_running():
        reloj = time.time()

        # --- leer la mano -------------------------------------------------
        pkt = rx.ultimo()
        if pkt is not None and pkt.get("ok"):
            mano, t_mano = pkt, reloj
        if not quejado and mano is None and reloj - t_arranque > 3.0:
            quejado = True
            print("\n(!) No llega nada de la camara. En otra terminal:")
            print("    uv run python vision/mano.py\n")

        # Un paquete de hace mas de 0.4 s ya no vale: o quitaste la mano o
        # el otro proceso se murio. En los dos casos, congelar.
        fresca = mano is not None and reloj - t_mano < 0.4
        siguiendo = fresca and not est["congelado"]

        # --- la mano arrastra el objetivo ---------------------------------
        if siguiendo:
            if est["ref"] is None:      # primera vez que la vemos: referencia
                est["ref"] = (mano["px"], mano["py"], mano["ancho"], objetivo.copy())
            rpx, rpy, rancho, robj = est["ref"]
            objetivo[:] = np.clip(robj + np.array([
                ESC_X * (mano["ancho"] - rancho),
                ESC_Y * (mano["px"] - rpx),
                -ESC_Z * (mano["py"] - rpy),
            ]), CAJA_LO, CAJA_HI)
            data.mocap_pos[mocap] = objetivo
        else:
            est["ref"] = None           # al volver la mano, referencia nueva

        if fresca:
            q[5] = np.clip(pellizco_a_pinza(mano["pellizco"]), lo[5], hi[5])

        # --- un paso de IK sobre la consigna actual (identico a wasd.py) ---
        ik.qpos[:6] = q
        mujoco.mj_kinematics(model, ik)
        mujoco.mj_comPos(model, ik)
        error = data.mocap_pos[mocap] - ik.site_xpos[tcp]
        mujoco.mj_jacSite(model, ik, jac, None, tcp)
        J = jac[:, :5]                                   # 5 juntas del brazo
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), error)
        q[:5] = np.clip(q[:5] + np.clip(dq, -0.05, 0.05), lo[:5], hi[:5])

        data.ctrl = q
        mujoco.mj_step(model, data)

        # --- espejo al brazo real, a HZ, con topes y slew ------------------
        if real and reloj - t_envio >= 1 / HZ:
            t_envio = reloj
            quiere = np.clip(real["offset"] + SIGNOS * K * q, real["smin"], real["smax"])
            real["goal"] += np.clip(quiere - real["goal"], -SLEW, SLEW)
            escribir_servos(real, real["goal"])

        visor.sync()

        if est["aviso"]:
            print(f"\n  >>> {est['aviso']}")
            est["aviso"] = ""
        if reloj - t_print > 0.5:
            t_print = reloj
            etiqueta = ("CONGELADO" if est["congelado"] else
                        "SIGUIENDO " if siguiendo else "  esperando")
            print(f"\r[{etiqueta}] objetivo={np.round(objetivo, 3)}  "
                  f"pinza={q[5]:+.2f}  errIK={np.linalg.norm(error)*1000:5.1f}mm   ",
                  end="", flush=True)

        time.sleep(max(0, model.opt.timestep - (time.time() - reloj)))

print()
if real:
    real["port"].closePort()
    print("El par sigue activo sujetando la pose. Corta la corriente para soltar.")
