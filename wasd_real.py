"""Mueve el brazo SO-101 REAL con el teclado: es wasd.py + el envio a los servos.

Antes, corre una vez  tareas/calibrar.py  para generar tareas/calibracion.json.

    uv run mjpython wasd_real.py                 # macOS
    uv run python   wasd_real.py /dev/ttyUSB0    # Linux / Windows (COM5, ...)

Controles (con el foco en la ventana del visor):

    W/S adelante/atras   A/D izquierda/derecha   Q/E arriba/abajo
    O/C abrir/cerrar pinza    R pose HOME    ESPACIO activar / pausar

El brazo arranca BLANDO (sin par). Al pulsar ESPACIO el simulador salta a la
pose real del brazo (arrancan iguales, sin tiron), se activa el par y a partir
de ahi el brazo sigue al simulador, sin pasar nunca de los topes min/max que
grabaste con calibrar.py y moviendose como mucho SLEW pasos por envio. Si una
junta gira al reves, cambia su signo en SIGNOS y recalibra.
"""

import glob
import json
import math
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from scservo_sdk import (
    GroupSyncWrite, PacketHandler, PortHandler, SCS_HIBYTE, SCS_LOBYTE,
)

PASO, PINZA = 0.01, 0.15                             # m y rad por pulsacion
HOME = np.array([0.0, -1.0, 0.56, 0.97, 0.0, 1.2])   # 5 juntas + pinza
SIGNOS = np.array([1, 1, 1, 1, 1, 1])    # -1 en la junta que gire al reves
SLEW, HZ = 12, 50                        # paso max por envio, envios/s
K = 4096 / (2 * math.pi)                 # pasos de servo por radian (STS3215, 1:1)
IDS = (1, 2, 3, 4, 5, 6)                 # del hombro a la pinza
MODE, TORQUE, ACC, GOAL_VEL, GOAL_POS, PRESENT_POS = 33, 40, 41, 46, 42, 56
ARCHIVO = Path(__file__).parent / "tareas" / "calibracion.json"

# --- calibracion grabada por calibrar.py -------------------------------
if not ARCHIVO.exists():
    sys.exit("Falta tareas/calibracion.json. Corre primero:  uv run python tareas/calibrar.py")
c = json.loads(ARCHIVO.read_text())
smin, smax = np.array(c["min"], float), np.array(c["max"], float)
offset = np.array(c["home"], float) - SIGNOS * K * HOME   # pasos = offset + SIGNOS*K*q

# --- servos: abrir puerto y dos funciones para leerlos y moverlos ------
PUERTO = sys.argv[1] if len(sys.argv) > 1 else next(iter(sorted(
    glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.usbmodem*")
    + glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))), "")
port = PortHandler(PUERTO)
if not PUERTO or not port.openPort() or not port.setBaudRate(1_000_000):
    sys.exit(f"No pude abrir el puerto ({PUERTO or 'no encontrado'}). Pasalo como argumento.")
pk = PacketHandler(0)                    # STS / SMS: protocol_end = 0
sync = GroupSyncWrite(port, pk, GOAL_POS, 2)


def leer():
    return np.array([pk.read2ByteTxRx(port, i, PRESENT_POS)[0] for i in IDS], float)


def escribir(pasos, bruto=False):
    v = pasos if bruto else np.clip(pasos, smin, smax)
    sync.clearParam()
    for i, p in zip(IDS, np.asarray(v).astype(int)):
        sync.addParam(i, [SCS_LOBYTE(int(p)), SCS_HIBYTE(int(p))])
    sync.txPacket()


for i in IDS:                            # arranca BLANDO (sin par)
    pk.write1ByteTxRx(port, i, TORQUE, 0)

# --- simulador (identico a wasd.py) --------------------------------------
model = mujoco.MjModel.from_xml_path(
    str(Path(__file__).parent / "models" / "so101" / "scene_ik.xml")
)
data = mujoco.MjData(model)
tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
mocap = model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")]
lo, hi = model.actuator_ctrlrange.T

q = HOME.copy()
data.qpos[:6] = q
data.ctrl = q
mujoco.mj_forward(model, data)
inicio = data.site_xpos[tcp].copy()
data.mocap_pos[mocap] = inicio
ik = mujoco.MjData(model)                # copia para resolver la IK sin fisica

print(f"Puerto: {PUERTO}   |   calibracion: {ARCHIVO.name}")
print("Brazo BLANDO. Pulsa ESPACIO en el visor para activarlo.\n")
estado = {"activo": False, "pausa": False, "goal": None}


def tecla(codigo):
    k = chr(codigo).upper() if 0 < codigo < 0x110000 else ""
    if codigo == 32:                    # ESPACIO
        if not estado["activo"]:
            pasos = leer()
            # el simulador salta a la pose real: arrancan iguales, sin tiron
            q[:] = np.clip((pasos - offset) / (SIGNOS * K), lo, hi)
            data.qpos[:6] = q
            data.ctrl = q
            mujoco.mj_forward(model, data)
            data.mocap_pos[mocap] = data.site_xpos[tcp]   # objetivo = donde ya esta
            for i in IDS:
                pk.write1ByteTxRx(port, i, MODE, 0)       # modo posicion
                pk.write1ByteTxRx(port, i, ACC, 20)       # aceleracion suave
                pk.write2ByteTxRx(port, i, GOAL_VEL, 300)
            escribir(pasos, bruto=True)                   # objetivo = pose actual, sin tiron
            for i in IDS:
                pk.write1ByteTxRx(port, i, TORQUE, 1)
            estado["goal"] = pasos
            estado["activo"] = True
            print("Activo y sincronizado. El brazo sigue al visor.")
        else:
            estado["pausa"] = not estado["pausa"]
            print("PAUSA" if estado["pausa"] else "reanudado")
        return
    ejes = {"W": (0, 1), "S": (0, -1), "A": (1, 1),
            "D": (1, -1), "Q": (2, 1), "E": (2, -1)}
    if k in ejes:
        eje, signo = ejes[k]
        data.mocap_pos[mocap][eje] += signo * PASO
    elif k in ("O", "C"):
        q[5] = np.clip(q[5] + (PINZA if k == "O" else -PINZA), lo[5], hi[5])
    elif k == "R":
        q[:] = HOME
        data.mocap_pos[mocap] = inicio


jac = np.zeros((3, model.nv))
t_envio = 0.0
with mujoco.viewer.launch_passive(model, data, key_callback=tecla) as visor:
    visor.cam.azimuth, visor.cam.elevation, visor.cam.distance = 140, -20, 0.9
    visor.cam.lookat[:] = [0.15, 0.0, 0.12]

    while visor.is_running():
        reloj = time.time()

        # --- un paso de IK sobre la consigna actual (igual que wasd.py)
        ik.qpos[:6] = q
        mujoco.mj_kinematics(model, ik)
        mujoco.mj_comPos(model, ik)
        error = data.mocap_pos[mocap] - ik.site_xpos[tcp]
        mujoco.mj_jacSite(model, ik, jac, None, tcp)
        J = jac[:, :5]
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(3), error)
        q[:5] = np.clip(q[:5] + np.clip(dq, -0.05, 0.05), lo[:5], hi[:5])
        data.ctrl = q
        mujoco.mj_step(model, data)

        # --- espejo al brazo real, a HZ, con topes min/max y slew ---------
        if estado["activo"] and not estado["pausa"] and reloj - t_envio >= 1 / HZ:
            t_envio = reloj
            quiere = np.clip(offset + SIGNOS * K * q, smin, smax)
            estado["goal"] += np.clip(quiere - estado["goal"], -SLEW, SLEW)
            escribir(estado["goal"])

        visor.sync()
        time.sleep(max(0, model.opt.timestep - (time.time() - reloj)))

print("\nEl par sigue activo sujetando la pose. Corta la corriente para soltar.")
