"""
Reproduce el CSV en el SO-101 fisico.

    uv run enviar_al_brazo.py --puerto COM5 --junta shoulder_pan   # prueba 1 junta
    uv run enviar_al_brazo.py --puerto COM5                        # completo

LEE ESTO ANTES DE CORRERLO
--------------------------
1. Corre primero con --junta. Mueve UNA sola junta, lento. Verifica
   direccion y magnitud. Es el 90% de los errores de la primera vez.
2. Ten la mano en el conector de corriente.
3. La API de LeRobot cambia entre versiones. Si algo no existe con este
   nombre, revisa la de tu instalacion; la logica de abajo no cambia.
"""

import argparse
import time

import numpy as np

CSV = "saludo.csv"
HZ = 50

ap = argparse.ArgumentParser()
ap.add_argument("--puerto", required=True, help="COM5, COM7, ...")
ap.add_argument("--junta", default=None, help="mover solo esta junta")
ap.add_argument("--lento", type=float, default=1.0, help="factor de tiempo")
args = ap.parse_args()

# --- cargar ------------------------------------------------------------------
with open(CSV, encoding="utf-8") as f:
    nombres = f.readline().strip().split(",")[1:]
datos = np.loadtxt(CSV, delimiter=",", skiprows=1)
t, Q_rad = datos[:, 0], datos[:, 1:]

# LeRobot maneja el SO-101 en GRADOS por defecto (use_degrees=True).
# El MJCF esta en radianes. Esta linea es la que se olvida.
Q = np.degrees(Q_rad)

# --- conectar ----------------------------------------------------------------
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

robot = SOFollower(SOFollowerRobotConfig(port=args.puerto, id="so101"))
robot.connect()

try:
    obs = robot.get_observation()
    q0 = {k: v for k, v in obs.items() if k.endswith(".pos")}
    print("Pose actual:", {k: round(v, 1) for k, v in q0.items()})

    # --- ir despacio a la pose inicial de la trayectoria ---------------------
    # Nunca saltes a la primera pose de golpe: puede estar lejos de donde
    # esta el brazo ahorita, y el servo va a tirar con todo.
    destino = {f"{n}.pos": float(Q[0, j]) for j, n in enumerate(nombres)}
    if args.junta:
        destino = {f"{args.junta}.pos": destino[f"{args.junta}.pos"]}

    print("Acercandose a la pose inicial (3 s)...")
    pasos = int(3 * HZ)
    for i in range(pasos):
        a = (i + 1) / pasos
        robot.send_action({
            k: (1 - a) * q0[k] + a * v for k, v in destino.items()
        })
        time.sleep(1 / HZ)

    input("Listo. Enter para ejecutar la trayectoria, Ctrl+C para abortar.")

    # --- ejecutar ------------------------------------------------------------
    inicio = time.perf_counter()
    for k in range(len(t)):
        accion = {f"{n}.pos": float(Q[k, j]) for j, n in enumerate(nombres)}
        if args.junta:
            accion = {f"{args.junta}.pos": accion[f"{args.junta}.pos"]}
        robot.send_action(accion)

        # reloj absoluto, no sleep fijo: no acumula deriva
        objetivo = inicio + t[k] * args.lento
        espera = objetivo - time.perf_counter()
        if espera > 0:
            time.sleep(espera)

    print("Listo.")

finally:
    robot.disconnect()
