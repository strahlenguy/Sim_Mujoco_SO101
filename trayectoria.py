"""
Convierte las poses clave en una trayectoria densa, la verifica contra los
limites del brazo, y la exporta a CSV.

Uso:
    uv run trayectoria.py              # verifica y exporta
    uv run trayectoria.py --ver        # ademas la reproduce en el visor
"""

import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.interpolate import CubicSpline

MODELO = "modelo/scene.xml"
ENTRADA = Path("saludo_poses.json")
SALIDA = Path("saludo.csv")
HZ = 50  # frecuencia de comando; la misma que usaras en el brazo real

# --- cargar poses -----------------------------------------------------------
datos = json.loads(ENTRADA.read_text(encoding="utf-8"))
nombres = datos["articulaciones"]
q_keys = np.array([p["q"] for p in datos["poses"]])
t_keys = np.cumsum([0.0] + [p["dur"] for p in datos["poses"][1:]])

if len(q_keys) < 3:
    sys.exit("Necesitas al menos 3 poses para un spline cubico.")

# --- interpolar -------------------------------------------------------------
# bc_type='clamped' fuerza velocidad cero al inicio y al final: sin tiron.
traj = CubicSpline(t_keys, q_keys, bc_type="clamped")

t = np.arange(0.0, t_keys[-1] + 1e-9, 1.0 / HZ)
Q = traj(t)        # posicion
V = traj(t, 1)     # velocidad
A = traj(t, 2)     # aceleracion

print(f"{len(t_keys)} poses -> {len(t)} puntos a {HZ} Hz "
      f"({t_keys[-1]:.2f} s)\n")

# --- verificacion 1: limites de junta ---------------------------------------
m = mujoco.MjModel.from_xml_path(MODELO)
d = mujoco.MjData(m)

rango = m.jnt_range[:m.nu]  # asume 1 actuador por junta, en orden
for j, nom in enumerate(nombres):
    lo, hi = rango[j]
    if hi > lo:  # junta limitada
        if Q[:, j].min() < lo or Q[:, j].max() > hi:
            print(f"  LIMITE  {nom}: la trayectoria sale del rango "
                  f"[{lo:+.3f}, {hi:+.3f}] "
                  f"(llega a [{Q[:, j].min():+.3f}, {Q[:, j].max():+.3f}])")
            print("          el spline sobrepasa entre poses aunque las "
                  "poses si esten dentro. Sube 'dur' o suaviza.")

# --- verificacion 2: par requerido (dinamica inversa) -----------------------
# Esto es lo que el simulador te da y el hardware no: saber ANTES si el
# STS3215 alcanza. Si el par pedido rebasa lo que da el servo, en el brazo
# real el movimiento se va a deformar.
tau = np.zeros_like(Q)
for k in range(len(t)):
    d.qpos[:m.nu] = Q[k]
    d.qvel[:m.nu] = V[k]
    d.qacc[:m.nu] = A[k]
    mujoco.mj_inverse(m, d)
    tau[k] = d.qfrc_inverse[:m.nu]

print("Par pico por junta (N*m):")
for j, nom in enumerate(nombres):
    pico = np.abs(tau[:, j]).max()
    print(f"  {nom:<16} {pico:6.3f}   (vel max {np.abs(V[:, j]).max():.2f} rad/s)")
print("\nCompara contra el par nominal de tu STS3215. Si algo se acerca,\n"
      "alarga la duracion de ese segmento antes de tocar el hardware.\n")

# --- exportar ---------------------------------------------------------------
# El header lleva los nombres: el mapeo es explicito, no posicional.
encabezado = "t," + ",".join(nombres)
np.savetxt(SALIDA, np.column_stack([t, Q]), delimiter=",",
           header=encabezado, comments="", fmt="%.6f")
print(f"-> {SALIDA}  (radianes)")

# --- opcional: reproducir en simulacion -------------------------------------
if "--ver" in sys.argv:
    import time
    import mujoco.viewer

    d2 = mujoco.MjData(m)
    with mujoco.viewer.launch_passive(m, d2) as v:
        while v.is_running():
            inicio = time.time()
            for k in range(len(t)):
                if not v.is_running():
                    break
                d2.ctrl[:] = Q[k]
                while (time.time() - inicio) < t[k]:
                    mujoco.mj_step(m, d2)
                    v.sync()
            time.sleep(0.5)
            inicio = time.time()
