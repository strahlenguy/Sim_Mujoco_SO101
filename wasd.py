"""Controla el SO-101 con el teclado: mueves un punto y el brazo lo persigue.

    uv run mjpython wasd.py     # macOS
    uv run python   wasd.py     # Linux / Windows

Con el foco en la ventana del visor:

    W / S    adelante / atras        O / C    abrir / cerrar la pinza
    A / D    izquierda / derecha     R        volver a la pose inicial
    Q / E    arriba / abajo

La esfera roja es el objetivo. Cada ciclo se calcula el jacobiano del TCP y se
da un paso de minimos cuadrados amortiguados hacia ella: eso es la cinematica
inversa, en cuatro lineas.
"""

import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

PASO = 0.01      # metros que avanza el objetivo por pulsacion
PINZA = 0.15     # radianes que abre/cierra la pinza por pulsacion
HOME = np.array([0.0, -1.0, 0.56, 0.97, 0.0, 1.2])   # 5 juntas + pinza

model = mujoco.MjModel.from_xml_path(
    str(Path(__file__).parent / "models" / "so101" / "scene_ik.xml")
)
data = mujoco.MjData(model)
tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "gripperframe")
mocap = model.body_mocapid[
    mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "target")
]
lo, hi = model.actuator_ctrlrange.T

q = HOME.copy()                  # consigna que se manda a los actuadores
data.qpos[:6] = q
data.ctrl = q
mujoco.mj_forward(model, data)
inicio = data.site_xpos[tcp].copy()
data.mocap_pos[mocap] = inicio

# Copia del estado para resolver la IK sin fisica: la consigna se calcula
# sobre donde el brazo *deberia* estar, no sobre donde esta (que va con
# retraso). Mezclar ambas cosas hace que la correccion se integre sin fin.
ik = mujoco.MjData(model)


def tecla(codigo: int) -> None:
    k = chr(codigo).upper() if 0 < codigo < 0x110000 else ""
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
with mujoco.viewer.launch_passive(model, data, key_callback=tecla) as visor:
    visor.cam.azimuth, visor.cam.elevation, visor.cam.distance = 140, -20, 0.9
    visor.cam.lookat[:] = [0.15, 0.0, 0.12]

    while visor.is_running():
        reloj = time.time()

        # --- Un paso de IK sobre la consigna actual.
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
        visor.sync()
        time.sleep(max(0, model.opt.timestep - (time.time() - reloj)))
