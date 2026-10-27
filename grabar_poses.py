"""
Graba poses clave del SO-101 desde el visor de MuJoCo.

Uso:
    uv run grabar_poses.py

En el visor, abre el panel "Control" (lado derecho) y mueve los sliders
hasta la pose que quieras. Luego:

    ESPACIO    captura la pose actual
    BACKSPACE  borra la ultima capturada
    cerrar     guarda el archivo

IMPORTANTE: usa el modelo con actuadores de POSICION (el original de
Menagerie), no el parcheado a torque de las practicas de control.
"""

import json
from pathlib import Path

import mujoco
import mujoco.viewer

MODELO = "modelo/scene.xml"
SALIDA = Path("saludo_poses.json")
DUR_DEFAULT = 1.0  # segundos por segmento; se edita despues en el JSON

m = mujoco.MjModel.from_xml_path(MODELO)
d = mujoco.MjData(m)

# Nombres de actuadores en el mismo orden que d.ctrl.
# Guardarlos es lo que evita el bug clasico de mandar las juntas revueltas.
NOMBRES = [
    mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)
]

poses = []


def on_key(keycode):
    if keycode == 32:  # ESPACIO
        poses.append({
            "dur": DUR_DEFAULT,
            "q": [float(v) for v in d.ctrl],
        })
        print(f"[{len(poses):>2}] capturada  ->  "
              + "  ".join(f"{n}={v:+.3f}" for n, v in zip(NOMBRES, d.ctrl)))
    elif keycode == 259:  # BACKSPACE
        if poses:
            poses.pop()
            print(f"     borrada. quedan {len(poses)}")


print(f"Actuadores ({m.nu}): {NOMBRES}")
print("ESPACIO = capturar   BACKSPACE = borrar   cerrar ventana = guardar\n")

with mujoco.viewer.launch_passive(m, d, key_callback=on_key) as v:
    while v.is_running():
        mujoco.mj_step(m, d)
        v.sync()

if not poses:
    print("No capturaste nada. No se escribio archivo.")
else:
    SALIDA.write_text(
        json.dumps({"articulaciones": NOMBRES, "poses": poses}, indent=2),
        encoding="utf-8",
    )
    print(f"\n{len(poses)} poses -> {SALIDA}")
    print("Edita el campo 'dur' de cada pose para ajustar el ritmo.")
