"""Abre el visor con el brazo SO-101 y los sliders por junta.

    uv run mjpython ver.py              # macOS
    uv run python   ver.py              # Linux / Windows
    uv run python   ver.py scene_ik     # otra escena

Pulsa Tab (o el boton ">") para abrir el panel lateral y ve a la pestana
"Control": hay un slider por junta. Como este script no escribe en data.ctrl,
los sliders mandan directo a los actuadores.

Se usa launch_passive y no mujoco.viewer.launch(): el visor gestionado aborta
en macOS con "RuntimeError: Caught an unknown exception!".
"""

import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer

escena = sys.argv[1] if len(sys.argv) > 1 else "scene"
ruta = Path(__file__).parent / "models" / "so101" / f"{escena}.xml"
if not ruta.exists():
    disponibles = sorted(p.stem for p in ruta.parent.glob("scene*.xml"))
    sys.exit(f"No existe {ruta.name}. Escenas: {', '.join(disponibles)}")

model = mujoco.MjModel.from_xml_path(str(ruta))
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as visor:
    while visor.is_running():
        reloj = time.time()
        mujoco.mj_step(model, data)
        visor.sync()
        time.sleep(max(0, model.opt.timestep - (time.time() - reloj)))
