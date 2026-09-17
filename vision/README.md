# vision/ — mover el brazo con la mano vista por la cámara

Pones la mano delante de la webcam y el brazo la sigue. La palma arrastra el
punto objetivo del TCP, la IK resuelve las juntas, y el pellizco pulgar-índice
abre y cierra la pinza. Funciona sobre el simulador y, con `--real`, también
sobre el SO-101 físico.

```
mano.py     cámara + MediaPipe -> UDP        (python normal)
brazo.py    UDP -> visor + IK + servos       (mjpython en macOS)
enlace.py   el protocolo UDP que comparten
```

## Por qué son dos procesos

macOS solo deja abrir ventanas desde el hilo principal, y aquí hay dos que lo
quieren: el visor de MuJoCo (que además necesita `mjpython` y su bucle de
Cocoa) y la ventana de vídeo de OpenCV. Separados no se pelean. Hablan por
datagramas JSON en `127.0.0.1:9101`:

```
mano.py  ──UDP──>  brazo.py
```

UDP y no TCP a propósito: si se pierde un paquete da igual, el siguiente llega
en 30 ms con la posición nueva. Lo que no queremos es que el visor se quede
esperando a la cámara.

## Uso

```bash
# Terminal 1 — cámara
uv run python vision/mano.py                 # con ventana de vídeo
uv run python vision/mano.py --sin-video     # sin ventana
uv run python vision/mano.py --cam 1         # otra cámara

# Terminal 2 — brazo
uv run mjpython vision/brazo.py              # solo simulador  (macOS)
uv run python   vision/brazo.py              # solo simulador  (Linux/Windows)
uv run mjpython vision/brazo.py --real       # además mueve el brazo físico
```

La primera vez, `mano.py` descarga el modelo de manos de MediaPipe
(`models/mediapipe/hand_landmarker.task`, ~8 MB). macOS pedirá permiso de
cámara para la terminal: **Ajustes → Privacidad y seguridad → Cámara**.

## Control

| Tu mano | Qué mueve |
|---|---|
| izquierda / derecha | objetivo en **Y** |
| arriba / abajo | objetivo en **Z** |
| acercar / alejar de la cámara | objetivo en **X** |
| pellizco pulgar-índice | abrir / cerrar la pinza |

La profundidad (X) sale del **ancho de nudillos**: al acercar la mano a la
cámara los nudillos se ven más separados. Es profundidad gratis, sin cámara 3D.

**Enganche automático.** Mientras la cámara te ve la mano, el objetivo la
sigue; no hay que pulsar nada. Al aparecer la mano se guarda dónde está ella y
dónde está el objetivo, y a partir de ahí el objetivo copia tus
*desplazamientos*, no tu posición absoluta. **Sacas la mano del encuadre y el
objetivo se congela**; al volver a mostrarla se toma una referencia nueva. Es
exactamente levantar el ratón para recolocarlo.

Teclas en el visor:

| Tecla | Acción |
|---|---|
| `F` o `ESPACIO` | congelar / descongelar el objetivo |
| `R` | objetivo de vuelta a la pose inicial |

`brazo.py` imprime una línea de estado:

```
[SIGUIENDO ] objetivo=[0.3 0.059 0.24]  pinza=-0.10  errIK=  0.1mm
```

Si pone `esperando` es que no llegan paquetes: comprueba que `mano.py` está
corriendo y con el mismo `--puerto`.

## Ajustes

Todo son constantes arriba de `brazo.py`:

| Constante | Qué es |
|---|---|
| `ESC_X`, `ESC_Y`, `ESC_Z` | metros de objetivo por unidad de mano. Súbelas si tienes que mover mucho el brazo para mover poco el robot |
| `CAJA_LO`, `CAJA_HI` | caja de trabajo del objetivo. Fuera de ahí la IK se estira y pega tirones |
| `P_CERRADO`, `P_ABIERTO` | pellizco que cuenta como pinza cerrada / abierta |
| `SUAVIZADO` (en `mano.py`) | filtro de la detección. 1 = crudo y tiembla, 0.2 = muy suave y va con retraso |

## `--real`

Necesita `tareas/calibracion.json`; si no lo tienes, corre antes una vez
`uv run python tareas/calibrar.py`.

Las mismas protecciones que `wasd_real.py`:

- **Arranque sin tirón**: lee la pose real, el simulador *salta* a ella, escribe
  objetivo = posición actual y solo entonces activa el par.
- **Topes**: cada junta se recorta a los `min`/`max` que grabaste al calibrar.
- **Slew**: como mucho `SLEW = 12` pasos de servo por envío, a 50 Hz. Nada de
  saltos bruscos aunque hagas un gesto rápido.
- Pide confirmación por terminal antes de energizar.

Al salir el par sigue activo sujetando la pose. Corta la corriente para soltar.

> El brazo se mueve en cuanto vea tu mano. Zona despejada y el corte de
> corriente a mano. Si una junta gira al revés, cambia su signo en `SIGNOS`
> (arriba de `brazo.py`) y vuelve a calibrar.

## Notas

- Se rastrea **una** mano.
- MediaPipe está clavado a **0.10.33** en `pyproject.toml`: las 0.10.35 / 1.0.x
  abortan en Apple Silicon (`DrishtiMetalHelper ... Service is unavailable`) y
  las ≤0.10.21 exigen `numpy<2`, que choca con mujoco. La 0.10.33 va en CPU y
  convive con numpy 2.
- La imagen se voltea en espejo, como un selfie, para que mover la mano a tu
  derecha mueva el objetivo a tu derecha.
