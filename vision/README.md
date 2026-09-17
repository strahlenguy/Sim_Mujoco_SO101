# vision/ — mueve el brazo con la mano

Pones la mano delante de la webcam y el brazo la sigue. La palma lleva la pinza
por el espacio, y juntar el pulgar con el índice la abre y la cierra.

Funciona en Windows, macOS y Linux, con la cámara que ya trae la laptop. Sirve
igual para el brazo simulado que para el de verdad.

## Antes de empezar

Una sola vez, en la carpeta del repo:

```bash
uv sync
```

Eso instala todo. No hay que activar nada ni descargar nada a mano: la primera
vez que corras `mano.py` se baja solo el modelo de detección de manos (~8 MB).

## Cómo se usa

Son dos programas y hacen falta **dos terminales**: uno mira por la cámara y el
otro mueve el brazo.

**Terminal 1 — la cámara:**

```bash
uv run python vision/mano.py
```

Se abre una ventana con tu mano y los puntos dibujados encima. Déjala abierta.

**Terminal 2 — el brazo:**

```bash
uv run python vision/brazo.py
```

En **macOS** esa segunda línea va con `mjpython` en lugar de `python`:

```bash
uv run mjpython vision/brazo.py
```

Se abre el visor con el brazo. Pon la mano delante de la cámara y ya está.

Para cerrar: `q` o `Esc` en la ventana de la cámara, y cierra el visor.

## Cómo se controla

| Tu mano | Qué hace el brazo |
|---|---|
| izquierda / derecha | se mueve a los lados |
| arriba / abajo | sube y baja |
| acercarte / alejarte de la cámara | se estira y se encoge |
| juntar pulgar e índice | cierra la pinza |
| separar pulgar e índice | abre la pinza |

**No hay que pulsar nada para engancharse.** Mientras la cámara te vea la mano,
el brazo la sigue. **Si sacas la mano del encuadre, el brazo se queda quieto**, y
cuando la vuelves a meter sigue desde donde se quedó. Sirve para recolocarte:
igual que levantas el ratón cuando llegas al borde de la mesa.

Teclas, con el visor del brazo seleccionado:

| Tecla | Acción |
|---|---|
| `F` o `ESPACIO` | congelar / descongelar (el brazo deja de hacerte caso) |
| `R` | devolver el brazo al centro |

La terminal del brazo va escribiendo cómo va:

```
[SIGUIENDO ] objetivo=[0.3 0.059 0.24]  pinza=-0.10  errIK=  0.1mm
```

## Si algo no funciona

| Qué ves | Qué hacer |
|---|---|
| `[  esperando]` en la terminal del brazo | La cámara no está mandando nada. Revisa que la Terminal 1 siga corriendo. |
| No pude abrir la cámara | Si tienes más de una cámara, prueba `uv run python vision/mano.py --cam 1` (y luego `--cam 2`). |
| Windows: la ventana de la cámara sale negra | Otra aplicación la está usando (Zoom, Teams, Meet). Ciérrala. |
| Windows: no aparece la ventana | Ajustes → Privacidad → Cámara → deja que las aplicaciones de escritorio accedan. |
| macOS: no aparece la ventana | Ajustes → Privacidad y seguridad → Cámara → dale permiso a la terminal. |
| `mano detectada 0.0%` | Falta luz, o la mano sale muy pequeña. Acércate y ponte de frente, con la palma hacia la cámara. |
| El brazo tiembla | Baja `SUAVIZADO` en `mano.py` (por ejemplo a `0.3`). |
| Tengo que mover mucho el brazo para mover poco el robot | Sube `ESC_X`, `ESC_Y`, `ESC_Z` en `brazo.py`. |
| El brazo se queda corto, no llega más lejos | Es la caja de trabajo: `CAJA_LO` y `CAJA_HI` en `brazo.py`. |

## Ajustes

Están todos juntos arriba de cada archivo, con su comentario:

| Dónde | Qué cambia |
|---|---|
| `ESC_X`, `ESC_Y`, `ESC_Z` en `brazo.py` | cuánto se mueve el brazo por cada centímetro de mano |
| `CAJA_LO`, `CAJA_HI` en `brazo.py` | hasta dónde puede llegar el brazo |
| `P_CERRADO`, `P_ABIERTO` en `brazo.py` | cuánto hay que pellizcar para cerrar la pinza |
| `SUAVIZADO` en `mano.py` | 1 = responde al instante pero tiembla · 0.2 = muy suave pero con retraso |

## Con el brazo de verdad

Añade `--real`:

```bash
uv run python vision/brazo.py --real          # Windows / Linux
uv run mjpython vision/brazo.py --real        # macOS
```

Antes hay que calibrar el brazo una vez (`uv run python tareas/calibrar.py`,
explicado en [`tareas/README.md`](../tareas/README.md)). El puerto se detecta
solo; si no lo encuentra, pásalo tú:

```bash
uv run python vision/brazo.py --real --serie COM5            # Windows
uv run python vision/brazo.py --real --serie /dev/ttyUSB0    # Linux
```

> ⚠️ **El brazo se mueve solo en cuanto vea tu mano.** Despeja la mesa y ten a
> la mano el cable de corriente. Empieza con movimientos lentos y pequeños.
> El programa pide confirmación antes de encender los motores.

Protecciones que ya lleva, las mismas que `wasd_real.py`:

- Arranca sin tirón: el simulador se coloca en la pose en la que esté el brazo
  antes de encender los motores.
- No se pasa de los topes que grabaste al calibrar.
- Va despacio aunque hagas un gesto brusco (como mucho 12 pasos de servo por
  envío, 50 veces por segundo).

Al cerrar, los motores se quedan encendidos sujetando la postura. Corta la
corriente para soltar el brazo.

## Notas

- Se rastrea **una** mano.
- La imagen va en espejo, como un selfie: mueves la mano a tu derecha y el
  brazo va a tu derecha.
- Los dos programas se pasan la posición de la mano por el puerto 9101 de tu
  propia computadora. Si ese puerto te da lata, cámbialo en los dos a la vez:
  `--puerto 9200`.
- La versión de MediaPipe está fijada a propósito en `pyproject.toml`. No la
  subas: las nuevas fallan en las Mac con chip M.
