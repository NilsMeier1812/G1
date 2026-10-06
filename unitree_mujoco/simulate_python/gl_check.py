"""OpenGL-Renderer pruefen: warnt laut, wenn der MuJoCo-Viewer auf der CPU rendert.

Bekommt der Container keine GPU (z.B. NVIDIA ohne Container Toolkit, siehe
g1pilot/docker/check_gpu.sh), faellt Mesa still auf llvmpipe zurueck. Die Sim
laeuft dann mit wenigen FPS, und das CPU-Rendering nimmt Physik + Reglern die
Kerne weg. Diese Pruefung faengt JEDE Ursache ab, nicht nur das fehlende Toolkit.

Nie fatal: jeder Fehler hier wird nur geloggt, der Start geht weiter.
"""

import ctypes

import mujoco

_GL_VENDOR = 0x1F00
_GL_RENDERER = 0x1F01

# Software-Renderer von Mesa (CPU statt GPU).
_SOFTWARE_RENDERERS = ("llvmpipe", "softpipe", "swrast", "software rasterizer")


def query_gl_renderer():
    """(vendor, renderer) des Standard-OpenGL-Kontexts, oder None bei Fehler."""
    ctx = mujoco.GLContext(16, 16)
    try:
        ctx.make_current()
        gl = ctypes.CDLL("libGL.so.1")
        gl.glGetString.restype = ctypes.c_char_p
        vendor = (gl.glGetString(_GL_VENDOR) or b"?").decode(errors="replace")
        renderer = (gl.glGetString(_GL_RENDERER) or b"?").decode(errors="replace")
        return vendor, renderer
    finally:
        ctx.free()


def check_gl_renderer():
    try:
        info = query_gl_renderer()
    except Exception as exc:  # noqa: BLE001 -- Diagnose darf den Start nie stoppen
        print(f"[gpu] OpenGL-Renderer nicht pruefbar ({exc}).")
        return
    vendor, renderer = info
    if any(s in renderer.lower() for s in _SOFTWARE_RENDERERS):
        bar = "=" * 72
        print(
            f"\n{bar}\n"
            f"[gpu] WARNUNG: MuJoCo rendert auf der CPU ({renderer})!\n"
            f"      Der Container hat keinen GPU-Zugriff -> niedrige FPS, CPU fehlt\n"
            f"      Physik und Reglern. Ursache pruefen: bash g1pilot/docker/check_gpu.sh\n"
            f"      (haeufigster Grund: NVIDIA-GPU ohne NVIDIA Container Toolkit)\n"
            f"{bar}\n",
            flush=True,
        )
    else:
        print(f"[gpu] OpenGL: {renderer} ({vendor})", flush=True)
