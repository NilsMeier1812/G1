#!/usr/bin/env python3
"""Vorschaubilder (Thumbnails) der Sim-Umgebungen fuer das Startmenue.

Rendert jede Szene einmal offscreen mit MuJoCo (schraeg von oben, Boden +
Licht dazu) nach scenes/thumbs/<name>.png und merkt sich in
scenes/thumbs/index.json einen Hash der XML. Das Startmenue ruft das Skript
im Hintergrund fuer neue oder geaenderte Szenen auf (scene_editor-venv,
dort ist mujoco installiert).

    .venv/bin/python render_thumbs.py                 # alle veralteten
    .venv/bin/python render_thumbs.py --all           # alle neu
    .venv/bin/python render_thumbs.py scenes/scene_A5.xml

Grafik: MUJOCO_GL=egl (GPU, ohne Fenster), sonst glfw (braucht DISPLAY);
klappt egl nicht, startet sich das Skript einmal mit glfw neu.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import struct
import sys
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
SCENES = HERE / "scenes"
THUMBS = SCENES / "thumbs"
STANDARD_XML = HERE.parent / "unitree_robots" / "g1" / "scene.xml"
STANDARD_NAME = "_standard"
W, H = 640, 400


def write_png(path: Path, img) -> None:
    """RGB-Bild (H x W x 3, uint8) als PNG schreiben -- ohne Pillow."""
    h, w = img.shape[:2]
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (struct.pack(">I", len(data)) + tag + data
                + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF))

    path.write_bytes(b"\x89PNG\r\n\x1a\n"
                     + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def xml_hash(path: Path) -> str:
    return hashlib.sha1(path.read_bytes()).hexdigest()[:16]


def load_index() -> dict:
    try:
        return json.loads((THUMBS / "index.json").read_text())
    except Exception:
        return {}


def save_index(idx: dict) -> None:
    THUMBS.mkdir(parents=True, exist_ok=True)
    (THUMBS / "index.json").write_text(json.dumps(idx, indent=1, sort_keys=True) + "\n")


def targets(all_: bool, explicit: list[str]) -> list[tuple[str, Path]]:
    if explicit:
        out = []
        for p in explicit:
            p = Path(p).resolve()
            out.append((STANDARD_NAME if p == STANDARD_XML.resolve() else p.stem, p))
        return out
    cand = [(STANDARD_NAME, STANDARD_XML)] + [(p.stem, p) for p in sorted(SCENES.glob("*.xml"))]
    if all_:
        return cand
    idx = load_index()
    return [(n, p) for n, p in cand
            if p.is_file() and (idx.get(n) != xml_hash(p) or not (THUMBS / f"{n}.png").is_file())]


def render_env(mujoco, np, xml: Path):
    """Umgebung ohne Roboter: Boden + Licht ergaenzen, Kamera auf die Moebel."""
    spec = mujoco.MjSpec.from_file(str(xml))
    tex = spec.add_texture(name="_thumb_grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
                           rgb1=[0.32, 0.35, 0.40], rgb2=[0.27, 0.30, 0.35],
                           width=300, height=300)
    mat = spec.add_material(name="_thumb_floor", texrepeat=[8, 8], reflectance=0.08)
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = tex.name
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[0, 0, 0.05],
                            material="_thumb_floor", contype=0, conaffinity=0,
                            name="_thumb_floor_geom")
    spec.worldbody.add_light(pos=[2, -2, 5], dir=[-0.3, 0.3, -1], diffuse=[0.5, 0.5, 0.5])
    spec.visual.headlight.ambient = [0.38, 0.38, 0.40]
    spec.visual.headlight.diffuse = [0.45, 0.45, 0.45]
    spec.visual.global_.offwidth = W
    spec.visual.global_.offheight = H
    m = spec.compile()
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    # Rahmen aus allen Geoms ausser dem Boden (m.stat ist bei Licht-Positionen ungenau)
    pts = []
    for i in range(m.ngeom):
        if m.geom_type[i] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        c, r = d.geom_xpos[i], float(m.geom_rbound[i])
        pts += [c - r, c + r]
    if pts:
        a = np.array(pts)
        lo, hi = a.min(axis=0), a.max(axis=0)
        center = (lo + hi) / 2
        extent = float(np.linalg.norm(hi[:2] - lo[:2]))
    else:
        center, extent = np.array([0, 0, 0.5]), 3.0
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [center[0], center[1], min(center[2], 0.8)]
    cam.distance = max(extent * 0.62, 2.5)
    cam.azimuth, cam.elevation = 135, -32
    return m, d, cam


def render_standard(mujoco, np, xml: Path):
    """Standard-Szene: der G1 in Grundstellung vor dem Gelaende."""
    m = mujoco.MjModel.from_xml_path(str(xml))
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, W)
    m.vis.global_.offheight = max(m.vis.global_.offheight, H)
    d = mujoco.MjData(m)
    if m.nkey:
        mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0, 0, 0.75]
    cam.distance, cam.azimuth, cam.elevation = 2.6, 150, -18
    return m, d, cam


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("xml", nargs="*", help="nur diese Szenen (sonst alle veralteten)")
    ap.add_argument("--all", action="store_true", help="alle neu rendern")
    a = ap.parse_args()

    todo = targets(a.all, a.xml)
    if not todo:
        print("[thumbs] alles aktuell")
        return 0
    os.environ.setdefault("MUJOCO_GL", "egl")
    try:
        import mujoco
        import numpy as np
    except Exception as exc:
        print(f"[thumbs] mujoco/numpy fehlt: {exc}", file=sys.stderr)
        return 3

    idx = load_index()
    THUMBS.mkdir(parents=True, exist_ok=True)
    failed = 0
    for name, xml in todo:
        try:
            fn = render_standard if name == STANDARD_NAME else render_env
            m, d, cam = fn(mujoco, np, xml)
            r = mujoco.Renderer(m, H, W)
            r.update_scene(d, cam)
            img = r.render()
            if hasattr(r, "close"):
                r.close()
        except Exception as exc:
            msg = str(exc)
            if os.environ.get("MUJOCO_GL") == "egl" and "_G1_THUMB_RETRY" not in os.environ \
                    and os.environ.get("DISPLAY") and ("EGL" in msg or "egl" in msg
                                                       or "OpenGL" in msg or "context" in msg):
                print("[thumbs] EGL geht nicht -> neuer Versuch mit glfw", flush=True)
                env = dict(os.environ, MUJOCO_GL="glfw", _G1_THUMB_RETRY="1")
                os.execve(sys.executable, [sys.executable, __file__, *sys.argv[1:]], env)
            print(f"[thumbs] FEHLER {name}: {msg}", file=sys.stderr, flush=True)
            failed += 1
            continue
        write_png(THUMBS / f"{name}.png", img)
        idx[name] = xml_hash(xml)
        save_index(idx)
        print(f"[thumbs] ok {name}", flush=True)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
