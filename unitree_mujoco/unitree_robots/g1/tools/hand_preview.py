#!/usr/bin/env python3
"""Vorschau/Justage eines Hand-Profils im MuJoCo-Viewer.

Zeigt den G1 schwebend (ohne Schwerkraft, kein Regler) mit den Haenden des
Profils. Sobald hand.yaml oder die STL gespeichert wird, erzeugt das Tool das
Modell neu (gen_hand_profile.py) und laedt die Ansicht neu -> Versatz/Drehung in
hand.yaml aendern, speichern, hinsehen.

Eingeblendet:
  blaue Achsen  = Flansch-Frame  (*_hand_flange: x rot = vom Flansch weg)
  gruene Achsen = Greifpunkt TCP (*_hand_tcp, darauf zielen IK/Jog/Planer)
  Taste 3 im Viewer (Geom-Gruppe 3) = Kollisions-Teile ein/aus

Aufruf (am besten ueber hand_tool.sh, das das scene_editor-venv nutzt):
  python3 hand_preview.py haken
  python3 hand_preview.py haken --snapshot vorschau.png    # ohne Fenster, Bild
"""
import argparse
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_hand_profile as gen  # noqa: E402

PREVIEW_XML = """<mujoco model="hand_preview_{name}">
  <include file="{robot}"/>
  <option gravity="0 0 0"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0 0 0"/>
    <global azimuth="160" elevation="-15"/>
    <scale framelength="0.06" framewidth="0.003"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="grid" builtin="checker" rgb1="0.25 0.3 0.35" rgb2="0.2 0.25 0.3"
      width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="grid" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>
"""


def _inputs(name):
    p = gen.HANDS_DIR / name
    files = [p / "hand.yaml"] + [f for f in p.iterdir() if f.suffix.lower() == ".stl"]
    return {f: f.stat().st_mtime for f in files if f.is_file()}


def _load(name):
    import mujoco
    if not gen.check(name):
        gen.build(name)
    xml = gen.G1_DIR / f".hand_preview_{name}.xml"
    xml.write_text(PREVIEW_XML.format(name=name, robot=gen.mjcf_path(name).name))
    m = mujoco.MjModel.from_xml_path(str(xml))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def _setup_opt(opt):
    import mujoco
    opt.frame = mujoco.mjtFrame.mjFRAME_SITE
    opt.sitegroup[4] = 1


def snapshot(name, out):
    import mujoco
    import numpy as np
    from PIL import Image
    m, d = _load(name)
    r = mujoco.Renderer(m, 480, 640)
    opt = mujoco.MjvOption()
    _setup_opt(opt)
    imgs = []
    torso = d.xpos[m.body("torso_link").id]
    for az, el in ((180, -10), (90, -5), (145, -35)):
        cam = mujoco.MjvCamera()
        cam.lookat[:] = torso + np.array([0.25, 0.0, -0.15])
        cam.distance, cam.azimuth, cam.elevation = 0.95, az, el
        r.update_scene(d, cam, opt)
        imgs.append(r.render())
    Image.fromarray(np.concatenate(imgs, axis=1)).save(out)
    print(f"Bild: {out}")


def run_viewer(name):
    import mujoco
    import mujoco.viewer
    cam_state = None
    while True:
        try:
            m, d = _load(name)
        except Exception as e:                                    # hand.yaml kaputt
            print(f"FEHLER: {e}\n  -> hand.yaml korrigieren und speichern ...", flush=True)
            seen = _inputs(name)
            while _inputs(name) == seen:
                time.sleep(0.5)
            continue
        seen = _inputs(name)
        reload_ = False
        with mujoco.viewer.launch_passive(m, d) as v:
            with v.lock():
                _setup_opt(v.opt)
                if cam_state:
                    v.cam.lookat[:], v.cam.distance, v.cam.azimuth, v.cam.elevation = cam_state
                else:
                    v.cam.lookat[:] = d.xpos[m.body("torso_link").id] + [0.25, 0.0, -0.15]
                    v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.0, 160, -15
            print(f"Vorschau '{name}': hand.yaml speichern = neu laden, Fenster schliessen = Ende.",
                  flush=True)
            while v.is_running():
                time.sleep(0.3)
                if _inputs(name) != seen:
                    cam_state = (v.cam.lookat.copy(), v.cam.distance, v.cam.azimuth,
                                 v.cam.elevation)
                    print("Aenderung erkannt -> erzeuge neu ...", flush=True)
                    try:
                        gen.build(name)
                    except Exception as e:
                        print(f"FEHLER: {e}", flush=True)
                    reload_ = True
                    break
                v.sync()
        if not reload_:
            return 0


def main():
    ap = argparse.ArgumentParser(description="Hand-Profil im MuJoCo-Viewer ansehen/justieren")
    ap.add_argument("profile")
    ap.add_argument("--snapshot", help="nur ein Bild (3 Ansichten) speichern, kein Fenster")
    a = ap.parse_args()
    if a.profile not in gen.list_profiles():
        sys.exit(f"Profil '{a.profile}' nicht gefunden (verfuegbar: "
                 f"{', '.join(gen.list_profiles())}).")
    gen._reexec_in_editor_venv()
    if a.snapshot:
        snapshot(a.profile, a.snapshot)
        return 0
    return run_viewer(a.profile)


if __name__ == "__main__":
    sys.exit(main())
