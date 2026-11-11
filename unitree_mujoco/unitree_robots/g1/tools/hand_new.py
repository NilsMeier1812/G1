#!/usr/bin/env python3
"""Neue Hand anlegen: 3D-Datei waehlen, Montage-Vorschlaege ansehen, uebernehmen.

Ablauf im Fenster:
  1. STL (oder OBJ/PLY/3MF/...) waehlen, Namen geben.
  2. Das Tool raet die Montage (hand_detect.py) und zeigt mehrere Vorschlaege als
     Bild: rechts die rechte Hand von nah, links beide Haende von vorne.
  3. Vorschlag waehlen -> "Anlegen": Ordner hands/<profil>/ mit Datei + hand.yaml,
     Modell erzeugen (gen_hand_profile.py), danach oeffnet das Bearbeitungstool
     (hand_adjust.py) zum Feinjustieren.

Die Vorschau-Bilder rendert ein eigener Prozess (MuJoCo-OpenGL und Tk vertragen
sich unter X11 nicht in einem Prozess, siehe hand_adjust.py).

  python3 hand_new.py [datei]                         Fenster
  python3 hand_new.py --create datei --name "Titel" [--pick 1] [--slug x] [--no-build]
                                                      ohne Fenster (Vorschlag Nr. pick)
"""
import argparse
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import threading
import unicodedata
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_hand_profile as gen  # noqa: E402
import hand_detect as det  # noqa: E402

PLA_DENSITY = 1240.0          # kg/m^3
INFILL = 0.5                  # grobe Annahme fuer gedruckte Teile
THUMB = (300, 225)            # Pixel je Ansicht
ARM_LINKS = ("wrist_yaw_link", "wrist_pitch_link", "wrist_roll_link", "elbow_link")


# ══════════════════════════════════════════════════════════════════════
# Profil anlegen
# ══════════════════════════════════════════════════════════════════════
def slugify(title: str) -> str:
    t = title.lower()
    for a, b in (("ä", "ae"), ("ö", "oe"), ("ü", "ue"), ("ß", "ss")):
        t = t.replace(a, b)
    t = unicodedata.normalize("NFKD", t).encode("ascii", "ignore").decode()
    t = re.sub(r"[^a-z0-9]+", "_", t).strip("_")
    return t[:32] or "hand"


def free_slug(slug: str) -> str:
    """slug, oder slug_2, slug_3 ..., falls der Ordner schon existiert."""
    if not (gen.HANDS_DIR / slug).exists():
        return slug
    i = 2
    while (gen.HANDS_DIR / f"{slug}_{i}").exists():
        i += 1
    return f"{slug}_{i}"


def mass_estimate(mesh, unit: str) -> float:
    scale = 0.001 if unit == "mm" else 1.0
    vol = det.volume_estimate(mesh) * scale ** 3
    return max(0.02, round(vol * PLA_DENSITY * INFILL, 2))


def _num(x, unit):
    nd = 2 if unit == "mm" else 5
    v = round(float(x), nd)
    return 0.0 if v == 0 else v


def _vec(v, unit=None, nd=4):
    if unit is not None:
        return "[" + ", ".join(f"{_num(x, unit)}" for x in v) + "]"
    return "[" + ", ".join(f"{(round(float(x), nd) or 0.0)}" for x in v) + "]"


HAND_YAML = """\
# Hand-Profil "{slug}": angelegt mit "Neue Hand anlegen" (tools/hand_new.py).
# Montage = Vorschlag {pick} der automatischen Erkennung: {why}.
#
# Feinjustage:  unitree_mujoco/unitree_robots/g1/tools/hand_tool.sh adjust {slug}
# Neu erzeugen: unitree_mujoco/unitree_robots/g1/tools/hand_tool.sh build {slug}
# Erklaerung aller Felder: unitree_mujoco/unitree_robots/g1/hands/README.md

name: {title}

mesh: {mesh}
unit: {unit}                          # Einheit der Datei: mm | m (geraten: {unit_guess})

# Masse JE Hand in kg. Geschaetzt aus dem Volumen ({vol_cm3:.0f} cm3) als PLA mit
# {infill:.0f} % Fuellung. Bitte wiegen!
mass_kg: {mass}
color: [0.95, 0.55, 0.10, 1.0]    # RGBA in der Sim

mount:
  # Punkt der Datei, der auf der Flansch-Mitte liegt
  origin: {origin}
  # Richtung in der Datei, die vom Flansch WEG zeigt (Verlaengerung des Unterarms)
  out: {out}
  # Richtung in der Datei zur HANDFLAECHE (dahin, wo die Inspire-Finger schliessen)
  palm: {palm}
  # Feinjustage im Hand-Frame (x = out, y = palm, z = quer): mm bzw. Grad
  offset_mm: [0.0, 0.0, 0.0]
  rotate_deg: [0.0, 0.0, 0.0]

# Eine Datei fuer beide Seiten: eine Seite ist das Spiegelbild der anderen.
mirror:
  left: {mirror_left}
  right: false

# Greifpunkt (TCP) in Datei-Koordinaten: darauf zielen IK, Jog und Planer.
tcp: {tcp}

collision:
  max_hulls: 12
"""


def create_profile(src: Path, title: str, suggestion: dict, *, unit: str, slug: str = "",
                   mirror_left: bool = True, mass: float = None, pick: int = 1,
                   unit_guess: str = "") -> str:
    """Ordner hands/<slug>/ mit Datei + hand.yaml anlegen. Gibt den slug zurueck."""
    src = Path(src)
    slug = slug or free_slug(slugify(title))
    if not gen.NAME_RE.match(slug):
        raise ValueError(f"Ordnername '{slug}': nur a-z, 0-9 und _ erlaubt.")
    pdir = gen.HANDS_DIR / slug
    if pdir.exists():
        raise ValueError(f"hands/{slug} gibt es schon.")
    mesh = det.load_mesh(src)
    pdir.mkdir(parents=True)
    try:
        name = slugify(src.stem) or "hand"
        if src.suffix.lower() == ".stl":
            dst = pdir / f"{name}.stl"
            shutil.copyfile(src, dst)
        else:                                   # andere Formate als STL ablegen
            dst = pdir / f"{name}.stl"
            mesh.export(str(dst))
        scale = 0.001 if unit == "mm" else 1.0
        vol = det.volume_estimate(mesh) * scale ** 3
        if mass is None:
            mass = mass_estimate(mesh, unit)
        title_q = json.dumps(title, ensure_ascii=False)
        (pdir / "hand.yaml").write_text(HAND_YAML.format(
            slug=slug, pick=pick, why=suggestion.get("why", "-"), title=title_q,
            mesh=dst.name, unit=unit, unit_guess=unit_guess or unit,
            vol_cm3=vol * 1e6, infill=INFILL * 100, mass=round(float(mass), 3),
            origin=_vec(suggestion["origin"], unit), out=_vec(suggestion["out"]),
            palm=_vec(suggestion["palm"]), tcp=_vec(suggestion["tcp"], unit),
            mirror_left="true" if mirror_left else "false"), encoding="utf-8")
        gen.load_profile(slug)                  # Plausibilitaet (wirft bei Fehlern)
    except Exception:
        shutil.rmtree(pdir, ignore_errors=True)
        raise
    return slug


# ══════════════════════════════════════════════════════════════════════
# Vorschau-Bilder (eigener Prozess: --analyse)
# ══════════════════════════════════════════════════════════════════════
def _profile_dict(s, unit, mirror_left):
    return {"origin": s["origin"], "out": s["out"], "palm": s["palm"],
            "offset_mm": [0, 0, 0], "rotate_deg": [0, 0, 0],
            "mirror": {"left": mirror_left, "right": False},
            "scale": 0.001 if unit == "mm" else 1.0}


def preview_model(mesh, s, unit, mirror_left):
    """G1 mit der Hand nach Vorschlag s (nur Optik, im Speicher gebaut)."""
    import mujoco
    import numpy as np
    spec = mujoco.MjSpec.from_file(str(gen.BASE_MJCF))
    spec.option.gravity = [0, 0, 0]
    p = _profile_dict(s, unit, mirror_left)
    for side in gen.SIDES:
        body = spec.body(f"{side}_wrist_yaw_link")
        for g in list(body.geoms):
            if g.meshname == f"{side}_rubber_hand":
                spec.delete(g)
        A, b = gen.side_transform(p, side)
        tm = gen.transform_mesh(mesh, A, b)
        me = spec.add_mesh()
        me.name = f"new_hand_{side}"
        me.uservert = np.asarray(tm.vertices, dtype=float).flatten().tolist()
        me.userface = np.asarray(tm.faces, dtype=int).flatten().tolist()
        g = body.add_geom()
        g.type = mujoco.mjtGeom.mjGEOM_MESH
        g.meshname = me.name
        g.name = f"{side}_new_hand"
        g.contype = g.conaffinity = 0
        g.density = 0
        g.rgba = [0.95, 0.55, 0.10, 1.0]
        # Flansch-Flaeche als blaue Scheibe, Greifpunkt als gruene Kugel
        f = body.add_geom()
        f.type = mujoco.mjtGeom.mjGEOM_CYLINDER
        f.size = [0.024, 0.0008, 0]
        f.pos = [gen.FLANGE_X, 0, 0]
        f.quat = [math.cos(math.pi / 4), 0, math.sin(math.pi / 4), 0]
        f.contype = f.conaffinity = 0
        f.density = 0
        f.rgba = [0.2, 0.45, 1.0, 0.9]
        t = body.add_geom()
        t.type = mujoco.mjtGeom.mjGEOM_SPHERE
        t.size = [0.006, 0, 0]
        t.pos = (A @ np.asarray(s["tcp"], dtype=float) + b).tolist()
        t.contype = t.conaffinity = 0
        t.density = 0
        t.rgba = [0.1, 0.9, 0.2, 1.0]
    tex = spec.add_texture()
    tex.name, tex.type, tex.builtin = ("sky", mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                                       mujoco.mjtBuiltin.mjBUILTIN_GRADIENT)
    tex.rgb1, tex.rgb2, tex.width, tex.height = [0.35, 0.5, 0.65], [0.05, 0.05, 0.08], 256, 1536
    spec.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1],
                             type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL)
    m = spec.compile()
    arm = {m.body(f"{sd}_{lk}").id for sd in gen.SIDES for lk in ARM_LINKS}
    for gi in range(m.ngeom):
        if m.geom_bodyid[gi] in arm:
            m.geom_rgba[gi, 3] = 0.3
            m.geom_matid[gi] = -1
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def render_suggestion(mesh, s, unit, mirror_left, out_png):
    import mujoco
    import numpy as np
    m, d = preview_model(mesh, s, unit, mirror_left)
    w, h = THUMB
    r = mujoco.Renderer(m, h, w)
    size = float(max(mesh.extents)) * (0.001 if unit == "mm" else 1.0)
    gr = m.geom("right_new_hand").id
    gl = m.geom("left_new_hand").id
    c_r = d.geom_xpos[gr]                   # Mesh-Mitte (MuJoCo zentriert Meshes)
    views = []
    cam = mujoco.MjvCamera()
    cam.lookat[:] = c_r
    cam.distance, cam.azimuth, cam.elevation = max(0.3, 3.2 * size), 120, -20
    r.update_scene(d, cam)
    views.append(r.render())
    cam = mujoco.MjvCamera()
    cam.lookat[:] = (d.geom_xpos[gr] + d.geom_xpos[gl]) / 2
    cam.distance, cam.azimuth, cam.elevation = 0.4 + 2.0 * size, 180, -25
    r.update_scene(d, cam)
    views.insert(0, r.render())
    r.close()
    img = np.concatenate(views, axis=1)
    try:
        from PIL import Image
        Image.fromarray(img).save(out_png)
    except ImportError:                    # ohne Pillow: PPM, kann Tk auch
        out_png = str(out_png)[:-4] + ".ppm"
        with open(out_png, "wb") as fh:
            fh.write(f"P6 {img.shape[1]} {img.shape[0]} 255\n".encode() + img.tobytes())
    return str(out_png)


def analyse(path, out_dir, unit=None, mirror_left=True, n=4, render=True):
    mesh = det.load_mesh(path)
    guess = det.guess_unit(mesh)
    unit = unit or guess
    res = {"unit": unit, "unit_guess": guess, "extents": det._clean(mesh.extents),
           "mass": mass_estimate(mesh, unit),
           "suggestions": det.suggest_mounts(mesh, n, unit)}
    if render:
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        for i, s in enumerate(res["suggestions"]):
            try:
                s["image"] = render_suggestion(mesh, s, unit, mirror_left,
                                               out_dir / f"vorschlag_{i + 1}.png")
            except Exception as e:                       # kein OpenGL -> ohne Bild
                s["image"] = ""
                s["image_error"] = str(e)
    return res


# ══════════════════════════════════════════════════════════════════════
# Fenster
# ══════════════════════════════════════════════════════════════════════
FILETYPES = [("3D-Modelle", " ".join(f"*{e} *{e.upper()}" for e in det.MESH_TYPES)),
             ("Alle Dateien", "*")]


def run_wizard(initial=None):
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    root = tk.Tk()
    root.title("Neue Hand anlegen")
    root.geometry("+40+40")
    st = {"res": None, "busy": False, "imgs": [], "tmp": tempfile.mkdtemp(prefix="hand_new_")}
    v_file = tk.StringVar(value=str(initial or ""))
    v_title = tk.StringVar()
    v_slug = tk.StringVar()
    v_unit = tk.StringVar(value="mm")
    v_mass = tk.StringVar()
    v_mirror = tk.BooleanVar(value=True)
    v_pick = tk.IntVar(value=0)
    v_status = tk.StringVar(value="Datei wählen.")

    top = ttk.Frame(root, padding=10)
    top.pack(fill="x")
    ttk.Label(top, text="Datei:").grid(row=0, column=0, sticky="w")
    ttk.Entry(top, textvariable=v_file, width=56, state="readonly").grid(row=0, column=1, sticky="we")
    ttk.Button(top, text="Datei wählen …", command=lambda: choose()).grid(row=0, column=2, padx=6)
    ttk.Label(top, text="Name:").grid(row=1, column=0, sticky="w", pady=4)
    ttk.Entry(top, textvariable=v_title, width=56).grid(row=1, column=1, sticky="we")
    ttk.Label(top, textvariable=v_slug, foreground="#666").grid(row=1, column=2, sticky="w", padx=6)
    opts = ttk.Frame(top)
    opts.grid(row=2, column=1, sticky="w", pady=4)
    ttk.Label(opts, text="Einheit:").pack(side="left")
    for u in ("mm", "m"):
        ttk.Radiobutton(opts, text=u, value=u, variable=v_unit,
                        command=lambda: start_analyse()).pack(side="left", padx=2)
    ttk.Label(opts, text="   Masse je Hand (kg):").pack(side="left")
    ttk.Entry(opts, textvariable=v_mass, width=6).pack(side="left")
    ttk.Checkbutton(opts, text="  Linke Hand gespiegelt (eine Datei für beide)",
                    variable=v_mirror, command=lambda: start_analyse()).pack(side="left")
    top.columnconfigure(1, weight=1)

    ttk.Label(root, text="Vorschläge (links beide Hände von vorn, rechts die rechte Hand; "
                         "blaue Scheibe = Flansch am Handgelenk, grüner Punkt = Greifpunkt):",
              padding=(10, 0)).pack(anchor="w")
    grid = ttk.Frame(root, padding=10)
    grid.pack(fill="both", expand=True)
    bottom = ttk.Frame(root, padding=10)
    bottom.pack(fill="x")
    ttk.Label(bottom, textvariable=v_status, wraplength=560).pack(side="left")
    b_ok = ttk.Button(bottom, text="Anlegen und Bearbeitungstool öffnen", command=lambda: create())
    b_ok.pack(side="right")
    ttk.Button(bottom, text="Abbrechen", command=root.destroy).pack(side="right", padx=6)
    b_ok.state(["disabled"])

    def sync_slug(*_):
        t = v_title.get().strip()
        v_slug.set(f"→ hands/{free_slug(slugify(t))}/" if t else "")
    v_title.trace_add("write", sync_slug)

    def choose():
        f = filedialog.askopenfilename(parent=root, title="3D-Datei der Hand",
                                       filetypes=FILETYPES)
        if f:
            v_file.set(f)
            v_unit.set("")
            if not v_title.get().strip():
                v_title.set(Path(f).stem.replace("_", " "))
            start_analyse()

    def start_analyse():
        if not v_file.get() or st["busy"]:
            return
        st["busy"] = True
        b_ok.state(["disabled"])
        v_status.set("Erkenne Montage und rendere Vorschläge …")
        cmd = [sys.executable, str(Path(__file__).resolve()), "--analyse", v_file.get(),
               "--out", st["tmp"], "--mirror-left", "1" if v_mirror.get() else "0"]
        if v_unit.get():
            cmd += ["--unit", v_unit.get()]

        def work():
            try:
                pr = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
                res = json.loads(pr.stdout.strip().splitlines()[-1]) if pr.returncode == 0 \
                    else {"error": (pr.stderr or pr.stdout).strip()[-800:]}
            except Exception as e:
                res = {"error": str(e)}
            root.after(0, lambda: done(res))
        threading.Thread(target=work, daemon=True).start()

    def done(res):
        st["busy"] = False
        if "error" in res:
            v_status.set("Fehler: " + res["error"])
            return
        st["res"] = res
        v_unit.set(res["unit"])
        if not v_mass.get().strip() or st.get("mass_auto") == v_mass.get():
            v_mass.set(str(res["mass"]))
            st["mass_auto"] = v_mass.get()
        show(res)
        ex = " × ".join(f"{x:g}" for x in res["extents"])
        v_status.set(f"Größe {ex} {res['unit']}. Vorschlag wählen; danach im "
                     "Bearbeitungstool fein einpassen.")
        b_ok.state(["!disabled"])

    def show(res):
        for w in grid.winfo_children():
            w.destroy()
        st["imgs"] = []
        if v_pick.get() >= len(res["suggestions"]):
            v_pick.set(0)
        for i, s in enumerate(res["suggestions"]):
            card = ttk.Frame(grid, padding=4, relief="groove")
            card.grid(row=i // 2, column=i % 2, padx=4, pady=4, sticky="n")
            if s.get("image"):
                img = tk.PhotoImage(file=s["image"])
                st["imgs"].append(img)
                tk.Label(card, image=img).pack()
            else:
                ttk.Label(card, text="(kein Bild: " + s.get("image_error", "?")[:80] + ")").pack()
            ttk.Radiobutton(card, text=f"Vorschlag {i + 1}" + ("  (wahrscheinlichster)" if i == 0
                                                                 else ""),
                            value=i, variable=v_pick).pack(anchor="w")
            ttk.Label(card, text=s["why"], foreground="#555", wraplength=2 * THUMB[0]).pack(
                anchor="w")

    def create():
        res = st["res"]
        title = v_title.get().strip()
        if not res or not title:
            messagebox.showerror("Neue Hand", "Bitte Datei wählen und einen Namen geben.",
                                 parent=root)
            return
        try:
            mass = float(v_mass.get().replace(",", "."))
            assert mass > 0
        except Exception:
            messagebox.showerror("Neue Hand", "Masse: Zahl > 0 in kg.", parent=root)
            return
        i = v_pick.get()
        try:
            slug = create_profile(Path(v_file.get()), title, res["suggestions"][i],
                                  unit=res["unit"], mirror_left=v_mirror.get(), mass=mass,
                                  pick=i + 1, unit_guess=res["unit_guess"])
        except Exception as e:
            messagebox.showerror("Neue Hand", str(e), parent=root)
            return
        b_ok.state(["disabled"])
        v_status.set(f"hands/{slug}/ angelegt. Erzeuge Modell (Kollisions-Zerlegung, "
                     "dauert etwas) …")

        def work():
            # ueber hand_tool.sh: nimmt das Szenen-Editor-venv (vhacdx fuer die Zerlegung)
            pr = subprocess.run(["bash", str(HERE / "hand_tool.sh"), "build", slug],
                                capture_output=True, text=True)
            root.after(0, lambda: built(slug, pr))
        threading.Thread(target=work, daemon=True).start()

    def built(slug, pr):
        if pr.returncode != 0:
            v_status.set("Erzeugen fehlgeschlagen: " + (pr.stderr or pr.stdout)[-600:])
            return
        print(pr.stdout, flush=True)
        print(f"Neue Hand: hands/{slug}/ -> Bearbeitungstool", flush=True)
        st["next"] = slug
        root.destroy()

    if initial:
        v_title.set(Path(initial).stem.replace("_", " "))
        v_unit.set("")
        root.after(100, start_analyse)
    root.mainloop()
    shutil.rmtree(st["tmp"], ignore_errors=True)
    if st.get("next"):
        # gleich weiter im Bearbeitungstool (selber Prozess -> die Konsole der
        # Start-GUI zeigt es weiter an, "Stoppen" schliesst es)
        adj = str(HERE / "hand_adjust.py")
        os.execv(sys.executable, [sys.executable, adj, st["next"]])
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="Neue Hand anlegen (Datei + Montage-Vorschlag)")
    ap.add_argument("file", nargs="?", help="3D-Datei (STL, OBJ, PLY, 3MF, ...)")
    ap.add_argument("--analyse", metavar="DATEI", help=argparse.SUPPRESS)
    ap.add_argument("--out", help=argparse.SUPPRESS)
    ap.add_argument("--unit", choices=("mm", "m"))
    ap.add_argument("--mirror-left", default="1", choices=("0", "1"))
    ap.add_argument("--create", metavar="DATEI", help="ohne Fenster anlegen")
    ap.add_argument("--name", help="Anzeigename (mit --create)")
    ap.add_argument("--slug", default="", help="Ordnername (Standard: aus dem Namen)")
    ap.add_argument("--pick", type=int, default=1, help="Vorschlag Nr. (mit --create)")
    ap.add_argument("--mass", type=float)
    ap.add_argument("--no-build", action="store_true")
    a = ap.parse_args(argv)
    mirror = a.mirror_left == "1"
    if a.analyse:
        print(json.dumps(analyse(a.analyse, a.out or tempfile.mkdtemp(), a.unit, mirror)))
        sys.stdout.flush()
        os._exit(0)                     # GL-Kontext nicht aufraeumen (GLFW-Absturz)
    if a.create:
        res = analyse(a.create, None, a.unit, mirror, render=False)
        sug = res["suggestions"]
        if not 1 <= a.pick <= len(sug):
            sys.exit(f"--pick: 1..{len(sug)}")
        slug = create_profile(Path(a.create), a.name or Path(a.create).stem, sug[a.pick - 1],
                              unit=res["unit"], slug=a.slug, mirror_left=mirror,
                              mass=a.mass, pick=a.pick, unit_guess=res["unit_guess"])
        print(f"Angelegt: hands/{slug}/hand.yaml  ({sug[a.pick - 1]['why']})")
        if not a.no_build:
            gen.build(slug)
        return 0
    try:
        import tkinter  # noqa: F401
    except ImportError:
        v = f"{sys.version_info.major}.{sys.version_info.minor}"
        sys.exit(f"tkinter fehlt fuer Python {v}: sudo apt install python{v}-tk "
                 "(oder ohne Fenster: hand_new.py --create <datei> --name <name>)")
    return run_wizard(a.file)


if __name__ == "__main__":
    sys.exit(main())
