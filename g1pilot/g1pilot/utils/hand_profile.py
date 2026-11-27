"""Welche Haende hat der G1 in DIESEM Lauf? (Sim und echter Roboter)

G1_HANDS waehlt das Hand-Profil:
  inspire   Inspire-FTP-Haende (Finger steuerbar, Bridge laeuft)
  rubber    starre Rubber-Hand (MuJoCo-Standardmodell)
  <profil>  eigenes starres Werkzeug aus unitree_robots/g1/hands/<profil>/
            (z. B. "haken"), erzeugt von tools/gen_hand_profile.py

Ist G1_HANDS leer, gilt das alte Verhalten ueber G1_INSPIRE_HANDS (1 = inspire,
sonst rubber). Rubber und Inspire teilen sich das Inspire-URDF.

Echter Roboter (G1_SIM_MODE != true, seit 2026-11-27): eigene Profile gelten
auch dort (Kugelmodell des Armplaners, Selbstkollisions-Gate, IK-TCP,
Schwerkraft-Feedforward und RViz nutzen dasselbe URDF). Weil ein falsches
Modell am echten Roboter gefaehrlich ist, gibt es real KEINEN Rueckfall auf
Inspire: ungueltiger Profilname oder fehlendes Profil-URDF -> urdf_path()
wirft HandProfileError und der Start bricht ab. In der Sim bleibt der
Rueckfall (mit Warnung) wie bisher. start.sh prueft/erzeugt das Profil schon
vorher auf dem Host und bricht real ebenfalls ab.

Dieselbe Regel steht (ohne g1pilot-Import) in unitree_mujoco/simulate_python/
config.py und scene_editor/build_env_scene.py.
"""
import os
import re

INSPIRE = "inspire"
RUBBER = "rubber"
BUILTIN = (INSPIRE, RUBBER)
DEFAULT_URDF = "g1_29dof_inspire_ftp.urdf"
_NAME_RE = re.compile(r"^[a-z0-9_]+$")
_TRUE = ("1", "true", "yes", "on")


class HandProfileError(RuntimeError):
    """Hand-Profil am echten Roboter nicht nutzbar -> Start abbrechen."""


def _sim_mode(env) -> bool:
    return str(env.get("G1_SIM_MODE", "false")).strip().lower() == "true"


def hand_profile(env=None) -> str:
    """Aktives Profil: 'inspire', 'rubber' oder der Name eines eigenen Profils."""
    env = os.environ if env is None else env
    p = str(env.get("G1_HANDS", "")).strip().lower()
    if p in BUILTIN:
        return p
    if p and _NAME_RE.match(p):
        return p
    return INSPIRE if str(env.get("G1_INSPIRE_HANDS", "0")).strip().lower() in _TRUE else RUBBER


def is_custom(profile: str) -> bool:
    return profile not in BUILTIN


def inspire_active(env=None) -> bool:
    """True, wenn die Inspire-Finger (Bridge, Finger-Gelenke) aktiv sind."""
    return hand_profile(env) == INSPIRE


def urdf_file_name(env=None) -> str:
    """URDF fuer IK, Armplaner, Kollisions-Gate und robot_state_publisher.
    Rubber und Inspire teilen sich wie bisher das Inspire-URDF (der TCP ist bei
    beiden gleich); nur eigene Profile haben ein eigenes URDF."""
    p = hand_profile(env)
    return f"g1_29dof_hand_{p}.urdf" if is_custom(p) else DEFAULT_URDF


def urdf_path(urdf_dir: str, env=None, log=print) -> str:
    """Pfad zum URDF des aktiven Profils.

    Sim: fehlt die Datei, Rueckfall aufs Inspire-URDF (mit Warnung) statt
    eines Absturzes beim Start. Real: kein Rueckfall -> HandProfileError."""
    env = os.environ if env is None else env
    raw = str(env.get("G1_HANDS", "")).strip().lower()
    sim = _sim_mode(env)
    if not sim and raw and raw not in BUILTIN and not _NAME_RE.match(raw):
        raise HandProfileError(
            f"[hand_profile] G1_HANDS='{raw}' ist kein gueltiger Profilname "
            f"(erlaubt: inspire, rubber oder a-z/0-9/_). Echter Roboter: Abbruch.")
    name = urdf_file_name(env)
    path = os.path.join(urdf_dir, name)
    if name != DEFAULT_URDF and not os.path.isfile(path):
        if not sim:
            raise HandProfileError(
                f"[hand_profile] {path} fehlt (Profil '{hand_profile(env)}' nicht erzeugt?). "
                f"Echter Roboter: kein Rueckfall auf {DEFAULT_URDF} -> Abbruch. "
                f"Erzeugen: unitree_mujoco/unitree_robots/g1/tools/hand_tool.sh build "
                f"{hand_profile(env)}")
        if log:
            log(f"[hand_profile] WARN: {path} fehlt (Profil nicht erzeugt / nicht gebaut?) "
                f"-> nutze {DEFAULT_URDF}.")
        path = os.path.join(urdf_dir, DEFAULT_URDF)
    return path


def mjcf_file_name(env=None) -> str:
    """MuJoCo-Robotermodell (ohne Szene) des aktiven Profils."""
    p = hand_profile(env)
    if p == INSPIRE:
        return "g1_29dof_inspire_ftp.xml"
    if p == RUBBER:
        return "g1_29dof.xml"
    return f"g1_29dof_hand_{p}.xml"
