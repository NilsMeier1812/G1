"""Welche Haende hat der G1 in DIESEM Lauf? (nur Simulation umschaltbar)

G1_HANDS waehlt das Hand-Profil:
  inspire   Inspire-FTP-Haende (Finger steuerbar, Bridge laeuft)
  rubber    starre Rubber-Hand (MuJoCo-Standardmodell)
  <profil>  eigenes starres Werkzeug aus unitree_robots/g1/hands/<profil>/
            (z. B. "haken"), erzeugt von tools/gen_hand_profile.py

Ist G1_HANDS leer, gilt das alte Verhalten ueber G1_INSPIRE_HANDS (1 = inspire,
sonst rubber). Eigene Profile gibt es NUR in der Sim (G1_SIM_MODE=true): am
echten Roboter bleibt alles beim Inspire-URDF wie bisher.

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


def _sim_mode(env) -> bool:
    return str(env.get("G1_SIM_MODE", "false")).strip().lower() == "true"


def hand_profile(env=None) -> str:
    """Aktives Profil: 'inspire', 'rubber' oder der Name eines eigenen Profils."""
    env = os.environ if env is None else env
    p = str(env.get("G1_HANDS", "")).strip().lower()
    if p in BUILTIN:
        return p
    if p and _NAME_RE.match(p) and _sim_mode(env):
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
    """Pfad zum URDF des aktiven Profils; fehlt die Datei, Rueckfall aufs
    Inspire-URDF (mit Warnung) statt eines Absturzes beim Start."""
    name = urdf_file_name(env)
    path = os.path.join(urdf_dir, name)
    if name != DEFAULT_URDF and not os.path.isfile(path):
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
