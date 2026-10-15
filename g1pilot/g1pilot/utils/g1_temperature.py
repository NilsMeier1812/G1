"""Temperaturen des echten G1 (rt/lowstate) -- reine Datenaufbereitung.

Quelle (unitree_hg LowState_): je Motor ``temperature[2]`` (zwei Sensoren,
int16 in Grad C) und ``vol``, dazu ``imu_state.temperature``. Der G1 hat
29 Koerper-Motoren -> 58 Motor-Sensoren + 1 IMU = 59 Messwerte. (Die Inspire-
Haende, BMS und Mainboard haben eigene Topics und sind hier nicht enthalten.)

Kein ROS, kein Unitree-SDK, kein Qt -> im Testcontainer direkt testbar.
robot_state.py baut daraus die JSON-Nachricht ``/g1pilot/temperatures``,
demo_gui.py wertet sie aus (Gruppen, Farbskala, heissester Motor).
"""
import json
import os

# Reihenfolge = G1_29_JointIndex / rt/lowstate motor_state[0..28].
MOTOR_NAMES = (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee",
    "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee",
    "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow", "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow", "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw",
)
N_MOTORS = len(MOTOR_NAMES)

# Anzeige-Gruppen: (Schluessel, Titel, Motor-Indizes). Hueften, Knie und
# Knoechel (Beine) sowie Taille tragen im Stand/Gehen die Last -> je Bein eine
# Kachel; Arme je Seite.
GROUPS = (
    ("leg_l", "Bein L", tuple(range(0, 6))),
    ("leg_r", "Bein R", tuple(range(6, 12))),
    ("waist", "Taille", tuple(range(12, 15))),
    ("arm_l", "Arm L", tuple(range(15, 22))),
    ("arm_r", "Arm R", tuple(range(22, 29))),
)

_DE = {
    "left": "links", "right": "rechts", "hip": "Hüfte", "knee": "Knie",
    "ankle": "Knöchel", "waist": "Taille", "shoulder": "Schulter",
    "elbow": "Ellbogen", "wrist": "Handgelenk",
    "pitch": "Nick", "roll": "Roll", "yaw": "Gier",
}

# Schwellen [Grad C]. ANNAHME (kein Herstellerwert im Repo): ab WARN gelb,
# ab CRIT rot. Per Umgebung anpassbar (G1_TEMP_WARN / G1_TEMP_CRIT).
DEFAULT_WARN_C = 60.0
DEFAULT_CRIT_C = 75.0
STALE_S = 3.0           # aelter -> Anzeige »keine Daten«


def thresholds():
    """(warn, crit) aus der Umgebung; ungueltige Werte -> Defaults."""
    def _f(name, default):
        try:
            v = float(os.environ.get(name, ""))
        except ValueError:
            return default
        return v if v > 0 else default
    warn = _f("G1_TEMP_WARN", DEFAULT_WARN_C)
    crit = _f("G1_TEMP_CRIT", DEFAULT_CRIT_C)
    if crit <= warn:
        return DEFAULT_WARN_C, DEFAULT_CRIT_C
    return warn, crit


def motor_label(idx):
    """Deutscher Anzeigename, z.B. 3 -> »Knie links«, 0 -> »Hüfte Nick links«."""
    parts = MOTOR_NAMES[idx].split("_")
    side = _DE[parts[0]] if parts[0] in ("left", "right") else None
    joint = _DE[parts[1]] if parts[0] in ("left", "right") else _DE[parts[0]]
    axis = parts[-1] if len(parts) > (2 if side else 1) and parts[-1] in _DE else None
    if parts[-1] == "knee" or parts[-1] == "elbow":
        axis = None
    name = joint + (f" {_DE[axis]}" if axis else "")
    return f"{name} {side}" if side else name


# ── Nachricht (robot_state -> GUI) ───────────────────────────────────────
def build_payload(motor_states, imu_temp):
    """JSON-String aus den DDS-Rohwerten.

    ``motor_states``: Folge mit ``.temperature`` (2 Werte) und ``.vol``
    (mindestens 29 Eintraege; weitere werden ignoriert).
    Format: {"t": [[t1, t2] * 29], "v": [vol * 29], "imu": t}
    """
    t, v = [], []
    for i in range(N_MOTORS):
        m = motor_states[i]
        temp = m.temperature
        try:
            pair = [float(temp[0]), float(temp[1])]
        except TypeError:                      # Skalar statt Array
            pair = [float(temp), float(temp)]
        t.append(pair)
        v.append(round(float(m.vol), 2))
    return json.dumps({"t": t, "v": v, "imu": float(imu_temp)},
                      separators=(",", ":"))


def parse_payload(text):
    """JSON-String -> dict oder None bei kaputten/unvollstaendigen Daten."""
    try:
        d = json.loads(text)
        t = d["t"]
        if len(t) != N_MOTORS or any(len(p) != 2 for p in t):
            return None
        return {"t": [[float(a), float(b)] for a, b in t],
                "v": [float(x) for x in d.get("v", [])],
                "imu": float(d.get("imu", 0.0))}
    except (ValueError, KeyError, TypeError):
        return None


# ── Auswertung (GUI) ─────────────────────────────────────────────────────
def motor_temp(data, idx):
    """Kritischer Wert eines Motors = Maximum seiner beiden Sensoren."""
    return max(data["t"][idx])


def group_summary(data):
    """Je Gruppe: {key, title, max, idx (heissester Motor), label, temps}."""
    out = []
    for key, title, idxs in GROUPS:
        hot = max(idxs, key=lambda i: motor_temp(data, i))
        out.append({"key": key, "title": title, "max": motor_temp(data, hot),
                    "idx": hot, "label": motor_label(hot),
                    "temps": {i: motor_temp(data, i) for i in idxs}})
    return out


def hottest(data):
    """(Motor-Index, Temperatur) des heissesten Motors insgesamt."""
    idx = max(range(N_MOTORS), key=lambda i: motor_temp(data, i))
    return idx, motor_temp(data, idx)


def level(temp, warn=DEFAULT_WARN_C, crit=DEFAULT_CRIT_C):
    """'ok' | 'warn' | 'crit'."""
    if temp >= crit:
        return "crit"
    if temp >= warn:
        return "warn"
    return "ok"


LEVEL_COLOR = {"ok": "#2e7d32", "warn": "#f9a825", "crit": "#c62828",
               "none": "#37474f"}


def is_stale(last_t, now, stale_s=STALE_S):
    return last_t is None or (now - last_t) > stale_s
