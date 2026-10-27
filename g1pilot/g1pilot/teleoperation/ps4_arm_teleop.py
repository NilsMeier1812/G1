#!/usr/bin/env python3
"""
ps4_arm_teleop — Oberkoerper (Arme + Haende) mit dem PS4-Controller steuern.

Liest den Controller in fester Belegung (ps4_layout.py) von /g1pilot/ps4/joy
(Quelle: ps4_joystick) und spricht DIESELBEN Schnittstellen wie RViz-Marker,
Demo-GUI und Streamdeck -- die Logik (IK, Kollisions-Gate, Haende) bleibt im
arm_controller bzw. in der Inspire-Bridge:

  /g1pilot/hand_goal/<side>     PoseStamped  kartesisches Hand-Ziel (pelvis)
  /g1pilot/hand_action/<side>   String       "open" / "close"
  /g1pilot/arms/enabled         Bool         Arme uebernehmen
  /g1pilot/start_balancing      Bool         Greifen-Modus (stehen, Arme frei; nur Sim)
  /g1pilot/arms/home            Bool         Grundstellung (Impuls)
  /g1pilot/pose_store/cancel    Bool         laufende geplante Bewegung abbrechen
  /g1pilot/emergency_stop       Bool         NOT-HALT (wie in den GUIs)
  /g1pilot/full_emergency_stop  Bool         VOLL-NOTAUS: ganzer G1 Damp (R1+L1+PS)
  /g1pilot/arm_command          String(JSON) Arm spiegeln ({"type": "mirror"}),
                                             Antwort auf /g1pilot/arm_command/status
  /g1pilot/sequence/next        Bool         Ablauf: naechster Schritt (L3); die
                                             Demo-GUI nimmt es nur an, wenn ein
                                             Ablauf auf Bestaetigung wartet
                                             (/g1pilot/sequence/state, latched)

KEIN Laufen: der Node publiziert nichts an loco_cmd_vel / joy_mux. Rueckmeldung:
  /g1pilot/ps4/feedback  String(JSON)  LED-Farbe + Vibration -> ps4_joystick
  /g1pilot/ps4/status    String(JSON)  Auswahl/Modus/Tempo (fuer GUIs/Logs)
  /g1pilot/ps4/markers   MarkerArray   farbige Kugel an der aktiven Hand
  /g1pilot/teleop_state  String(JSON)  gemeinsamer Zustand mit der Demo-GUI
                         (Hand-Auswahl, Tempo, Modus; shared_state.py) --
                         publiziert eigene Wechsel, uebernimmt die der GUI

Belegung (Details: g1pilot/docs/11-A-Controller.md):
  Steuerkreuz  links/rechts/oben/unten = linke / rechte / beide / beide gespiegelt
  Dreieck      Umschalten VERSCHIEBEN <-> DREHEN (kurze Vibration nur beim
               Wechsel zu VERSCHIEBEN -- so ohne GUI unterscheidbar)
  Linker Stick VERSCHIEBEN: vor/zurueck + links/rechts | DREHEN: kippen + schwenken
  Rechter Stick VERSCHIEBEN: hoch/runter (vertikal)    | DREHEN: rollen (horizontal)
  L1 / R1      Tempo langsamer / schneller (Langsam, Normal, Schnell);
               im GEHEN bzw. mit gehaltenem L2: Geh-Tempo (ps4_walk)
  Kreis / Viereck  Hand oeffnen / schliessen (ausgewaehlte Hand/Haende)
  Kreuz        Stopp: Hand haelt sofort an, geplante Bewegung abbrechen
  Options      Controller SPERREN / ENTSPERREN (zum Ablegen: dann loest ein
               versehentlich gedrueckter L2/R2 nichts aus). Gesperrt wirken
               nur noch PS (NOT-HALT), R1 + L1 + PS (Voll-Notaus) und Options
               selbst; LED fast aus (dunkelgrau), lange Vibration beim
               Sperren, kurze beim Entsperren, jede andere Taste (auch
               L2/R2) vibriert gesperrt kurz und leicht. Nach dem Entsperren erst Sticks
               in die Mitte und L2 einmal los, bevor sie wieder wirken.
               (Bis 2026-10: Arme uebernehmen -- das macht jetzt R2.)
  Share (1 s halten)  Grundstellung
  L3 (linken Stick druecken)  Ablauf »Schritt für Schritt«: naechster Schritt
               (wie »Weiter ▶« in der Demo-GUI). Kurze Vibration, sobald ein
               Ablauf wartet; Kreuz bricht ihn ab, PS (NOT-HALT) sowieso.
               Solange ein Ablauf-Schritt / eine Pose der GUI faehrt, bewegen
               die Sticks die Haende nicht (ein Hand-Ziel braeche die Fahrt ab).
  PS           NOT-HALT (ps_estop=false: nur Stopp wie Kreuz)
  R1 + L1 + PS (2 s halten, Controller vibriert)  VOLL-NOTAUS: ganzer G1
               in Damp (auch die Beine, /g1pilot/full_emergency_stop).
               Vorzeitig loslassen bricht ab. Zurueck: START quittiert,
               der G1 bleibt in Damp; Aufstehen nur ueber die Startleiste
               (Preparation -> Walk).
  Steuerkreuz links/rechts + Kreuz (2 s halten, Controller vibriert)
               ARM SPIEGELN: der Arm dieser Seite faehrt geplant (Planer,
               Kollisionsschutz) in die gespiegelte Pose des anderen Arms.
               Vorzeitig loslassen bricht ab; die Fahrt selbst stoppt Kreuz,
               PS oder ein Stick-Ausschlag. Nur mit uebernommenen Armen.
  L2 halten    Gehen (ps4_walk, nur mit walk_deadman): Sticks + Steuerkreuz
               gehoeren dann dem Gehen, die Arme halten ihre Lage
  R2 halten    HUEFTE (waist_yaw, nur im GREIFEN): rechter Stick links/rechts
               dreht den Oberkoerper (Taillen-Yaw), R2 + R3 = zurueck zur Mitte.
               Loslassen = Huefte steht. Sticks/Steuerkreuz bewegen dabei die Arme
               nicht; die Arme halten ihre Gelenkstellung und drehen mit.
               Jog-Richtungen gelten danach im Oberkoerper-Bezug.
  L2 druecken  Modus GEHEN, R2 druecken = Modus GREIFEN (trigger_modes), wie die
               Kacheln der Demo-GUI. R2 im GREIFEN mit abgegebenen Armen
               (z. B. nach NOT-HALT, /g1pilot/arms/state False) = Arme
               uebernehmen (Sim: + stehen, options_starts_balancing); ohne
               trigger_modes macht R2 nur das. Im GEHEN laeuft der G1 nur, solange L2
               gehalten wird; losgelassen bleibt GEHEN, die Arm-Tasten und
               Sticks sind dann gesperrt (LED blau wie die Kachel GEHEN)
               bis R2.
               Real nur, wenn der G1 schon im Walk-Modus steht -- der
               Controller stellt die Beine nie selbst auf.
"""
import json
import math
import time

from g1pilot.teleoperation import ps4_layout as L
from g1pilot.teleoperation import shared_state
from g1pilot.teleoperation import sequence_runner as sr
from g1pilot.teleoperation.ps4_walk import deadman_held
from g1pilot.teleoperation.hand_jog import (
    JOG_FRAME, JOG_TF, JOG_SPEEDS, JOG_SPEED_ORDER, JOG_CMD_TOPIC, CommandedPoses,
    HoldStreamer, JogReference, hold_pose, jog_reference_mode, lead_limits, jog_step,
)
from g1pilot.utils.common import is_sim_mode
from g1pilot.manipulation.waist_yaw import (
    WAIST_RATE_TOPIC, WAIST_CENTER_TOPIC, WAIST_STATE_TOPIC, STATE_STALE_S,
    parse_state, rotate_twist_yaw,
)

# Auswahl -> betroffene Seiten
SELECTIONS = {
    "left":   ("left",),
    "right":  ("right",),
    "both":   ("left", "right"),
    "mirror": ("left", "right"),
}
DPAD_SELECTION = ((L.DPAD_LEFT, "left"), (L.DPAD_RIGHT, "right"),
                  (L.DPAD_UP, "both"), (L.DPAD_DOWN, "mirror"))
SELECTION_LABEL = {"left": "LINKS", "right": "RECHTS", "both": "BEIDE",
                   "mirror": "BEIDE gespiegelt"}
MODE_LABEL = {"move": "Verschieben", "turn": "Drehen"}

# Farben (RGB 0..255) -- Controller-LED und RViz-Marker gleich.
COLORS = {
    "left":   (255, 200, 0),    # gelb (blau ist GEHEN)
    "right":  (0, 200, 60),     # gruen
    "both":   (170, 0, 255),    # violett
    "mirror": (0, 200, 200),    # tuerkis
    "estop":  (255, 0, 0),      # rot
    # blau wie die Kachel GEHEN der Demo-GUI (MODE_COLOR): Modus GEHEN, Arme
    # am Controller gesperrt. (Bis 2026-10 orange -- das ist dort GREIFEN.)
    "walk":   (30, 136, 229),
    "waist":  (255, 255, 255),  # weiss: R2 gehalten = Huefte drehen
    "locked": (40, 40, 40),     # fast aus: Controller per Options gesperrt
}
TURN_DIM = 0.35   # DREHEN: LED gedimmt (gleiche Farbe), Verschieben = voll


def shape_axis(x, deadzone, expo):
    """Stick-Wert -> Sollwert: Totzone (mit Neuskalierung, kein Sprung am Rand)
    und Expo-Kurve (feinfuehlig um die Mitte, voller Ausschlag bleibt 1)."""
    a = abs(x)
    if a <= deadzone:
        return 0.0
    a = min(1.0, (a - deadzone) / (1.0 - deadzone))
    a = (1.0 - expo) * a + expo * a ** 3
    return math.copysign(a, x)


def snap_stick(x, y, snap_deg):
    """Achsen-Einrasten eines Sticks: liegt die Auslenkung hoechstens
    `snap_deg` neben einer Achse, zaehlt nur diese Achse (die andere -> 0).
    Ein Daumen drueckt nie exakt gerade -- ohne Einrasten fuhr "vor" immer
    ein wenig seitlich mit (bei 10 Grad schraeg ~3-5 % des Tempos, trotz
    Totzone). Dazwischen (echte Diagonale) bleiben beide Achsen. <= 0 = aus."""
    if snap_deg <= 0.0 or (x == 0.0 and y == 0.0):
        return x, y
    ang = math.degrees(math.atan2(abs(y), abs(x)))     # 0 = reine x-Achse
    if ang <= snap_deg:
        return x, 0.0
    if ang >= 90.0 - snap_deg:
        return 0.0, y
    return x, y


def mirror_twist(lin, ang):
    """Spiegelung an der Sagittalebene (x-z) des Roboters: Verschiebung
    (vx, -vy, vz), Drehvektor (-wx, wy, -wz) -- Drehungen sind axial."""
    return (lin[0], -lin[1], lin[2]), (-ang[0], ang[1], -ang[2])


class ArmTeleopLogic:
    """Reine Bedienlogik (ohne ROS): Joy-Zustand rein, Befehle + Sollbewegung raus.

    update(axes, buttons, now) -> Liste von Befehlen (Tupel), z.B.
      ("hand", side, "open"|"close"), ("enable",), ("home",), ("cancel",),
      ("estop",), ("hold", sides), ("changed",), ("lock", bool), ("locked_tap",),
      ("full_arm",), ("full_abort",), ("full_estop",)   -- Voll-Notaus
      ("mirror_arm", side), ("mirror_abort",), ("mirror", side)  -- Spiegeln
      ("seq_next",)                                          -- Ablauf weiter (L3)
    twist(axes) -> {side: (lin, ang)} fuer die gerade aktiven Seiten (oder {}).
    """

    def __init__(self, deadzone=0.12, expo=0.4, speed="Normal", home_hold_s=1.0,
                 selection="right", max_speed="Schnell", ps_estop=True,
                 full_estop_hold_s=2.0, full_estop_gap_s=0.3,
                 walk_deadman=False, trigger_modes=False, axis_snap_deg=20.0,
                 mirror_hold_s=2.0, mirror_gap_s=0.3, waist_yaw=False):
        self.deadzone = float(deadzone)
        self.expo = float(expo)
        # Linker Stick: Achsen-Einrasten (snap_stick), damit "vor" nur vor
        # faehrt bzw. "kippen" nur kippt. 0 = aus (alter Stand).
        self.axis_snap_deg = float(axis_snap_deg)
        self.home_hold_s = float(home_hold_s)
        self.selection = selection
        self.mode = "move"
        # Hoechstes per R1 erreichbares Tempo (Real: "Normal", siehe bringup_real).
        self.max_idx = (JOG_SPEED_ORDER.index(max_speed) if max_speed in JOG_SPEED_ORDER
                        else len(JOG_SPEED_ORDER) - 1)
        idx = JOG_SPEED_ORDER.index(speed) if speed in JOG_SPEED_ORDER else 1
        self.speed_idx = min(idx, self.max_idx)
        # Gewuenschtes Tempo (gemeinsamer Zustand mit der GUI): kann ueber
        # max_speed liegen, gefahren wird dann max_speed.
        self.speed_req = self.speed
        # PS: True = NOT-HALT (wie die GUIs), False = nur Stopp wie Kreuz.
        self.ps_estop = bool(ps_estop)
        self.estop = False
        # Voll-Notaus (R1 + L1 + PS halten): <= 0 schaltet ihn ab.
        self.full_hold_s = float(full_estop_hold_s)
        # Laengere Luecke im Joy-Strom waehrend des Haltens -> Abbruch (sonst
        # loeste ein nach einem Funkabriss weiter "gedrueckt" gemeldeter
        # Zustand sofort aus, ohne dass jemand die 2 s gehalten hat).
        self.full_gap_s = float(full_estop_gap_s)
        self.full_estop = False
        self._full_since = None      # Beginn des Haltens (None = nicht gehalten)
        self._full_last = None       # letzte Joy-Nachricht waehrend des Haltens
        self._full_sent = False      # in diesem Halten schon ausgeloest
        # Tempo vor dem ersten L1/R1-Schritt, solange L1 oder R1 gehalten:
        # wird daraus der Voll-Notaus (R1 + L1 + PS), zuruecknehmen.
        self._speed_undo = None
        # Arm spiegeln (Steuerkreuz links/rechts + Kreuz halten): <= 0 = aus.
        # Gleiche Halte-Logik wie der Voll-Notaus (Luecke im Strom -> neu).
        self.mirror_hold_s = float(mirror_hold_s)
        self.mirror_gap_s = float(mirror_gap_s)
        self._mir_side = None        # Seite, die gerade gehalten wird
        self._mir_since = None
        self._mir_last = None
        self._mir_sent = False
        # L2 = Deadman fuers Gehen (ps4_walk): solange gehalten, pausiert die
        # Arm-Steuerung (Sticks/Steuerkreuz). False = L2 frei (alter Stand).
        self.walk_deadman = bool(walk_deadman)
        self.walking = False
        # L2 druecken = Modus GEHEN, R2 druecken = GREIFEN (Flanke, mit Hysterese:
        # die Trigger sind analog).
        self.trigger_modes = bool(trigger_modes)
        self._trig_held = {L.L2: False, L.R2: False}
        # Modus des Roboters, soweit bekannt: None (unbekannt, Arme frei wie
        # bisher), "walk" (GEHEN: Arme am Controller gesperrt) oder "grasp".
        # Der Node gleicht ihn mit start_walking/start_balancing (GUIs) ab.
        self.robot_mode = None
        # R2 gehalten (im GREIFEN) = Huefte drehen: rechter Stick X -> Taillen-
        # Yaw, R3 = Mitte. Braucht trigger_modes (R2 ist sonst ohne Bedeutung).
        self.waist_yaw = bool(waist_yaw)
        self._waist_was = False
        self._waist_stop = False     # Kreuz: bis der Stick zurueck ist, kein Drehen
        self._prev = [0] * L.NUM_BUTTONS
        # L3 (Ablauf weiter) = linken Stick DRUECKEN: dabei kippt er leicht ->
        # bis L3 los und der Stick in der Mitte ist, kein Jog (ein Hand-Ziel
        # braeche die gerade gestartete geplante Fahrt ab).
        self._stick_block = False
        # Ablauf/Pose der Demo-GUI faehrt (/g1pilot/sequence/state sent|moving|
        # pause): Sticks bewegen die Haende NICHT -- ein Hand-Ziel braeche die
        # geplante Fahrt ab ("manueller Vorrang"). Stopp bleibt Kreuz.
        self.seq_busy = False
        self._share_since = None
        self._home_sent = False
        # Options = Controller gesperrt (zum Ablegen): nur PS/Voll-Notaus und
        # Options selbst wirken. Nur im Speicher -- Node-Neustart = entsperrt.
        self.locked = False
        # Nach dem Entsperren mit noch gehaltenem R2: Huefte erst nach einem
        # "R2 los" (wie ps4_walk beim L2).
        self._r2_block = False
        # /g1pilot/arms/state des arm_controllers (None = unbekannt): R2 im
        # GREIFEN uebernimmt die Arme nur, wenn sie abgegeben sind (ENABLE
        # bricht geplante Fahrten ab, also nicht bei jedem Hueft-Druck).
        self.arms_enabled = None

    # ── Zustand ─────────────────────────────────────────────────────────
    @property
    def speed(self):
        return JOG_SPEED_ORDER[self.speed_idx]

    @property
    def sides(self):
        return SELECTIONS[self.selection]

    @property
    def arms_locked(self):
        """GEHEN-Modus: Arm-Funktionen am Controller gesperrt."""
        return self.trigger_modes and self.robot_mode == "walk"

    @property
    def waist_mode(self):
        """R2 gehalten im GREIFEN: der rechte Stick dreht die Huefte."""
        return (self.waist_yaw and self.trigger_modes and self.robot_mode == "grasp"
                and self._trig_held[L.R2] and not self.estop and not self.walking
                and not self.locked and not self._r2_block)

    def waist_rate(self, axes):
        """-> Hueft-Drehrate -1..1 (+ = links, Stick links), solange R2
        gehalten wird, sonst None."""
        if not self.waist_mode:
            return None
        a = list(axes) + [0.0] * max(0, L.NUM_AXES - len(axes))
        r = shape_axis(a[L.RX], self.deadzone, self.expo)
        if self._waist_stop:
            if r != 0.0:
                return 0.0
            self._waist_stop = False
        return r

    def set_selection(self, sel):
        """Hand-Auswahl von aussen (GUI). -> Befehle wie update() (Halten der
        abgewaehlten Seiten, "changed") oder []."""
        if sel not in SELECTIONS or sel == self.selection:
            return []
        dropped = tuple(s for s in self.sides if s not in SELECTIONS[sel])
        self.selection = sel
        return ([("hold", dropped)] if dropped else []) + [("changed",)]

    def set_speed(self, name):
        """Tempo von aussen (GUI), auf max_speed begrenzt. -> geaendert?"""
        if name not in JOG_SPEED_ORDER:
            return False
        before = (self.speed_idx, self.speed_req)
        self.speed_req = name
        self.speed_idx = min(JOG_SPEED_ORDER.index(name), self.max_idx)
        return before != (self.speed_idx, self.speed_req)

    def set_robot_mode(self, mode):
        """Modus von aussen (GUI-Kachel) oder vom Node zuruecknehmen. -> geaendert?"""
        if mode == self.robot_mode:
            return False
        self.robot_mode = mode
        return True

    def color(self):
        if self.estop or self.full_estop:
            return COLORS["estop"]
        if self.locked:
            return COLORS["locked"]
        if self.arms_locked:
            return COLORS["walk"]
        if self.waist_mode:
            return COLORS["waist"]
        c = COLORS[self.selection]
        if self.mode == "turn":
            c = tuple(int(round(v * TURN_DIM)) for v in c)
        return c

    def label(self):
        if self.full_estop:
            return "PS4 · VOLL-NOTAUS (G1 in Damp, mit START quittieren)"
        if self.estop:
            return "PS4 · NOT-HALT (in GUI quittieren)"
        if self.locked:
            return "PS4 · GESPERRT (Options = entsperren, PS-Notaus aktiv)"
        if self.arms_locked:
            return "PS4 · GEHEN (L2 halten = laufen, R2 = Greifen)"
        if self.waist_mode:
            return "PS4 · HÜFTE (rechter Stick ←/→, R3 = Mitte)"
        if self.seq_busy:
            return "PS4 · ABLAUF fährt (Sticks gesperrt, Kreuz = Stopp)"
        return f"PS4 · {SELECTION_LABEL[self.selection]} · {MODE_LABEL[self.mode]} · {self.speed}"

    def status(self):
        return {"selection": self.selection, "sides": list(self.sides),
                "mode": self.mode, "speed": self.speed, "speed_req": self.speed_req,
                "max_speed": JOG_SPEED_ORDER[self.max_idx], "estop": self.estop,
                "ps_estop": self.ps_estop, "full_estop": self.full_estop,
                "full_arming": self.full_arming,
                "mirror_arming": self._mir_side if self.mirror_arming else None,
                "robot_mode": self.robot_mode,
                "arms_locked": self.arms_locked, "waist_mode": self.waist_mode,
                "locked": self.locked,
                "seq_busy": self.seq_busy, "label": self.label()}

    def clear_estop(self):
        """NOT-HALT quittiert (START aus GUI/Streamdeck gesehen)."""
        if self.estop or self.full_estop:
            self.estop = False
            self.full_estop = False
            return True
        return False

    def latch_estop(self):
        """NOT-HALT von aussen (GUI/Streamdeck): Controller sperren wie bei PS."""
        if not self.estop:
            self.estop = True
            return True
        return False

    @property
    def full_arming(self):
        """True, solange R1 + L1 + PS gehalten werden (noch nicht ausgeloest)."""
        return self._full_since is not None and not self._full_sent

    @staticmethod
    def full_combo(axes, buttons):
        """R1 + L1 + PS gleichzeitig gedrueckt? (Xbox: RB + LB + Xbox-Taste)"""
        b = list(buttons) + [0] * max(0, L.NUM_BUTTONS - len(buttons))
        return bool(b[L.R1]) and bool(b[L.L1]) and bool(b[L.PS])

    def abort_full(self):
        """Halten abbrechen (Loslassen, Joy-Timeout). True, wenn es lief."""
        was = self.full_arming
        self._full_since = self._full_last = None
        self._full_sent = False
        return was

    def _update_full(self, axes, buttons, now):
        """Voll-Notaus: R1 + L1 + PS ununterbrochen full_hold_s lang halten."""
        if self.full_hold_s <= 0.0:
            return []
        if not self.full_combo(axes, buttons):
            return [("full_abort",)] if self.abort_full() else []
        if self._full_last is not None and now - self._full_last > self.full_gap_s:
            # Luecke im Strom: neu anfangen (Vibration laeuft weiter/neu).
            self._full_since = None
            self._full_sent = False
        self._full_last = now
        if self._full_since is None:
            self._full_since = now
            return [("full_arm",)]
        if not self._full_sent and now - self._full_since >= self.full_hold_s:
            self._full_sent = True
            self.full_estop = True
            self.estop = True
            return [("hold", ("left", "right")), ("full_estop",), ("changed",)]
        return []

    # ── Arm spiegeln ────────────────────────────────────────────────────
    @property
    def mirror_arming(self):
        """True, solange Steuerkreuz links/rechts + Kreuz gehalten werden
        (noch nicht ausgeloest)."""
        return self._mir_since is not None and not self._mir_sent

    @staticmethod
    def mirror_combo(buttons):
        """-> "left"/"right" (Seite, die spiegeln soll), wenn Kreuz und GENAU
        eine der Steuerkreuz-Tasten links/rechts gedrueckt sind, sonst None.
        Xbox: A + Steuerkreuz (A liegt auf Kreuz, siehe ps4_layout)."""
        b = list(buttons) + [0] * max(0, L.NUM_BUTTONS - len(buttons))
        if not b[L.CROSS] or bool(b[L.DPAD_LEFT]) == bool(b[L.DPAD_RIGHT]):
            return None
        return "left" if b[L.DPAD_LEFT] else "right"

    def abort_mirror(self):
        """Halten abbrechen (Loslassen, Sperre, Joy-Timeout). True, wenn es lief."""
        was = self.mirror_arming
        self._mir_side = self._mir_since = self._mir_last = None
        self._mir_sent = False
        return was

    def _abort_mirror_cmds(self):
        return [("mirror_abort",)] if self.abort_mirror() else []

    def _update_mirror(self, b, now):
        """Steuerkreuz links/rechts + Kreuz ununterbrochen mirror_hold_s lang
        halten -> ("mirror", side). Seitenwechsel waehrend des Haltens zaehlt
        von vorn (die neue Seite vibriert neu)."""
        if self.mirror_hold_s <= 0.0:
            return []
        side = self.mirror_combo(b)
        if side is None:
            return self._abort_mirror_cmds()
        out = []
        if side != self._mir_side or (self._mir_last is not None
                                      and now - self._mir_last > self.mirror_gap_s):
            if self._mir_side is not None and side != self._mir_side:
                out += self._abort_mirror_cmds()
            self._mir_since = None
            self._mir_sent = False
        self._mir_side = side
        self._mir_last = now
        if self._mir_since is None:
            self._mir_since = now
            return out + [("mirror_arm", side)]
        if not self._mir_sent and now - self._mir_since >= self.mirror_hold_s:
            self._mir_sent = True
            return out + [("hold", ("left", "right")), ("mirror", side)]
        return out

    # ── Sperre (Options) ────────────────────────────────────────────────
    def set_locked(self, locked):
        """Controller sperren/entsperren. -> Befehle wie update() oder [].
        Sperren haelt laufende Hand-Jogs an (nur die -- eine geplante Fahrt
        der GUI laeuft weiter), beendet Huefte und Spiegeln-Halten."""
        locked = bool(locked)
        if locked == self.locked:
            return []
        self.locked = locked
        out = []
        if locked:
            out += self._abort_mirror_cmds()
            out.append(("hold", ("left", "right")))
            if self._waist_was:
                self._waist_was = False
                out.append(("waist", "end"))
            self.walking = False
            self._share_since = None
            self._home_sent = False
        else:
            # Ein beim Ablegen ausgelenkter Stick / gedrueckter R2 wirkt
            # nicht sofort.
            self._stick_block = True
            self._r2_block = self._trig_held[L.R2]
        out.append(("lock", locked))
        return out

    # ── Eingabe ─────────────────────────────────────────────────────────
    def update(self, axes, buttons, now):
        b = list(buttons) + [0] * max(0, L.NUM_BUTTONS - len(buttons))
        rise = [1 if (b[i] and not self._prev[i]) else 0 for i in range(L.NUM_BUTTONS)]
        self._prev = b[:L.NUM_BUTTONS]
        # Voll-Notaus zuerst: wirkt auch im NOT-HALT (PS hat den Oberkoerper
        # beim Druecken schon gedaempft) und unabhaengig von ps_estop.
        if not b[L.L1] and not b[L.R1]:
            self._speed_undo = None
        trig_rise = self._trigger_edges(axes, b)
        if not self._trig_held[L.R2]:
            self._r2_block = False
        out = self._update_full(axes, b, now)
        if ("full_arm",) in out and self._speed_undo is not None:
            # L1/R1 vor PS gedrueckt: deren Tempo-Schritt gehoerte zum Notaus.
            self.speed_idx = self._speed_undo
            self.speed_req = self.speed
            self._speed_undo = None
            out.append(("changed",))
        if self.full_arming or self._full_sent:
            # Waehrend R1 + L1 + PS gehalten werden: nur PS-NOT-HALT (unten),
            # keine anderen Tasten (L1/R1 waeren sonst "langsamer/schneller").
            out[:0] = self._abort_mirror_cmds()
            if rise[L.PS] and self.ps_estop:
                out[:0] = [("hold", ("left", "right")), ("estop",)]
                if not self.estop:
                    self.estop = True
                    out.append(("changed",))
            elif rise[L.PS] and not self.estop:
                out[:0] = [("hold", ("left", "right")), ("cancel",)]
            return out

        # NOT-HALT zuerst und unabhaengig von allem anderen.
        if rise[L.PS] and self.ps_estop:
            out += self._abort_mirror_cmds()
            out.append(("hold", ("left", "right")))
            out.append(("estop",))
            if not self.estop:
                self.estop = True
                out.append(("changed",))
            return out
        # Options = sperren/entsperren. PS und Voll-Notaus (oben) wirken auch
        # gesperrt; die L2/R2-Flanken laufen oben weiter mit, ein beim
        # Entsperren noch gedrueckter Trigger schaltet also keinen Modus.
        # Die Options-Nachricht selbst bewirkt sonst nichts (ein im selben
        # Moment gedrueckter Trigger zaehlt dann als schon gehalten).
        if rise[L.OPTIONS]:
            return out + self.set_locked(not self.locked)
        if self.locked:
            # Jede andere Taste (auch L2/R2) vibriert leicht: "gesperrt".
            if any(rise[i] for i in range(L.NUM_BUTTONS)
                   if i not in (L.OPTIONS, L.PS)) or any(trig_rise.values()):
                out.append(("locked_tap",))
            return out
        if self.estop:
            # bis zur Quittung in der GUI: nichts bewegen
            return out + self._abort_mirror_cmds()

        if self.trigger_modes:
            if trig_rise[L.L2] and self.robot_mode != "walk":
                self.robot_mode = "walk"
                out += [("hold", ("left", "right")), ("mode", "walk"), ("changed",)]
            elif trig_rise[L.R2] and self.robot_mode != "grasp":
                self.robot_mode = "grasp"
                out += [("mode", "grasp"), ("changed",)]
            elif trig_rise[L.R2] and self.arms_enabled is not True:
                # Schon GREIFEN, Arme aber abgegeben (z. B. nach NOT-HALT):
                # R2 uebernimmt sie wieder (frueher Options).
                out.append(("enable",))
        elif trig_rise[L.R2] and self.arms_enabled is not True:
            out.append(("enable",))
        self.walking = self.walk_deadman and deadman_held(axes, b)
        if self.arms_locked or self.walking:
            # GEHEN / L2 gehalten: Steuerkreuz gehoert dem Gehen, kein Spiegeln.
            out += self._abort_mirror_cmds()
        waist = self.waist_mode
        if waist != self._waist_was:
            self._waist_was = waist
            self._waist_stop = False
            if waist:
                # Hueft-Modus: laufende Hand-Jogs anhalten, Arme halten.
                out += [("hold", ("left", "right")), ("waist", "start")]
            else:
                out.append(("waist", "end"))
        if waist:
            out += self._abort_mirror_cmds()     # Steuerkreuz gehoert nicht den Armen
            # R2 gehalten: Sticks/Steuerkreuz/Dreieck/Tempo gehoeren nicht den
            # Armen (Share/Grundstellung auch nicht). R3 = Mitte, Kreuz =
            # Stopp (auch die Huefte); Haende wirken weiter.
            if rise[L.R3]:
                out.append(("waist_center",))
            if rise[L.CROSS]:
                self._waist_stop = True
                out += [("hold", ("left", "right")), ("cancel",), ("waist_stop",)]
            if rise[L.CIRCLE]:
                out.extend(("hand", s, "open") for s in self.sides)
            if rise[L.SQUARE]:
                out.extend(("hand", s, "close") for s in self.sides)
            self._share_since = None
            self._home_sent = False
            return out
        if self.arms_locked:
            # GEHEN: nur Kreuz (Stopp); zurueck zu GREIFEN mit R2 (oben).
            self._share_since = None
            if rise[L.CROSS] or (rise[L.PS] and not self.ps_estop):
                out += [("hold", ("left", "right")), ("cancel",)]
            return out
        # Auswahl der Hand / Haende (Steuerkreuz; mit L2 = Geh-Tempo)
        for btn, sel in (() if self.walking else DPAD_SELECTION):
            if rise[btn] and sel != self.selection:
                dropped = tuple(s for s in self.sides if s not in SELECTIONS[sel])
                if dropped:
                    out.append(("hold", dropped))
                self.selection = sel
                out.append(("changed",))

        if rise[L.TRIANGLE]:
            self.mode = "turn" if self.mode == "move" else "move"
            out.append(("jog_mode", self.mode))
        if self.walking:
            pass   # L1/R1 = Geh-Tempo (ps4_walk), solange L2 gehalten
        elif rise[L.L1] and self.speed_idx > 0:
            if self._speed_undo is None:
                self._speed_undo = self.speed_idx
            self.speed_idx -= 1
            self.speed_req = self.speed
            out.append(("changed",))
        if not self.walking and rise[L.R1] and self.speed_idx < self.max_idx:
            if self._speed_undo is None:
                self._speed_undo = self.speed_idx
            self.speed_idx += 1
            self.speed_req = self.speed
            out.append(("changed",))

        if rise[L.CIRCLE]:
            out.extend(("hand", s, "open") for s in self.sides)
        if rise[L.SQUARE]:
            out.extend(("hand", s, "close") for s in self.sides)

        if rise[L.CROSS] or (rise[L.PS] and not self.ps_estop):
            out.append(("hold", ("left", "right")))
            out.append(("cancel",))
        if rise[L.L3] and not self.walking:
            self._stick_block = True
            out.append(("seq_next",))

        # Grundstellung nur nach bewusstem Halten (versehentliches Antippen)
        if b[L.SHARE]:
            if self._share_since is None:
                self._share_since = now
            elif not self._home_sent and now - self._share_since >= self.home_hold_s:
                self._home_sent = True
                out.append(("home",))
        else:
            self._share_since = None
            self._home_sent = False
        # Spiegeln zuletzt: Auswahl (Steuerkreuz) und Stopp (Kreuz) desselben
        # Drueckens sind dann schon raus, die Halte-Vibration kommt danach.
        if not self.walking:
            out += self._update_mirror(b, now)
        return out

    TRIGGER_ON, TRIGGER_OFF = 0.5, 0.3

    def _trigger_edges(self, axes, b):
        """L2/R2 gedrueckt (Taste oder Achse >= 0.5, los erst < 0.3) -> Flanken."""
        a = list(axes) + [0.0] * max(0, L.NUM_AXES - len(axes))
        rise = {}
        for btn, ax in ((L.L2, L.L2_AXIS), (L.R2, L.R2_AXIS)):
            was = self._trig_held[btn]
            thr = self.TRIGGER_OFF if was else self.TRIGGER_ON
            now = bool(b[btn]) or a[ax] >= thr
            self._trig_held[btn] = now
            rise[btn] = now and not was
        return rise

    def twist(self, axes):
        """-> {side: ((vx, vy, vz) m/s, (wx, wy, wz) rad/s)} im JOG_FRAME."""
        if (self.estop or self.locked or self.arms_locked or self.waist_mode
                or self.seq_busy) or (
                self.walk_deadman and (self.walking or deadman_held(axes, ()))):
            return {}
        a = list(axes) + [0.0] * max(0, L.NUM_AXES - len(axes))
        if self._stick_block:
            if self._prev[L.L3] or max(abs(a[L.LX]), abs(a[L.LY])) > self.deadzone:
                return {}
            self._stick_block = False
        a[L.LX], a[L.LY] = snap_stick(a[L.LX], a[L.LY], self.axis_snap_deg)
        s = lambda i: shape_axis(a[i], self.deadzone, self.expo)   # noqa: E731
        v, w = JOG_SPEEDS[self.speed]
        if self.mode == "move":
            # Stick oben = vor (+x), links = links (+y); rechter Stick oben = hoch (+z)
            lin, ang = (s(L.LY) * v, s(L.LX) * v, s(L.RY) * v), (0.0, 0.0, 0.0)
        else:
            # Stick oben = Finger hoch: NEGATIVE Drehung um y (wie Demo-GUI);
            # links = schwenken nach links (+z); rechter Stick rechts = rollen
            # im Uhrzeigersinn von hinten gesehen (+x).
            lin = (0.0, 0.0, 0.0)
            ang = (-s(L.RX) * w, -s(L.LY) * w, s(L.LX) * w)
        if max(abs(c) for c in lin + ang) < 1e-6:
            return {}
        out = {}
        for side in self.sides:
            if self.selection == "mirror" and side == "left":
                out[side] = mirror_twist(lin, ang)   # Referenz = rechte Hand
            else:
                out[side] = (lin, ang)
        return out


# ════════════════════════════════════════════════════════════════════════
#  ROS-Node
# ════════════════════════════════════════════════════════════════════════
def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Bool, Float32, String
    from geometry_msgs.msg import PoseStamped
    from visualization_msgs.msg import Marker, MarkerArray
    from tf2_ros import Buffer, TransformListener

    class Ps4ArmTeleop(Node):
        def __init__(self):
            super().__init__("ps4_arm_teleop")
            self.declare_parameter("joy_topic", "/g1pilot/ps4/joy")
            self.declare_parameter("rate_hz", 30.0)
            self.declare_parameter("deadzone", 0.12)
            self.declare_parameter("expo", 0.4)
            self.declare_parameter("speed", "Normal")
            # Hoechstes Tempo per R1 (Real: "Normal" -- 15 cm/s bleibt unter
            # dem EE-Limit des arm_controllers von 0.25 m/s).
            self.declare_parameter("max_speed", "Schnell")
            # PS = NOT-HALT wie in den GUIs (Real: Damp, Roboter sackt
            # zusammen). false -> PS wirkt nur wie Kreuz (Stopp).
            self.declare_parameter("ps_estop", True)
            self.declare_parameter("home_hold_s", 1.0)
            # Voll-Notaus: R1 + L1 + PS so lange halten (s), Controller vibriert
            # waehrenddessen. 0 = Voll-Notaus am Controller abgeschaltet.
            self.declare_parameter("full_estop_hold_s", 2.0)
            # Arm spiegeln: Steuerkreuz links/rechts + Kreuz so lange halten (s),
            # Controller vibriert waehrenddessen. 0 = Spiegeln abgeschaltet.
            self.declare_parameter("mirror_hold_s", 2.0)
            # L2 = Deadman fuers Gehen (Node ps4_walk laeuft mit): solange
            # gehalten, pausiert die Arm-Steuerung. false = L2 ohne Wirkung.
            self.declare_parameter("walk_deadman", False)
            # L2 druecken = Modus GEHEN (start_walking), R2 = GREIFEN (wie die
            # Kacheln der Demo-GUI); im GEHEN sind die Arm-Tasten gesperrt.
            self.declare_parameter("trigger_modes", False)
            # Beine nie per Controller aufstellen: start_walking/start_balancing
            # nur, wenn /g1pilot/onboard_state den Walk-Modus meldet (Real).
            self.declare_parameter("modes_need_onboard_walk", not is_sim_mode())
            # Bleibt der Joy-Stream laenger aus (Funkabriss, Akku leer), haelt
            # die Hand an -- der ps4_joystick publiziert nur bei Verbindung.
            self.declare_parameter("joy_timeout_s", 0.5)
            # R2 = Arme uebernehmen (frueher Options): zusaetzlich start_balancing senden (wie die
            # Kachel GREIFEN der Demo-GUI). false -> nur Arme uebernehmen.
            # Real: false -- der Controller aendert den Zustand der Beine nie
            # (Aufstehen/Balancing bleibt eine Aktion in GUI/Streamdeck).
            self.declare_parameter("options_starts_balancing", True)
            # Bezug der Jog-Ziele (hand_jog.JOG_REFERENCES): "command" = Start,
            # Vorlauf und Anhalten an der kommandierten Hand (/g1pilot/hand_cmd_pose,
            # wie der RViz-Marker; Fallback TF), "start" = nur der Start dort,
            # "tf" = alles an der gemessenen Hand (alter Real-Stand).
            # Default sim + real "command" (Env G1_JOG_REFERENCE).
            self.declare_parameter("jog_reference", jog_reference_mode())
            # Linker Stick rastet auf die Achse ein, wenn er hoechstens so viele
            # Grad daneben liegt (nur eine Richtung statt leicht schraeg). 0 = aus.
            self.declare_parameter("axis_snap_deg", 20.0)
            # R2 halten (GREIFEN) = Huefte drehen (rechter Stick X), R2 + R3 =
            # Mitte; nur mit trigger_modes. false = R2 halten ohne Wirkung.
            self.declare_parameter("waist_yaw", True)
            # Jog-Richtungen im Oberkoerper-Bezug (um den gemessenen Taillen-Yaw
            # gedreht): "vor" = Richtung Brust. false = Becken wie bisher.
            self.declare_parameter("jog_torso_frame", True)
            gp = lambda n: self.get_parameter(n).value   # noqa: E731

            self.logic = ArmTeleopLogic(gp("deadzone"), gp("expo"), gp("speed"),
                                        gp("home_hold_s"), max_speed=gp("max_speed"),
                                        ps_estop=gp("ps_estop"),
                                        full_estop_hold_s=gp("full_estop_hold_s"),
                                        walk_deadman=gp("walk_deadman"),
                                        trigger_modes=gp("trigger_modes"),
                                        axis_snap_deg=gp("axis_snap_deg"),
                                        mirror_hold_s=gp("mirror_hold_s"),
                                        waist_yaw=gp("waist_yaw"))
            self.need_onboard_walk = bool(gp("modes_need_onboard_walk"))
            self.onboard = self.onboard_t = None
            self.joy_timeout = float(gp("joy_timeout_s"))
            self.options_balancing = bool(gp("options_starts_balancing"))
            self.cmd_poses = CommandedPoses()
            self.axes, self.t_joy = [], None
            self.connected = False
            self.targets = {}            # side -> [pos, quat] laufendes Jog-Ziel
            self.last_twist = {}         # side -> (lin, ang) letzte Sollbewegung
            self.last_hand = {}          # side -> (pos, quat) letzter Bezug (Hand)
            self.jog_ref = JogReference(gp("jog_reference"))
            self.holds = HoldStreamer()  # Halte-Ziele nach dem Loslassen
            self.t_tick = time.monotonic()
            self._tf_warned = False
            self._home_off_at = None
            self.jog_torso = bool(gp("jog_torso_frame"))
            self.waist = None            # letzter /g1pilot/waist/state
            self.waist_t = None
            self._waist_sending = False  # Rate-Strom laeuft (0 beim Ende senden)
            self._mirror_n = 0
            self._mirror_id = None       # ID des laufenden Spiegel-Kommandos

            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            self.pub_goal = {s: self.create_publisher(PoseStamped, f"/g1pilot/hand_goal/{s}", 10)
                             for s in ("left", "right")}
            self.pub_hand = {s: self.create_publisher(String, f"/g1pilot/hand_action/{s}", 10)
                             for s in ("left", "right")}
            self.pub_enable = self.create_publisher(Bool, "/g1pilot/arms/enabled", 10)
            self.pub_balance = self.create_publisher(Bool, "/g1pilot/start_balancing", 10)
            self.pub_walk = self.create_publisher(Bool, "/g1pilot/start_walking", 10)
            self.pub_home = self.create_publisher(Bool, "/g1pilot/arms/home", 10)
            self.pub_cancel = self.create_publisher(Bool, "/g1pilot/pose_store/cancel", 10)
            self.pub_estop = self.create_publisher(Bool, "/g1pilot/emergency_stop", 10)
            self.pub_full_estop = self.create_publisher(Bool, "/g1pilot/full_emergency_stop", 10)
            self.pub_waist_rate = self.create_publisher(Float32, WAIST_RATE_TOPIC, 10)
            self.pub_waist_center = self.create_publisher(Bool, WAIST_CENTER_TOPIC, 10)
            self.create_subscription(String, WAIST_STATE_TOPIC, self._on_waist_state, 10)
            self.pub_arm_cmd = self.create_publisher(String, "/g1pilot/arm_command", 10)
            self.pub_feedback = self.create_publisher(String, "/g1pilot/ps4/feedback", 10)
            latched = QoSProfile(depth=1)
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.pub_status = self.create_publisher(String, "/g1pilot/ps4/status", latched)
            self.pub_markers = self.create_publisher(MarkerArray, "/g1pilot/ps4/markers", 10)
            # Gemeinsamer Zustand mit der Demo-GUI (shared_state.py). Start-
            # Parameter (Real: Langsam) gelten ab Node-Start fuer beide Seiten.
            self.t_start_wall = time.time()
            self.shared = shared_state.SharedState("ps4_arm_teleop")
            self.shared.set("arm_side", self.logic.selection)
            self.shared.set("arm_speed", self.logic.speed)
            self.pub_shared = self.create_publisher(String, shared_state.TOPIC, latched)
            self.create_subscription(String, shared_state.TOPIC, self._on_shared, latched)
            self.pub_shared.publish(String(data=self.shared.to_json()))

            self.create_subscription(Joy, gp("joy_topic"), self._on_joy, 10)
            for s in ("left", "right"):
                self.create_subscription(
                    PoseStamped, JOG_CMD_TOPIC.format(side=s),
                    lambda m, s=s: self._on_cmd_pose(s, m), 10)
            # START (GUI/Streamdeck) quittiert den NOT-HALT im arm_controller.
            self.create_subscription(Bool, "/g1pilot/start", self._on_start, 10)
            # NOT-HALT aus GUI/Streamdeck sperrt auch den Controller (LED rot).
            self.create_subscription(Bool, "/g1pilot/emergency_stop", self._on_estop, 10)
            # Modus-Wechsel aus GUIs/Streamdeck (auch die eigenen) mitverfolgen.
            self.create_subscription(Bool, "/g1pilot/start_walking",
                                     lambda m: self._on_mode_topic("walk", m), 10)
            self.create_subscription(Bool, "/g1pilot/start_balancing",
                                     lambda m: self._on_mode_topic("grasp", m), 10)
            self.create_subscription(String, "/g1pilot/onboard_state", self._on_onboard, latched)
            self.create_subscription(String, "/g1pilot/arm_command/status",
                                     self._on_cmd_status, 10)
            # Arme uebernommen? (arm_controller, latched) -> R2 im GREIFEN.
            self.create_subscription(Bool, "/g1pilot/arms/state", self._on_arms_state, latched)
            # Ablauf (Demo-GUI): L3 = Weiter; vibrieren, sobald er wartet.
            self.pub_seq_next = self.create_publisher(Bool, sr.NEXT_TOPIC, 10)
            self.seq_waiting = False
            self.create_subscription(String, sr.STATE_TOPIC, self._on_seq_state, latched)

            self.create_timer(1.0 / float(gp("rate_hz")), self._tick)
            self.create_timer(0.2, self._publish_markers)
            self.create_timer(1.0, self._publish_status)
            self._send_feedback(rumble_ms=0)
            self.get_logger().info(
                f"PS4-Arm-Steuerung bereit ({gp('joy_topic')}): {self.logic.label()} "
                f"(max. {JOG_SPEED_ORDER[self.logic.max_idx]}, PS="
                f"{'NOT-HALT' if self.logic.ps_estop else 'Stopp'}, R1+L1+PS "
                f"{f'{self.logic.full_hold_s:.1f} s = VOLL-NOTAUS' if self.logic.full_hold_s > 0 else 'aus'}"
                f", Steuerkreuz+Kreuz "
                f"{f'{self.logic.mirror_hold_s:.1f} s = Spiegeln' if self.logic.mirror_hold_s > 0 else 'aus'}"
                f", Options = sperren, R2 Arme"
                f"{' + start_balancing' if self.options_balancing else ' = nur Arme'})")

        # ── Eingaenge ──────────────────────────────────────────────────
        def _on_joy(self, msg):
            now = time.monotonic()
            if not self.connected:
                self.connected = True
                self.get_logger().info("PS4-Controller: Daten kommen an.")
                self._send_feedback()
            self.axes, self.t_joy = list(msg.axes), now
            lg = self.logic
            before = (lg.selection, lg.speed, lg.robot_mode)
            for cmd in lg.update(msg.axes, msg.buttons, now):
                self._run(cmd)
            # Wechsel am Controller -> gemeinsamer Zustand (GUI zieht nach).
            # Modus erst NACH _run: real ohne Walk nimmt "mode" ihn zurueck.
            changed = False
            if lg.selection != before[0]:
                changed |= self.shared.set("arm_side", lg.selection)
            if lg.speed != before[1]:
                changed |= self.shared.set("arm_speed", lg.speed)
            if lg.robot_mode != before[2] and lg.robot_mode in ("walk", "grasp"):
                changed |= self.shared.set("mode", lg.robot_mode)
            if changed:
                self.pub_shared.publish(String(data=self.shared.to_json()))

        def _on_shared(self, msg):
            """Wechsel aus der GUI uebernehmen (ohne Vibration: die LED zeigt
            es). Eigene Nachrichten aendern nichts (gleicher Zeitstempel)."""
            ch = self.shared.merge(msg.data)
            lg, quiet_change = self.logic, False
            if "arm_side" in ch:
                for cmd in lg.set_selection(ch["arm_side"]):
                    if cmd[0] == "hold":
                        self._hold(cmd[1])
                    else:
                        quiet_change = True
            if "arm_speed" in ch and lg.set_speed(ch["arm_speed"]):
                quiet_change = True
            # Modus nur, wenn er NACH dem Node-Start gewechselt wurde (ein
            # alter latched Stand beschreibt den Roboter nicht mehr).
            if ("mode" in ch and lg.trigger_modes
                    and self.shared.stamps.get("mode", 0.0) > self.t_start_wall
                    and lg.set_robot_mode(ch["mode"])):
                if lg.arms_locked:
                    self._hold(("left", "right"))
                quiet_change = True
            if quiet_change:
                self.get_logger().info(f"Von der GUI: {lg.label()}")
                self._send_feedback()
                self._publish_status()
                self._publish_markers()

        def _on_cmd_pose(self, side, msg):
            if msg.header.frame_id and msg.header.frame_id != JOG_FRAME:
                return
            p, q = msg.pose.position, msg.pose.orientation
            self.cmd_poses.update(side, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                                  time.monotonic())

        def _on_mode_topic(self, mode, msg):
            if msg.data and self.logic.trigger_modes and self.logic.set_robot_mode(mode):
                self._changed()

        def _on_arms_state(self, msg):
            self.logic.arms_enabled = bool(msg.data)

        def _on_waist_state(self, msg):
            st = parse_state(msg.data)
            if st is not None:
                self.waist, self.waist_t = st, time.monotonic()

        def _torso_yaw(self):
            """Gemessener Taillen-Yaw [rad] fuer den Jog-Bezug (0 = Becken)."""
            if (not self.jog_torso or self.waist is None or self.waist_t is None
                    or time.monotonic() - self.waist_t > STATE_STALE_S):
                return 0.0
            try:
                return float(self.waist.get("yaw_rad") or 0.0)
            except (TypeError, ValueError):
                return 0.0

        def _send_waist_rate(self, rate):
            self.pub_waist_rate.publish(Float32(data=float(rate)))

        def _on_onboard(self, msg):
            try:
                self.onboard = json.loads(msg.data)
            except ValueError:
                return
            self.onboard_t = time.monotonic()

        def _onboard_walk(self):
            from g1pilot.teleoperation import status_line   # (importiert dieses Modul)
            if self.onboard is None or self.onboard_t is None:
                return False
            if time.monotonic() - self.onboard_t > status_line.STALE_S:
                return False
            return self.onboard.get("fsm") in status_line.WALK_FSM

        def _on_cmd_status(self, msg):
            """Antwort des arm_controllers auf das eigene Spiegel-Kommando:
            abgelehnt/fehlgeschlagen -> lange Vibration (sonst fuehlte sich
            ein verweigertes Spiegeln wie ein ausgefuehrtes an)."""
            try:
                st = json.loads(msg.data)
            except ValueError:
                return
            if not self._mirror_id or st.get("id") != self._mirror_id:
                return
            state, why = st.get("state"), st.get("reason", "")
            if state in ("rejected", "failed"):
                self._mirror_id = None
                self.get_logger().warn(f"Spiegeln nicht ausgefuehrt ({state}): {why}")
                self._send_feedback(rumble_ms=600, strong=1.0, weak=0.0)
            elif state == "reached":
                self._mirror_id = None
                self.get_logger().info("Spiegeln: Spiegelpose erreicht.")
                self._send_feedback(rumble_ms=120, strong=0.0, weak=0.5)
            elif state == "cancelled":
                self._mirror_id = None
                self.get_logger().info(f"Spiegeln abgebrochen: {why}")

        def _on_start(self, msg):
            if msg.data and self.logic.clear_estop():
                self.get_logger().info("NOT-HALT quittiert (START) -- PS4 wieder aktiv.")
                self._changed()

        def _on_seq_state(self, msg):
            """Stand des Ablaufs der Demo-GUI: beim Uebergang zu »wartet auf
            Weiter« zweimal kurz vibrieren (ohne Blick auf die GUI erkennbar)."""
            try:
                data = json.loads(msg.data)
            except ValueError:
                return
            state = data.get("state") if isinstance(data, dict) else None
            waiting = state == sr.WAITING
            if waiting and not self.seq_waiting and not self.logic.estop:
                self._send_feedback(rumble_ms=250, strong=0.3, weak=0.8)
            self.seq_waiting = waiting
            busy = state in (sr.SENT, sr.MOVING, sr.PAUSE)
            if busy != self.logic.seq_busy:
                self.logic.seq_busy = busy
                if busy:
                    # Fahrt laeuft schon: laufenden Jog still beenden (KEIN
                    # Halte-Ziel mehr -- die geplante Bahn hat Vorrang).
                    self._settle_jog(publish=False)
                self._changed(rumble_ms=0)

        def _on_estop(self, msg):
            if msg.data and self.logic.latch_estop():
                self._hold(("left", "right"))
                self.holds.clear()   # Arme sind ohnehin abgeschaltet
                self.get_logger().warn("NOT-HALT (von aussen) -- PS4 gesperrt bis START.")
                self._send_feedback(rumble_ms=600, strong=1.0, weak=1.0)
                self._publish_status()
                self._publish_markers()

        def _run(self, cmd):
            kind = cmd[0]
            if kind == "hold":
                self._hold(cmd[1])
            elif kind == "hand":
                self.pub_hand[cmd[1]].publish(String(data=cmd[2]))
                self.get_logger().info(f"Hand {cmd[1]}: {cmd[2]}")
                self._send_feedback(rumble_ms=60, strong=0.0, weak=0.4)
            elif kind == "enable":
                if self.options_balancing:
                    self.pub_balance.publish(Bool(data=True))
                self.pub_enable.publish(Bool(data=True))
                self.get_logger().info("R2: Arme uebernommen"
                                       + (" (Greifen-Modus)." if self.options_balancing else "."))
                self._send_feedback(rumble_ms=150)
            elif kind == "locked_tap":
                # Gesperrt: kurze, leichte Vibration als Rueckmeldung.
                self._send_feedback(rumble_ms=80, strong=0.0, weak=0.3)
            elif kind == "lock":
                if cmd[1]:
                    self.get_logger().warn(
                        "Options: Controller GESPERRT (nur PS-Notaus und R1 + L1 + PS "
                        "wirken; Options = entsperren).")
                    self._send_feedback(rumble_ms=400, strong=0.8, weak=0.0)
                else:
                    self.get_logger().info("Options: Controller entsperrt -- "
                                           + self.logic.label())
                    self._send_feedback(rumble_ms=100, strong=0.0, weak=0.6)
                self._publish_status()
                self._publish_markers()
            elif kind == "mode":
                legs_ok = not self.need_onboard_walk or self._onboard_walk()
                if cmd[1] == "walk":
                    if not legs_ok:
                        # Real, G1 nicht im Walk: nicht aufstellen, Modus zuruecknehmen.
                        self.logic.set_robot_mode(None)
                        self.get_logger().warn(
                            "L2: GEHEN gesperrt -- G1 steht nicht im Walk-Modus "
                            "(erst Startleiste Preparation -> Walk).")
                        self._send_feedback(rumble_ms=600, strong=1.0, weak=0.0)
                        return
                    self.pub_walk.publish(Bool(data=True))
                    self.get_logger().info("L2: Modus GEHEN (L2 halten = laufen, Arme gesperrt).")
                    self._send_feedback(rumble_ms=150)
                else:
                    if legs_ok:
                        self.pub_balance.publish(Bool(data=True))
                    self.pub_enable.publish(Bool(data=True))
                    self.get_logger().info("R2: Modus GREIFEN (Arme uebernommen"
                                           + (")." if legs_ok else ", Beine unveraendert)."))
                    self._send_feedback(rumble_ms=150)
            elif kind == "home":
                self._hold(("left", "right"))
                # KEIN Halte-Strom: jedes Ziel braeche die geplante Grundstellung
                # ab ("manueller Vorrang" im arm_controller).
                self.holds.clear()
                self.pub_home.publish(Bool(data=True))
                # Impuls wie _pulse der GUIs: nach 1 s False (in _tick)
                self._home_off_at = time.monotonic() + 1.0
                self.get_logger().info("Grundstellung angefordert (Share gehalten).")
                self._send_feedback(rumble_ms=300)
            elif kind == "seq_next":
                if not self.seq_waiting:
                    return      # kein Ablauf wartet -> L3 ohne Wirkung
                # Lief gerade ein Jog: EIN Halte-Ziel jetzt (vor dem goto), dann
                # kein Halte-Strom mehr -- jedes spaetere Ziel braeche die
                # naechste geplante Fahrt ab ("manueller Vorrang").
                self._settle_jog(publish=True)
                self.pub_seq_next.publish(Bool(data=True))
                self.get_logger().info("L3: Ablauf weiter (naechster Schritt).")
                self._send_feedback(rumble_ms=120, strong=0.0, weak=0.6)
            elif kind == "cancel":
                self.pub_cancel.publish(Bool(data=True))
                self.get_logger().info("STOPP: Hand haelt, geplante Bewegung abgebrochen.")
                self._send_feedback(rumble_ms=80, strong=0.6, weak=0.0)
            elif kind == "estop":
                self.holds.clear()
                self.pub_estop.publish(Bool(data=True))
                self.get_logger().warn("NOT-HALT (PS-Taste). Quittieren in GUI/Streamdeck.")
                self._send_feedback(rumble_ms=600, strong=1.0, weak=1.0)
            elif kind == "full_arm":
                # Vibration ueber die ganze Haltezeit: "jetzt wird es ernst".
                self._send_feedback(rumble_ms=self.logic.full_hold_s * 1000.0 + 300.0,
                                    strong=1.0, weak=1.0, force=True)
                self.get_logger().warn(
                    f"R1 + L1 + PS gehalten: VOLL-NOTAUS in {self.logic.full_hold_s:.1f} s "
                    "(loslassen = Abbruch).")
                self._publish_status()
            elif kind == "full_abort":
                self._send_feedback(rumble_ms=0, stop_rumble=True)
                self.get_logger().info("Voll-Notaus abgebrochen (vor Ablauf losgelassen).")
                self._publish_status()
            elif kind == "full_estop":
                self.holds.clear()
                # Zuerst der Voll-Notaus (loco_client: Damp), dann der normale
                # NOT-HALT (arm_controller/GUIs sperren wie bei PS).
                self.pub_full_estop.publish(Bool(data=True))
                self.pub_estop.publish(Bool(data=True))
                self.get_logger().error(
                    "VOLL-NOTAUS (R1 + L1 + PS): ganzer G1 in Damp. Quittieren mit "
                    "START, Aufstehen danach nur ueber die Startleiste.")
                self._send_feedback(rumble_ms=1500, strong=1.0, weak=1.0, force=True)
            elif kind == "mirror_arm":
                # Vibration ueber die ganze Haltezeit (wie beim Voll-Notaus).
                self._send_feedback(rumble_ms=self.logic.mirror_hold_s * 1000.0 + 300.0,
                                    strong=1.0, weak=1.0, force=True)
                src = "rechten" if cmd[1] == "left" else "linken"
                self.get_logger().info(
                    f"Spiegeln gehalten: {'linker' if cmd[1] == 'left' else 'rechter'} Arm "
                    f"faehrt in {self.logic.mirror_hold_s:.1f} s in die Spiegelpose des "
                    f"{src} Arms (loslassen = Abbruch).")
                self._publish_status()
            elif kind == "mirror_abort":
                self._send_feedback(stop_rumble=True)
                self.get_logger().info("Spiegeln abgebrochen (vor Ablauf losgelassen).")
                self._publish_status()
            elif kind == "mirror":
                # Kein Halte-Strom: jedes Hand-Ziel braeche die geplante Fahrt
                # ab ("manueller Vorrang" im arm_controller), wie bei "home".
                self.holds.clear()
                self._mirror_n += 1
                self._mirror_id = f"ps4_mirror_{self._mirror_n}"
                self.pub_arm_cmd.publish(String(data=json.dumps(
                    {"id": self._mirror_id, "type": "mirror", "side": cmd[1]})))
                self.get_logger().info(
                    f"Spiegeln: {cmd[1]} -> Spiegelpose angefordert ({self._mirror_id}; "
                    "Stopp mit Kreuz/PS oder Stick).")
                self._send_feedback(rumble_ms=250, strong=0.0, weak=1.0, force=True)
                self._publish_status()
            elif kind == "changed":
                self._changed()
            elif kind == "waist":
                if cmd[1] == "start":
                    # Kein Halte-Strom: Halte-Ziele (Becken-Bezug) wuerden die
                    # Arme nach dem Drehen zurueck an die alte Stelle ziehen.
                    self.holds.clear()
                    self.get_logger().info("R2 gehalten: HUEFTE (rechter Stick links/rechts, "
                                           "R3 = Mitte).")
                elif self._waist_sending:
                    self._waist_sending = False
                    self._send_waist_rate(0.0)
                self._changed(rumble_ms=0)
            elif kind == "waist_center":
                self.pub_waist_center.publish(Bool(data=True))
                self.get_logger().info("R2 + R3: Huefte zur Mitte.")
                self._send_feedback(rumble_ms=120, strong=0.0, weak=0.5)
            elif kind == "waist_stop":
                self._send_waist_rate(0.0)
            elif kind == "jog_mode":
                # Dreieck: nur VERSCHIEBEN vibriert kurz, DREHEN nicht -- so
                # ist der Modus auch ohne GUI/LED eindeutig.
                self._changed(rumble_ms=100 if cmd[1] == "move" else 0)

        def _changed(self, rumble_ms=100):
            self.get_logger().info(self.logic.label())
            self._send_feedback(rumble_ms=rumble_ms)
            self._publish_status()
            self._publish_markers()

        # ── Rueckmeldung ───────────────────────────────────────────────
        def _send_feedback(self, rumble_ms=0, strong=0.0, weak=0.5, force=False,
                           stop_rumble=False):
            fb = {"rgb": list(self.logic.color())}
            if stop_rumble:
                fb["rumble"] = {"ms": 0}
            elif rumble_ms > 0 and (force or not (self.logic.full_arming
                                                  or self.logic.mirror_arming)):
                # Waehrend R1 + L1 + PS (Voll-Notaus) oder Steuerkreuz + Kreuz
                # (Spiegeln) gehalten werden, darf keine kurze Quittungs-
                # Vibration die Halte-Vibration ueberschreiben.
                fb["rumble"] = {"ms": int(rumble_ms), "strong": strong, "weak": weak}
            self.pub_feedback.publish(String(data=json.dumps(fb)))

        def _publish_status(self):
            st = self.logic.status()
            st["connected"] = self.connected
            self.pub_status.publish(String(data=json.dumps(st)))

        def _publish_markers(self):
            arr = MarkerArray()
            stamp = self.get_clock().now().to_msg()
            r, g, b = (c / 255.0 for c in self.logic.color())
            if self.logic.estop:
                r, g, b = 1.0, 0.0, 0.0
            active = self.logic.sides if not self.logic.estop else ()
            for i, side in enumerate(("left", "right")):
                m = Marker()
                m.header.frame_id = JOG_TF[side]
                m.header.stamp = stamp
                m.ns, m.id = "ps4_hand", i
                m.frame_locked = True
                if side in active:
                    m.type, m.action = Marker.SPHERE, Marker.ADD
                    m.pose.orientation.w = 1.0
                    m.scale.x = m.scale.y = m.scale.z = 0.09
                    m.color.r, m.color.g, m.color.b = r, g, b
                    m.color.a = 0.55 if self.connected else 0.2
                else:
                    m.action = Marker.DELETE
                arr.markers.append(m)
            # Kein Status-Text mehr ueber dem Roboter (stand in RViz im Weg).
            # Einen evtl. noch angezeigten Text einer aelteren Version loeschen;
            # der Status steht weiter auf /g1pilot/ps4/status und in der LED.
            t = Marker()
            t.header.frame_id = JOG_FRAME
            t.header.stamp = stamp
            t.ns, t.id = "ps4_label", 0
            t.action = Marker.DELETE
            arr.markers.append(t)
            self.pub_markers.publish(arr)

        # ── Hand bewegen ───────────────────────────────────────────────
        def _hand_pose(self, side):
            try:
                tr = self.tf_buffer.lookup_transform(JOG_FRAME, JOG_TF[side], rclpy.time.Time())
            except Exception:   # noqa: BLE001 -- Lookup/Connectivity/Extrapolation
                return None
            p, q = tr.transform.translation, tr.transform.rotation
            return [p.x, p.y, p.z], (q.x, q.y, q.z, q.w)

        def _jog_pose(self, side, new):
            """-> (start, ref) fuer einen Jog-Schritt (hand_jog.JogReference):
            Startpose eines neuen Ziels und Bezugshand fuer Vorlauf/Anhalten.
            Live umschaltbar: ros2 param set /ps4_arm_teleop jog_reference
            command|start|tf (gilt ab der naechsten Fahrt)."""
            mode = jog_reference_mode(self.get_parameter("jog_reference").value)
            if mode != self.jog_ref.mode:
                self.jog_ref = JogReference(mode)
            cmd = (self.cmd_poses.get(side, time.monotonic())
                   if mode != "tf" else None)
            return self.jog_ref.pose(side, cmd, self._hand_pose(side), new)

        def _publish_goal(self, side, pos, rot):
            msg = PoseStamped()
            msg.header.frame_id = JOG_FRAME
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
            o = msg.pose.orientation
            o.x, o.y, o.z, o.w = map(float, rot)
            self.pub_goal[side].publish(msg)

        def _hold(self, sides):
            """Ziel auf die Hand (Bezug: kommandiert, siehe jog_reference) setzen
            -> Arm bleibt sofort stehen statt den Vorlauf noch abzufahren
            (hold_pose: entlang der Bewegung an der Hand, quer auf der Ziel-Linie). Das Halte-Ziel wird JOG_HOLD_S lang
            wiederholt (HoldStreamer), sonst faehrt der arm_controller wegen
            seiner Ziel-Glaettung noch einen Grossteil des Vorlaufs ab."""
            now = time.monotonic()
            for side in sides:
                target = self.targets.pop(side, None)
                lin, ang = self.last_twist.pop(side, ((0.0,) * 3, (0.0,) * 3))
                cached = self.last_hand.pop(side, None)
                if target is None:
                    continue
                _, hand = self._jog_pose(side, new=False)
                self.jog_ref.end(side)
                if hand is None:
                    # TF-Luecke genau beim Loslassen: NICHT gar nichts senden --
                    # sonst faehrt der arm_controller das alte Ziel (ganzer
                    # Vorlauf) ab. Die Hand aus dem letzten Takt (<= 1 Takt alt)
                    # gibt es immer, sobald es ein Jog-Ziel gibt.
                    hand = cached
                    self.get_logger().warn(
                        f"Handposition {side} beim Anhalten unbekannt (TF) -- "
                        "halte an der zuletzt bekannten Hand.")
                if hand is not None:
                    pose = hold_pose(target, hand, lin, ang)
                    self.holds.hold(side, pose, now)
                    self._publish_goal(side, *pose)

        def _settle_jog(self, publish):
            """Laufenden Jog ohne Halte-Strom beenden: publish=True schickt je
            Seite EIN Halte-Ziel (vor einer neuen geplanten Fahrt), False gar
            nichts (Fahrt laeuft schon)."""
            sides = tuple(self.targets)
            if publish:
                self._hold(sides)
            for side in sides:
                self.targets.pop(side, None)
                self.last_twist.pop(side, None)
                self.last_hand.pop(side, None)
                self.jog_ref.end(side)
            self.holds.clear()

        def _stream_holds(self, now):
            for side, pose in self.holds.due(now).items():
                self._publish_goal(side, *pose)

        def _tick(self):
            now = time.monotonic()
            dt = min(0.1, now - self.t_tick)
            self.t_tick = now
            if self._home_off_at is not None and now >= self._home_off_at:
                self._home_off_at = None
                self.pub_home.publish(Bool(data=False))
            if self.t_joy is not None and now - self.t_joy > self.joy_timeout:
                if self.logic.abort_full():
                    self.get_logger().warn("Voll-Notaus-Halten abgebrochen: keine "
                                           "Controller-Daten mehr.")
                    self._send_feedback(stop_rumble=True)
                if self.logic.abort_mirror():
                    self.get_logger().warn("Spiegeln-Halten abgebrochen: keine "
                                           "Controller-Daten mehr.")
                    self._send_feedback(stop_rumble=True)
                if self.connected:
                    self.connected = False
                    self.get_logger().warn("PS4-Controller: keine Daten mehr -> Hand haelt an.")
                    self._publish_status()
                self._hold(tuple(self.targets))
                self._stream_holds(now)
                if self._waist_sending:
                    self._waist_sending = False
                    self._send_waist_rate(0.0)
                return
            rate = self.logic.waist_rate(self.axes) if self.connected else None
            if rate is not None:
                # Totmann: Strom mit ~30 Hz, der arm_controller haelt ohne ihn an.
                self._send_waist_rate(rate)
                self._waist_sending = True
            elif self._waist_sending:
                self._waist_sending = False
                self._send_waist_rate(0.0)
            tw = self.logic.twist(self.axes) if self.connected else {}
            yaw = self._torso_yaw()
            if yaw:
                tw = {side: rotate_twist_yaw(lin, ang, yaw) for side, (lin, ang) in tw.items()}
            # Seiten, die nicht mehr bewegt werden (Stick losgelassen): anhalten
            self._hold(tuple(s for s in self.targets if s not in tw))
            for side in tw:
                self.holds.cancel(side)
            self._stream_holds(now)
            if not tw:
                return
            v, w = JOG_SPEEDS[self.logic.speed]
            lead_m, lead_rad = lead_limits(v, w)
            for side, (lin, ang) in tw.items():
                new = side not in self.targets
                start, hand = self._jog_pose(side, new)
                if hand is None:
                    if not self._tf_warned:
                        self._tf_warned = True
                        self.get_logger().warn(
                            f"Handposition {side} unbekannt (TF {JOG_FRAME}->{JOG_TF[side]} "
                            "fehlt) -- laeuft robot_state?")
                    continue
                self._tf_warned = False
                self.last_hand[side] = hand
                p, q = jog_step(self.targets.get(side), hand, lin, ang, dt, lead_m, lead_rad,
                                start=start)
                self.targets[side] = [p, q]
                self.last_twist[side] = (lin, ang)
                self._publish_goal(side, p, q)

    rclpy.init(args=args)
    node = Ps4ArmTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
