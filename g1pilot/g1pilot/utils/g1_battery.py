"""Akku des echten G1 (rt/lf/bmsstate) -- reine Datenaufbereitung.

Quelle: DDS-Topic ``rt/lf/bmsstate`` (``unitree_hg`` ``BmsState_``, niedrige
Frequenz). Der G1 hat im ``rt/lowstate`` (anders als der Go2) KEIN BMS-Feld;
der Akku kommt ueber dieses eigene Topic.

Verwendete Felder:
  ``soc``          Ladestand in Prozent (0..100)  -> Hauptanzeige
  ``soh``          Gesundheitszustand in Prozent
  ``bmsvoltage[0]`` Pack-Spannung (ANNAHME: mV, siehe ``pack_voltage``)
  ``current``      Strom (ANNAHME: mA, Vorzeichen nicht dokumentiert)
  ``cell_vol[40]`` Zellspannungen in mV (unbenutzte Plaetze = 0)
  ``temperature[12]`` BMS-Temperaturen in Grad C (unbenutzte = 0)
  ``cycle``        Ladezyklen

Einheiten von Spannung/Strom sind von Unitree fuer den G1 nicht dokumentiert;
nur ``soc`` ist eindeutig. Deshalb zeigt die GUI den Ladestand gross und
Spannung/Strom nur als Zusatz im Tooltip (mit Plausibilitaetspruefung).

Kein ROS, kein Unitree-SDK, kein Qt -> im Testcontainer direkt testbar.
robot_state.py baut daraus ``/g1pilot/battery`` (JSON), demo_gui.py zeigt es.
"""
import json
import os

TOPIC_DDS = "rt/lf/bmsstate"
TOPIC_ROS = "/g1pilot/battery"

# Schwellen [% Ladestand]. ANNAHME (kein Herstellerwert im Repo): bis WARN
# gelb, bis CRIT rot. Per Umgebung anpassbar (G1_BATT_WARN / G1_BATT_CRIT).
DEFAULT_WARN_PCT = 30.0
DEFAULT_CRIT_PCT = 15.0
STALE_S = 5.0           # aelter -> Anzeige »keine Daten« (Topic ist »lf«)

# Plausibler Bereich der Pack-Spannung [V] (G1-Akku ca. 13s Li-Ion, ~43..55 V;
# grosszuegig, damit nur offensichtlich falsch interpretierte Werte rausfliegen).
_V_MIN, _V_MAX = 20.0, 70.0


def thresholds():
    """(warn, crit) aus der Umgebung; ungueltige Werte -> Defaults."""
    def _f(name, default):
        try:
            v = float(os.environ.get(name, ""))
        except ValueError:
            return default
        return v if 0 < v < 100 else default
    warn = _f("G1_BATT_WARN", DEFAULT_WARN_PCT)
    crit = _f("G1_BATT_CRIT", DEFAULT_CRIT_PCT)
    if crit >= warn:
        return DEFAULT_WARN_PCT, DEFAULT_CRIT_PCT
    return warn, crit


def pack_voltage(bmsvoltage, cell_vol):
    """Pack-Spannung in V oder None.

    Zuerst ``bmsvoltage[0]`` (als mV, sonst als V gedeutet), sonst Summe der
    belegten Zellspannungen (mV). Nur plausible Werte (20..70 V)."""
    cands = []
    try:
        raw = float(bmsvoltage[0])
        cands += [raw / 1000.0, raw]
    except (TypeError, IndexError, ValueError):
        pass
    cells = [c for c in _floats(cell_vol) if c > 0]
    if cells:
        cands.append(sum(cells) / 1000.0)
    for v in cands:
        if _V_MIN <= v <= _V_MAX:
            return round(v, 2)
    return None


def _floats(seq):
    try:
        return [float(x) for x in seq]
    except (TypeError, ValueError):
        return []


# ── Nachricht (robot_state -> GUI) ───────────────────────────────────────
def build_payload(bms):
    """JSON-String aus einem ``BmsState_`` (oder Objekt mit gleichen Feldern).

    Format: {"soc": %, "soh": %, "v": V|None, "i": A|None, "cycle": n,
             "tmax": °C|None, "cell": [min, max] mV|None,
             "raw": {"bmsvoltage": [...], "current": n}}
    """
    soc = int(bms.soc)
    cells = [c for c in _floats(getattr(bms, "cell_vol", [])) if c > 0]
    temps = [t for t in _floats(getattr(bms, "temperature", [])) if t != 0]
    cur = getattr(bms, "current", None)
    bv = _floats(getattr(bms, "bmsvoltage", []))
    return json.dumps({
        "soc": soc,
        "soh": int(getattr(bms, "soh", 0)),
        "v": pack_voltage(bv, cells),
        "i": round(float(cur) / 1000.0, 2) if cur is not None else None,
        "cycle": int(getattr(bms, "cycle", 0)),
        "tmax": max(temps) if temps else None,
        "cell": [min(cells), max(cells)] if cells else None,
        "raw": {"bmsvoltage": bv, "current": int(cur) if cur is not None else None},
    }, separators=(",", ":"))


def parse_payload(text):
    """JSON-String -> dict oder None bei kaputten/unplausiblen Daten."""
    try:
        d = json.loads(text)
        soc = float(d["soc"])
    except (ValueError, KeyError, TypeError):
        return None
    if not 0 <= soc <= 100:
        return None
    d["soc"] = soc
    return d


# ── Auswertung (GUI) ─────────────────────────────────────────────────────
def level(soc, warn=DEFAULT_WARN_PCT, crit=DEFAULT_CRIT_PCT):
    """'ok' | 'warn' | 'crit' (niedriger Ladestand = schlechter)."""
    if soc <= crit:
        return "crit"
    if soc <= warn:
        return "warn"
    return "ok"


def subtitle(data):
    """Kurzzeile unter dem Prozentwert: Spannung, sonst Zyklen."""
    if data.get("v") is not None:
        return f"{data['v']:.1f} V"
    return f"{data.get('cycle', 0)} Zyklen"


def details(data):
    """Mehrzeiliger Tooltip-Text."""
    lines = [f"Ladestand: {data['soc']:.0f} %"]
    if data.get("soh"):
        lines.append(f"Zustand (SOH): {data['soh']} %")
    if data.get("v") is not None:
        lines.append(f"Spannung: {data['v']:.1f} V")
    if data.get("i") is not None:
        lines.append(f"Strom: {data['i']:.2f} A (Einheit/Vorzeichen Annahme)")
    if data.get("cell"):
        lo, hi = data["cell"]
        lines.append(f"Zellen: {lo / 1000:.3f} … {hi / 1000:.3f} V")
    if data.get("tmax") is not None:
        lines.append(f"Akku-Temperatur max.: {data['tmax']:.0f} °C")
    lines.append(f"Ladezyklen: {data.get('cycle', 0)}")
    return "\n".join(lines)


def is_stale(last_t, now, stale_s=STALE_S):
    return last_t is None or (now - last_t) > stale_s
