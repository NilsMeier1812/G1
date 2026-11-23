"""Roboter-LAN: Verbindung pruefen und feste IP per NetworkManager einrichten.

Ohne Qt, nur Standardbibliothek (laeuft auf dem Host, Tests pruefen die Logik
direkt). Zwei Teile:

* ``probe()``: blockierende Zustandsabfrage (Interface/IP/Kabel, Ping zum G1,
  TCP zu den Inspire-Haenden). Das Startmenue ruft sie periodisch in einem
  Hintergrund-Thread auf, die GUI blockiert also nie.
* ``plan()``: die nmcli-Schritte fuer das dauerhafte Profil ``g1-robot``
  (wie docs/50-A-Echter-Roboter.md Abschnitt B). Der Assistent zeigt sie vor
  dem Ausfuehren als Befehle an. Root holt sich nmcli selbst ueber polkit
  (Passwort-Abfrage des Desktops); nur wenn das scheitert, ``pkexec``.
"""
from __future__ import annotations

import ipaddress
import os
import re
import shlex
import shutil
import socket
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

ROBOT_NET = ipaddress.ip_network("192.168.123.0/24")
G1_IP = "192.168.123.161"
PROFILE = "g1-robot"
DEFAULT_PC_IP = "192.168.123.222"
# Adressen im Roboter-LAN, die der eigene PC nicht nehmen darf.
RESERVED = {
    "192.168.123.161": "G1-Onboard-PC",
    "192.168.123.164": "G1-Entwickler-PC (PC2)",
    "192.168.123.120": "LiDAR im Kopf",
    "192.168.123.210": "Inspire-Hand links",
    "192.168.123.211": "Inspire-Hand rechts",
}
UFW_CONF = Path("/etc/ufw/ufw.conf")
SYS_NET = Path("/sys/class/net")
# nmcli-/polkit-Meldungen, nach denen der Schritt mit pkexec wiederholt wird.
PRIV_ERR = re.compile(r"insufficient privileges|not authorized|permission denied|"
                      r"nicht berechtigt|unzureichende berechtigung", re.I)


def have(cmd: str) -> bool:
    return shutil.which(cmd) is not None


# ════════════════════════════════════════════════════════════════════════
#  Adressen
# ════════════════════════════════════════════════════════════════════════
def in_robot_lan(ip_cidr: str) -> bool:
    """'192.168.123.222/24' oder '192.168.123.222' -> True."""
    try:
        return ipaddress.ip_interface(ip_cidr.strip()).ip in ROBOT_NET
    except ValueError:
        return False


def validate_pc_ip(text: str) -> tuple[str, str]:
    """Eingabe der PC-Adresse pruefen -> (adresse, fehlertext). Fehlertext '' = ok."""
    t = text.strip().split("/")[0]
    try:
        ip = ipaddress.ip_address(t)
    except ValueError:
        return "", "Keine gültige IPv4-Adresse (z. B. 192.168.123.222)."
    if ip.version != 4 or ip not in ROBOT_NET:
        return "", "Die Adresse muss im Roboter-LAN 192.168.123.x liegen."
    if ip in (ROBOT_NET.network_address, ROBOT_NET.broadcast_address):
        return "", "Netz- und Broadcast-Adresse (.0/.255) gehen nicht."
    if str(ip) in RESERVED:
        return "", f"{ip} gehört schon dem {RESERVED[str(ip)]}."
    return str(ip), ""


def robot_ifaces(nics: list[tuple[str, str]]) -> list[str]:
    """Alle Interfaces mit einer 192.168.123.x-Adresse."""
    return [n for n, ip in nics if in_robot_lan(ip)]


# ════════════════════════════════════════════════════════════════════════
#  Host-Abfragen (blockierend, nur im Hintergrund aufrufen)
# ════════════════════════════════════════════════════════════════════════
def carrier(iface: str, sys_net: Path = SYS_NET) -> bool | None:
    """Kabel/Link da? None = unbekannt (Interface fehlt oder ist abgeschaltet)."""
    if not iface or "/" in iface:
        return None
    try:
        return (sys_net / iface / "carrier").read_text().strip() == "1"
    except (OSError, ValueError):
        return None


def ping(host: str, timeout_s: int = 1) -> bool | None:
    """Ein ICMP-Echo. None = kein ping-Programm."""
    if not have("ping"):
        return None
    try:
        r = subprocess.run(["ping", "-c", "1", "-W", str(timeout_s), host],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                           timeout=timeout_s + 2)
        return r.returncode == 0
    except Exception:
        return False


def tcp_open(host: str, port: int, timeout_s: float = 0.6) -> bool:
    try:
        with socket.create_connection((host, int(port)), timeout=timeout_s):
            return True
    except (OSError, ValueError):
        return False


def ufw_enabled(conf: Path = UFW_CONF) -> bool:
    """ufw aktiv? (ufw.conf ist ohne root lesbar, `ufw status` nicht)."""
    try:
        return bool(re.search(r"^\s*ENABLED\s*=\s*yes", conf.read_text(), re.M))
    except OSError:
        return False


def _nmcli(*args: str, timeout: float = 5) -> subprocess.CompletedProcess | None:
    if not have("nmcli"):
        return None
    try:
        return subprocess.run(["nmcli", *args], capture_output=True, text=True,
                              timeout=timeout, env={**os.environ, "LC_ALL": "C"})
    except Exception:
        return None


def nm_running() -> bool:
    """NetworkManager installiert und aktiv."""
    r = _nmcli("-t", "-f", "RUNNING", "general")
    return bool(r and r.returncode == 0 and r.stdout.strip() == "running")


def parse_profile(text: str) -> dict:
    """Ausgabe von `nmcli -t -f <felder> connection show <name>` -> dict."""
    out: dict = {}
    for line in text.splitlines():
        key, sep, val = line.partition(":")
        if sep:
            out[key.strip()] = val.strip()
    return out


def nm_profile(name: str = PROFILE) -> dict | None:
    """Vorhandenes NM-Profil (iface, ip, method, active) oder None."""
    r = _nmcli("-t", "-f", "connection.interface-name,ipv4.method,ipv4.addresses,"
               "GENERAL.STATE", "connection", "show", name)
    if r is None or r.returncode != 0:
        return None
    d = parse_profile(r.stdout)
    return {
        "iface": d.get("connection.interface-name", ""),
        "method": d.get("ipv4.method", ""),
        "ip": d.get("ipv4.addresses", "").split(",")[0].strip(),
        "active": d.get("GENERAL.STATE", "") == "activated",
    }


# ════════════════════════════════════════════════════════════════════════
#  Einrichtungs-Plan
# ════════════════════════════════════════════════════════════════════════
@dataclass
class Step:
    title: str
    argv: list[str]
    root: bool = False          # braucht sicher root (ufw) -> direkt pkexec
    optional: bool = False      # Fehler bricht den Ablauf nicht ab
    note: str = ""              # Hinweis, wenn der optionale Schritt scheitert

    def shell(self) -> str:
        return ("sudo " if self.root else "") + shlex.join(self.argv)


def plan(iface: str, ip: str, existing: dict | None, *, link_up: bool | None = True,
         open_firewall: bool = False, prefix: int = 24) -> list[Step]:
    """nmcli-Schritte fuer das Profil g1-robot. Kein Gateway, kein DNS: das
    Internet laeuft weiter ueber WLAN. Ein vorhandenes Profil wird geaendert,
    nicht geloescht (nichts anderes am PC wird angefasst)."""
    addr = f"{ip}/{prefix}"
    common = ["ipv4.method", "manual", "ipv4.addresses", addr,
              "ipv4.gateway", "", "ipv4.dns", "", "ipv4.never-default", "yes",
              "ipv6.method", "disabled",
              "connection.autoconnect", "yes", "connection.autoconnect-priority", "10"]
    steps: list[Step] = []
    if existing is not None:
        steps.append(Step(f"Profil »{PROFILE}« anpassen ({iface}, {addr})",
                          ["nmcli", "connection", "modify", PROFILE,
                           "connection.interface-name", iface, *common]))
    else:
        steps.append(Step(f"Profil »{PROFILE}« anlegen ({iface}, {addr})",
                          ["nmcli", "connection", "add", "type", "ethernet",
                           "ifname", iface, "con-name", PROFILE, *common]))
    steps.append(Step(
        f"Profil »{PROFILE}« aktivieren", ["nmcli", "connection", "up", PROFILE],
        optional=True,
        note=("Ohne eingestecktes Kabel geht das noch nicht. Das Profil "
              "aktiviert sich von selbst, sobald der G1 angeschlossen ist."
              if link_up is False else
              "Aktivieren fehlgeschlagen. Kabel und Interface prüfen.")))
    if open_firewall:
        steps.append(Step(f"Firewall (ufw) für {iface} öffnen",
                          ["ufw", "allow", "in", "on", iface], root=True, optional=True,
                          note="Firewall-Regel nicht gesetzt. Von Hand: "
                               f"sudo ufw allow in on {iface}"))
    return steps


def remove_plan() -> list[Step]:
    return [Step(f"Profil »{PROFILE}« entfernen",
                 ["nmcli", "connection", "delete", PROFILE])]


def manual_commands(iface: str, ip: str, prefix: int = 24) -> str:
    """Fallback ohne NetworkManager (geht bei Neustart/Kabelziehen verloren)."""
    q = shlex.quote(iface)
    return (f"sudo ip addr add {ip}/{prefix} dev {q}\n"
            f"sudo ip link set {q} up")


def with_root(step: Step) -> list[str]:
    """argv mit pkexec (grafische Passwort-Abfrage), wenn nicht schon root."""
    if os.geteuid() == 0 or not have("pkexec"):
        return list(step.argv)
    exe = shutil.which(step.argv[0]) or step.argv[0]
    return ["pkexec", exe, *step.argv[1:]]


def needs_root_retry(output: str) -> bool:
    return bool(PRIV_ERR.search(output or ""))


# ════════════════════════════════════════════════════════════════════════
#  Verbindungs-Zustand fuer die Anzeige
# ════════════════════════════════════════════════════════════════════════
@dataclass
class LinkStatus:
    iface: str
    ip: str = ""
    link: bool | None = None            # Kabel/Link
    g1: bool | None = None              # Ping 192.168.123.161 (None = nicht geprueft)
    hands: list[tuple[str, str, bool]] = field(default_factory=list)  # (name, ip, ok)
    others: list[str] = field(default_factory=list)   # weitere Ifaces im Roboter-LAN
    nics: list[tuple[str, str]] = field(default_factory=list)

    @property
    def in_lan(self) -> bool:
        return in_robot_lan(self.ip)

    @property
    def level(self) -> str:
        """ok (gruen) | warn (orange) | off (orange, kein Netz)."""
        if not self.iface or not self.in_lan or self.link is False:
            return "off"
        if self.g1 is False or self.others or any(not ok for _n, _i, ok in self.hands):
            return "warn"
        return "ok"

    def text(self) -> str:
        if not self.iface:
            return "Kein Netzwerk-Interface gewählt."
        if self.link is False and not self.in_lan:
            return (f"An {self.iface} steckt kein Kabel (Link aus). G1 einschalten und "
                    "per LAN-Kabel anschließen.")
        if not self.in_lan:
            return (f"Keine 192.168.123.x-Adresse auf {self.iface}. Mit »Netzwerk "
                    "einrichten« eine feste IP vergeben.")
        ip = self.ip.split("/")[0]
        if self.link is False:
            return f"{self.iface} hat {ip}, aber kein Kabel steckt (Link aus)."
        if self.g1 is False:
            msg = (f"Im Roboter-LAN ({ip}), aber der G1 ({G1_IP}) antwortet nicht. "
                   "Eingeschaltet und fertig hochgefahren?")
        elif self.g1 is True:
            msg = f"Im Roboter-LAN ({ip}). G1 antwortet."
        else:
            msg = f"Im Roboter-LAN ({ip})."
        if self.others:
            msg += (" Achtung: auch " + ", ".join(self.others) +
                    " hat eine 192.168.123.x-Adresse, das stört DDS.")
        return msg


def probe(iface: str, hands: list[tuple[str, str]] | None = None, port: int = 6000, *,
          nics_fn=None, ping_fn=ping, tcp_fn=tcp_open, carrier_fn=carrier) -> LinkStatus:
    """Kompletter Zustand (blockiert bis ~2 s). `hands`: [(name, ip)]."""
    if nics_fn is None:
        from .core import detect_nics
        nics_fn = detect_nics
    nics = list(nics_fn())
    st = LinkStatus(iface=iface, nics=nics)
    st.ip = dict(nics).get(iface, "")
    st.link = carrier_fn(iface) if iface else None
    st.others = [n for n in robot_ifaces(nics) if n != iface]
    if iface and st.in_lan and st.link is not False:
        st.g1 = ping_fn(G1_IP)
        for name, hip in hands or []:
            st.hands.append((name, hip, bool(tcp_fn(hip, port))))
    return st
