#!/usr/bin/env python3
"""
lidar_config — Netzwerk-Konfiguration fuer den Livox MID-360 im G1-Kopf.

Der livox_ros_driver2 schreibt dem LiDAR beim Start, an welche IP er Punkte,
IMU und Status schicken soll (host_net_info). Frueher lud livox_launcher die
Werks-Konfiguration aus dem Treiberpaket (192.168.1.x) -- im Roboter-LAN
192.168.123.x kamen so keine Daten an. Jetzt baut livox_launcher die Datei
aus config/livox_mid.json mit:

  Host-IP   G1_LIDAR_HOST_IP, sonst die IPv4 von ROBOT_INTERFACE (der PC,
            der per LAN am G1 haengt), sonst die Vorlage (192.168.123.123)
  LiDAR-IP  G1_LIDAR_IP (Default 192.168.123.120 = MID-360 im G1)

Achtung: der LiDAR schickt danach an DIESEN PC. Ob Unitrees Onboard-Software
den MID-360 selbst braucht, ist ungeprueft (docs/70_echtroboter_anleitung.md).
"""
import copy
import ipaddress
import json
import os
import socket
import struct

DEFAULT_LIDAR_IP = "192.168.123.120"
OUT_PATH = "/tmp/g1pilot_livox_mid360.json"


def interface_ipv4(iface):
    """IPv4-Adresse eines Netzwerk-Interfaces (Linux) oder None."""
    if not iface:
        return None
    try:
        import fcntl
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            req = struct.pack("256s", iface[:15].encode())
            res = fcntl.ioctl(s.fileno(), 0x8915, req)   # SIOCGIFADDR
            return socket.inet_ntoa(res[20:24])
        finally:
            s.close()
    except (OSError, ImportError):
        return None


def _check_ip(ip, what):
    try:
        return str(ipaddress.IPv4Address(ip.strip()))
    except (ipaddress.AddressValueError, AttributeError):
        raise ValueError(f"{what}: keine gueltige IPv4-Adresse: {ip!r}")


def build_mid360_config(template, host_ip=None, lidar_ip=None):
    """Kopie der Vorlage mit gesetzter Host-/LiDAR-IP (Logdaten-IP bleibt)."""
    cfg = copy.deepcopy(template)
    host = cfg["MID360"]["host_net_info"]
    if host_ip:
        ip = _check_ip(host_ip, "Host-IP")
        for k in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
            host[k] = ip
    if lidar_ip:
        cfg["lidar_configs"][0]["ip"] = _check_ip(lidar_ip, "LiDAR-IP")
    return cfg


def resolve_host_ip(env=os.environ, iface_lookup=interface_ipv4):
    """-> (ip oder None, Quelle)."""
    ip = env.get("G1_LIDAR_HOST_IP", "").strip()
    if ip:
        return ip, "G1_LIDAR_HOST_IP"
    iface = env.get("ROBOT_INTERFACE", "").strip()
    ip = iface_lookup(iface)
    if ip:
        return ip, f"Interface {iface}"
    return None, "Vorlage"


def same_subnet(a, b, prefix=24):
    try:
        na = ipaddress.IPv4Network(f"{a}/{prefix}", strict=False)
        return ipaddress.IPv4Address(b) in na
    except (ipaddress.AddressValueError, ValueError):
        return False


def write_config(template_path, out_path=OUT_PATH, env=os.environ,
                 iface_lookup=interface_ipv4):
    """Datei schreiben -> (Pfad, host_ip, lidar_ip, Hinweise)."""
    with open(template_path) as f:
        template = json.load(f)
    host_ip, src = resolve_host_ip(env, iface_lookup)
    lidar_ip = env.get("G1_LIDAR_IP", "").strip() or DEFAULT_LIDAR_IP
    cfg = build_mid360_config(template, host_ip, lidar_ip)
    used_host = cfg["MID360"]["host_net_info"]["point_data_ip"]
    notes = [f"Host-IP {used_host} ({src}), LiDAR-IP {lidar_ip}"]
    if not same_subnet(used_host, lidar_ip):
        notes.append(f"WARNUNG: Host {used_host} und LiDAR {lidar_ip} nicht im selben /24 "
                     "-- es kommen vermutlich keine Punkte an.")
    with open(out_path, "w") as f:
        json.dump(cfg, f, indent=2)
    return out_path, used_host, lidar_ip, notes
