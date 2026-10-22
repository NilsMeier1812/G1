#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR-Netz (lidar_config): MID-360 schickt an den PC am Roboter-LAN.

Frueher lud livox_launcher die Werks-Konfiguration des Treibers (192.168.1.x)
und config/livox_mid.json wurde nie benutzt -> keine Punkte im Roboter-LAN.
"""
import json
import os

import pytest

from g1pilot.navigation import lidar_config as lc

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEMPLATE = os.path.join(REPO, "config", "livox_mid.json")


def load():
    with open(TEMPLATE) as f:
        return json.load(f)


def test_build_sets_all_host_ips_and_lidar_ip():
    cfg = lc.build_mid360_config(load(), "192.168.123.99", "192.168.123.121")
    host = cfg["MID360"]["host_net_info"]
    for k in ("cmd_data_ip", "push_msg_ip", "point_data_ip", "imu_data_ip"):
        assert host[k] == "192.168.123.99"
    assert cfg["lidar_configs"][0]["ip"] == "192.168.123.121"
    assert load()["MID360"]["host_net_info"]["point_data_ip"] == "192.168.123.123", \
        "Vorlage bleibt unveraendert"


def test_invalid_ip_rejected():
    with pytest.raises(ValueError):
        lc.build_mid360_config(load(), "192.168.123.300", None)


def test_host_ip_priority():
    env = {"G1_LIDAR_HOST_IP": "192.168.123.50", "ROBOT_INTERFACE": "eth0"}
    assert lc.resolve_host_ip(env, lambda i: "192.168.123.7") == ("192.168.123.50",
                                                                   "G1_LIDAR_HOST_IP")
    env = {"ROBOT_INTERFACE": "eth0"}
    assert lc.resolve_host_ip(env, lambda i: "192.168.123.7")[0] == "192.168.123.7"
    assert lc.resolve_host_ip({}, lambda i: None) == (None, "Vorlage")


def test_write_config_and_subnet_warning(tmp_path):
    out = tmp_path / "mid360.json"
    path, host, lidar, notes = lc.write_config(
        TEMPLATE, str(out), env={"ROBOT_INTERFACE": "enp3s0"},
        iface_lookup=lambda i: "192.168.123.42")
    cfg = json.loads(out.read_text())
    assert host == "192.168.123.42" and lidar == lc.DEFAULT_LIDAR_IP
    assert cfg["MID360"]["host_net_info"]["imu_data_ip"] == "192.168.123.42"
    assert not any("WARNUNG" in n for n in notes)
    _, _, _, notes = lc.write_config(
        TEMPLATE, str(out), env={"G1_LIDAR_HOST_IP": "10.0.0.5"})
    assert any("WARNUNG" in n for n in notes)


def test_interface_ipv4_loopback():
    assert lc.interface_ipv4("lo") in ("127.0.0.1", None)
    assert lc.interface_ipv4("gibtesnicht0") is None
    assert lc.interface_ipv4("") is None
