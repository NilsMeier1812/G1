#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""dds_api_check -- Welche RPC-Dienste bietet der echte G1 per DDS an?

Hintergrund (Real-Test 2026-10-16): loco_client bekam fuer jeden Loco-RPC
Code 3102 (RPC_ERR_CLIENT_SEND). Der kommt, wenn auf "rt/api/sport/request"
innerhalb des Timeouts KEIN Leser des Roboters gefunden wird -- der Befehl
geht also gar nicht erst raus. rt/lowstate und rt/arm_sdk laufen trotzdem,
weil das reine Topics ohne RPC-Dienst sind.

Dieses Skript lauscht ein paar Sekunden auf der Unitree-Domain (0) und listet
alle Topics "rt/api/<dienst>/request", fuer die der Roboter einen LESER hat
(= Dienst wartet auf Anfragen), und alle uebrigen rt/-Topics. Danach versucht
es einmal GetFsmId ueber den LocoClient.

Aufruf im laufenden Real-Container:
    docker compose exec g1pilot-real python3 -m g1pilot.tools.dds_api_check enp0s31f6
Interface = ROBOT_INTERFACE (im Log: "[DDS] ... iface=...").
Nur lesend: sendet keinen Befehl ausser einer GetFsmId-Abfrage.
"""
import sys
import time


def main():
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    listen_s = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0

    from unitree_sdk2py.core.channel import ChannelFactory, ChannelFactoryInitialize
    from cyclonedds.builtin import (BuiltinDataReader, BuiltinTopicDcpsPublication,
                                    BuiltinTopicDcpsSubscription)

    ChannelFactoryInitialize(0, iface)
    dp = ChannelFactory()._ChannelFactory__participant
    subs = BuiltinDataReader(dp, BuiltinTopicDcpsSubscription)
    pubs = BuiltinDataReader(dp, BuiltinTopicDcpsPublication)

    readers, writers = {}, {}
    t_end = time.monotonic() + listen_s
    while time.monotonic() < t_end:
        for s in subs.take(N=200):
            if getattr(s, "topic_name", None):
                readers.setdefault(s.topic_name, set()).add(str(s.participant_key))
        for p in pubs.take(N=200):
            if getattr(p, "topic_name", None):
                writers.setdefault(p.topic_name, set()).add(str(p.participant_key))
        time.sleep(0.1)

    print(f"\n== DDS-Domain 0 ueber {iface}, {listen_s:.0f} s gelauscht ==")
    services = sorted(t[len("rt/api/"):-len("/request")] for t in readers
                      if t.startswith("rt/api/") and t.endswith("/request"))
    print("RPC-Dienste mit Leser (Anfragen moeglich):")
    for name in services:
        print(f"   {name:<20} rt/api/{name}/request")
    if not services:
        print("   (keine) -> kein RPC-Dienst des Roboters sichtbar")
    print("'sport' (LocoClient):", "JA" if "sport" in services else "NEIN")

    others = sorted(t for t in set(readers) | set(writers)
                    if t.startswith("rt/") and not t.startswith("rt/api/"))
    print("\nUebrige rt/-Topics (R = Leser, W = Schreiber):")
    for t in others:
        print(f"   {'R' if t in readers else ' '}{'W' if t in writers else ' '}  {t}")

    print("\nGetFsmId ueber LocoClient (Timeout 3 s):")
    import unitree_sdk2py.g1.loco.g1_loco_client as mod
    from unitree_sdk2py.g1.loco.g1_loco_api import ROBOT_API_ID_LOCO_GET_FSM_ID
    print(f"   installiertes SDK: Dienst '{mod.LOCO_SERVICE_NAME}' -- getestet wird 'sport'")
    mod.LOCO_SERVICE_NAME = "sport"
    c = mod.LocoClient()
    c.SetTimeout(3.0)
    c.Init()
    code, data = c._Call(ROBOT_API_ID_LOCO_GET_FSM_ID, "{}")
    print(f"   Code {code}, Antwort {data!r}")
    print("   0 = ok, 3102 = kein Leser auf rt/api/sport/request, 3104 = keine Antwort")


if __name__ == "__main__":
    main()
