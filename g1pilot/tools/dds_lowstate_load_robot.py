# Messung 2026-10-21 (Docker, --cpus begrenzt): Fake-G1 fuer dds_lowstate_load_client.py.
"""Fake-G1: rt/lowstate 500 Hz + Loco-RPC-Server (rt/api/sport/*)."""
import sys, time, threading
sys.path.insert(0, "/ws/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_, unitree_api_msg_dds__Response_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_, Response_
ChannelFactoryInitialize(0, "lo")
low = ChannelPublisher("rt/lowstate", LowState_); low.Init()
resp = ChannelPublisher("rt/api/sport/response", Response_); resp.Init()
def on_req(req):
    from unitree_sdk2py.idl.default import unitree_api_msg_dds__ResponseHeader_ as RH
    r = Response_(RH(), "", [])
    r.header.identity.id = req.header.identity.id
    r.header.identity.api_id = req.header.identity.api_id
    r.data = '{"data": 4}'
    resp.Write(r)
sub = ChannelSubscriber("rt/api/sport/request", Request_); sub.Init(on_req, 10)
msg = unitree_hg_msg_dds__LowState_()
t = time.monotonic()
while True:
    t += 0.002
    msg.tick += 1
    msg.motor_state[3].tau_est = -14.0
    low.Write(msg)
    d = t - time.monotonic()
    if d > 0: time.sleep(d)
