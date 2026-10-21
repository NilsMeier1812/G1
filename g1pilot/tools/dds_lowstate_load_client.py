# Messung 2026-10-21: RPC-Latenz loco_client-artig mit lowstate-Listener (alt) vs. Abfrage (neu).
# Aufruf: python3 dds_lowstate_load_robot.py & python3 dds_lowstate_load_client.py listener|poll|none 40
# Ergebnis --cpus=0.25: listener 30/72 Zeitueberschreitungen, p50 700 ms; poll 0, p50 84 ms; none p50 8 ms.
"""loco_client-Nachbildung: 2-Hz-GetFsmId per LocoClient-RPC, dazu entweder
lowstate-Listener (alt) oder 10-Hz-Abfrage ohne Listener (neu), plus
rclpy-aehnliche Python-Last (Timer-Threads)."""
import sys, time, threading, json
sys.path.insert(0, "/ws/unitree_sdk2_python")
mode = sys.argv[1]; dur = float(sys.argv[2])
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
import unitree_sdk2py.g1.loco.g1_loco_client as lm
lm.LOCO_SERVICE_NAME = "sport"
ChannelFactoryInitialize(0, "lo")
c = lm.LocoClient(); c.SetTimeout(1.0); c.Init()
n_low = [0]
def feed(m):
    _ = (abs(m.motor_state[3].tau_est), abs(m.motor_state[9].tau_est)); n_low[0] += 1
sub = ChannelSubscriber("rt/lowstate", LowState_)
if mode == "listener":
    sub.Init(feed, 10)
elif mode == "poll":
    sub.Init()
    def poll():
        while True:
            m = sub.Read(0.05)
            if m is not None: feed(m)
            time.sleep(0.1)
    threading.Thread(target=poll, daemon=True).start()
# Python-Last wie rclpy-Executor (cmd_vel 20 Hz, publish 2 Hz ...)
def busy():
    while True:
        s = 0
        for i in range(20000): s += i
        time.sleep(0.05)
for _ in range(3): threading.Thread(target=busy, daemon=True).start()
time.sleep(1.0)
lat, fail = [], 0
t_end = time.monotonic() + dur
while time.monotonic() < t_end:
    t0 = time.monotonic()
    code, data = c._Call(7001, "{}")
    dt = time.monotonic() - t0
    if code == 0: lat.append(dt)
    else: fail += 1
    time.sleep(max(0, 0.5 - dt))
lat.sort()
p = lambda q: round(1000 * lat[int(q * (len(lat) - 1))], 1) if lat else None
print(json.dumps({"mode": mode, "calls": len(lat) + fail, "timeouts": fail,
                  "p50_ms": p(0.5), "p95_ms": p(0.95), "max_ms": p(1.0),
                  "lowstate_per_s": round(n_low[0] / (dur + 1), 1)}))
