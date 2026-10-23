"""walk_diag: Auswertung und Umrechnung ohne Roboter (DDS/ROS nicht noetig)."""
import csv
import json
import math
import struct
from types import SimpleNamespace as NS

from g1pilot.tools import walk_diag as wd


def _remote_bytes(lx, rx, ry, l2, ly):
    b = bytearray(40)
    struct.pack_into("<5f", b, 4, lx, rx, ry, l2, ly)
    return list(b)


def _lowstate(roll=0.0, knee_l=14.0, knee_r=14.0, temp=40):
    motors = [NS(q=0.0, dq=0.0, tau_est=0.0, temperature=[temp, temp - 2], motorstate=0)
              for _ in range(35)]
    motors[3].tau_est = knee_l
    motors[9].tau_est = knee_r
    motors[1].q = 0.02            # L hip roll
    motors[7].q = -0.02           # R hip roll (gespiegelt)
    imu = NS(rpy=[roll, 0.0, 0.5], gyroscope=[0, 0, 0.01], accelerometer=[0, 0, 9.8],
             temperature=35)
    return NS(tick=7, imu_state=imu, motor_state=motors,
              wireless_remote=_remote_bytes(0.1, -0.2, 0.3, 0.0, 0.4))


def test_parse_remote_layout():
    assert wd.parse_remote(_remote_bytes(0.1, -0.2, 0.3, 0.0, 0.4)) == \
        tuple(struct.unpack("<4f", struct.pack("<4f", 0.1, 0.4, -0.2, 0.3)))
    assert wd.parse_remote([0] * 10) is None
    assert wd.parse_remote(None) is None
    # Murks (z. B. Fernbedienung aus, Speicher voller NaN/riesiger Werte)
    assert wd.parse_remote(_remote_bytes(float("nan"), 0, 0, 0, 0)) is None


def test_lowstate_fields_and_columns():
    row = wd.lowstate_fields(_lowstate(roll=0.01))
    assert row["L_knee_tau"] == 14.0 and row["R_knee_tau"] == 14.0
    assert row["L_knee_temp"] == 40.0
    assert row["imu_roll"] == 0.01 and row["tick"] == 7
    assert math.isclose(row["rc_ly"], 0.4, rel_tol=1e-6)
    cols = wd.columns()
    assert set(row) <= set(cols)
    assert len(cols) == len(set(cols))


def test_body_velocity_rotates_into_robot_frame():
    fwd, left = wd.body_velocity(0.0, 1.0, math.pi / 2)   # Welt +y, Roboter schaut +y
    assert math.isclose(fwd, 1.0, abs_tol=1e-9) and math.isclose(left, 0.0, abs_tol=1e-9)


def _synthetic(tmp_path):
    """3 s Stehen (Last links hoeher), 4 s geradeaus mit Linksdrift, 3 s rechts."""
    rows = []
    t = 0.0
    px = py = 0.0
    for phase, (vx, vy, n) in enumerate(((0.0, 0.0, 150), (0.5, 0.0, 200), (0.0, -0.5, 150))):
        for _ in range(n):
            r = wd.lowstate_fields(_lowstate(roll=math.radians(1.0), knee_l=18.0, knee_r=10.0))
            odom_vy = 0.08 if vx else (-0.02 if vy else 0.0)
            odom_vx = 0.2 if vx else 0.0
            px += odom_vx * 0.02
            py += odom_vy * 0.02
            r.update({"t": round(t, 3), "fsm": 501, "cmd_vx": vx, "cmd_vy": vy, "cmd_wz": 0.0,
                      "odom_vx": odom_vx, "odom_vy": odom_vy, "odom_wz": 0.0,
                      "odom_px": px, "odom_py": py, "imu_yaw": 0.0,
                      "L_ankle_roll_qcmd": 0.0, "L_ankle_roll_q": 0.03,
                      "R_ankle_roll_qcmd": 0.0, "R_ankle_roll_q": 0.005})
            rows.append(r)
            t += 0.02
    path = tmp_path / "walk_diag_x.csv"
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=wd.columns(), extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    (tmp_path / "walk_diag_x.json").write_text(json.dumps(
        {"label": "synth", "max_vx": 0.4, "max_vy": 0.3, "max_vyaw": 0.4,
         "sources": ["rt/lowstate"]}))
    return str(path)


def test_segments_and_report(tmp_path):
    rows, meta = wd.load_csv(_synthetic(tmp_path))
    kinds = [s["kind"] for s in wd.segments(rows)]
    assert kinds == ["stand", "+vx", "-vy"]
    text = "\n".join(wd.analyze(rows, meta))
    assert "Stehen, Befehl 0" in text
    assert "Roll +1.00 deg" in text
    # Knie-Last links 18 / rechts 10 -> Asymmetrie +8
    assert "knee" in text and "+18.00 /   +10.00 /   +8.00" in text
    # Geradeaus: Befehl 0.5 * 0.4 = 0.2 m/s, Odometrie seitlich +0.08 (links)
    assert "Befehl  vx +0.200 m/s" in text
    assert "Abweichung seitlich (Odometrie - Befehl): +0.080 m/s" in text
    assert "Seiten ungleich" in text                  # ankle_roll L 1.7 deg vs R 0.3 deg
    # Seitlich rechts: Befehl -0.15 m/s, erreicht -0.02 -> +0.13 Abweichung
    assert "+0.130 m/s" in text


def test_non_walk_fsm_is_ignored():
    rows = [{"t": str(i * 0.02), "fsm": "4", "cmd_vx": "0.5", "cmd_vy": "0", "cmd_wz": "0"}
            for i in range(200)]
    assert wd.segments(rows) == []


def test_side_compare_mirrored_joints():
    rows = [{"L_hip_roll_q": "0.05", "R_hip_roll_q": "-0.05",
             "L_knee_q": "0.3", "R_knee_q": "0.2"}]
    comp = wd.side_compare(rows, "q")
    assert math.isclose(comp["hip_roll"][2], 0.0, abs_tol=1e-12)   # symmetrisch
    assert math.isclose(comp["knee"][2], 0.1)
