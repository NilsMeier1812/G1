#!/usr/bin/env python3
"""
nav_supervisor — EINE Stelle, an der die Autonomie (AUTO NAV) freigegeben wird.

Sitzt zwischen dem Pfadfolger und dem Joy-Multiplexer:

  nav2point --/g1pilot/nav/raw_joy--> nav_supervisor --/g1pilot/auto_joy--> joy_mux
                                             |--/g1pilot/nav/auto_active--> joy_mux (Freigabe)
                                             '--/g1pilot/nav/state (JSON, 10 Hz) --> GUIs

Der Wunsch "AUTO NAV an" (/g1pilot/auto_enable aus Demo-GUI/Streamdeck)
ist nur eine ANFRAGE. Durchgereicht wird der virtuelle Joystick erst, wenn
alle Waechter gruen sind; faellt einer aus, sendet der Node EINEN Stop-Joy,
nimmt die Freigabe weg und bleibt im Zustand "stopped" (mit Grund), bis die
Anfrage neu gestellt wird (AUTO NAV aus und wieder an).

Zustaende:
  off      keine Anfrage
  queued   Anfrage da, wartet (auf Ziel/Pfad, Walk-Modus, Totmann + Stick)
  running  Joystick wird durchgereicht
  paused   Totmann (L2) losgelassen -> Stopp; L2 + linker Stick laeuft weiter
  stopped  ein Waechter hat ausgeloest (reason); neue Anfrage noetig

Totmann (require_deadman, real Pflicht): ps4_walk meldet auf
/g1pilot/ps4/deadman {"held": L2 gehalten, "go": linker Stick ausgelenkt}.
Losgehen nur mit L2 + Stick, Weiterlaufen nur mit gehaltenem L2. Bleiben die
Meldungen aus (Funkabriss -> ps4_joystick/ps4_walk melden nichts mehr), gilt
der Totmann als losgelassen.

Waechter (jeder einzeln per Parameter, Grenzwerte siehe declare_parameter):
  Totmann, GUI-Herzschlag, Pose zu alt / Pose-Sprung, Pfadfolger antwortet
  nicht, NOT-HALT, G1 nicht im Walk-Modus, Bahnabweichung, kein Fortschritt,
  Geofence (Ziel-Entfernung, Karte), Neigung, Lokalisierungs-Guete und
  Stopp-Feld (Eingaenge fuer spaetere Stufen, default aus), Tempo-Deckel.

Die Logik (SupervisorLogic) ist ROS-frei und bekommt die Zeit von aussen ->
test/test_nav_supervisor.py prueft jeden Waechter mit Test-Uhr.
Siehe g1pilot/docs/51_navigation_technik.md.
"""
import json
import math

OFF, QUEUED, RUNNING, PAUSED, STOPPED = "off", "queued", "running", "paused", "stopped"

# Index der Joy-Achsen/Taste, wie nav2point sie belegt (loco_client liest so).
AX_VY, AX_VX, AX_YAW, BTN_DEADMAN = 0, 1, 2, 8

DEFAULTS = {
    "require_deadman": True,
    "deadman_timeout_s": 0.3,
    "heartbeat_timeout_s": 1.0,     # 0 = kein GUI-Herzschlag noetig
    "pose_timeout_s": 0.3,
    "pose_jump_m": 0.2,
    "pose_jump_deg": 15.0,
    "raw_joy_timeout_s": 0.3,
    "require_walk_ready": True,
    "max_path_deviation_m": 0.3,    # 0 = aus
    "progress_timeout_s": 5.0,      # 0 = aus
    "progress_min_m": 0.03,
    "max_goal_distance_m": 10.0,    # 0 = aus
    "max_tilt_deg": 15.0,           # 0 = aus
    "imu_timeout_s": 0.5,
    "require_imu": False,
    "check_map": True,              # Pose/Ziel auf belegter oder unbekannter Zelle -> Stopp
    "require_loc_quality": False,   # Stufe 4
    "min_loc_quality": 0.5,
    "loc_quality_timeout_s": 1.0,
    "require_obstacle_field": False,  # Stufe 5
    "obstacle_timeout_s": 0.3,
    # Tempo-Deckel als Anteil der normierten Joy-Achsen (1.0 = kein Deckel).
    "axis_cap_lin": 1.0,
    "axis_cap_yaw": 1.0,
}

REASON_TEXT = {
    "deadman_lost": "Totmann losgelassen (L2)",
    "deadman_silent": "Controller meldet sich nicht (Funk?)",
    "heartbeat": "Bedienoberflaeche meldet sich nicht",
    "pose_timeout": "keine aktuelle Pose (Lokalisierung)",
    "pose_jump": "Pose ist gesprungen",
    "no_pose": "noch keine Pose",
    "follower_silent": "Pfadfolger antwortet nicht",
    "estop": "NOT-HALT",
    "not_walking": "G1 nicht im Walk-Modus",
    "deviation": "zu weit vom Pfad",
    "no_progress": "kein Fortschritt zum Ziel",
    "goal_far": "Ziel zu weit entfernt",
    "goal_blocked": "Ziel liegt auf belegter/unbekannter Kartenzelle",
    "off_map": "G1 steht auf belegter/unbekannter Kartenzelle",
    "tilt": "Neigung zu gross",
    "imu_timeout": "keine IMU-Daten",
    "loc_quality": "Lokalisierung unsicher",
    "obstacle": "Hindernis im Stopp-Feld",
    "obstacle_silent": "Stopp-Feld meldet sich nicht",
    "no_path": "kein Weg gefunden",
    "arrived": "Ziel erreicht",
    "wait_go": "bereit: L2 halten + linken Stick bewegen",
}


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def point_segment_distance(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def path_distance(path, x, y):
    if not path:
        return None
    if len(path) == 1:
        return math.hypot(x - path[0][0], y - path[0][1])
    return min(point_segment_distance(x, y, *path[i], *path[i + 1])
               for i in range(len(path) - 1))


def cap_axes(axes, cap_lin, cap_yaw):
    """Tempo-Deckel: Translation als Vektor skalieren (Richtung bleibt), Drehen
    einzeln begrenzen. Gibt eine neue Liste zurueck."""
    a = list(axes)
    while len(a) <= AX_YAW:
        a.append(0.0)
    mag = math.hypot(a[AX_VX], a[AX_VY])
    if cap_lin < 1.0 and mag > cap_lin > 0.0:
        k = cap_lin / mag
        a[AX_VX] *= k
        a[AX_VY] *= k
    elif cap_lin <= 0.0:
        a[AX_VX] = a[AX_VY] = 0.0
    a[AX_YAW] = max(-cap_yaw, min(cap_yaw, a[AX_YAW]))
    return a


class GridCheck:
    """Belegungskarte: Zelle frei? (unbekannt = nicht frei)."""

    def __init__(self, width, height, resolution, ox, oy, data, occ_threshold=50):
        self.w, self.h, self.res = int(width), int(height), float(resolution)
        self.ox, self.oy = float(ox), float(oy)
        self.data = list(data)
        self.th = int(occ_threshold)

    def free(self, x, y):
        if self.res <= 0.0:
            return True
        ix = int(math.floor((x - self.ox) / self.res))
        iy = int(math.floor((y - self.oy) / self.res))
        if not (0 <= ix < self.w and 0 <= iy < self.h):
            return False
        v = self.data[iy * self.w + ix]
        return 0 <= v < self.th


class SupervisorLogic:
    """ROS-freie Zustandsmaschine. Alle Eingaenge mit Zeitstempel `now`
    (Sekunden, monoton). tick(now) -> (out_joy oder None, events)."""

    def __init__(self, **params):
        self.p = dict(DEFAULTS)
        unknown = set(params) - set(DEFAULTS)
        if unknown:
            raise ValueError(f"unbekannte Parameter: {sorted(unknown)}")
        self.p.update(params)
        self.state = OFF
        self.reason = None
        self.requested = False
        self.t_deadman = None
        self.deadman_held = False
        self.deadman_go = False
        self.t_heartbeat = None
        self.t_pose = None
        self.pose = None            # (x, y, yaw)
        self.raw_joy = None
        self.t_raw = None
        self.estop = False
        self.walk_ready = None      # None = unbekannt
        self.path = []
        self.have_path = False
        self.goal = None            # (x, y, yaw oder None)
        self.grid = None
        self.t_imu = None
        self.tilt_deg = 0.0
        self.loc_quality = None
        self.t_loc = None
        self.obstacle = False
        self.t_obstacle = None
        self._best = None
        self._t_best = None
        self._pending_stop = False

    # ── Eingaenge ──────────────────────────────────────────────────────
    def on_request(self, on, now):
        on = bool(on)
        if on == self.requested:
            return
        self.requested = on
        if on:
            self.estop = False      # neue Anfrage = bewusste Quittung
            self._set(QUEUED, None)
        else:
            self._set(OFF, None)

    def on_deadman(self, held, go, now):
        self.deadman_held, self.deadman_go, self.t_deadman = bool(held), bool(go), now

    def on_heartbeat(self, now):
        self.t_heartbeat = now

    def on_pose(self, x, y, yaw, now):
        if self.pose is not None and self.state == RUNNING:
            jump = math.hypot(x - self.pose[0], y - self.pose[1])
            djaw = abs(math.degrees(_wrap(yaw - self.pose[2])))
            if jump > self.p["pose_jump_m"] or djaw > self.p["pose_jump_deg"]:
                self.pose, self.t_pose = (x, y, yaw), now
                self._fault("pose_jump")
                return
        self.pose, self.t_pose = (x, y, yaw), now

    def on_raw_joy(self, axes, buttons, now):
        self.raw_joy, self.t_raw = (list(axes), list(buttons)), now

    def on_estop(self, on, now):
        if on:
            self.estop = True
            if self.state in (QUEUED, RUNNING, PAUSED):
                self._fault("estop")

    def on_walk_ready(self, ready, now):
        self.walk_ready = None if ready is None else bool(ready)

    def on_path(self, points, now):
        self.path = [(float(x), float(y)) for x, y in points]
        self.have_path = bool(self.path)
        self._best, self._t_best = None, None
        if self.state == RUNNING:
            # Neuer Pfad waehrend der Fahrt: anhalten, neu mit L2 + Stick
            # (ohne Totmann laeuft er im naechsten tick direkt weiter).
            self._set(QUEUED, None)

    def on_goal(self, x, y, yaw, now):
        self.goal = (float(x), float(y), yaw)
        self._best, self._t_best = None, None

    def on_nav_status(self, status, now):
        if status == "arrived" and self.state in (RUNNING, PAUSED):
            self.have_path = False
            self._set(QUEUED, "arrived")
        elif status == "no_path" and self.state in (QUEUED, RUNNING, PAUSED):
            self.have_path = False
            self._set(QUEUED, "no_path")

    def on_map(self, grid):
        self.grid = grid

    def on_imu(self, roll, pitch, now):
        self.tilt_deg = math.degrees(max(abs(roll), abs(pitch)))
        self.t_imu = now

    def on_loc_quality(self, q, now):
        self.loc_quality, self.t_loc = float(q), now

    def on_obstacle(self, blocked, now):
        self.obstacle, self.t_obstacle = bool(blocked), now

    # ── Pruefungen ─────────────────────────────────────────────────────
    def _fresh(self, t, timeout, now):
        return t is not None and (now - t) <= timeout

    def _deadman_ok(self, now):
        """-> None (ok) oder Grund."""
        if not self.p["require_deadman"]:
            return None
        if not self._fresh(self.t_deadman, self.p["deadman_timeout_s"], now):
            return "deadman_silent"
        return None if self.deadman_held else "deadman_lost"

    def _hard_checks(self, now, running):
        """Waechter, die in queued/paused das Losgehen sperren und in running
        sofort stoppen. -> erster Grund oder None."""
        p = self.p
        if self.estop:
            return "estop"
        if p["heartbeat_timeout_s"] > 0 and not self._fresh(
                self.t_heartbeat, p["heartbeat_timeout_s"], now):
            return "heartbeat"
        if self.pose is None:
            return "no_pose"
        if not self._fresh(self.t_pose, p["pose_timeout_s"], now):
            return "pose_timeout"
        if p["require_walk_ready"] and self.walk_ready is not True:
            return "not_walking"
        if p["require_imu"] and not self._fresh(self.t_imu, p["imu_timeout_s"], now):
            return "imu_timeout"
        if p["max_tilt_deg"] > 0 and self._fresh(self.t_imu, p["imu_timeout_s"], now) \
                and self.tilt_deg > p["max_tilt_deg"]:
            return "tilt"
        if p["require_loc_quality"]:
            if not self._fresh(self.t_loc, p["loc_quality_timeout_s"], now) \
                    or self.loc_quality < p["min_loc_quality"]:
                return "loc_quality"
        if p["require_obstacle_field"]:
            if not self._fresh(self.t_obstacle, p["obstacle_timeout_s"], now):
                return "obstacle_silent"
            if self.obstacle:
                return "obstacle"
        x, y, _ = self.pose
        if self.goal is not None:
            if p["max_goal_distance_m"] > 0 and \
                    math.hypot(self.goal[0] - x, self.goal[1] - y) > p["max_goal_distance_m"]:
                return "goal_far"
            if p["check_map"] and self.grid is not None and not self.grid.free(*self.goal[:2]):
                return "goal_blocked"
        if p["check_map"] and self.grid is not None and not self.grid.free(x, y):
            return "off_map"
        if running and p["max_path_deviation_m"] > 0:
            d = path_distance(self.path, x, y)
            if d is not None and d > p["max_path_deviation_m"]:
                return "deviation"
        return None

    def _progress_metric(self):
        if self.goal is None or self.pose is None:
            return None
        x, y, yaw = self.pose
        m = math.hypot(self.goal[0] - x, self.goal[1] - y)
        if self.goal[2] is not None:
            m += 0.5 * abs(_wrap(self.goal[2] - yaw))
        return m

    def _raw_active(self):
        if self.raw_joy is None:
            return False
        _, b = self.raw_joy
        return len(b) > BTN_DEADMAN and b[BTN_DEADMAN] == 1

    # ── Zustand ────────────────────────────────────────────────────────
    def _set(self, state, reason):
        if self.state == RUNNING and state != RUNNING:
            self._pending_stop = True
        self.state, self.reason = state, reason
        if state == RUNNING:
            self._best, self._t_best = None, None

    def _fault(self, reason):
        self._set(STOPPED, reason)

    def tick(self, now):
        """-> (joy oder None, stop): joy = (axes, buttons) zum Durchreichen;
        stop = True, wenn genau jetzt EIN Stop-Joy faellig ist."""
        p = self.p
        st = self.state
        if st in (QUEUED, PAUSED):
            why = self._hard_checks(now, running=False)
            if not self.have_path:
                if self.reason not in ("arrived", "no_path"):
                    self.reason = why          # None = wartet auf Ziel
            elif why is not None:
                self.reason = why              # Losgehen gesperrt (kein Fehler)
            elif not self._raw_active():
                self.reason = None             # Pfadfolger noch nicht bereit
            elif not p["require_deadman"]:
                self._set(RUNNING, None)
            else:
                dm = self._deadman_ok(now)
                if dm is None and self.deadman_go:
                    self._set(RUNNING, None)
                else:
                    self.reason = dm if dm == "deadman_silent" else "wait_go"
        elif st == RUNNING:
            why = self._hard_checks(now, running=True)
            if why is None and self._raw_active() and not self._fresh(
                    self.t_raw, p["raw_joy_timeout_s"], now):
                why = "follower_silent"
            if why is None and p["progress_timeout_s"] > 0:
                m = self._progress_metric()
                if m is not None:
                    if self._best is None or m < self._best - p["progress_min_m"]:
                        self._best, self._t_best = m, now
                    elif now - self._t_best > p["progress_timeout_s"]:
                        why = "no_progress"
            if why is not None:
                self._fault(why)
            else:
                dm = self._deadman_ok(now)
                if dm is not None:
                    self._set(PAUSED, dm)

        stop = self._pending_stop
        self._pending_stop = False
        if self.state != RUNNING:
            return None, stop
        if self.raw_joy is None or not self._fresh(self.t_raw, p["raw_joy_timeout_s"], now):
            return None, stop
        axes, buttons = self.raw_joy
        return (cap_axes(axes, p["axis_cap_lin"], p["axis_cap_yaw"]), list(buttons)), stop

    def status(self):
        return {"state": self.state, "reason": self.reason,
                "text": REASON_TEXT.get(self.reason, "") if self.reason else "",
                "deadman": self.deadman_held, "require_deadman": bool(self.p["require_deadman"]),
                "have_path": self.have_path}


# ════════════════════════════════════════════════════════════════════════
#  ROS-Node
# ════════════════════════════════════════════════════════════════════════
def main(args=None):
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Odometry, Path, OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import Joy, Imu
    from std_msgs.msg import Bool, String, Float32

    def yaw_of(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    class NavSupervisor(Node):
        def __init__(self):
            super().__init__("nav_supervisor")
            for k, v in DEFAULTS.items():
                self.declare_parameter(k, v)
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("pose_topic", "/lidar_odometry/pose_fixed")
            self.declare_parameter("raw_joy_topic", "/g1pilot/nav/raw_joy")
            self.declare_parameter("out_joy_topic", "/g1pilot/auto_joy")
            self.declare_parameter("active_topic", "/g1pilot/nav/auto_active")
            gp = lambda n: self.get_parameter(n).value   # noqa: E731
            params = {}
            for k, v in DEFAULTS.items():
                val = gp(k)
                params[k] = type(v)(val) if not isinstance(v, bool) else bool(val)
            self.logic = SupervisorLogic(**params)
            now = time.monotonic

            qos = QoSProfile(depth=10)
            latched = QoSProfile(depth=1)
            latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

            self.pub_joy = self.create_publisher(Joy, gp("out_joy_topic"), qos)
            self.pub_active = self.create_publisher(Bool, gp("active_topic"), latched)
            self.pub_state = self.create_publisher(String, "/g1pilot/nav/state", latched)

            L = self.logic
            self.create_subscription(Bool, "/g1pilot/auto_enable",
                                     lambda m: L.on_request(m.data, now()), qos)
            self.create_subscription(String, "/g1pilot/ps4/deadman", self._on_deadman, qos)
            self.create_subscription(Bool, "/g1pilot/nav/heartbeat",
                                     lambda m: m.data and L.on_heartbeat(now()), qos)
            self.create_subscription(Odometry, gp("pose_topic"), self._on_pose, qos)
            self.create_subscription(Joy, gp("raw_joy_topic"),
                                     lambda m: L.on_raw_joy(m.axes, m.buttons, now()), qos)
            for t in ("/g1pilot/emergency_stop", "/g1pilot/full_emergency_stop"):
                self.create_subscription(Bool, t, lambda m: L.on_estop(m.data, now()), qos)
            self.create_subscription(String, "/g1pilot/onboard_state", self._on_onboard, latched)
            self.create_subscription(String, "/g1pilot/loco_state",
                                     lambda m: L.on_walk_ready(m.data == "walk", now()), latched)
            self.create_subscription(Path, "/g1pilot/path", self._on_path, qos)
            self.create_subscription(PoseStamped, "/g1pilot/goal", self._on_goal, qos)
            self.create_subscription(String, "/g1pilot/nav_status",
                                     lambda m: L.on_nav_status(m.data, now()), latched)
            self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
            self.create_subscription(Imu, "/g1pilot/imu", self._on_imu, qos)
            self.create_subscription(Float32, "/g1pilot/nav/loc_quality",
                                     lambda m: L.on_loc_quality(m.data, now()), qos)
            self.create_subscription(Bool, "/g1pilot/nav/obstacle_stop",
                                     lambda m: L.on_obstacle(m.data, now()), qos)

            self._last_state = None
            self._active = None
            self._publish_active(False)
            self.create_timer(1.0 / float(gp("rate_hz")), self._tick)
            self.create_timer(0.1, self._publish_state)
            self.get_logger().info(
                "nav_supervisor bereit: Totmann %s, Herzschlag %.1f s, Deckel %.2f/%.2f"
                % ("Pflicht (L2 + Stick)" if params["require_deadman"] else "aus",
                   params["heartbeat_timeout_s"], params["axis_cap_lin"], params["axis_cap_yaw"]))

        def _on_deadman(self, msg):
            try:
                d = json.loads(msg.data)
            except (TypeError, ValueError):
                return
            self.logic.on_deadman(d.get("held"), d.get("go"), time.monotonic())

        def _on_pose(self, msg):
            p = msg.pose.pose
            self.logic.on_pose(p.position.x, p.position.y, yaw_of(p.orientation),
                               time.monotonic())

        def _on_onboard(self, msg):
            try:
                self.logic.on_walk_ready(json.loads(msg.data).get("balanced"),
                                         time.monotonic())
            except (TypeError, ValueError, AttributeError):
                pass

        def _on_path(self, msg):
            self.logic.on_path([(q.pose.position.x, q.pose.position.y) for q in msg.poses],
                               time.monotonic())

        def _on_goal(self, msg):
            self.logic.on_goal(msg.pose.position.x, msg.pose.position.y,
                               yaw_of(msg.pose.orientation), time.monotonic())

        def _on_map(self, msg):
            i = msg.info
            self.logic.on_map(GridCheck(i.width, i.height, i.resolution,
                                        i.origin.position.x, i.origin.position.y, msg.data))

        def _on_imu(self, msg):
            q = msg.orientation
            roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
            pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
            self.logic.on_imu(roll, pitch, time.monotonic())

        def _publish_active(self, on):
            if on != self._active:
                self._active = on
                self.pub_active.publish(Bool(data=bool(on)))

        def _stop_joy(self):
            j = Joy()
            j.header.stamp = self.get_clock().now().to_msg()
            j.axes = [0.0] * 8
            j.buttons = [0] * 14
            self.pub_joy.publish(j)

        def _tick(self):
            out, stop = self.logic.tick(time.monotonic())
            if stop:
                self._stop_joy()
            if out is not None:
                j = Joy()
                j.header.stamp = self.get_clock().now().to_msg()
                j.axes = [float(a) for a in out[0]]
                j.buttons = [int(b) for b in out[1]]
                self.pub_joy.publish(j)
            self._publish_active(self.logic.state == RUNNING)
            st = (self.logic.state, self.logic.reason)
            if st != self._last_state:
                self._last_state = st
                text = REASON_TEXT.get(st[1], "") if st[1] else ""
                line = f"AUTO NAV: {st[0]}" + (f" ({text})" if text else "")
                # Zwei Aufrufstellen: rclpy verbietet wechselnde Stufe an EINER Stelle
                # (ValueError -> Node tot).
                if st[0] == STOPPED:
                    self.get_logger().warn(line)
                else:
                    self.get_logger().info(line)
                self._publish_state()

        def _publish_state(self):
            self.pub_state.publish(String(data=json.dumps(self.logic.status())))

    rclpy.init(args=args)
    node = NavSupervisor()
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
