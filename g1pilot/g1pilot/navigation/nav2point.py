#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, Bool, String

def yaw_from_quat(x, y, z, w):
    s = 2.0 * (w * z + x * y)
    c = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(s, c)

class Nav2Point(Node):
    def __init__(self):
        super().__init__('nav2point')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('pos_kp', 0.8)
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('waypoint_tolerance', 0.20)
        self.declare_parameter('goal_tolerance', 0.03)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('joy_topic', '/g1pilot/auto_joy')
        self.declare_parameter('path_topic', '/g1pilot/path')
        self.declare_parameter('auto_enable_topic', '/g1pilot/auto_enable')
        self.declare_parameter('vx_limit', 0.6)
        self.declare_parameter('vy_limit', 0.6)
        self.declare_parameter('wz_limit', 0.5)
        
        self.rate = self.get_parameter('publish_rate').value
        self.pos_kp = self.get_parameter('pos_kp').value
        self.yaw_kp = self.get_parameter('yaw_kp').value
        self.wp_tol = self.get_parameter('waypoint_tolerance').value
        self.goal_tol = self.get_parameter('goal_tolerance').value
        self.frame_id = self.get_parameter('frame_id').value
        self.joy_topic = self.get_parameter('joy_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.vx_lim = self.get_parameter('vx_limit').value
        self.vy_lim = self.get_parameter('vy_limit').value
        self.wz_lim = self.get_parameter('wz_limit').value
        self.auto_enable_topic = self.get_parameter('auto_enable_topic').value

        # Endausrichtung: am Ziel auf den vorgegebenen Ziel-Yaw (Pfeil im RViz-
        # "2D Goal Pose") drehen -- auf der Stelle, kuerzester Weg. Und: kurz vor
        # dem Ziel NICHT mehr zum Punkt drehen (sonst umkreist der Roboter ihn).
        self.declare_parameter('goal_topic', '/g1pilot/goal')
        self.declare_parameter('final_align', True)     # am Ziel auf Ziel-Yaw drehen
        self.declare_parameter('yaw_tol_deg', 3.0)      # Toleranz Endausrichtung [deg] (Greifen braucht < 5)
        self.declare_parameter('align_yaw_kp', 1.2)     # Dreh-Regler beim Ausrichten
        self.declare_parameter('yaw_hold_dist', 0.5)    # ab hier Yaw-Jagen aus (Orbit-Fix) [m]
        # loco_sim schaltet bei ||cmd|| < stand_eps (0.1) auf Stehen -> zu kleine
        # Kommandos kurz vor dem Ziel bringen den Roboter 10-20 cm davor zum
        # Stehen, die Endausrichtung startet nie. Daher Mindest-Ausschlag:
        self.declare_parameter('min_axis', 0.35)        # Translation, normiert (-> >= 0.17 m/s)
        self.declare_parameter('min_yaw_axis', 0.25)    # Endausrichtung, normiert
        self.declare_parameter('stall_s', 3.0)          # kein Fortschritt nahe Ziel -> ausrichten
        self.declare_parameter('stall_dist', 0.35)      # [m]
        goal_topic = self.get_parameter('goal_topic').value
        self.final_align = bool(self.get_parameter('final_align').value)
        self.yaw_tol = math.radians(float(self.get_parameter('yaw_tol_deg').value))
        self.align_yaw_kp = float(self.get_parameter('align_yaw_kp').value)
        self.yaw_hold_dist = float(self.get_parameter('yaw_hold_dist').value)

        qos = QoSProfile(depth=10)
        self.sub_odom = self.create_subscription(Odometry, '/lidar_odometry/pose_fixed', self.cb_odom, qos)
        self.sub_auto_enable = self.create_subscription(Bool, self.auto_enable_topic, self.cb_auto_enable, qos)
        self.sub_path = self.create_subscription(Path, self.path_topic, self.cb_path, qos)
        # Ziel-Yaw direkt aus dem Ziel lesen (dijkstra ueberschreibt die Pfad-
        # Orientierung mit Segmentrichtungen -> der Ziel-Yaw ginge dort verloren).
        self.sub_goal_pose = self.create_subscription(PoseStamped, goal_topic, self.cb_goal_pose, qos)
        self.pub_joy = self.create_publisher(Joy, self.joy_topic, qos)
        self.pub_wp_marker = self.create_publisher(Marker, '/g1pilot/waypoint_marker', qos)
        self.pub_goal_marker = self.create_publisher(Marker, '/g1pilot/goal_marker', qos)
        # Fortschritt fuer die Bedienoberflaeche (demo_gui): Ereignisse
        #   moving  = neuer Pfad, faehrt (sobald AUTO NAV an ist) zum Ziel
        #   arrived = Ziel erreicht (inkl. Endausrichtung), Pfad verworfen
        #   no_path = Planer hat keinen Weg gefunden (leerer Pfad)
        #   idle    = noch kein Ziel seit Start
        # TRANSIENT_LOCAL: eine spaeter startende GUI bekommt den letzten Stand.
        qos_status = QoSProfile(depth=1)
        qos_status.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub_status = self.create_publisher(String, '/g1pilot/nav_status', qos_status)
        self.timer = self.create_timer(1.0 / self.rate, self.loop)
        self.min_axis = float(self.get_parameter('min_axis').value)
        self.min_yaw_axis = float(self.get_parameter('min_yaw_axis').value)
        self.stall_s = float(self.get_parameter('stall_s').value)
        self.stall_dist = float(self.get_parameter('stall_dist').value)
        self._best_dist = None
        self._best_t = None

        self.have_pose = False
        self.auto_enabled = False
        self.path = []
        self.path_frame = self.frame_id
        self.idx = 0
        self.x = self.y = self.yaw = 0.0
        self.goal_yaw = None      # Ziel-Yaw aus /g1pilot/goal (None = keiner)
        self.aligning = False     # True = am Ziel, dreht auf Ziel-Yaw

        self.logged_no_pose = False
        self.logged_no_path = False
        self.logged_end_path = False
        self.publish_status('idle')

    def publish_status(self, state):
        self.pub_status.publish(String(data=state))

    def publish_stop(self):
        joy = Joy()
        joy.header.stamp = self.get_clock().now().to_msg()
        joy.axes = [0.0] * 8
        joy.buttons = [0] * 14
        self.pub_joy.publish(joy)

    def cb_odom(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self.yaw = yaw_from_quat(qx, qy, qz, qw)
        self.have_pose = True
        self.logged_no_pose = False

    def cb_auto_enable(self, msg: Bool):
        self.auto_enabled = msg.data

    def cb_path(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_frame = msg.header.frame_id if msg.header.frame_id else self.frame_id
        self.idx = 0
        self.aligning = False      # neuer Pfad -> Ausrichtungsphase zuruecksetzen
        self._best_dist = None
        self.logged_no_path = False
        self.logged_end_path = False
        if self.path:
            self.publish_goal_marker(self.path[-1][0], self.path[-1][1])
            self.publish_status('moving')
        else:
            # Planer fand keinen Weg -> anhalten statt dem alten Pfad weiter
            # zu folgen (sonst liefe der Roboter zum vorherigen Ziel).
            self.publish_stop()
            self.publish_status('no_path')

    def cb_goal_pose(self, msg: PoseStamped):
        o = msg.pose.orientation
        self.goal_yaw = yaw_from_quat(o.x, o.y, o.z, o.w)
        self.aligning = False      # neues Ziel -> Ausrichtungsphase zuruecksetzen

    @staticmethod
    def _wrap(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def publish_goal_marker(self, gx, gy):
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'g1pilot_goal'
        m.id = 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = gx
        m.pose.position.y = gy
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.12
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
        self.pub_goal_marker.publish(m)

    def publish_wp_marker(self, wx, wy):
        m = Marker()
        m.header.frame_id = self.path_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'g1pilot_wp'
        m.id = 2
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = wx
        m.pose.position.y = wy
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.10
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.9
        self.pub_wp_marker.publish(m)

    def loop(self):
        try:
            if (len(self.path) == 0):
                if not self.logged_no_path:
                    self.get_logger().warn('No path available.')
                    self.logged_no_path = True
                return
            
            if not self.have_pose and self.auto_enabled:
                if not self.logged_no_pose:
                    self.get_logger().warn('No pose available.')
                    self.logged_no_pose = True
                return

            if (not self.path or len(self.path) == 0) and self.auto_enabled:
                if not self.logged_no_path:
                    self.get_logger().warn('No path available.')
                    self.logged_no_path = True
                return

            if self.idx >= len(self.path) and self.auto_enabled:
                if not self.logged_end_path:
                    self.get_logger().warn('Reached the end of the path.')
                    self.logged_end_path = True
                return

            self.logged_no_pose = False
            self.logged_no_path = False
            self.logged_end_path = False

            wx, wy = self.path[self.idx]
            dx = wx - self.x
            dy = wy - self.y
            dist_wp = math.hypot(dx, dy)

            if self.idx < len(self.path) - 1 and dist_wp <= self.wp_tol:
                self.idx += 1
                wx, wy = self.path[self.idx]
                dx = wx - self.x
                dy = wy - self.y
                dist_wp = math.hypot(dx, dy)

            self.publish_wp_marker(wx, wy)

            dist_goal = math.hypot(self.path[-1][0] - self.x, self.path[-1][1] - self.y)

            # Haengt der Roboter kurz vor dem Ziel fest (kein Fortschritt), trotzdem
            # in die Endausrichtung gehen -- sonst bleibt er schraeg stehen.
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._best_dist is None or dist_goal < self._best_dist - 0.02:
                self._best_dist, self._best_t = dist_goal, now
            elif (not self.aligning and dist_goal <= self.stall_dist
                  and now - self._best_t > self.stall_s):
                self.get_logger().info(
                    f'Kein Fortschritt {dist_goal:.2f} m vor dem Ziel -> Endausrichtung.')
                self.aligning = True

            joy = Joy()
            joy.header.stamp = self.get_clock().now().to_msg()
            axes = [0.0] * 8
            buttons = [0] * 14

            # ── Endphase: am Ziel auf der Stelle auf den Ziel-Yaw drehen ──────
            #  aligning latcht, sobald der Zielpunkt erreicht war -> KEINE
            #  Translation mehr (kein Umkreisen), nur noch Drehen auf Ziel-Yaw
            #  (kuerzester Weg -> nie 270°). Ohne Ziel-Yaw / final_align: nur Stop.
            if self.aligning or dist_goal <= self.goal_tol:
                self.aligning = True
                if self.final_align and self.goal_yaw is not None:
                    yaw_err = self._wrap(self.goal_yaw - self.yaw)
                    if abs(yaw_err) <= self.yaw_tol:
                        self.pub_joy.publish(joy)     # axes/buttons = 0 -> Stop
                        self.path = []
                        self.aligning = False
                        self.publish_status('arrived')
                        return
                    wz = max(-self.wz_lim, min(self.wz_lim, self.align_yaw_kp * yaw_err))
                    n = wz / self.wz_lim
                    if abs(n) < self.min_yaw_axis:      # sonst steht loco_sim nur
                        n = math.copysign(self.min_yaw_axis, n)
                    axes[2] = max(-1.0, min(1.0, -n))
                    buttons[8] = 1
                    joy.axes = axes
                    joy.buttons = buttons
                    self.pub_joy.publish(joy)
                    return
                self.pub_joy.publish(joy)
                self.path = []
                self.aligning = False
                self.publish_status('arrived')
                return

            # ── Anfahrt: holonom zum Zielpunkt, Yaw in Fahrtrichtung ──────────
            desired_yaw = math.atan2(dy, dx)
            yaw_err = self._wrap(desired_yaw - self.yaw)

            vx_w = max(-self.vx_lim, min(self.vx_lim, self.pos_kp * dx))
            vy_w = max(-self.vy_lim, min(self.vy_lim, self.pos_kp * dy))

            c = math.cos(-self.yaw)
            s = math.sin(-self.yaw)
            vx_b = c * vx_w - s * vy_w
            vy_b = s * vx_w + c * vy_w

            # Orbit-Fix: nahe am Ziel NICHT mehr zum (dann instabilen) Punkt
            # drehen, sondern schon auf den Ziel-Yaw (holonom: seitlich/rueckwaerts
            # reingleiten) -- so steht er auch dann richtig, wenn die letzten
            # Zentimeter nicht ganz aufgehen.
            if dist_goal < self.yaw_hold_dist:
                if self.final_align and self.goal_yaw is not None:
                    wz = max(-self.wz_lim, min(self.wz_lim,
                             self.align_yaw_kp * self._wrap(self.goal_yaw - self.yaw)))
                else:
                    wz = 0.0
            else:
                wz = max(-self.wz_lim, min(self.wz_lim, self.yaw_kp * yaw_err))

            nx, ny = vx_b / self.vx_lim, vy_b / self.vy_lim
            mag = math.hypot(nx, ny)
            if 1e-6 < mag < self.min_axis:          # nicht unter die Steh-Schwelle fallen
                nx, ny = nx * self.min_axis / mag, ny * self.min_axis / mag
            axes[1] = max(-1.0, min(1.0, -nx))
            axes[0] = max(-1.0, min(1.0, -ny))
            axes[2] = max(-1.0, min(1.0, -wz / self.wz_lim))
            buttons[8] = 1

            joy.axes = axes
            joy.buttons = buttons
            self.pub_joy.publish(joy)

        except Exception as e:
            self.get_logger().error(f'Error in loop: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Point()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
