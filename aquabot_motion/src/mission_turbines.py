#!/usr/bin/env python3
#
# Continuous-circle inspection mission.
#
# Instead of driving 8 discrete stop-and-go waypoints around each turbine
# (which makes a cruise tracker saturate the steering and oscillate), this
# builds ONE continuous path per turbine:  a planner-generated approach leg
# (rock-aware) concatenated with a dense analytic circle, published once to
# /aquabot/plan.  The MPPI controller then tracks the whole thing smoothly.
#
# Contract with the (separate) QR node:
#   - OUT: /aquabot/current_target  (PoseStamped)  -> which turbine we're orbiting
#   - IN : /aquabot/qr_facing       (PoseStamped)  -> QR decoded + facing direction

import rclpy
from rclpy.node import Node
import math
from enum import Enum

from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class MissionState(Enum):
    WAITING_FOR_DATA = 0
    PLAN_APPROACH = 1
    WAITING_FOR_PLAN = 2
    ORBITING = 3
    MISSION_COMPLETE = 4


class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.get_logger().info("Mission Control Online (continuous-circle)!")

        # ---------- tunables ----------
        self.orbit_radius = 18.0          # inspection ring radius [m]
        self.orbit_points = 36            # density of the analytic circle
        self.orbit_arc_deg = 340.0        # path arc; < 360 so start/end don't overlap
        self.orbit_direction = +1         # +1 = counter-clockwise, -1 = clockwise
        self.completion_arc_deg = 330.0   # swept angle that counts as "orbit done"
        self.reach_circle_tol = 3.0       # how near the ring before we start counting laps

        # ---------- state ----------
        self.state = MissionState.WAITING_FOR_DATA
        self.turbines = []
        self.current_turbine_index = 0
        self.boat_pose = None

        self.final_waypoint = None        # last point of the current orbit (for reference)
        self.swept_angle = 0.0            # accumulated angle around the turbine this lap
        self.prev_angle = None
        self.on_circle = False            # have we reached the ring yet?
        self.latest_qr_facing = None      # filled by the QR node (future)
        self._completed_logged = False

        # ---------- I/O ----------
        self.turbine_sub = self.create_subscription(
            PoseArray, '/aquabot/turbines', self.turbine_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/aquabot/odom', self.odom_callback, 10)
        # hook for the separate QR node (no-op today)
        self.qr_sub = self.create_subscription(
            PoseStamped, '/aquabot/qr_facing', self.qr_facing_callback, 10)

        self.path_pub = self.create_publisher(Path, '/aquabot/plan', 10)
        # tells the QR node which turbine is currently being inspected
        self.target_pub = self.create_publisher(PoseStamped, '/aquabot/current_target', 10)

        self.planner_client = self.create_client(GetPlan, '/aquabot/get_plan')

        self.timer = self.create_timer(0.1, self.state_machine_loop)

    # ------------------------------------------------------------------ helpers
    def generate_circle_path(self, cx, cy, entry_angle):
        """Dense analytic circle starting at entry_angle, covering orbit_arc_deg."""
        path = Path()
        path.header.frame_id = "world"
        path.header.stamp = self.get_clock().now().to_msg()

        arc = math.radians(self.orbit_arc_deg) * self.orbit_direction
        n = self.orbit_points
        for i in range(n + 1):
            theta = entry_angle + arc * (i / n)
            px = cx + self.orbit_radius * math.cos(theta)
            py = cy + self.orbit_radius * math.sin(theta)
            # tangent heading = direction of travel along the circle
            heading = theta + self.orbit_direction * (math.pi / 2.0)

            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = px
            ps.pose.position.y = py
            ps.pose.orientation = yaw_to_quaternion(heading)
            path.poses.append(ps)
        return path

    def current_turbine_xy(self):
        t = self.turbines[self.current_turbine_index].position
        return t.x, t.y

    # ------------------------------------------------------------ subscriptions
    def turbine_callback(self, msg):
        if len(self.turbines) == 0:
            self.turbines = msg.poses
            self.get_logger().info(f"Received {len(self.turbines)} turbine locations!")

    def odom_callback(self, msg):
        self.boat_pose = msg.pose.pose

    def qr_facing_callback(self, msg):
        # TODAY: just record + log.
        # LATER: this is where ORBITING hands off to the 30s station-keep phase.
        self.latest_qr_facing = msg
        self.get_logger().info("QR facing received (hook -- not yet wired into the flow).")

    def plan_received_callback(self, future):
        approach = future.result().plan

        tx, ty = self.current_turbine_xy()
        entry_angle = math.atan2(self.boat_pose.position.y - ty,
                                 self.boat_pose.position.x - tx)
        circle = self.generate_circle_path(tx, ty, entry_angle)

        # concatenate approach + circle into one continuous plan
        full = Path()
        full.header.frame_id = "world"
        full.header.stamp = self.get_clock().now().to_msg()
        full.poses = list(approach.poses) + list(circle.poses)

        if len(full.poses) == 0:
            self.get_logger().error("Empty concatenated plan -- retrying approach.")
            self.state = MissionState.PLAN_APPROACH
            return

        # reset lap tracking for this turbine
        self.final_waypoint = (circle.poses[-1].pose.position.x,
                               circle.poses[-1].pose.position.y)
        self.swept_angle = 0.0
        self.prev_angle = None
        self.on_circle = False

        self.path_pub.publish(full)
        self.get_logger().info(
            f"Published approach+circle for turbine {self.current_turbine_index} "
            f"({len(approach.poses)} approach + {len(circle.poses)} circle pts).")
        self.state = MissionState.ORBITING

    # --------------------------------------------------------------- main loop
    def state_machine_loop(self):
        if self.state == MissionState.WAITING_FOR_DATA:
            if self.boat_pose is not None and len(self.turbines) > 0:
                self.get_logger().info("Mission data ready.")
                self.state = MissionState.PLAN_APPROACH

        elif self.state == MissionState.PLAN_APPROACH:
            if not self.planner_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("Waiting for /aquabot/get_plan service...")
                return

            tx, ty = self.current_turbine_xy()
            entry_angle = math.atan2(self.boat_pose.position.y - ty,
                                     self.boat_pose.position.x - tx)
            entry_x = tx + self.orbit_radius * math.cos(entry_angle)
            entry_y = ty + self.orbit_radius * math.sin(entry_angle)

            req = GetPlan.Request()
            req.start.header.frame_id = "map"
            req.start.header.stamp = self.get_clock().now().to_msg()
            req.start.pose = self.boat_pose

            req.goal.header.frame_id = "map"
            req.goal.header.stamp = self.get_clock().now().to_msg()
            req.goal.pose.position.x = float(entry_x)
            req.goal.pose.position.y = float(entry_y)
            req.goal.pose.orientation.w = 1.0
            req.tolerance = 2.0

            self.get_logger().info(
                f"Planning approach to ring entry ({entry_x:.1f}, {entry_y:.1f}) "
                f"for turbine {self.current_turbine_index}.")
            future = self.planner_client.call_async(req)
            future.add_done_callback(self.plan_received_callback)
            self.state = MissionState.WAITING_FOR_PLAN

        elif self.state == MissionState.WAITING_FOR_PLAN:
            pass  # waiting on the async planner response

        elif self.state == MissionState.ORBITING:
            tx, ty = self.current_turbine_xy()

            # tell the QR node which turbine we're inspecting
            tgt = PoseStamped()
            tgt.header.frame_id = "world"
            tgt.header.stamp = self.get_clock().now().to_msg()
            tgt.pose.position.x = tx
            tgt.pose.position.y = ty
            tgt.pose.orientation.w = 1.0
            self.target_pub.publish(tgt)

            bx = self.boat_pose.position.x
            by = self.boat_pose.position.y
            dist_to_turbine = math.hypot(bx - tx, by - ty)

            # start counting laps only once we've actually reached the ring
            if not self.on_circle and abs(dist_to_turbine - self.orbit_radius) < self.reach_circle_tol:
                self.on_circle = True
                self.prev_angle = math.atan2(by - ty, bx - tx)
                self.get_logger().info("Reached inspection ring -- orbiting.")

            if self.on_circle:
                angle = math.atan2(by - ty, bx - tx)
                d = normalize_angle(angle - self.prev_angle)
                self.swept_angle += d * self.orbit_direction   # count motion in orbit direction
                self.prev_angle = angle

                if self.swept_angle >= math.radians(self.completion_arc_deg):
                    self.get_logger().info(
                        f"Completed orbit of turbine {self.current_turbine_index} "
                        f"({math.degrees(self.swept_angle):.0f} deg swept).")
                    self.advance_to_next_turbine()

        elif self.state == MissionState.MISSION_COMPLETE:
            if not self._completed_logged:
                self.get_logger().info("All turbines orbited. Mission complete.")
                self._completed_logged = True

    def advance_to_next_turbine(self):
        self.current_turbine_index += 1
        if self.current_turbine_index >= len(self.turbines):
            self.state = MissionState.MISSION_COMPLETE
        else:
            # Going straight to PLAN_APPROACH publishes the next turbine's path
            # before the controller reaches the old orbit's endpoint, so the
            # boat flows turbine-to-turbine without ever stopping.
            self.state = MissionState.PLAN_APPROACH


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()