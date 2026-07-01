#!/usr/bin/env python3
#
# Bare QR reader -- STEP 1 of the 4.5 challenge (prove decode works).
#
# This node does the minimum: subscribe to the camera, decode any QR code in
# frame with OpenCV's built-in detector, log what it found (and how big it was
# in the frame, so we learn the usable range), and republish the decoded text
# to the checkup topic.
#
# It does NOT aim the camera, compute facing, or talk to the mission yet.
# Drive the boat near a turbine with a 2D Goal Pose, point the QR into view,
# and confirm the decode fires with the right string. Aiming + facing + mission
# wiring come in step 2, once decode is proven.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Quaternion

import math
import cv2
import numpy as np
from cv_bridge import CvBridge


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class QRReader(Node):
    def __init__(self):
        super().__init__('qr_reader')
        self.get_logger().info("QR Reader online (bare decode mode).")

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        # remember what we've already reported so we don't spam the checkup topic
        self.seen = set()

        # Facing discovery: a flat panel looks squarest (aspect ~1.0) only from
        # the one bearing directly in front of it. We don't know where on the
        # turbine the panel is, so instead of guessing we track the squarest
        # sighting seen so far this lap -- whichever bearing produced it IS the
        # facing, regardless of which side of the tower it turned out to be on.
        # This updates even on sightings that FAIL to decode (blur, distance) --
        # a failed read still carries the geometry of where the boat was standing.
        self.boat_xy = None
        self.target_xy = None
        self.best_facing_score = float('inf')   # lower = squarer = better
        self.facing_gate = 0.5                   # |log(aspect)| must be under this to trust

        self.image_sub = self.create_subscription(
            Image,
            '/aquabot/sensors/cameras/main_camera_sensor/image_raw',
            self.image_callback,
            10)

        # boat pose + which turbine the mission is currently orbiting -- needed
        # to turn "the panel just looked square-on" into an actual bearing
        self.odom_sub = self.create_subscription(
            Odometry, '/aquabot/odom', self.odom_callback, 10)
        self.target_sub = self.create_subscription(
            PoseStamped, '/aquabot/current_target', self.target_callback, 10)

        self.checkup_pub = self.create_publisher(
            String, '/vrx/windturbinesinspection/windturbine_checkup', 10)
        self.facing_pub = self.create_publisher(
            PoseStamped, '/aquabot/qr_facing', 10)

        # throttle "nothing seen" logging so the console stays readable
        self._frames = 0

    def try_decode(self, gray):
        """
        Try several decode strategies, easiest/cheapest first. Returns
        (data, points, strategy_name) on the first success, else
        ('', points_or_None, None) -- keeping the last localization for logging.
        """
        last_points = None

        # 1. plain
        data, points, _ = self.detector.detectAndDecode(gray)
        if points is not None:
            last_points = points
        if data:
            return data, points, "plain"

        # 2. curved decoder -- REMOVED. detectAndDecodeCurved() has documented
        # segfault bugs in this OpenCV version range on certain inputs (a Python
        # try/except can't catch a real segfault -- it's a hard crash in the C++
        # code underneath, not a Python exception). The three remaining
        # strategies (plain / upscaled / binarized) all use the stable
        # detectAndDecode() path instead.

        # 2. upscale 2x -- helps when the code is small or the modules are blurry
        big = cv2.resize(gray, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        data, points, _ = self.detector.detectAndDecode(big)
        if data:
            return data, (points / 2.0 if points is not None else None), "upscaled"

        # 3. binarize (Otsu) -- crisp black/white modules for the decoder
        _, binimg = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        data, points, _ = self.detector.detectAndDecode(binimg)
        if points is not None:
            last_points = points
        if data:
            return data, points, "binarized"

        return '', last_points, None

    def odom_callback(self, msg):
        self.boat_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def target_callback(self, msg):
        new_xy = (msg.pose.position.x, msg.pose.position.y)
        # a new turbine to inspect -> forget the old best sighting and start fresh
        if self.target_xy is None or math.hypot(new_xy[0] - self.target_xy[0],
                                                  new_xy[1] - self.target_xy[1]) > 1.0:
            self.target_xy = new_xy
            self.best_facing_score = float('inf')
            self.get_logger().info(
                f"Now inspecting turbine at ({new_xy[0]:.1f}, {new_xy[1]:.1f}) "
                f"-- facing search reset.")

    def update_facing(self, aspect):
        """Called on every localized sighting, decoded or not. Keeps the
        squarest sighting seen so far this lap and publishes the boat->turbine
        bearing at that moment as the panel's outward-facing direction."""
        if self.boat_xy is None or self.target_xy is None or aspect <= 0.0:
            return

        score = abs(math.log(aspect))   # 0 == perfectly square, bigger == more oblique
        if score > self.facing_gate:
            return                       # too oblique to trust as "in front of the panel"
        if score >= self.best_facing_score:
            return                       # not an improvement on the best sighting so far

        self.best_facing_score = score
        tx, ty = self.target_xy
        bx, by = self.boat_xy
        facing = math.atan2(by - ty, bx - tx)   # turbine -> boat == panel's outward normal

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = tx
        msg.pose.position.y = ty
        msg.pose.orientation = yaw_to_quaternion(facing)
        self.facing_pub.publish(msg)

        self.get_logger().info(
            f"FACING update: panel faces {math.degrees(facing):.0f} deg "
            f"(aspect {aspect:.2f}, score {score:.2f}) -- published.")

    def image_callback(self, msg):
        self._frames += 1

        # encoding is rgb8 -> ask cv_bridge for rgb8, then hand a gray image to
        # the detector (QR decode is intensity-based, channel order irrelevant once gray)
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge failed: {e}")
            return

        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)

        data, points, strategy = self.try_decode(gray)

        if points is None:
            # nothing localized at all this frame
            if self._frames % 30 == 0:
                self.get_logger().info("...no QR in frame")
            return

        # report how much of the frame the code covers + its aspect (squareness)
        pts = points.reshape(-1, 2)
        w = float(np.ptp(pts[:, 0]))
        h = float(np.ptp(pts[:, 1]))
        frac_w = w / msg.width
        frac_h = h / msg.height
        aspect = (w / h) if h > 1e-3 else 0.0

        # Feed the facing tracker on EVERY localization, whether or not it
        # decoded -- a square-on failed read is still useful geometry.
        self.update_facing(aspect)


        if data:
            self.get_logger().info(
                f"DECODED [{strategy}]: '{data}'  |  size {w:.0f}x{h:.0f}px "
                f"({frac_w*100:.0f}%x{frac_h*100:.0f}%), aspect {aspect:.2f}")
            if data not in self.seen:
                self.seen.add(data)
                self.checkup_pub.publish(String(data=data))
                self.get_logger().info(
                    f"--> published to checkup topic. Total unique codes: {len(self.seen)}")
        else:
            # localized but no strategy decoded it. aspect far from 1.0 => too oblique.
            self.get_logger().info(
                f"QR seen but NOT decoded | size {w:.0f}x{h:.0f}px "
                f"({frac_w*100:.0f}%x{frac_h*100:.0f}%), aspect {aspect:.2f} "
                f"{'(too oblique - view more face-on)' if (aspect > 1.6 or aspect < 0.6) else '(near face-on, try closer/steadier)'}")


def main(args=None):
    rclpy.init(args=args)
    node = QRReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()