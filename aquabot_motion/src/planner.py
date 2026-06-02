#!/usr/bin/env python3

from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import TransformListener, Buffer
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from astar import Astar
import bitangents as bt
from numpy import cos, sin, arctan2

radius = 10
me = 'aquabot/base_link'


def distance2(p1, p2):
    return (p1.x - p2.x)**2 + (p1.y - p2.y)**2


class Planner(Node):

    def __init__(self):
        super().__init__('planner')
        param = Parameter('use_sim_time', Parameter.Type.BOOL, True)
        self.set_parameters([param])

        self.path_pub = self.create_publisher(Path, 'plan', 1)
        self.planner = Astar()
        obstacles = (
            (120, -50, 35),  # aquabot_lighthouse_island
            (-152, -6, 55),  # aquabot_island_1
            (110, 135, 50),  # aquabot_island_2
            (12, -102, 30),  # aquabot_rock_island_0
            (92, 170, 30),  # aquabot_rock_island_1
            (-92, 176, 40),  # aquabot_rock_0
            (-40, 220, 20),  # aquabot_rock_1
            (-44, -95, 25),  # aquabot_rock_2
            (-30, -150, 27)  # aquabot_rock_3
            )

        for x,y,rad in obstacles:
            self.planner.add_obstacle(x,y,rad)
        self.N = len(self.planner.no_go_zones)
        self.planner.init_graph()

        self.sub = self.create_subscription(PoseArray, 'turbines', self.add_turbines, 10)
        self.turbines = []

        self.marker_pub = self.create_publisher(MarkerArray, 'obstacles', 1)

        # also allow setGoal from RViz
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node = self)
        self.goal_pub = self.create_subscription(PoseStamped, '/aquabot/goal_pose', self.goal_cb, 1)

    def add_turbines(self, msg: PoseArray):

        if len(self.planner.no_go_zones) - self.N == len(msg.poses):
            # no new turbines
            return

        for pose in msg.poses:
            self.planner.add_obstacle(pose.position.x, pose.position.y, radius)

            for turb in self.turbines:
                if distance2(turb, pose.position) < 10:
                    break
            else:
                self.turbines.append(bt.Point(pose.position.x, pose.position.y))
        self.planner.init_graph()
        self.pub_markers()
        # we cannot have this service before getting the turbines
        self.srv = self.create_service(GetPlan, 'get_plan', self.plan_cb)

    def pub_markers(self):

        markers = MarkerArray()

        now = rclpy.time.Time()
        inspected = None

        if self.buffer.can_transform('world', 'pinger', now):
            tf = self.buffer.lookup_transform('world', 'pinger', now).transform

            def distTF(turbine):
                return (turbine.x-tf.translation.x)**2 + (turbine.y-tf.translation.y)**2

            inspected = min(self.turbines, key = distTF)

            if distTF(inspected) > 400:
                inspected = None

        for i,obs in enumerate(self.planner.no_go_zones):

            rock = isinstance(obs.x, int)

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = now.to_msg()
            marker.id = i
            marker.type = marker.CYLINDER
            marker.action = marker.ADD
            marker.pose.position.x = float(obs.x)
            marker.pose.position.y = float(obs.y)
            marker.scale.x = marker.scale.y = 2.*(obs.rad if rock else 3.)
            marker.scale.z = 1. if rock else 40.
            marker.pose.position.z = marker.scale.z/2
            marker.color.r = marker.color.g = marker.color.b = 0.2 if rock else 1.

            if inspected is not None and obs.x == inspected.x and obs.y == inspected.y:
                # second phase, display the relevant turbine
                marker.color.g = marker.color.b = 0.05
            marker.color.a = 1.
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

    def plan_cb(self, req: GetPlan.Request, res: GetPlan.Response):

        self.get_logger().info(f'Got plan request ({req.start.pose.position.x,req.start.pose.position.y}) -> ({req.goal.pose.position.x,req.goal.pose.position.y})')

        # XY path
        plan = self.get_plan(req.start.pose.position.x, req.start.pose.position.y,
                                     req.goal.pose.position.x, req.goal.pose.position.y)
        if plan is None:
            self.get_logger().warn('   could not get plan')
            return

        res.plan = plan
        self.get_logger().info('    plan computed')
        return res

    def goal_cb(self, msg: PoseStamped):
        now = rclpy.time.Time()
        if not self.buffer.can_transform('world', me, now):
            return
        start = self.buffer.lookup_transform('world', me, now).transform.translation
        goal = msg.pose.position
        self.get_plan(start.x, start.y, goal.x, goal.y)

    def get_plan(self, x1, y1, x2, y2):

        # XY path
        path2D = self.planner.set_task(x1, y1, x2, y2, discretize=True)

        if path2D is None:
            return

        # to PoseStamped
        path = Path()

        path.poses = []
        for i,p in enumerate(path2D):
            pose = PoseStamped()
            pose.header.frame_id = 'world'
            pose.pose.position.x = p.x
            pose.pose.position.y = p.y

            if i not in (0, len(path2D)-1):
                pn = path2D[i+1]
                pp =  path2D[i-1]
                theta = arctan2(pn.y-pp.y, pn.x - pp.x)
                pose.pose.orientation.w = cos(theta/2)
                pose.pose.orientation.z = sin(theta/2)
            if i == 1:
                path.poses[0].pose.orientation = pose.pose.orientation
            elif i == len(path2D)-1:
                pose.pose.orientation = path.poses[-1].pose.orientation

            path.poses.append(pose)
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'world'

        self.path_pub.publish(path)
        return path


rclpy.init()
rclpy.spin(Planner())
rclpy.shutdown()
