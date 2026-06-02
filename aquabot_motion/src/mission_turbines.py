#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from enum import Enum

from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan

class MissionState(Enum):
    WAITING_FOR_DATA = 0
    PLANNING_PATH = 1
    WAITING_FOR_PLAN = 2
    FOLLOWING_PATH = 3
    INSPECTING = 4
    MISSION_COMPLETE = 5

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.get_logger().info("Mission Control Online!")

        self.state = MissionState.WAITING_FOR_DATA

        self.turbines = []
        self.current_turbine_index = 0
        self.boat_pose = None
        self.inspection_start_time = None
        self.current_sub_waypoints = []
        self.active_target_x = None
        self.active_target_y = None

        self.turbine_sub = self.create_subscription(PoseArray, '/aquabot/turbines', self.turbine_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/aquabot/odom', self.odom_callback, 10)

        self.path_pub = self.create_publisher(Path, '/aquabot/plan', 10)

        self.planner_client = self.create_client(GetPlan, '/aquabot/get_plan')

        self.timer = self.create_timer(0.1, self.state_machine_loop)

    def generate_inspection_circle(self, center_x, center_y, radius=15.0, num_points=8):
        """Generates a list of (x, y) tuples forming a circle around a target."""
        circle_waypoints = []
        for i in range(num_points):
            # Calculate the angle for this specific point
            angle = (2.0 * math.pi / num_points) * i
            
            # Use trigonometry to find the X and Y at that angle
            point_x = center_x + (radius * math.cos(angle))
            point_y = center_y + (radius * math.sin(angle))
            
            circle_waypoints.append((point_x, point_y))
            
        return circle_waypoints

    def turbine_callback(self, msg):
        if len(self.turbines) == 0:
            self.turbines = msg.poses
            self.get_logger().info(f"Received {len(self.turbines)} turbine locations!")

    def odom_callback(self, msg):
        self.boat_pose = msg.pose.pose

    def plan_received_callback(self, future):
        response = future.result()
        
        if len(response.plan.poses) > 0:
            self.get_logger().info("Path received! Sending to controller...")
            self.path_pub.publish(response.plan)
            self.state = MissionState.FOLLOWING_PATH
        else:
            self.get_logger().error("Planner failed to find a path!")
            self.state = MissionState.PLANNING_PATH 

    def state_machine_loop(self):
        if self.state == MissionState.WAITING_FOR_DATA:
            if self.boat_pose is not None and len(self.turbines) > 0:
                self.get_logger().info("Received mission data.")
                self.state = MissionState.PLANNING_PATH

        elif self.state == MissionState.PLANNING_PATH:
            if not self.planner_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("Waiting for /aquabot/get_plan service to be available...")
                return
            
            if len(self.current_sub_waypoints) == 0:
                tx = self.turbines[self.current_turbine_index].position.x
                ty = self.turbines[self.current_turbine_index].position.y
                self.current_sub_waypoints = self.generate_inspection_circle(tx, ty, radius=15.0, num_points=8)

            target_x, target_y = self.current_sub_waypoints.pop(0)
            self.active_target_x = target_x
            self.active_target_y = target_y

            self.get_logger().info(f"Requesting path to waypoint ({target_x:.2f}, {target_y:.2f}) around turbine {self.current_turbine_index}...")

            req = GetPlan.Request()
            
            req.start.header.frame_id = "map"
            req.start.header.stamp = self.get_clock().now().to_msg()
            req.start.pose = self.boat_pose
            
            req.goal.header.frame_id = "map"
            req.goal.header.stamp = self.get_clock().now().to_msg()
            req.goal.pose.position.x = float(target_x)
            req.goal.pose.position.y = float(target_y)
            req.goal.pose.orientation.w = 1.0  # Make it a valid unrotated quaternion
            
            req.tolerance = 2.0

            future = self.planner_client.call_async(req)
            future.add_done_callback(self.plan_received_callback)
            
            self.state = MissionState.WAITING_FOR_PLAN

        elif self.state == MissionState.WAITING_FOR_PLAN:
            pass

        elif self.state == MissionState.FOLLOWING_PATH:
            dist = math.hypot(self.active_target_x - self.boat_pose.position.x, 
                              self.active_target_y - self.boat_pose.position.y)
            
            if dist < 2.0:
                self.get_logger().info(f"Waypoint reached. Initiating inspection scan...")
                self.state = MissionState.INSPECTING
                self.inspection_start_time = self.get_clock().now()

        elif self.state == MissionState.INSPECTING:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.inspection_start_time).nanoseconds / 1e9
            
            if elapsed_time >= 5.0:
                if len(self.current_sub_waypoints) > 0:
                    self.get_logger().info("Inspection complete. Moving to next waypoint.")
                    self.state = MissionState.PLANNING_PATH
                else:
                    self.get_logger().info(f"Finished circular inspection of turbine {self.current_turbine_index}.")
                    self.current_turbine_index += 1
                    
                    if self.current_turbine_index >= len(self.turbines):
                        self.state = MissionState.MISSION_COMPLETE
                    else:
                        self.state = MissionState.PLANNING_PATH
            
        elif self.state == MissionState.MISSION_COMPLETE:
            self.get_logger().info("All turbines inspected. Mission complete.")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()