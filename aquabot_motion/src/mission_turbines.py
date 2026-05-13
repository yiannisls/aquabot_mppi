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

        self.turbine_sub = self.create_subscription(PoseArray, '/aquabot/turbines', self.turbine_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.path_pub = self.create_publisher(Path, '/aquabot/plan', 10)

        self.planner_client = self.create_client(GetPlan, '/aquabot/get_plan')

        self.timer = self.create_timer(0.1, self.state_machine_loop)

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
            req = GetPlan.Request()
            
            req.start.header.frame_id = "map"
            req.start.header.stamp = self.get_clock().now().to_msg()
            req.start.pose = self.boat_pose
            
            req.goal.header.frame_id = "map"
            req.goal.header.stamp = self.get_clock().now().to_msg()
            req.goal.pose = self.turbines[self.current_turbine_index]
            
            req.tolerance = 2.0

            future = self.planner_client.call_async(req)
            future.add_done_callback(self.plan_received_callback)
            
            self.state = MissionState.WAITING_FOR_PLAN

        elif self.state == MissionState.WAITING_FOR_PLAN:
            pass

        elif self.state == MissionState.FOLLOWING_PATH:
            target_pos = self.turbines[self.current_turbine_index].position
            dist = math.hypot(target_pos.x - self.boat_pose.position.x, 
                              target_pos.y - self.boat_pose.position.y)
            
            if dist < 2.0:
                self.get_logger().info(f"Turbine {self.current_turbine_index} reached. Initiating inspection.")
                self.state = MissionState.INSPECTING
                self.inspection_start_time = self.get_clock().now()

        elif self.state == MissionState.INSPECTING:
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.inspection_start_time).nanoseconds / 1e9
            
            if elapsed_time >= 5.0:
                self.get_logger().info(f"Inspection of turbine {self.current_turbine_index} complete.")
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