#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from enum import Enum

# Import messages and services
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry, Path
from nav_msgs.srv import GetPlan

# 1. Define our States
class MissionState(Enum):
    WAITING_FOR_DATA = 0
    PLANNING_PATH = 1
    WAITING_FOR_PLAN == 2
    FOLLOWING_PATH = 3
    INSPECTING = 4
    MISSION_COMPLETE = 5

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.get_logger().info("Mission Control Online!")

        # Start in the WAITING state
        self.state = MissionState.WAITING_FOR_DATA

        # Mission Memory
        self.turbines = []
        self.current_turbine_index = 0
        self.boat_pose = None

        # --- Subscribers ---
        # The gz2ros node handles the raw data and publishes the metric positions of the turbines here [cite: 58, 67]
        self.turbine_sub = self.create_subscription(PoseArray, '/aquabot/turbines', self.turbine_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # --- Service Client ---
        # Instead of subscribing, we call the planner service to get a path on demand 
        self.planner_client = self.create_client(GetPlan, '/aquabot/get_plan')

        # --- The Heartbeat ---
        # Run the state machine loop every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.state_machine_loop)

    # --- Callbacks to update our memory ---
    def turbine_callback(self, msg):
        """ Save the list of wind turbines if we don't have them yet """
        if len(self.turbines) == 0:
            self.turbines = msg.poses
            self.get_logger().info(f"Received {len(self.turbines)} turbine locations!")

    def odom_callback(self, msg):
        """ Always keep track of where the boat is """
        self.boat_pose = msg.pose.pose

    def plan_received_callback(self, future):
        """ This triggers automatically when the planner finally answers our request """
        response = future.result()
        
        if len(response.plan.poses) > 0:
            self.get_logger().info("Path received! Sending to follower...")
            
            # We need a publisher to send this path to the follower node
            # (We will create this publisher in the __init__ next)
            self.path_pub.publish(response.plan)
            
            # Shift into driving mode!
            self.state = MissionState.FOLLOWING_PATH
        else:
            self.get_logger().error("Planner failed to find a path!")
            self.state = MissionState.PLANNING_PATH # Try again
    # --- The Brain ---
    def state_machine_loop(self):
        """ This function runs 10 times a second and controls the mission flow """
        
        if self.state == MissionState.WAITING_FOR_DATA:
            """ Check if we have both the boat's position and the turbine list.
            if we do change state 0->1 """
            if self.boat_pose is not None and len(self.turbines) > 0:
                self.get_logger().info("got mission data")
                self.state = MissionState.PLANNING_PATH
            if self.state == MissionState.WAITING_FOR_DATA:


        elif self.state == MissionState.PLANNING_PATH:

            req = GetPlan.Request()

            req.start.h
            # TODO: Send a request to the /aquabot/get_plan service.
            # Start = boat_pose, Goal = turbines[current_turbine_index].
            # Once we get the path, change state to FOLLOWING_PATH.
            pass

        elif self.state == MissionState.FOLLOWING_PATH:
            # TODO: Monitor the distance between the boat and the current turbine.
            # If distance < 2.0 meters, change state to INSPECTING.
            pass

        elif self.state == MissionState.INSPECTING:
            # TODO: Perform a small mission (e.g., wait 5 seconds).
            # Then, cross this turbine off the list (current_turbine_index += 1).
            # Check if we are done with all turbines. If yes, MISSION_COMPLETE. Else, PLANNING_PATH.
            pass
            
        elif self.state == MissionState.MISSION_COMPLETE:
            self.get_logger().info("All turbines inspected. Returning to base or idling!")
            # Stop the timer so we don't spam the logs
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
