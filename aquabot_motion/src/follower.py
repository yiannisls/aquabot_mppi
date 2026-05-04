#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# Import the message types we need
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist

class PathFollower(Node):
    def __init__(self):
        # Name this node 'path_follower'
        super().__init__('path_follower')
        
        self.get_logger().info("Path Follower Node Started!")

        # store the path from the planner here once we receive it
        self.current_path = [] 
        # publisher, where 10 = queue size (holds up to 10 messages in line)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # sub to plan
        self.plan_sub = self.create_subscription(Path, '/aquabot/plan', self.plan_callback, 10)
        #sub to odomtery
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
    def plan_callback(self, msg):
            """save pose from rviz"""
            self.current_path = msg.poses
            self.get_logger().info(f"Received a new path with {len(self.current_path)} waypoints!")
            
    def get_yaw_from_quaternion(self, q):
        """ quaternions to a 2D yaw angle (in radians) """
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        
        # if we don't have a path yet, just sit still
        if not self.current_path:
            return

        # find out where the boat is right now
        boat_x = msg.pose.pose.position.x
        boat_y = msg.pose.pose.position.y
        
        # get the boat's heading (yaw)
        q = msg.pose.pose.orientation
        boat_yaw = self.get_yaw_from_quaternion(q)

        # find the Look-Ahead Point
        look_ahead_distance = 3.0  # meters ahead we want to target
        target_x = None
        target_y = None

        # loop through the path to find the first point that is far enough away
        for pose_stamped in self.current_path:
            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y
            dist = math.hypot(px - boat_x, py - boat_y)
            
            if dist >= look_ahead_distance:
                target_x = px
                target_y = py
                break
        
        # if no point is far enough, target the very last point in the path
        if target_x is None:
            last_pose = self.current_path[-1].pose.position
            target_x = last_pose.x
            target_y = last_pose.y
            
            # If we are really close to that last point, we reached the goal!
            if math.hypot(target_x - boat_x, target_y - boat_y) < 1.0:
                self.get_logger().info("Goal reached! Stopping.")
                self.current_path = []  # Clear the path
                
                # Publish a zero-velocity command to stop
                stop_cmd = Twist()
                self.cmd_pub.publish(stop_cmd)
                return

        # calculate the steering commands
        # find the angle from the boat directly to the target point
        angle_to_target = math.atan2(target_y - boat_y, target_x - boat_x)
        
        # Find the difference between where we are looking and where we want to look
        heading_error = angle_to_target - boat_yaw
        
        # Normalize the error to stay between -PI and PI
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 5. Send the commands to cmd_vel
        cmd = Twist()
        
        # Proportional controller for steering
        kp_steering = 1.5 
        cmd.angular.z = kp_steering * heading_error 
            
        self.cmd_pub.publish(cmd)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    
    # Keep the node alive and listening
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
