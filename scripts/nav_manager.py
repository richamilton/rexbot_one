#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from enum import Enum
import math

class ActionResult(Enum):
    NONE = -1
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6

REQUEST_ACCEPTED_TIMEOUT = 5.0  # seconds
NAVIGATION_RESULT_TIMEOUT = 120.0  # seconds



class NavManager(Node):
    def __init__(self):
        super().__init__("nav2_goal_service")
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available, initialization failed.')
            raise RuntimeError('navigate_to_pose action server not available')

    def handle_goal_request(self, x, y, yaw):
        # Create a goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        # Convert yaw to quaternion using proper formula
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        return self.navigate_to_goal(goal_msg)
    
    def navigate_to_goal(self, goal_msg):
        # NOTE:
        # 1. Send goal
        # 2. Wait for response
        # 3. Wait for result
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=REQUEST_ACCEPTED_TIMEOUT)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=NAVIGATION_RESULT_TIMEOUT)

        result = result_future.result()
        if result.status == ActionResult.STATUS_SUCCEEDED.value:
            self.get_logger().info("Goal succeeded!")
            return True
        else:
            self.get_logger().warn(f"Goal failed with status: {result}")
            return False

def navigate_to_goal(goal: dict) -> dict:
    node = NavManager()
    try:
        x = goal.get("x")
        y = goal.get("y")
        yaw = goal.get("yaw")
        success = node.handle_goal_request(x, y, yaw)
    finally:
        node.destroy_node()
    return {"success": success}

if __name__ == "__main__":
    rclpy.init()
    navigate_to_goal({'x': -1.0, 'y': 0.0, 'yaw': 0.0})
    rclpy.shutdown()