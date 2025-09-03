#!/usr/bin/env python3
"""
set_initial_pose_once.py - Set initial pose for AMCL localization and exit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math

class InitialPoseSetterOnce(Node):
    def __init__(self):
        super().__init__('initial_pose_setter_once')
        
        # Publisher for initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Get parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0) 
        self.declare_parameter('yaw', 0.0)
        
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        
        self.get_logger().info(f'Setting initial pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        
        # Set the initial pose immediately
        self.set_initial_pose(x, y, yaw)
        
        # Set a timer to exit after a short delay
        self.create_timer(1.0, self.shutdown_node)
        
    def set_initial_pose(self, x, y, yaw):
        """Set the initial pose for AMCL"""
        pose_msg = PoseWithCovarianceStamped()
        
        # Header
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Orientation (yaw to quaternion)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Covariance matrix (6x6 = 36 elements)
        # Small values indicate good initial estimate confidence
        covariance = [0.0] * 36
        covariance[0] = 0.25   # x variance (0.5m std dev)
        covariance[7] = 0.25   # y variance (0.5m std dev)
        covariance[35] = 0.06854  # yaw variance (~15 degrees std dev)
        pose_msg.pose.covariance = covariance
        
        # Publish the pose multiple times to ensure AMCL receives it
        for i in range(5):
            self.initial_pose_pub.publish(pose_msg)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info(f'✅ Initial pose set successfully: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        
    def shutdown_node(self):
        """Shutdown the node after setting the pose"""
        self.get_logger().info('Initial pose setter shutting down')
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitialPoseSetterOnce()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()