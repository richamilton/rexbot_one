#!/usr/bin/env python3
"""
Map Manager - Handle switching between floor maps

This node manages switching between ground floor and residential floor maps
as the robot moves between floors during delivery operations.
"""

import os
from typing import Dict, Optional
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty

class MapManager(Node):
    """
    Manages map switching for multi-floor navigation.
    
    Handles:
    - Loading ground floor vs residential floor maps
    - Updating map frame IDs
    - Resetting localization when switching maps
    - Publishing map change notifications
    """
    
    def __init__(self):
        super().__init__('map_manager')
        
        # Map configuration
        self.map_configs = {
            'ground_floor': {
                'map_file': 'ground_floor_map_save.yaml',
                'frame_id': 'map',
                'description': 'Lobby, reception, elevators, loading area'
            },
            'residential_floor': {
                'map_file': 'residential_floor_save.yaml', 
                'frame_id': 'map',
                'description': 'Standard residential floor layout (floors 1-4)'
            }
        }

        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        self.map_loader_client = self.create_client(LoadMap, '/map_server/load_map')
        if not self.map_loader_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('map_server/load_map service not available, initialization failed.')
            raise RuntimeError('map_server/load_map service not available')

        self.clear_costmaps_client = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        if not self.clear_costmaps_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('global_costmap/clear_entirely_global_costmap service not available, initialization failed.')
            raise RuntimeError('global_costmap/clear_entirely_global_costmap service not available')

        self.clear_local_costmaps_client = self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        if not self.clear_local_costmaps_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('local_costmap/clear_entirely_local_costmap service not available, initialization failed.')
            raise RuntimeError('local_costmap/clear_entirely_local_costmap service not available')

        self.get_logger().info("Map manager initialized")

    def setup_map(self, map_name: str, initial_pose: Dict[str, float]):
        """Handle map switch requests"""
        requested_map = map_name.lower()

        self.get_logger().info(f"Map switch request: {requested_map}")
        
        # TODO: Check if already on requested map
        # if requested_map == self.current_map:
        #     self.get_logger().info(f"Already using map: {requested_map}")
        #     return True
        
        if requested_map not in self.map_configs.keys():
            self.get_logger().info(f"Unknown map: {requested_map}")
            return False

        map_path = self.map_configs[requested_map]['map_file']

        if not self.load_map(map_path):
            return False

        if not self.clear_costmaps():
            return False

        if not self.set_initial_pose(requested_map, initial_pose):
            return False
        
        self.get_logger().info(f"Switched to map: {requested_map}")
        return True

    def load_map(self, map_path: str):
        """Load a specific map using nav2 map server"""
        try:
            # Create load map request
            request = LoadMap.Request()
            request.map_url = map_path
            
            self.get_logger().info(f"Loading map: {map_path}")
            
            future = self.map_loader_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            if future._result.result == LoadMap.Response.RESULT_SUCCESS:
                self.get_logger().info(f"Map loaded successfully: {map_path}")
                return True
            else:
                self.get_logger().error(f"Failed to load map: {map_path}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error loading map: {e}")
            return False
    
    def clear_costmaps(self):
        """Clear global and local costmaps after map change"""
        try:
            # Clear global costmap
            request = ClearEntireCostmap.Request()
            future = self.clear_costmaps_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

            # Clear local costmap
            future = self.clear_local_costmaps_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
            self.get_logger().info("Cleared costmaps")
            return True
            
        except Exception as e:
            self.get_logger().warning(f"Failed to clear costmaps: {e}")
            return False

    def set_initial_pose(self, map_name: str, initial_pose: Dict[str, float]):
        """Set initial pose for localization after map switch"""
        if map_name not in self.map_configs:
            return
            
        config = self.map_configs[map_name]
        
        try:
            # Create initial pose message
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = config['frame_id']
            
            # Set position
            pose_msg.pose.pose.position.x = float(initial_pose['x'])
            pose_msg.pose.pose.position.y = float(initial_pose['y'])
            pose_msg.pose.pose.position.z = 0.0
            
            # Set orientation (from yaw)
            import math
            yaw = float(initial_pose.get('yaw', 0.0))
            pose_msg.pose.pose.orientation.x = 0.0
            pose_msg.pose.pose.orientation.y = 0.0
            pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Set covariance (diagonal matrix with reasonable uncertainties)
            covariance = [0.0] * 36  # 6x6 matrix flattened
            covariance[0] = 0.25   # x variance
            covariance[7] = 0.25   # y variance  
            covariance[35] = 0.07  # yaw variance
            pose_msg.pose.covariance = covariance

            self.initial_pose_pub.publish(pose_msg)

            self.get_logger().info(
                f"Set initial pose for {map_name}: "
                f"({initial_pose['x']:.1f}, {initial_pose['y']:.1f}, {initial_pose['yaw']:.2f})"
            )

            return True
            
        except Exception as e:
            self.get_logger().error(f"Failed to set initial pose: {e}")
            return False

    def _get_map_path(self, map_name: str) -> Optional[str]:
        """Get full path to map file"""
        maps_dir = self.get_parameter('maps_directory').value
        
        if map_name in self.map_configs:
            map_file = self.map_configs[map_name]['map_file']
            # map_path = os.path.join(maps_dir, map_file)
            map_path = map_file
            
            if os.path.exists(map_path):
                return map_path
            else:
                self.get_logger().warning(f"Map file not found: {map_path}")
                return None
        else:
            self.get_logger().error(f"Unknown map: {map_name}")
            return None


def setup_map(map_name: str, initial_pose: Dict[str, float]):
    """Convenience function to call the clear costmap service once and return the response."""
    client = MapManager()
    response = client.setup_map(map_name, initial_pose)
    client.get_logger().info(f'Result of setup map: {response}')
    client.destroy_node()
    return response


if __name__ == '__main__':
    rclpy.init()
    setup_map("ground_floor", {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
    rclpy.shutdown()