#!/usr/bin/env python3
"""
Delivery Coordinator - Implements the practical delivery workflow

This node coordinates the entire delivery process:
1. Parse unit ID -> floor and unit
2. Navigate to loading-zone  
3. Wait for loading complete
4. Navigate to lobby-elevator
5. Take elevator (with elevator detection/entry)
6. Detect arrival at floor
7. Exit elevator
8. Navigate to unit
"""

import re
import time
import yaml
from enum import Enum
from typing import Dict, Optional, Tuple
import uuid
import os
import signal
import subprocess
import sys


import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String, Int32
from std_srvs.srv import Empty
from ament_index_python.packages import get_package_share_directory

# Would import our custom services in real implementation:
from rexbot_one.srv import DeliverToUnit, LoadingComplete

class DeliveryState(Enum):
    """States in the delivery workflow"""
    IDLE = "idle"
    PARSING_UNIT = "parsing_unit"  
    NAVIGATING_TO_LOADING = "navigating_to_loading"
    WAITING_FOR_LOADING = "waiting_for_loading"
    NAVIGATING_TO_ELEVATOR = "navigating_to_elevator"
    WAITING_FOR_ELEVATOR = "waiting_for_elevator"
    ENTERING_ELEVATOR = "entering_elevator"
    RIDING_ELEVATOR = "riding_elevator"
    EXITING_ELEVATOR = "exiting_elevator"
    NAVIGATING_TO_UNIT = "navigating_to_unit"
    DELIVERY_COMPLETE = "delivery_complete"
    ERROR_STATE = "error"


class DeliveryCoordinator(Node):
    """
    Coordinates the complete delivery workflow using simple goal pose publishing.
    
    Handles unit parsing, map switching, and sequential navigation goals.
    """
    
    def __init__(self):
        # Add unique timestamp to node name to prevent conflicts
        timestamp = int(time.time() * 1000) % 10000  # Last 4 digits of timestamp
        node_name = f'delivery_coordinator_{timestamp}'
        super().__init__(node_name)

        # Current delivery state
        self.current_state = DeliveryState.IDLE
        self.current_delivery_id = None
        self.target_floor = None
        self.target_unit = None
        
        # Load location coordinates
        self.locations = {}
        self.load_location_config()
        
        # ROS2 parameters
        self.declare_parameter('location_config', 'location_coordinates.yaml')
        self.declare_parameter('navigation_timeout', 60.0)
        self.declare_parameter('loading_timeout', 600.0)
        
        # Navigation action client (nav2)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers for status and map switching
        self.state_publisher = self.create_publisher(String, 'delivery_state', 10)
        self.map_switch_publisher = self.create_publisher(String, 'switch_map', 10)
        
        # Subscribers for elevator and loading signals
        self.create_subscription(String, 'elevator_status', self.elevator_status_callback, 10)
        self.create_subscription(Int32, 'current_floor', self.floor_detection_callback, 10)
        
        # Service servers
        self.deliver_service = self.create_service(
            DeliverToUnit, 'deliver_to_unit', self.deliver_to_unit_callback
        )
        self.loading_service = self.create_service(
            LoadingComplete, 'loading_complete', self.loading_complete_callback
        )
        
        # State tracking
        self.current_floor = 0  # Start on ground floor
        self.elevator_door_open = False
        self.loading_complete = False
        
        self.get_logger().info("Delivery coordinator initialized")
        
    def load_location_config(self):
        """Load predefined location coordinates from YAML config"""
        try:            
            try:
                # Try to get package path (for installed package)
                package_share = get_package_share_directory('rexbot_one')
                config_path = os.path.join(package_share, 'config', 'location_coordinates.yaml')
            except:
                # Fallback to development workspace path
                config_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'location_coordinates.yaml')
            
            with open(config_path, 'r') as f:
                self.locations = yaml.safe_load(f)
            
            self.get_logger().info(f"Loaded location coordinates from {config_path}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load location config: {e}")
            self.locations = {}

        self.get_logger().info(f"Loaded location coordinates: {yaml.dump(self.locations, default_flow_style=False)}")
    
    def parse_unit_id(self, unit_id: str) -> Tuple[Optional[int], Optional[int]]:
        """
        Parse unit ID to extract floor and unit number.
        
        Examples:
        - "0411" -> floor 4, unit 11
        - "0205" -> floor 2, unit 5  
        - "1022" -> floor 10, unit 22
        
        Returns:
            (floor, unit) or (None, None) if parsing fails
        """
        try:
            # Remove any non-digit characters
            digits_only = re.sub(r'[^\d]', '', unit_id)
            
            if len(digits_only) < 3:
                self.get_logger().error(f"Unit ID too short: {unit_id}")
                return None, None
            
            # Standard format: FFUU where FF=floor, UU=unit
            if len(digits_only) == 4:
                floor = int(digits_only[:2])  
                unit = int(digits_only[2:])
            elif len(digits_only) == 3:
                # Format: FUU where F=floor, UU=unit  
                floor = int(digits_only[0])
                unit = int(digits_only[1:])
            else:
                self.get_logger().error(f"Cannot parse unit ID format: {unit_id}")
                return None, None
            
            # Validation
            if floor < 1 or floor > 20:  # Reasonable floor limits
                self.get_logger().error(f"Invalid floor number: {floor}")
                return None, None
                
            if unit < 1 or unit > 50:  # Reasonable unit limits
                self.get_logger().error(f"Invalid unit number: {unit}")
                return None, None
            
            self.get_logger().info(f"Parsed '{unit_id}' -> Floor {floor}, Unit {unit}")
            return floor, unit
            
        except ValueError as e:
            self.get_logger().error(f"Unit ID parsing error: {e}")
            return None, None
    
    def create_goal_pose(self, location_key: str, floor_type: str = 'ground_floor') -> Optional[PoseStamped]:
        """Create a PoseStamped goal from location configuration"""
        try:
            if floor_type == 'ground_floor':
                location_data = self.locations['ground_floor'].get(location_key)
            else:
                # Residential floor locations
                if location_key.startswith('unit_'):
                    location_data = self.locations['residential_floors']['units'].get(location_key)
                else:
                    location_data = self.locations['residential_floors'].get(location_key)
            
            if not location_data:
                self.get_logger().error(f"Location '{location_key}' not found in config")
                return None
            
            goal = PoseStamped()
            goal.header.frame_id = 'map'  # Will be updated based on current map
            goal.header.stamp = self.get_clock().now().to_msg()
            
            goal.pose.position.x = float(location_data['x'])
            goal.pose.position.y = float(location_data['y'])
            goal.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            yaw = float(location_data.get('yaw', 0.0))
            goal.pose.orientation.z = (yaw / 2.0) ** 0.5
            goal.pose.orientation.w = (1.0 - (yaw / 2.0) ** 2) ** 0.5
            
            return goal
            
        except Exception as e:
            self.get_logger().error(f"Failed to create goal pose for {location_key}: {e}")
            return None
    
    def publish_nav_goal(self, goal_pose: PoseStamped) -> bool:
        """Publish navigation goal to nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation server not available")
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(f"Sending navigation goal: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")
        
        future = self.nav_client.send_goal_async(goal_msg)
        
        # In a full implementation, we'd properly handle the async result
        # For now, just return True to indicate goal was sent
        return True
    
    def switch_to_ground_floor_map(self):
        """Switch to ground floor map"""
        self.get_logger().info("Switching to ground floor map")
        map_msg = String()
        map_msg.data = "ground_floor"
        self.map_switch_publisher.publish(map_msg)
    
    def switch_to_residential_floor_map(self, floor: int):
        """Switch to residential floor map"""
        self.get_logger().info(f"Switching to residential floor map (floor {floor})")
        map_msg = String()
        map_msg.data = f"residential_floor_{floor}"
        self.map_switch_publisher.publish(map_msg)
    
    def update_state(self, new_state: DeliveryState):
        """Update delivery state and publish status"""
        self.current_state = new_state
        state_msg = String()
        state_msg.data = new_state.value
        self.state_publisher.publish(state_msg)
        self.get_logger().info(f"State: {new_state.value}")
    
    def start_delivery(self, unit_id: str) -> Dict:
        """
        Start the delivery workflow for a given unit.
        
        This implements your 9-step workflow:
        1. Parse unit ID
        2. Navigate to loading-zone  
        3. Wait for loading complete
        4. Navigate to lobby-elevator
        5. Take elevator
        6. Detect floor arrival
        7. Exit elevator  
        8. Navigate to unit
        """
        try:
            # Generate unique delivery ID
            delivery_id = str(uuid.uuid4())[:8]
            self.current_delivery_id = delivery_id
            
            self.get_logger().info(f"Starting delivery {delivery_id} to unit {unit_id}")
            
            # Step 1: Parse unit ID
            self.update_state(DeliveryState.PARSING_UNIT)
            floor, unit = self.parse_unit_id(unit_id)
            
            if floor is None or unit is None:
                return {"success": False, "error": f"Invalid unit ID: {unit_id}"}
            
            self.target_floor = floor
            self.target_unit = unit
            
            # Step 2: Navigate to loading-zone
            self.update_state(DeliveryState.NAVIGATING_TO_LOADING)
            self.switch_to_ground_floor_map()  # Ensure we're on ground floor map
            
            loading_goal = self.create_goal_pose('loading-zone', 'ground_floor')
            if not loading_goal:
                return {"success": False, "error": "Cannot create loading zone goal"}
            
            if not self.publish_nav_goal(loading_goal):
                return {"success": False, "error": "Failed to send loading navigation goal"}
            
            # Step 3: Wait for loading complete (this will be handled by service callback)
            self.update_state(DeliveryState.WAITING_FOR_LOADING)
            
            return {
                "success": True,
                "delivery_id": delivery_id,
                "target_floor": floor,
                "target_unit": unit,
                "estimated_duration": 300.0  # 5 minutes estimate
            }
            
        except Exception as e:
            self.get_logger().error(f"Failed to start delivery: {e}")
            self.update_state(DeliveryState.ERROR_STATE)
            return {"success": False, "error": str(e)}
    
    def continue_after_loading(self):
        """Continue delivery workflow after loading is complete"""
        if self.current_state != DeliveryState.WAITING_FOR_LOADING:
            self.get_logger().warning("Loading complete signal received in wrong state")
            return
        
        try:
            # Step 4: Navigate to lobby-elevator
            self.update_state(DeliveryState.NAVIGATING_TO_ELEVATOR)
            
            elevator_goal = self.create_goal_pose('lobby-elevator', 'ground_floor')
            if not elevator_goal:
                self.get_logger().error("Cannot create elevator goal")
                self.update_state(DeliveryState.ERROR_STATE)
                return
            
            if not self.publish_nav_goal(elevator_goal):
                self.get_logger().error("Failed to send elevator navigation goal")
                self.update_state(DeliveryState.ERROR_STATE)
                return
            
            # Steps 5-7 will be handled by elevator detection callbacks
            self.update_state(DeliveryState.WAITING_FOR_ELEVATOR)
            
        except Exception as e:
            self.get_logger().error(f"Failed to continue after loading: {e}")
            self.update_state(DeliveryState.ERROR_STATE)
    
    def handle_elevator_arrival_at_floor(self):
        """Handle arrival at target floor (Step 8)"""
        if self.current_floor != self.target_floor:
            return  # Not at target floor yet
        
        try:
            # Step 8: Exit elevator and navigate to unit
            self.update_state(DeliveryState.EXITING_ELEVATOR)
            
            # Switch to residential floor map
            self.switch_to_residential_floor_map(self.target_floor)
            
            # Wait a moment for map switch
            time.sleep(2.0)
            
            # Navigate to unit
            self.update_state(DeliveryState.NAVIGATING_TO_UNIT)
            unit_key = f"unit_{self.target_unit:02d}"  # Format as unit_01, unit_22, etc.
            
            unit_goal = self.create_goal_pose(unit_key, 'residential_floor')
            if not unit_goal:
                self.get_logger().error(f"Cannot create goal for {unit_key}")
                self.update_state(DeliveryState.ERROR_STATE)
                return
            
            if not self.publish_nav_goal(unit_goal):
                self.get_logger().error("Failed to send unit navigation goal")
                self.update_state(DeliveryState.ERROR_STATE)
                return
            
            # Delivery will be complete when nav goal is reached
            self.get_logger().info(f"Navigating to unit {self.target_unit} on floor {self.target_floor}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to handle elevator arrival: {e}")
            self.update_state(DeliveryState.ERROR_STATE)
    
    def complete_delivery(self):
        """Mark delivery as complete and return to home base"""
        self.update_state(DeliveryState.DELIVERY_COMPLETE)
        
        self.get_logger().info(f"Delivery {self.current_delivery_id} completed successfully!")
        
        # Optional: Return to home base
        # This would involve taking elevator back to ground floor and navigating to home-base
        
        # Reset state
        self.current_delivery_id = None
        self.target_floor = None
        self.target_unit = None
        self.update_state(DeliveryState.IDLE)
    
    # Callback functions
    def elevator_status_callback(self, msg):
        """Handle elevator status updates"""
        status = msg.data.lower()
        
        if status == "doors_open":
            self.elevator_door_open = True
            if self.current_state == DeliveryState.WAITING_FOR_ELEVATOR:
                self.update_state(DeliveryState.ENTERING_ELEVATOR)
            elif self.current_state == DeliveryState.RIDING_ELEVATOR:
                self.handle_elevator_arrival_at_floor()
                
        elif status == "doors_closed":
            self.elevator_door_open = False
            if self.current_state == DeliveryState.ENTERING_ELEVATOR:
                self.update_state(DeliveryState.RIDING_ELEVATOR)
    
    def floor_detection_callback(self, msg):
        """Handle floor detection updates"""
        self.current_floor = msg.data
        self.get_logger().info(f"Detected floor: {self.current_floor}")
        
        if (self.current_state == DeliveryState.RIDING_ELEVATOR and 
            self.current_floor == self.target_floor and 
            self.elevator_door_open):
            self.handle_elevator_arrival_at_floor()
    
    # Service callbacks (would be uncommented in real implementation)
    def deliver_to_unit_callback(self, request, response):
        """Service callback to start delivery"""
        result = self.start_delivery(request.unit_id)
        
        response.success = result["success"]
        response.error_message = result.get("error", "")
        if result["success"]:
            response.target_floor = result["target_floor"]
            response.target_unit = result["target_unit"]
            response.estimated_duration = result["estimated_duration"]
            response.delivery_id = result["delivery_id"]
        
        return response
    
    def loading_complete_callback(self, request, response):
        """Service callback for loading completion"""
        if request.delivery_id == self.current_delivery_id and request.loading_confirmed:
            self.continue_after_loading()
            response.proceed_to_delivery = True
            response.instructions = f"Proceeding to floor {self.target_floor}, unit {self.target_unit}"
        else:
            response.proceed_to_delivery = False
            response.instructions = "Loading not confirmed or delivery ID mismatch"
        
        return response

def main(args=None):
    """Main entry point for delivery coordinator"""
    
    coordinator = None
    rclpy.init(args=args)
    
    try:
        coordinator = DeliveryCoordinator()
        coordinator.get_logger().info("Delivery coordinator ready for service calls")
        coordinator.get_logger().info("Example usage:")
        coordinator.get_logger().info("  ros2 service call /deliver_to_unit rexbot_one/DeliverToUnit \"{unit_id: '0411'}\"")
        
        rclpy.spin(coordinator)
        
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if coordinator is not None:
            coordinator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    # Graceful shutdown on signals
    def signal_handler(sig, frame):
        """Handle system signals for clean shutdown"""
        print(f"\n🛑 Received signal {sig}, shutting down...")
        sys.exit(0)
    
    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Termination

    main()