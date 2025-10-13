#!/usr/bin/env python3
import os
import sys
import time
import yaml
import re
import uuid
from enum import Enum
from typing import Dict, Tuple, Optional
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from nav2_msgs.srv import LoadMap, ClearEntireCostmap
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from dataclasses import dataclass

from rexbot_one.srv import DeliverToUnit
from example_interfaces.srv import AddTwoInts

from map_manager import setup_map
from nav_manager import navigate_to_goal

# TODO: Get from params
NUMBER_OF_FLOORS = 5
NUMBER_OF_UNITS_PER_FLOOR = 20
REQUEST_ACCEPTED_TIMEOUT = 5.0  # seconds
NAVIGATION_RESULT_TIMEOUT = 120.0  # seconds

class DeliveryState(Enum):
    """States in the delivery workflow"""
    IDLE = "idle"
    STARTING = "starting"
    NAVIGATING_TO_LOADING = "navigating_to_loading"
    WAITING_FOR_LOADING = "waiting_for_loading"
    NAVIGATING_TO_ELEVATOR = "navigating_to_elevator"
    REQUESTING_ELEVATOR = "requesting_elevator"
    WAITING_FOR_ELEVATOR = "waiting_for_elevator"
    ENTERING_ELEVATOR = "entering_elevator"
    RIDING_ELEVATOR = "riding_elevator"
    EXITING_ELEVATOR = "exiting_elevator"
    NAVIGATING_TO_UNIT = "navigating_to_unit"
    DELIVERY_COMPLETE = "delivery_complete"
    ERROR_STATE = "error"

class ActionResult(Enum):
    NONE = -1
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6

class SingleElevatorRequestDirection(Enum):
    UP = 1
    DOWN = -1

@dataclass
class ElevatorStatus:
    """Tracks the status of a single elevator"""
    elevator_id: int
    current_floor: int = 0
    doors_open: bool = True

class DeliveryCoordinator(Node):
    def __init__(self):
        super().__init__('delivery_coordinator')
        self.map_configs = {
            'ground_floor': {
                'map_file': 'ground_floor_map_save.yaml',
                'frame_id': 'ground_floor_map',
                'description': 'Lobby, reception, elevators, loading area'
            },
            'residential_floor': {
                'map_file': 'residential_floor_save.yaml', 
                'frame_id': 'residential_floor_map',
                'description': 'Standard residential floor layout (floors 1-4)'
            }
        }
        self.num_elevators = 4 # TODO: Move to a configuration file
        self.state = DeliveryState.IDLE
        self.future = None

        # Initialize delivery parameters
        self.current_floor = 0 # Ground floor
        self.target_floor = None
        self.target_unit = None
        self.requested_elevator_id = None

        # Load location coordinates
        self.locations = {}
        self._load_location_config()

        # Initialize elevator status tracking
        self.lock = threading.Lock()
        self.elevators: Dict[int, ElevatorStatus] = {}
        for i in range(1, self.num_elevators + 1):
            self.elevators[i] = ElevatorStatus(elevator_id=i)
        

        # Service clients
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

        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_to_pose action server not available, initialization failed.')
            raise RuntimeError('navigate_to_pose action server not available')
        
        self.request_elevator_client = self.create_client(AddTwoInts, 'request_elevator')
        if not self.request_elevator_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('request_elevator service not available, initialization failed.')
            raise RuntimeError('request_elevator service not available')
        
        self.floor_subscribers = {}
        self.door_subscribers = {}
        for elevator_id in range(1, self.num_elevators + 1):
            # Subscribe to current floor
            self.floor_subscribers[elevator_id] = self.create_subscription(
                Int32,
                f'/elevator_{elevator_id}/current_floor',
                lambda msg, eid=elevator_id: self.floor_callback(msg, eid),
                10
            )
            
            # Subscribe to door state
            self.door_subscribers[elevator_id] = self.create_subscription(
                Bool,
                f'/elevator_{elevator_id}/door_state',
                lambda msg, eid=elevator_id: self.door_callback(msg, eid),
                10
            )

        # Service servers
        self.deliver_service = self.create_service(
            DeliverToUnit, 'deliver_to_unit', self.delivery_request_callback
        )

        # Run step() every second
        self.timer = self.create_timer(1.0, self.step)
    
    # @property
    # def state(self):
    #     return self._state

    # @state.setter
    # def state(self, new_state):
    #     if self._state != new_state:
    #         self._state = new_state
    #         self.step()
    
    def step(self):
        if self.state == DeliveryState.IDLE:
            self.get_logger().info("Currently IDLE")
        
        elif self.state == DeliveryState.STARTING:
            self.get_logger().info("Starting delivery process")
            self.handle_starting()
        
        elif self.state == DeliveryState.NAVIGATING_TO_LOADING:
            self.get_logger().info("Navigating to loading area")
            self.handle_navigating_to_loading()
        
        elif self.state == DeliveryState.WAITING_FOR_LOADING:
            self.get_logger().info("Waiting for loading")
            # Check if loading is complete
            # On completion, transition to NAVIGATING_TO_ELEVATOR state
            # self.state = DeliveryState.NAVIGATING_TO_ELEVATOR
        
        elif self.state == DeliveryState.NAVIGATING_TO_ELEVATOR:
            self.get_logger().info("Navigating to elevator")
            self.handle_navigating_to_elevator()
        
        elif self.state == DeliveryState.REQUESTING_ELEVATOR:
            self.get_logger().info("Requesting elevator")
            self.handle_requesting_elevator()
        
        elif self.state == DeliveryState.WAITING_FOR_ELEVATOR:
            self.get_logger().info("Waiting for elevator")
            threading.Thread(target=lambda: self.handle_waiting_for_elevator(self.current_floor), daemon=True).start()
        
        elif self.state == DeliveryState.ENTERING_ELEVATOR:
            self.get_logger().info("Entering elevator")
        #     # Navigate into the elevator
        #     # On success, transition to RIDING_ELEVATOR state
        #     self.state = DeliveryState.RIDING_ELEVATOR
        
        # elif self.state == DeliveryState.RIDING_ELEVATOR:
        #     self.get_logger().info("Riding elevator")
        #     # Wait for elevator to reach destination floor
        #     # On arrival, transition to EXITING_ELEVATOR state
        #     self.state = DeliveryState.EXITING_ELEVATOR
        
        # elif self.state == DeliveryState.EXITING_ELEVATOR:
        #     self.get_logger().info("Exiting elevator")
        #     # Navigate

    def delivery_request_callback(self, request, response):
        if self.state == DeliveryState.IDLE:
            self.get_logger().info("Received new delivery request")

            # Parse floor and unit from request.unit_id
            floor, unit = self._parse_unit_id(request.unit_id)

            if floor is None or unit is None:
                response.success = False
                response.error = f"Invalid unit ID: {request.unit_id}"
                return response

            self.delivery_id = str(uuid.uuid4())[:8]
            self.delivery_floor = floor
            self.delivery_unit = unit

            self.state = DeliveryState.STARTING
            

            # # Set single elevator request state to EN_ROUTE
            # def start():
            #     self.state = DeliveryState.STARTING
            # threading.Thread(target=start, daemon=True).start()

            response.success = True
            response.target_floor = self.delivery_floor
            response.target_unit = self.delivery_unit
            response.estimated_duration = 300.0
            response.delivery_id = self.delivery_id

            return response
            
        else:
            self.get_logger().warn("Failed to accept new delivery request, already busy!")
            response.success = False
            response.error = f"Delivery coordinator busy with another request."
            return response

    def handle_starting(self):
        if self.state != DeliveryState.STARTING:
            self.get_logger().error("handle_starting called in wrong state")
            return
        
        # Setup ground floor map
        result = setup_map("ground_floor", self.locations.get("ground_floor").get("lobby-center"))  # TODO: set to home

        if not result:
            self.get_logger().error("Failed to setup ground floor map")
            self.state = DeliveryState.ERROR_STATE
            return

        self.state = DeliveryState.NAVIGATING_TO_ELEVATOR
    
    def handle_navigating_to_loading(self):
        # Navigate to loading zone
        result = navigate_to_goal(self.locations.get("ground_floor").get("loading-zone"))

        if not result:
            self.get_logger().error("Failed to navigate to loading zone.")
            self.state = DeliveryState.ERROR_STATE
            return
        
        self.state = DeliveryState.WAITING_FOR_LOADING

    def handle_navigating_to_elevator(self):
        # Navigate to elevator
        result = navigate_to_goal(self.locations.get("ground_floor").get("lobby-elevator"))

        if not result:
            self.get_logger().error("Failed to navigate to lobby elevator.")
            self.state = DeliveryState.ERROR_STATE
            return

        self.state = DeliveryState.REQUESTING_ELEVATOR
    
    def handle_requesting_elevator(self):
        request = AddTwoInts.Request()
        request.a = self.current_floor
        request.b = SingleElevatorRequestDirection.UP.value

        future = self.request_elevator_client.call_async(request)
        future.add_done_callback(self.handle_requesting_elevator_callback)
    
    def handle_requesting_elevator_callback(self, future):
        result = future.result()
        if result.sum != -1:
            self.get_logger().info("Elevator request successful, waiting for elevator to arrive")
            self.requested_elevator_id = result.sum
            self.state = DeliveryState.WAITING_FOR_ELEVATOR
        else:
            self.get_logger().warn(f"Elevator request failed with response: {result}")
            self.state = DeliveryState.ERROR_STATE

    def handle_waiting_for_elevator(self, target_floor):
        if self.state == DeliveryState.WAITING_FOR_ELEVATOR:
            while self.elevators[self.requested_elevator_id].current_floor != target_floor or not self.elevators[self.requested_elevator_id].doors_open:
                self.get_logger().info(f"Waiting for elevator {self.requested_elevator_id} to arrive at floor {target_floor} with doors open...")
                time.sleep(1.0)
            
            self.get_logger().info(f"Elevator {self.requested_elevator_id} has arrived with doors open.")
            self.state = DeliveryState.ENTERING_ELEVATOR
            
        else:
            self.get_logger().error(f"Delivery state is not {self.state}, and is expected to be {DeliveryState.WAITING_FOR_ELEVATOR}.")
            self.state = DeliveryState.ERROR_STATE

    # ------------------ Utility functions ------------------
    def floor_callback(self, msg: Int32, elevator_id: int):
        """Update elevator floor information"""
        with self.lock:
            if elevator_id in self.elevators:
                self.elevators[elevator_id].current_floor = msg.data
                # if elevator_id == self.requested_elevator_id:
                #     self.get_logger().info(f"Elevator {elevator_id} is now at floor {msg.data}")
    
    def door_callback(self, msg: Bool, elevator_id: int):
        """Update elevator door state"""
        with self.lock:
            if elevator_id in self.elevators:
                self.elevators[elevator_id].doors_open = msg.data

    # def send_navigate_to_goal_request(self, goal: Dict[str, float]):
    #     # Create a goal pose
    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = "map"
    #     goal_pose.header.stamp = self.get_clock().now().to_msg()
    #     goal_pose.pose.position.x = goal.get("x")
    #     goal_pose.pose.position.y = goal.get("y")

    #     # Convert yaw to quaternion
    #     goal_pose.pose.orientation.z = (goal.get("yaw") / 2.0) ** 0.5
    #     goal_pose.pose.orientation.w = (1.0 - (goal.get("yaw") / 2.0) ** 2) ** 0.5

    #     # Send goal
    #     goal_msg = NavigateToPose.Goal()
    #     goal_msg.pose = goal_pose

    #     # NOTE:
    #     # 1. Send goal
    #     # 2. Wait for accepted response
    #     future = self._action_client.send_goal_async(goal_msg)
    #     return future

    #     goal_handle = send_goal_future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().warn("Goal rejected")
    #         return False
        
    #     self.get_logger().info("Goal accepted, waiting for result...")
    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.handle_navigating_to_loading_callback)


    def _load_location_config(self):
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

    # TODO: move to utils
    def _parse_unit_id(self, unit_id: str) -> Tuple[Optional[int], Optional[int]]:
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
            if floor < 1 or floor > NUMBER_OF_FLOORS:  # Reasonable floor limits
                self.get_logger().error(f"Invalid floor number: {floor}")
                return None, None

            if unit < 1 or unit > NUMBER_OF_UNITS_PER_FLOOR:  # Reasonable unit limits
                self.get_logger().error(f"Invalid unit number: {unit}")
                return None, None
            
            self.get_logger().info(f"Parsed '{unit_id}' -> Floor {floor}, Unit {unit}")
            return floor, unit
            
        except ValueError as e:
            self.get_logger().error(f"Unit ID parsing error: {e}")
            return None, None
    

def main(args=None):
    """Main entry point for delivery coordinator"""

    coordinator = None
    rclpy.init(args=args)

    coordinator = DeliveryCoordinator()
    executor = MultiThreadedExecutor()
    executor.add_node(coordinator)

    try:
        executor.spin()

        coordinator.get_logger().info("Delivery coordinator ready for service calls")
        coordinator.get_logger().info("Example usage:")
        coordinator.get_logger().info("  ros2 service call /deliver_to_unit rexbot_one/DeliverToUnit \"{unit_id: '0411'}\"")
        
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received. Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if coordinator is not None:
            coordinator.destroy_node()
        if rclpy.ok():
            executor.shutdown()
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