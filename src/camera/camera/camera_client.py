#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from camera_interface.srv import CameraSrv
from geometry_msgs.msg import Point
import sys

class CameraClient(Node):
    def __init__(self):
        super().__init__('camera_client')
        self.client = self.create_client(CameraSrv, 'camera_service')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Service available! Ready for commands.')

    def send_request(self, input_str):
        """Process user input and send service request"""
        input_str = input_str.strip()
        
        # Handle empty input
        if not input_str:
            self.get_logger().warn("Empty input received")
            return None
            
        # Split into command and parameters
        parts = input_str.split(maxsplit=1)
        command = parts[0].lower()
        
        # Validate command
        if command not in ['detect', 'other_command']:  # Add other commands as needed
            self.get_logger().error(f"Invalid command: {command}. Valid commands are: detect")
            return None
            
        # Parse identifier (param1)
        identifier = None
        if len(parts) > 1:
            try:
                identifier = int(parts[1])
            except ValueError:
                self.get_logger().error(f"Invalid identifier: {parts[1]}. Must be an integer.")
                return None
        else:
            self.get_logger().error("Missing required identifier parameter")
            return None

        # Create and send request
        try:
            request = CameraSrv.Request()
            request.command = command
            request.identifier = identifier
            
            self.get_logger().info(f"Sending request - Command: {command}, ID: {identifier}")
            
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.log_detection_results(response)
                else:
                    self.get_logger().error(f"Service failed: {response.message}")
                return response
            else:
                self.get_logger().error("Service call failed: No response received")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")
            return None

    def log_detection_results(self, response):
        """Format and log detection results"""
        self.get_logger().info("\n=== Detection Results ===")
        self.get_logger().info(f"Status: {response.message}")
        self.get_logger().info(f"Objects found: {len(response.coordinates)}")
        
        for i, point in enumerate(response.coordinates):
            self.get_logger().info(
                f"  Object {i+1}: "
                f"X={point.x:.3f}m, Y={point.y:.3f}m, Z={point.z:.3f}m"
            )
        
        self.get_logger().info("=======================\n")

        #   (47 for apple in COCO)
        #   (64 for mouse in COCO)

def print_usage():
    """Display usage instructions"""
    print("\nUsage:")
    print("  Enter commands in format: <command> <identifier>")
    print("  Example: 'detect 47' - Detect objects of class 47 (apple)")
    print("  Commands:")
    print("    detect <class_id> - Detect objects of specified class")
    print("    exit                 - Quit the program")
    print()

def main(args=None):
    rclpy.init(args=args)
    client = CameraClient()
    
    print_usage()

    try:
        while True:
            try:
                user_input = input("Enter command (or 'exit' to quit): ").strip()
                
                if user_input.lower() == 'exit':
                    break
                    
                if not user_input:
                    continue

                client.send_request(user_input)
                
            except KeyboardInterrupt:
                print("\nOperation cancelled by user")
                break
            except Exception as e:
                client.get_logger().error(f"Unexpected error: {str(e)}")
                continue
                
    finally:
        client.destroy_node()
        rclpy.shutdown()
        print("Client shutdown complete.")

if __name__ == '__main__':
    main()