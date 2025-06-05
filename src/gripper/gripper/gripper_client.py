#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interface.srv import GripperCmd

class GripperClient(Node):
    def __init__(self):
        super().__init__('gripper_client')
        self.cli = self.create_client(GripperCmd, 'gripper_command')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Connected to gripper server')
        
    def send_request(self, width, force):
        req = GripperCmd.Request()
        req.width = width
        req.force = force
        self.future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    
    client = GripperClient()
    
    try:
        while True:
            # Get user input
            user_input = input("Enter width and force (default 40N) separated by space: ").strip()
            
            if not user_input:
                continue
                
            parts = user_input.split()
            width = int(parts[0])
            force = 40 if len(parts) < 2 else int(parts[1])
            
            # Send request
            response = client.send_request(width, force)
            
            # Display result
            if response.success:
                print(f"Success: {response.message}")
            else:
                print(f"Error: {response.message}")
                
    except KeyboardInterrupt:
        print("\nShutting down client...")
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()