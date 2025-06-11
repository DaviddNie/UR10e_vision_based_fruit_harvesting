import rclpy
from rclpy.node import Node
from custom_interface.srv import CameraSrv, MovementRequest, GripperCmd
from geometry_msgs.msg import Point
import time

class DemoRoutine(Node):
    def __init__(self):
        super().__init__('demo_routine')
        
        # Initialize all service clients
        self.camera_client = self.create_client(CameraSrv, '/camera_srv')
        self.gripper_client = self.create_client(GripperCmd, '/gripper_cmd')
        self.movement_client = self.create_client(MovementRequest, '/moveit_path_plan')
        
        # Wait for services to be available
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera Service not available, waiting again...')
            
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper Service not available, waiting again...')
            
        while not self.movement_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Movement Service not available, waiting again...')
        
        self.get_logger().info('DEMO ready!')

    def send_camera_request(self, command, identifier):
        request = CameraSrv.Request()
        request.command = command
        request.identifier = identifier
        self.get_logger().info(f'Sending Camera request: {command} {identifier}')

        future = self.camera_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            self.get_logger().info(f'Camera Response: {response.success} - {response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'Camera Service call failed: {e}')
            return None

    def send_gripper_request(self, width, force=40):
        force = max(min(force, 40), 3)  # Clamp force between 3 and 40N
        
        request = GripperCmd.Request()
        request.width = width
        request.force = force
        self.get_logger().info(f'Sending Gripper request: width={width}, force={force}')

        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            self.get_logger().info(f'Gripper Response: {response.success} - {response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'Gripper Service call failed: {e}')
            return None

    def send_movement_request(self, positions):
        request = MovementRequest.Request()
        request.positions = positions
        self.get_logger().info(f'Sending Movement request: {positions}')

        future = self.movement_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            self.get_logger().info(f'Movement Response: {response.success}')
            return response
        except Exception as e:
            self.get_logger().error(f'Movement Service call failed: {e}')
            return None

    def run_demo(self):
        # Initial bird's eye view position [x, y, z, roll, pitch, yaw]
        bird_eye_position = [0.822, 0.183, 0.856, 0.0, 3.14, 0.0]
        drop_position = [0.822, 0.583, 0.556, 0.0, 3.14, 0.0]
        
        while True:
            # 1. Move to bird's eye view
            self.get_logger().info("Moving to bird's eye view")
            self.send_movement_request(bird_eye_position)
            
            # 2. Take photo and detect apples (identifier 47)
            self.get_logger().info("Detecting apples")
            camera_response = self.send_camera_request("detect", 47)
            
            if not camera_response or not camera_response.success or not camera_response.coordinates:
                self.get_logger().info("No apples detected. Ending demo.")
                break
                
            self.get_logger().info("gripper init")
            self.send_gripper_request(100)  # Open gripper

            # 3. Process each detected apple
            for apple in camera_response.coordinates:
                x, y, z = apple.x, apple.y, apple.z
                self.get_logger().info(f"Processing apple at position: {x}, {y}, {z}")
                
                # 3.1 Move above the apple
                above_apple = [-x, -y, 0.35, 0.0, 3.14, 0.0]
                self.get_logger().info("Moving above apple")
                self.send_movement_request(above_apple)
                
                # 3.2 Lower to picking height
                pick_position = [-x, -y, 0.25, 0.0, 3.14, 0.0]
                self.get_logger().info("Lowering to pick height")
                self.send_movement_request(pick_position)
                
                # 3.3 Grip the apple
                self.get_logger().info("Gripping apple")
                self.send_gripper_request(60)  # Close gripper # 78mm is used as 0mm would trigger a safety fault
                
                # 3.4 Lift the apple
                self.get_logger().info("Lifting apple")
                self.send_movement_request(above_apple)
                
                # 3.5 Move to drop position
                self.get_logger().info("Moving to drop position")
                self.send_movement_request(drop_position)
                
                # 3.6 Release the apple
                self.get_logger().info("Releasing apple")
                self.send_gripper_request(100)  # Open gripper
                
                # 3.7 Return to bird's eye view for next apple
                self.get_logger().info("Returning to bird's eye view")
                self.send_movement_request(bird_eye_position)
                
            # After processing all apples, loop will repeat detection
            
        self.get_logger().info("Demo routine completed")

def main(args=None):
    rclpy.init(args=args)
    
    node = DemoRoutine()
    node.run_demo()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()