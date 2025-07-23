import rclpy
from rclpy.node import Node
from custom_interface.srv import CameraSrv, MovementRequest, GripperCmd, ResetGripperCmd
from geometry_msgs.msg import Point
import time
import copy 
NO_CONSTRAINT = "NONE"
DOWN_CONSTRAINT = "DOWN"

# Horizontal Gripping Stretegy: Greedy + Visual Servoing

# Since depth camera is tuned to be accurate at a certain height, but apples are usually of various height
# The stretegy for now is perform (x,y,z) calibration based on the first instance of the detected_array such that
# we can guarentee that we can sccuessfuly grip the first apple

# where (x,y) is the center coordinate of the detected apple
# z is tuned to the optimal detection height

# before we start picking, we filter the detected apples by height, and only leave the apples that are (+- 0.1m) of the target height
# the reason for this is that we know the depth camera is not accurate enough for apples outside the height range

### Alternative solutions
# multiple passes visual servoing -> Done by Weichen Tie

# position [x, y, z, roll, pitch, yaw]
bird_eye_position = [0.822, 0.183, 0.856, 0.0, 3.14, 0.0]
drop_position = [0.822, 0.583, 0.556, 0.0, 3.14, 0.0]
max_attempts = 3

class DemoRoutine(Node):
    def __init__(self):
        super().__init__('demo_routine')
        
        # Initialize all service clients
        self.camera_client = self.create_client(CameraSrv, '/camera_srv')
        self.gripper_client = self.create_client(GripperCmd, '/gripper_cmd')
        self.reset_gripper_client = self.create_client(ResetGripperCmd, '/reset_gripper_cmd')
        self.movement_client = self.create_client(MovementRequest, '/moveit_path_plan')
        
        # Wait for services to be available
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Camera Service not available, waiting again...')
            
        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper Service not available, waiting again...')

        while not self.reset_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ResetGripper Service not available, waiting again...')
            
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


    def send_reset_gripper_request(self, reset):
        
        request = ResetGripperCmd.Request()
        request.reset_gripper = reset
        self.get_logger().info(f'Sending ResetGripper request: reset_gripper={reset}')

        future = self.reset_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            self.get_logger().info(f'Reset Gripper Response: {response.success} - {response.message}')
            return response
        except Exception as e:
            self.get_logger().error(f'Reset Gripper Service call failed: {e}')
            return None

    def send_movement_request(self, positions, constraint = DOWN_CONSTRAINT):
        request = MovementRequest.Request()
        request.command = "cartesian"
        request.positions = positions
        request.constraints_identifier = constraint

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
        while True:
            birds_eye_position_copy = copy.deepcopy(bird_eye_position)
            detected_apples = self.run_detection_at_pos(birds_eye_position_copy)

            if not detected_apples:
                break

            # Tune the birds eye to focus on the first apple to guarentee gripping success
            extracted_apple_height = detected_apples[0].z
            adjusted_birds_eye_position = copy.deepcopy(bird_eye_position)
            adjusted_birds_eye_position[0] = detected_apples[0].x
            adjusted_birds_eye_position[1] = detected_apples[0].y
            adjusted_birds_eye_position[2] = extracted_apple_height + 0.67

            adjusted_above_drop_off_position = copy.deepcopy(drop_position)

            gripping_pos_array = self.run_detection_at_pos(adjusted_birds_eye_position)
            filtered_pos_array = self.filter_apples_for_pickup(gripping_pos_array, extracted_apple_height)

            if not filtered_pos_array:
                break

            self.get_logger().info("gripper init")
            self.send_gripper_request(100)  # Open gripper

            # 3. Process each detected apple
            for apple in filtered_pos_array:
                x, y, z = apple.x, apple.y, apple.z
                above_apple_height = z + 0.5

                above_apple = [x, y, above_apple_height, 0.0, 3.14, 0.0]
                adjusted_above_drop_off_position[2] = above_apple_height
                pick_position = [x, y, z + 0.19, 0.0, 3.14, 0.0]


                self.get_logger().info(f"Processing apple at position: {x}, {y}, {z}")
                
                # 3.1 Move above the apple
                self.get_logger().info("Moving above apple")
                self.send_movement_request(above_apple)
                
                # 3.2 Lower to picking height
                self.get_logger().info("Lowering to pick height")
                self.send_movement_request(pick_position)
                
                # 3.3 Grip the apple
                self.get_logger().info("Gripping apple")
                self.send_gripper_request(0)  # Close gripper # 78mm is used as 0mm would trigger a safety fault
                
                time.sleep(0.5)
                
                # 3.4 Lift the apple
                self.get_logger().info("Lifting apple")
                self.send_movement_request(above_apple)
                
                # 3.3.1 Reset Gripper in case of a safety fault
                self.get_logger().info("reset gripper")
                self.send_reset_gripper_request(True)

                # 3.4 Move horizontally to above the drop position
                self.get_logger().info("Moving to drop position")
                self.send_movement_request(adjusted_above_drop_off_position)

                # 3.5 Move to drop position
                self.send_movement_request(drop_position)
                
                # 3.6 Release the apple
                self.get_logger().info("Releasing apple")
                self.send_gripper_request(100)  # Open gripper
                                
            # After processing all apples, loop will repeat detection
        
        self.send_movement_request(bird_eye_position)

        self.get_logger().info("Demo routine completed")

    def run_detection_at_pos(self, position):
            # 2. Take photo and detect apples (identifier 47)
            self.get_logger().info("Detecting apples")
            
            attempt = 0
            while attempt < max_attempts:
                attempt += 1
                self.get_logger().info(f"Detection attempt {attempt}/{max_attempts}")
                
                # Sometimes YOLO doesn't work well when it's directly above
                position[0] -= 0.01
                position[1] -= 0.01
                position[2] -= 0.02

                # 1. Move to bird's eye view
                self.get_logger().info("Moving to bird's eye view")
                self.send_movement_request(position)
                
                time.sleep(1.5)

                # Take photo and detect apples (identifier 47)
                camera_response = self.send_camera_request("detect", 47)
                
                # If successful detection, return coordinates immediately
                if camera_response and camera_response.success and camera_response.coordinates:
                    self.get_logger().info(f"Successfully detected {len(camera_response.coordinates)} apples")
                    return camera_response.coordinates
                    
                self.get_logger().info(f"Detection failed on attempt {attempt}")
                time.sleep(0.5)  # Brief pause between attempts

            self.get_logger().info("Max detection attempts reached with no apples found")
            return None

    def filter_apples_for_pickup(self, pos_array, target_height, height_tolerance=0.05):
        """
        Filters apples to only those within height tolerance of target height.
        
        Args:
            pos_array: List of Point messages or arrays containing apple positions
            target_height: Desired Z height (in meters)
            height_tolerance: Allowed ± variation from target height (default: 0.15m)
        
        Returns:
            List of filtered apples meeting height criteria
        """
        filtered_apples = []
        
        for apple in pos_array: 
            if abs(apple.z - target_height) <= height_tolerance:
                filtered_apples.append(apple)
        
        return filtered_apples

def main(args=None):
    rclpy.init(args=args)
    
    node = DemoRoutine()
    node.run_demo()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()