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
bird_eye_position = [0.822, 0.183, 0.656, 0.0, 3.14, 0.0]
drop_position = [0.822, 0.583, 0.556, 0.0, 3.14, 0.0]
max_attempts = 3

class DemoRoutine(Node):
    def __init__(self):
        super().__init__('demo_routine')
        
        # Initialize all service clients
        self.movement_client = self.create_client(MovementRequest, '/moveit_path_plan')
        
        # Wait for services to be available
        while not self.movement_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Movement Service not available, waiting again...')
        
        self.get_logger().info('DEMO ready!')

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

            # 3. Process each detected apple
            for apple in gripping_pos_array:
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
                
                time.sleep(0.5)
                
                # 3.4 Lift the apple
                self.get_logger().info("Lifting apple")
                self.send_movement_request(above_apple)

                # 3.4 Move horizontally to above the drop position
                self.get_logger().info("Moving to drop position")
                self.send_movement_request(adjusted_above_drop_off_position)

                # 3.5 Move to drop position
                self.send_movement_request(drop_position)
        
        self.send_movement_request(bird_eye_position)

        self.get_logger().info("Demo routine completed")

    def run_detection_at_pos(self, position):    
            attempt = 0
            while attempt < max_attempts:
                attempt += 1
                
                # Sometimes YOLO doesn't work well when it's directly above
                position[0] -= 0.01
                position[1] -= 0.01
                position[2] -= 0.02

                # 1. Move to bird's eye view
                self.get_logger().info("Moving to bird's eye view")
                self.send_movement_request(position)
                
                time.sleep(1.5)

            self.get_logger().info("Max detection attempts reached with no apples found")
            return None


def main(args=None):
    rclpy.init(args=args)
    
    node = DemoRoutine()
    node.run_demo()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()