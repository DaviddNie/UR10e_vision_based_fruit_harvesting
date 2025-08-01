import rclpy
from rclpy.node import Node
from custom_interface.srv import CameraSrv, MovementRequest, GripperCmd, ResetGripperCmd
from geometry_msgs.msg import Point
import time
import copy 
NO_CONSTRAINT = "NONE"
VER_ELBOW = "ELBOW"
VER_ELBOW_WRIST1 = "ELBOW_WRIST1"

# position [x, y, z, roll, pitch, yaw]
bird_eye_position = [0.54, 0.174, 0.9, -1.56, -0.0, -1.571]
# birds_eye_joint_pos = [-1.64, 1.66, -3.17, -1.57, 0.0, 0.0]
birds_eye_joint_pos = [-1.9, 1.2, -2.0, -1.57, 0.0, 0.0]

# bird_eye_position = [0.64, 0.174, 1.04, -1.56, -0.0, -1.571]
# bird_eye_position = [0.44, 0.16, 0.83, -1.68, 0.0, -1.61]
initial_bird_eye_position = [0.471,0.149, 1.044, -1.978, 0.058, -1.549]
birds_eye_via_point = [0.822, 0.183, 0.856, 0.0, 3.14, 0.0]
drop_position = [0.822, 0.583, 0.556, 0.0, 3.14, 0.0]
drop_pos_joint = [-1.05, 1.12, -1.64, -1.57, 2.01, 0.44]
max_attempts = 3
SCAN_MODE = 'scan'
GRIP_MODE = 'grip'

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

    def send_camera_request(self, command, identifier, conf):
        request = CameraSrv.Request()
        request.command = command
        request.identifier = identifier
        request.conf = conf
        self.get_logger().info(f'Sending Camera request: {command} {identifier} {conf}')

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

    def send_movement_request(self, positions, command = "cartesian", constraint = NO_CONSTRAINT):
        request = MovementRequest.Request()
        request.command = command
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

            self.send_movement_request(birds_eye_joint_pos, "joint", NO_CONSTRAINT)

            # birds_eye_position_copy = copy.deepcopy(bird_eye_position)
            # detected_apples = self.run_detection_at_pos(birds_eye_position_copy, SCAN_MODE)
            detected_apples = self.run_detection_at_curr_pos()
            print(f"detected_apples are {detected_apples}")
            
            if not detected_apples:
                print(f"no apple detected, ending")
                break

            # Tune the birds eye to focus on the first apple to guarentee gripping success
            extracted_apple_distance = detected_apples[0].x
            adjusted_birds_eye_position = copy.deepcopy(bird_eye_position)
            adjusted_birds_eye_position[0] = extracted_apple_distance - 0.5
            adjusted_birds_eye_position[1] = detected_apples[0].y
            adjusted_birds_eye_position[2] = detected_apples[0].z - 0.1

            adjusted_above_drop_off_position = copy.deepcopy(drop_position)

            gripping_pos_array = self.run_detection_at_pos(adjusted_birds_eye_position, SCAN_MODE)

            if not gripping_pos_array:
                print(f"no apples detected, ending")
                break

            filtered_pos_array = self.filter_apples_for_pickup(gripping_pos_array, extracted_apple_distance)
            print(f"gripper pose detected_apples are {gripping_pos_array}")

            if not filtered_pos_array:
                print(f"bad photo, redetecting")
                continue

            self.get_logger().info("gripper init")
            self.send_gripper_request(100)  # Open gripper

            # 3. Process each detected apple
            for apple in filtered_pos_array:
                x, y, z = apple.x, apple.y, apple.z
                above_apple_distance = x - 0.5
                y_compensation = y + 0.06
                z_compensation = z - 0.03

                above_apple = [above_apple_distance, y_compensation, z_compensation, -1.56, -0.0, -1.571]
                adjusted_above_drop_off_position[0] = above_apple_distance
                # pre_pick_position = [x - 0.23, y_compensation, z_compensation, -1.56, -0.0, -1.571]
                pick_position = [x - 0.18, y_compensation, z_compensation, -1.56, -0.0, -1.571]


                self.get_logger().info(f"Processing apple at position: {x}, {y}, {z}")
                
                # # 3.1 Move above the apple
                # self.get_logger().info("Moving to hover position")
                # self.send_movement_request(above_apple)

                # 3.2 Lower to picking height
                self.get_logger().info("Lowering to pick height")
                self.send_movement_request(pick_position)
                
                # 3.3 Grip the apple
                self.get_logger().info("Gripping apple")
                self.send_gripper_request(0)  # Close gripper # 78mm is used as 0mm would trigger a safety fault
                
                time.sleep(2)
                
                # 3.4 Lift the apple
                # self.get_logger().info("back to hovering height")
                # self.send_movement_request(above_apple)
                
                # 3.3.1 Reset Gripper in case of a safety fault
                self.get_logger().info("reset gripper")
                self.send_reset_gripper_request(True)

                self.send_gripper_request(100)


                # 3.4 Move horizontally to above the drop position
                self.get_logger().info("Moving to drop position")
                # self.send_movement_request(adjusted_above_drop_off_position)

                # 3.5 Move to drop position
                # self.send_movement_request(drop_pos_joint, "joint")
                self.send_movement_request(drop_position)
                
                # 3.6 Release the apple
                self.get_logger().info("Releasing apple")
                self.send_gripper_request(100)  # Open gripper
                                
            # After processing all apples, loop will repeat detection
        
        self.send_movement_request(birds_eye_joint_pos, "joint", NO_CONSTRAINT)

        self.get_logger().info("Demo routine completed")

    def run_detection_at_pos(self, position, mode):
            # 2. Take photo and detect apples (identifier 47)
            self.get_logger().info("Detecting apples")
            
            attempt = 0
            while attempt < max_attempts:
                attempt += 1
                self.get_logger().info(f"Detection attempt {attempt}/{max_attempts}")
                
                if (attempt != 1 and mode == SCAN_MODE):
                    position[1] -= 0.08
                    position[2] += 0.15
                    position[3] -= 0.3
                elif (mode == GRIP_MODE):
                    position[1] += 0.05
                    position[2] += 0.02
                    position[2] -= 0.05

                # 1. Move to bird's eye view
                self.get_logger().info("Moving to bird's eye view")
                self.send_movement_request(position)
                
                time.sleep(1.5)

                # Take photo and detect apples (identifier 47)
                
                max_detect_attempt = 20
                conf = 0.5
                for attempt in range(1, max_detect_attempt + 1):
                    camera_response = self.send_camera_request("detect", 0, conf)
                    # If successful detection, return coordinates immediately
                    if camera_response and camera_response.success and camera_response.coordinates:
                        self.get_logger().info(f"Successfully detected {len(camera_response.coordinates)} apples")
                        return camera_response.coordinates
                    else:
                        self.get_logger().info(f"YOLO detection attempt {attempt} with conf: {conf} failed")
                        time.sleep(0.5)  # Brief pause between attempts

                        conf = conf - 0.05 if conf >= 0.2 else conf
                    
                self.get_logger().info(f"Detection failed on attempt {attempt}")
                time.sleep(0.5)  # Brief pause between attempts

            self.get_logger().info("Max detection attempts reached with no apples found")
            return None
    

    def run_detection_at_curr_pos(self):
            # 2. Take photo and detect apples (identifier 47)
            self.get_logger().info("Detecting apples")
            
            # Take photo and detect apples (identifier 47)
            
            max_detect_attempt = 20
            conf = 0.5
            for attempt in range(1, max_detect_attempt + 1):
                camera_response = self.send_camera_request("detect", 0, conf)
                # If successful detection, return coordinates immediately
                if camera_response and camera_response.success and camera_response.coordinates:
                    self.get_logger().info(f"Successfully detected {len(camera_response.coordinates)} apples")
                    return camera_response.coordinates
                else:
                    self.get_logger().info(f"YOLO detection attempt {attempt} with conf: {conf} failed")
                    conf = conf - 0.05 if conf >= 0.2 else conf
                time.sleep(0.5)
                
            self.get_logger().info("Max detection attempts reached with no apples found")
            return None

    def filter_apples_for_pickup(self, pos_array, target_distance, distance_tolerance=0.06):
        """
        Filters apples to only those within height tolerance of target height.
        
        Args:
            pos_array: List of Point messages or arrays containing apple positions
            target_distance: Desired distance between camera and item (in meters)
            height_tolerance: Allowed ± variation from target height (default: 0.15m)
        
        Returns:
            List of filtered apples meeting height criteria
        """
        filtered_apples = []

        if len(pos_array) == 1:
            return pos_array
        
        for apple in pos_array: 
            if abs(apple.x - target_distance) <= distance_tolerance:
                filtered_apples.append(apple)
            else:
                print(f"filtered out, difference is {abs(apple.x - target_distance)}")
        
        return filtered_apples

def main(args=None):
    rclpy.init(args=args)
    
    node = DemoRoutine()
    node.run_demo()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# 0.9 flat
# ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{command: 'cartesian', positions: [0.64, 0.174, 0.9, -1.56, -0.0, -1.571], constraints_identifier: '0'}"

# ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{positions: [0.595, 0.183, 0.811, 1.492, 0.06, 1.507]}"

# HOME
# ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{command: 'joint', positions: [-1.3, 1.57, -1.83, -1.57, 0, 0], constraints_identifier: '0'}"

# BIRDS EYE JOINT
# ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{command: 'joint', positions: [-1.64, 1.66, -3.17, -1.57, 0, 0], constraints_identifier: '0'}"

# DROP POINT JOINT
# ros2 service call /moveit_path_plan custom_interface/srv/MovementRequest "{command: 'joint', positions: [-1.05, 1.12, -1.64, -1.57, 2.01, 0.44], constraints_identifier: '0'}"