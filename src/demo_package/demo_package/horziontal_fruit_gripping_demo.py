import rclpy
from rclpy.node import Node
from custom_interface.srv import CameraSrv, GripperCmd
import time
import random
import string
from tm_msgs.srv import GripperCmd


WAITAFTER_COMMAND = "waitAfter"
WAITCOMPLETE_COMMAND = "waitComplete"
WAIT_CMDS = {WAITAFTER_COMMAND, WAITCOMPLETE_COMMAND}
QUERYJOB_COMMAND = "queryjob"
EXIT_COMMAND = "exit"
AMR = "AMR"
TM = "TM"
WAIT = "wait"

class DemoRoutine(Node):
	def __init__(self):
		super().__init__('demo_routine')
		self.cameraClient = self.create_client(CameraSrv, '/camera_srv')
		self.gripperClient = self.create_client(GripperCmd, '/gripper_cmd')

		while not self.cameraClient.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Camera Service not available, waiting again...')

		while not self.gripperClient.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('Gripper Service not available, waiting again...')
		
		self.get_logger().info('DEMO ready!')

	def send_request_camera(self, command, identifier):
		# Create the request message
		request = CameraSrv.Request()
		request.command = command
		request.identifier = identifier
		self.get_logger().info(f'Sending Camera request: {command} {identifier}')

		future = self.client.call_async(request)

		while rclpy.ok():
			rclpy.spin_once(self, timeout_sec=0.1)
			
			if future.done():
				try:
					response = future.result()
					self.get_logger().info(f'Response: {response}')
					return response
				except Exception as e:
					self.get_logger().error(f'Service call failed: {e}')
					return None
				
	def send_request_gripper(self, width, force=None):
		force = int(force) if force is not None else 40
		force = max(min(force, 40), 3) # minimum 3N

		request = GripperCmd.Request()
		request.width = width
		request.force = force

		self.get_logger().info(f'Sending Gripper request: {width} {force}')

		future = self.gripperClient.call_async(request)

		while rclpy.ok():
			rclpy.spin_once(self, timeout_sec=0.1)
			
			if future.done():
				try:
					response = future.result()
					self.get_logger().info(f'Response: {response}')
					return response
				except Exception as e:
					self.get_logger().error(f'Service call failed: {e}')
					return None

	def run_demo(self):
		return
	
def main(args=None):
	rclpy.init(args=args)

	node = DemoRoutine()
	node.run_demo()

if __name__ == '__main__':
	main()