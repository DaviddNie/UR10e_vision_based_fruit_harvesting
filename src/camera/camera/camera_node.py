import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from camera_interface.srv import CameraSrv
from .detection_utils import DetectionHandler
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
import pyrealsense2 as rs
from .tf_utils import TFHandler
from .visualisation import VisualisationHandler

class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        
        # Setup callback groups
        self.service_group = ReentrantCallbackGroup()
        self.image_group = ReentrantCallbackGroup()

        # Setup components
        self.tf_handler = TFHandler(self)
        self.visualiser = VisualisationHandler(
            node=self,
            tf_handler=self.tf_handler,
        )

        self.detector = DetectionHandler(
            node=self,
            tf_handler=self.tf_handler,
            visualiser=self.visualiser
        )


        self.setup_subscribers()


        # Service
        self.srv = self.create_service(
            CameraSrv, 
            'camera_service', 
            self.handle_camera_request,
            callback_group=self.service_group
        )
        
        self.get_logger().info("Camera Server ready")

    def setup_subscribers(self):
        """Configure image subscribers and synchronizer"""
        self.color_sub = Subscriber(
            self, 
            Image, 
            '/camera/camera/color/image_raw',
            callback_group=self.image_group
        )
        self.depth_sub = Subscriber(
            self, 
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw',
            callback_group=self.image_group
        )
                
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1,
        )
        self.ts.registerCallback(self.detector.update_frames)
        
    async def handle_camera_request(self, request, response):
        result = await self.detector.handle_request(request)

        response.coordinates = result.get('coordinates', [])
        response.success = result.get('success', False)
        response.message = result.get('message', '')
        return response

def main():
    rclpy.init()
    server = CameraServer()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)

    # Start the executor in a background thread
    def spin_executor():
        executor.spin()
    executor_thread = threading.Thread(target=spin_executor, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            rclpy.spin_once(server, timeout_sec=0.1)
    except KeyboardInterrupt:
        server.get_logger().info("Shutting down server")
    finally:
        server.destroy_node()
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()