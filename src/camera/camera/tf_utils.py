import rclpy
import tf2_ros
from geometry_msgs.msg import Point, Pose, TransformStamped
import tf_transformations
from rclpy.time import Time
import pyrealsense2 as rs
from sensor_msgs.msg import CameraInfo
from rclpy.callback_groups import ReentrantCallbackGroup
import tf2_geometry_msgs
from geometry_msgs.msg import Quaternion

class TFHandler:
    def __init__(self, node):
        self.node = node
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        self.broadcaster = tf2_ros.TransformBroadcaster(self.node)
        self.intrinsics = None
        
        self.cam_info_sub = self.node.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info',
            self.camera_info_callback,
            10,
        )

    def camera_info_callback(self, msg):
        """Store camera intrinsics when available"""
        if self.intrinsics is None:
            self.intrinsics = rs.intrinsics()
            self.intrinsics.width = msg.width
            self.intrinsics.height = msg.height
            self.intrinsics.ppx = msg.k[2]
            self.intrinsics.ppy = msg.k[5]
            self.intrinsics.fx = msg.k[0]
            self.intrinsics.fy = msg.k[4]
            self.intrinsics.model = rs.distortion.brown_conrady
            self.intrinsics.coeffs = list(msg.d)
            self.node.get_logger().info("Camera intrinsics received")

    def transform_to_base(self, point, from_frame='camera_link'):
        try:
            if self.tf_buffer.can_transform('base', from_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=2.0)):
                transform = self.tf_buffer.lookup_transform(
                    'base', from_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=2.0))
                
                pose = Pose()
                pose.position = point
                pose.orientation = Quaternion()  # Add this line to prevent NoneType errors
                
                transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
                return transformed.position
            else:
                self.node.get_logger().error('TF transform failed: could not find transform from %s to base' % from_frame)
                return None
        except Exception as e:
            self.node.get_logger().error(f"TF error: {str(e)}")
            return None
            
    def publish_transform(self, frame_id, child_frame_id, point, orientation=None):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = point[0]
        t.transform.translation.y = point[1]
        t.transform.translation.z = point[2]
        
        if orientation is None:
            q = tf_transformations.quaternion_from_euler(0, 0, 0)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
        else:
            t.transform.rotation = orientation
            
        self.broadcaster.sendTransform(t)
        
    def transform_camera_to_world(self, point):
        """Proper coordinate transformation from camera to world frame"""
        return [
            point[2],   # Camera Z -> World X (forward)
            -point[1] + 0.038 + 0.18,   # Camera Y -> World Z (up)
            -point[0] - 0.2,  # Camera X -> World Y (left)
        ]
    
    def pixel_to_3d(self, pixel_x, pixel_y, depth_value):
        """Convert pixel+depth to 3D point in camera frame"""            
        # Convert to camera frame coordinates (X right, Y down, Z forward)
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics,
            [pixel_y, pixel_x],
            depth_value * 0.001  # mm to meters
        )
        return self.transform_camera_to_world(point_3d)
