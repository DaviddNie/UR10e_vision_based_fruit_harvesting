from ultralytics import YOLO
import numpy as np
import asyncio
from cv_bridge import CvBridge
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Pose, TransformStamped
from ament_index_python.packages import get_package_share_directory
import os
import tf2_ros

class DetectionHandler:
    def __init__(self, node, tf_handler, visualiser):
        self.node = node
        self.tf_handler = tf_handler
        self.visualiser = visualiser

        self.bridge = CvBridge()

        camera_pkg_dir = get_package_share_directory('camera')
        model_path = os.path.join(camera_pkg_dir, 'models', 'yolo11m.pt')
        # Load YOLO model from parameter
        self.model = YOLO(model_path)
        self.model.fuse()

        self.last_detections = None
        self.rviz_vis_timer = self.node.create_timer(
            0.1, 
            lambda: self.visualiser.update_rviz_visualization(self.last_detections)
        )
        # Current frame data
        self.current_frame = None
        self.current_depth = None
        
    async def handle_request(self, request):
        if request.command == "detect":
            return await self._detect_objects(request)
        else:
            return {'success': False, 'message': f"Unknown command: {request.command}"}
    
    async def _detect_objects(self, request):
        """Async handler for detect command"""
        if self.current_frame is None or self.current_depth is None:
            return {
                'success': False,
                'message': "No frame available"
            }
            
        try:
            results = self.model(self.current_frame, verbose=False)[0]
            boxes = results.boxes.xyxy.cpu().numpy()
            class_ids = results.boxes.cls.cpu().numpy()
            confidences = results.boxes.conf.cpu().numpy()
            
            detections = []
            for i, (box, cls_id, conf) in enumerate(zip(boxes, class_ids, confidences)):
                if cls_id == request.identifier and conf > 0.5:
                    x_center = int((box[0] + box[2]) / 2)
                    y_center = int((box[1] + box[3]) / 2)
                    depth = self.current_depth[int(y_center), int(x_center)]
                    
                    point_3d = self.tf_handler.pixel_to_3d(x_center, y_center, depth)

                    if point_3d:
                        point_msg = Point()
                        point_msg.x = point_3d[0]
                        point_msg.y = point_3d[1]
                        point_msg.z = point_3d[2]
                        
                        try:
                            # Transform the point to base frame
                            base_pose = self.tf_handler.transform_to_base(point_msg)
                            
                            # Create new point with transformed coordinates
                            transformed_point = Point()
                            transformed_point.x = base_pose.x
                            transformed_point.y = base_pose.y
                            transformed_point.z = base_pose.z
                            
                            detections.append(transformed_point)
                            
                            # Print the transformed coordinates
                            self.node.get_logger().info(
                                f"Transformed coordinates (base frame): "
                                f"X: {transformed_point.x:.3f}, "
                                f"Y: {transformed_point.y:.3f}, "
                                f"Z: {transformed_point.z:.3f}")
                                
                        except (tf2_ros.LookupException, 
                                tf2_ros.ConnectivityException, 
                                tf2_ros.ExtrapolationException) as e:
                            self.node.get_logger().error(f"TF transform failed: {str(e)}")
                            detections.append(point_msg)  # Fall back to camera frame coordinates
            
            # Update visualization state
            self.last_detections = [{
                'box': box,
                'center': (x_center, y_center),
                'point_3d': point_3d,
                'confidence': conf
            } for box, cls_id, conf, (x_center, y_center, point_3d) in 
              zip(boxes, class_ids, confidences, 
                  [(int((b[0]+b[2])/2), int((b[1]+b[3])/2), 
                    self.tf_handler.pixel_to_3d(int((b[0]+b[2])/2), int((b[1]+b[3])/2), 
                                    self.current_depth[int((b[1]+b[3])/2), int((b[0]+b[2])/2)]))
                   for b in boxes])
              if cls_id == request.identifier and conf > 0.5]
            
            self.visualiser.update_cv_visualization(self.current_frame, self.last_detections)

            return {
                'coordinates': detections,
                'success': True,
                'message': f"Found {len(detections)} objects"
            }
            
        except Exception as e:
            self.node.get_logger().error(f"Detection error: {str(e)}")
            return {
                'success': False,
                'message': f"Detection failed: {str(e)}"
            }
    
    def update_frames(self, color_msg, depth_msg):
        try:
            self.current_frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.current_depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception as e:
            self.node.get_logger().error(f"Image conversion failed: {str(e)}")