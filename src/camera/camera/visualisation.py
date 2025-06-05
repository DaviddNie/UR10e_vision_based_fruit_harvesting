import cv2
import threading
import tf2_ros
import tf_transformations
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, TransformStamped


class VisualisationHandler:
    def __init__(self, node, tf_handler):
        self.node = node
        self._lock = threading.Lock()
        self.tf_handler = tf_handler
        self.marker_pub = node.create_publisher(MarkerArray, 'detected_objects', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        
        # OpenCV setup
        cv2.namedWindow("Object Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Object Detection", 1280, 720)
                
    def update_cv_visualization(self, display_frame, detections):
        """Update the visualization window with frame and detections passed in"""
        if display_frame is None:
            self.node.get_logger().info("No frame available for visualization", 
                                 throttle_duration_sec=1.0)
            return
        
        # Draw detections
        for det in detections:
            box = det['box']
            center = det['center']
            point_3d = det['point_3d']
            conf = det['confidence']

            # Draw bounding box
            cv2.rectangle(display_frame, 
                         (int(box[0]), int(box[1])),
                         (int(box[2]), int(box[3])),
                         (0, 255, 0), 2)
            
            # Draw center point
            cv2.circle(display_frame, 
                      (int(center[0]), int(center[1])), 
                      5, (0, 255, 0), -1)
            
            # Draw coordinates text
            coord_text = f"X:{point_3d[0]:.2f}, Y:{point_3d[1]:.2f}, Z:{point_3d[2]:.2f}"
            cv2.putText(display_frame, coord_text,
                       (int(box[0]), int(box[1]) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            
            # Draw confidence
            conf_text = f"Conf: {conf:.2f}"
            cv2.putText(display_frame, conf_text,
                       (int(box[2]), int(box[3]) + 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Show the frame
        cv2.imshow("Object Detection", display_frame)
        cv2.waitKey(1)

    def update_rviz_visualization(self, detections):
        """Update RViz visualization with TF frames and markers"""
        if not detections:
            return
            
        try:
            # Create marker array
            marker_array = MarkerArray()
            
            for i, detection in enumerate(detections):
                point_3d = detection['point_3d']

                self.tf_handler.publish_transform(frame_id="camera_link", child_frame_id=f"detected_object_{i}", point=point_3d)

                # Create RViz marker
                marker = Marker()
                marker.header.frame_id = "camera_link"
                marker.header.stamp = self.node.get_clock().now().to_msg()
                marker.ns = "detections"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                
                # Set marker position
                marker.pose.position.x = point_3d[0]
                marker.pose.position.y = point_3d[1]
                marker.pose.position.z = point_3d[2]
                
                # Set marker scale (size)
                marker.scale.x = 0.05  # 5cm diameter
                marker.scale.y = 0.05
                marker.scale.z = 0.05
                
                # Set marker color (green)
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                
                # Set marker lifetime
                marker.lifetime.sec = 1
                
                marker_array.markers.append(marker)
            
            self.marker_pub.publish(marker_array)
            
        except Exception as e:
            self.node.get_logger().error(f"RViz Visualization error: {str(e)}")
        
    def cleanup(self):
        cv2.destroyAllWindows()