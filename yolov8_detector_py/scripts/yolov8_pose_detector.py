#!/usr/bin/env python3
import numpy as np
# Monkey-patch for compatibility with ros_numpy
np.float = np.float64

import rospy, cv2
import numpy as np  # reimport uses the monkey-patched version
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO
from image_geometry import PinholeCameraModel
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker  # for visualization marker
import ros_numpy  # for point cloud conversion

class YOLOv8PoseDetector:
    def __init__(self):
        rospy.init_node('yolov8_pose_detector', anonymous=True)

        # Load the YOLOv8 model
        model_path = rospy.get_param('~model_path',
                                     '/home/megha/git/RS2/yolov8_detector_py/models/shapes/best.onnx')
        rospy.loginfo("Loading YOLOv8 model from: %s", model_path)
        self.model = YOLO(model_path)

        # Helpers and state variables
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.depth_image = None
        self.point_cloud = None  # will store an organized point cloud (with color if available)
        self.color_image = None  # store the latest color image
        self.info_received = False
        self.depth_scale = 0.001  # adjust as needed (e.g., from mm to m)
        self.img_width = None
        self.img_height = None

        # Publishers
        self.pose_pub = rospy.Publisher('/detected_object_pose', PoseStamped, queue_size=1)
        self.roi_pc_pub = rospy.Publisher('/segmented_roi', PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher('/roi_marker', Marker, queue_size=1)

        # Subscribers
        rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.cam_info_cb, queue_size=1)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_cb, queue_size=1)
        rospy.Subscriber('/camera/color/image_raw', Image, self.color_cb, queue_size=1)
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pc_cb, queue_size=1)

        rospy.loginfo("YOLOv8PoseDetector with segmentation ready.")
        rospy.spin()

    def cam_info_cb(self, info):
        if not self.info_received:
            self.cam_model.fromCameraInfo(info)
            self.info_received = True
            # Save resolution for reorganization
            self.img_width = info.width
            self.img_height = info.height
            #rospy.loginfo("Camera info received. Resolution: %dx%d", info.width, info.height)

    def depth_cb(self, msg):
        if not self.info_received:
            return
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr("Depth cv_bridge error: %s", e)

    def color_cb(self, msg):
        if not self.info_received:
            return
        try:
            # Convert and store the latest color image (BGR)
            self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.cv_image = self.color_image.copy()  # for drawing
        except CvBridgeError as e:
            rospy.logerr("Color cv_bridge error: %s", e)
            return

        # Run YOLO inference on the color image
        results = self.model(self.cv_image)

        # Loop over detections
        for i, (box, cls) in enumerate(zip(results[0].boxes.xyxy, results[0].boxes.cls)):
            x1, y1, x2, y2 = map(int, box.tolist())
            height, width, _ = self.cv_image.shape
            x1 = max(0, min(x1, width-1))
            x2 = max(0, min(x2, width-1))
            y1 = max(0, min(y1, height-1))
            y2 = max(0, min(y2, height-1))

            # Use center pixel depth as default
            u = (x1 + x2) // 2
            v = (y1 + y2) // 2
            Z_center = float(self.depth_image[v, u]) * self.depth_scale

            # Initialize refined position as center result
            refined_X, refined_Y, refined_Z = None, None, Z_center

            # If organized point cloud is available, perform segmentation for the ROI.
            if self.point_cloud is not None:
                try:
                    pad = 5
                    x1_pad = max(0, x1 - pad)
                    y1_pad = max(0, y1 - pad)
                    x2_pad = min(width-1, x2 + pad)
                    y2_pad = min(height-1, y2 + pad)
                    pc_roi = self.point_cloud[y1_pad:y2_pad, x1_pad:x2_pad]
                    valid_mask = np.isfinite(pc_roi['z'])
                    valid_points = pc_roi[valid_mask]
                    #rospy.loginfo("Found %d valid points in ROI", valid_points.size)
                    if valid_points.size > 0:
                        refined_X = np.median(valid_points['x'])
                        refined_Y = np.median(valid_points['y'])
                        refined_Z = np.median(valid_points['z'])
                        #rospy.loginfo("Refined coordinates: X=%.3f, Y=%.3f, Z=%.3f", refined_X, refined_Y, refined_Z)
                        # Publish the segmented ROI for visualization in RViz.
                        roi_msg = ros_numpy.point_cloud2.array_to_pointcloud2(
                            valid_points, stamp=msg.header.stamp, frame_id=msg.header.frame_id)
                        self.roi_pc_pub.publish(roi_msg)
                    else:
                        refined_X, refined_Y = None, None
                except Exception as e:
                    rospy.logwarn("Segmentation failed for ROI: %s", e)

            # If segmentation did not yield valid results, fall back to center projection.
            if refined_X is None or refined_Y is None:
                ray = self.cam_model.projectPixelTo3dRay((u, v))
                refined_X, refined_Y, _ = [r * Z_center for r in ray]

            # Create zero-rotation orientation.
            qx, qy, qz, qw = quaternion_from_euler(0, 0, 0)

            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = refined_X
            pose.pose.position.y = refined_Y
            pose.pose.position.z = refined_Z
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.pose_pub.publish(pose)

            # Publish a visualization marker.
            marker = Marker()
            marker.header = msg.header
            marker.ns = "roi_marker"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = refined_X
            marker.pose.position.y = refined_Y
            marker.pose.position.z = refined_Z
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.marker_pub.publish(marker)

            # Logging detection
            cls_id = int(cls)
            cls_name = self.model.names[cls_id] if hasattr(self.model, 'names') else "object"
            q = pose.pose.orientation
            rospy.loginfo(
                f"Detected {cls_name} → X={refined_X:.3f}, Y={refined_Y:.3f}, Z={refined_Z:.3f}   "
                f"quat=({q.x:.2f},{q.y:.2f},{q.z:.2f},{q.w:.2f})"
            )
            cv2.rectangle(self.cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            conf = results[0].boxes.conf[i] * 100 if i < len(results[0].boxes.conf) else 0
            cv2.putText(self.cv_image, f"{cls_name} {conf:.0f}%", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLOv8 Pose Detections", self.cv_image)
        cv2.waitKey(1)

    def pc_cb(self, msg):
        try:
            pc_array = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            # If the incoming point cloud is unorganized (height==1) and we have resolution info,
            # then try to reconstruct an organized point cloud.
            if msg.height == 1 and self.img_width and self.img_height:
                expected_points = self.img_width * self.img_height
                if pc_array.size != expected_points:
                    rospy.logwarn("Unorganized point cloud size (%d) does not match expected (%d). " "Reconstructing from depth image.", pc_array.size, expected_points)
                    if self.depth_image is not None:
                        pc_array = self.generate_point_cloud_from_depth()
                    else:
                        rospy.logwarn("Depth image unavailable. Using unorganized point cloud as-is.")
                else:
                    pc_array = pc_array.reshape((self.img_height, self.img_width))
                    #rospy.loginfo("Reshaped point cloud to organized format: %dx%d", self.img_height, self.img_width)
            self.point_cloud = pc_array
        except Exception as e:
            rospy.logerr("PointCloud conversion error: %s", e)

    def generate_point_cloud_from_depth(self):
        """
        Reconstruct an organized, colored point cloud from the depth image using the camera model and
        the stored color image.
        """
        h = self.img_height
        w = self.img_width
        # Create an empty array with fields: x, y, z, and rgb.
        organized_pc = np.zeros((h * w,), dtype=[('x', np.float32), ('y', np.float32),
                                                   ('z', np.float32), ('rgb', np.float32)])
        depth_scaled = self.depth_image * self.depth_scale  # depth in meters
        # Generate coordinates for each pixel
        for i in range(h * w):
            u = i % w
            v = i // w
            Z = depth_scaled[v, u]
            if Z > 0:
                ray = self.cam_model.projectPixelTo3dRay((u, v))
                organized_pc[i]['x'] = ray[0] * Z
                organized_pc[i]['y'] = ray[1] * Z
                organized_pc[i]['z'] = Z
                # If a color image is available, get its color. Assume color image is BGR.
                if self.color_image is not None:
                    pixel = self.color_image[v, u]
                    B, G, R = int(pixel[0]), int(pixel[1]), int(pixel[2])
                    # Pack RGB into an integer (using 24 bits) and then interpret as float.
                    rgb_int = (R << 16) | (G << 8) | B
                    organized_pc[i]['rgb'] = np.float32(rgb_int)
                else:
                    organized_pc[i]['rgb'] = np.float32(0)
            else:
                organized_pc[i]['x'] = 0.0
                organized_pc[i]['y'] = 0.0
                organized_pc[i]['z'] = 0.0
                organized_pc[i]['rgb'] = np.float32(0)
        organized_pc = organized_pc.reshape((h, w))
        #rospy.loginfo("Reconstructed organized point cloud from depth image (with color).")
        return organized_pc

if __name__ == '__main__':
    try:
        YOLOv8PoseDetector()
    except rospy.ROSInterruptException:
        pass