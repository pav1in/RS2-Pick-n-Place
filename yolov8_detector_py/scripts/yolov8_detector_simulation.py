#!/usr/bin/env python3
import numpy as np
# Monkey‑patch for compatibility with ros_numpy
np.float = np.float64

import rospy, cv2
import numpy as np  # reimport uses the monkey‑patched version
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from ultralytics import YOLO
from image_geometry import PinholeCameraModel
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import ros_numpy  # for point cloud conversion

class YOLOv8PoseDetectorSim:
    def __init__(self):
        rospy.init_node('yolov8_pose_detector_sim', anonymous=True)

        # Load the YOLOv8 ONNX model (override with rosparam if needed)
        model_path = rospy.get_param('~model_path',
                                     '/home/megha/git/RS2/yolov8_detector_py/models/simulation/best.onnx')
        rospy.loginfo("Loading YOLOv8 model from: %s", model_path)
        self.model = YOLO(model_path)

        # Helpers and state
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.color_image = None
        self.depth_image = None
        self.point_cloud = None
        self.info_ready = False
        self.depth_scale = 0.001  # adjust if your depth is in mm
        self.img_width = 0
        self.img_height = 0

        # Publishers
        self.pose_pub = rospy.Publisher('/detected_object_pose', PoseStamped, queue_size=1)
        self.roi_pc_pub = rospy.Publisher('/segmented_roi', PointCloud2, queue_size=1)
        self.marker_pub = rospy.Publisher('/roi_marker', Marker, queue_size=1)

        # Subscribers (simulation topics)
        rospy.Subscriber('/camera/rgb/rgb_camera/camera_info', CameraInfo,
                         self.cam_info_cb, queue_size=1)
        rospy.Subscriber('/camera/depth/depth_camera/depth/image_raw', Image,
                         self.depth_cb, queue_size=1)
        rospy.Subscriber('/camera/rgb/rgb_camera/image_raw', Image,
                         self.color_cb, queue_size=1)
        rospy.Subscriber('/camera/depth/depth_camera/points', PointCloud2,
                         self.pc_cb, queue_size=1)

        rospy.loginfo("YOLOv8PoseDetectorSim ready.")
        rospy.spin()

    def cam_info_cb(self, info: CameraInfo):
        if not self.info_ready:
            self.cam_model.fromCameraInfo(info)
            self.img_width = info.width
            self.img_height = info.height
            self.info_ready = True
            rospy.loginfo("Camera info received: %dx%d", self.img_width, self.img_height)

    def depth_cb(self, msg: Image):
        if not self.info_ready:
            return
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr("Depth cv_bridge error: %s", e)

    def color_cb(self, msg: Image):
        if not self.info_ready:
            return
        try:
            self.color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            frame = self.color_image.copy()
        except CvBridgeError as e:
            rospy.logerr("Color cv_bridge error: %s", e)
            return

        # Run inference
        results = self.model(frame)

        for i, (box, cls) in enumerate(zip(results[0].boxes.xyxy, results[0].boxes.cls)):
            x1, y1, x2, y2 = map(int, box.tolist())
            h, w, _ = frame.shape
            x1, x2 = np.clip([x1, x2], 0, w-1)
            y1, y2 = np.clip([y1, y2], 0, h-1)

            # Center pixel depth
            u, v = (x1 + x2)//2, (y1 + y2)//2
            Zc = float(self.depth_image[v, u]) * self.depth_scale

            # Try ROI‐based point cloud median
            Xc = Yc = None
            if self.point_cloud is not None:
                roi = self.point_cloud[y1:y2, x1:x2]
                mask = np.isfinite(roi['z'])
                pts = roi[mask]
                if pts.size:
                    Xc = np.median(pts['x'])
                    Yc = np.median(pts['y'])
                    Zc = np.median(pts['z'])
                    # publish ROI cloud
                    roi_msg = ros_numpy.point_cloud2.array_to_pointcloud2(
                        pts, stamp=msg.header.stamp, frame_id=msg.header.frame_id)
                    self.roi_pc_pub.publish(roi_msg)

            # fallback to center‐ray if needed
            if Xc is None or Yc is None:
                ray = self.cam_model.projectPixelTo3dRay((u, v))
                Xc, Yc = ray[0]*Zc, ray[1]*Zc

            # publish pose
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = Xc
            ps.pose.position.y = Yc
            ps.pose.position.z = Zc
            qx, qy, qz, qw = quaternion_from_euler(0,0,0)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            self.pose_pub.publish(ps)

            
            # marker
            m = Marker()
            m.header = msg.header
            m.ns = "roi_marker"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = ps.pose
            m.scale.x = m.scale.y = m.scale.z = 0.05
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            self.marker_pub.publish(m)

            # draw on image
            cls_id = int(cls)
            cls_name = self.model.names[cls_id]
            conf = results[0].boxes.conf[i] * 100

                        # ——— NOW we can safely log ———
            rospy.loginfo(
                "[sim] Detected %s (%.1f%%) → X=%.3f, Y=%.3f, Z=%.3f",
                cls_name, conf, Xc, Yc, Zc
            )
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, f"{cls_name} {conf:.0f}%", (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                        

        cv2.imshow("YOLOv8 (sim)", frame)
        cv2.waitKey(1)

    def pc_cb(self, msg: PointCloud2):
        try:
            pc = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
            if msg.height == 1 and self.img_width and self.img_height:
                if pc.size == self.img_width*self.img_height:
                    pc = pc.reshape((self.img_height, self.img_width))
                else:
                    # fallback to reconstruct from depth
                    pc = self._reconstruct_pc()
            self.point_cloud = pc
        except Exception as e:
            rospy.logerr("PC convert error: %s", e)

    def _reconstruct_pc(self):
        h, w = self.img_height, self.img_width
        arr = np.zeros((h*w,), dtype=[('x', np.float32),('y',np.float32),
                                       ('z',np.float32),('rgb',np.float32)])
        depth = self.depth_image * self.depth_scale
        for i in range(h*w):
            u = i % w; v = i // w
            Z = depth[v,u]
            if Z > 0:
                ray = self.cam_model.projectPixelTo3dRay((u, v))
                arr[i]['x'] = ray[0] * Z
                arr[i]['y'] = ray[1] * Z
                arr[i]['z'] = Z

                # pack RGB into an integer
                if self.color_image is not None:
                    b, g, r = self.color_image[v, u]
                    rgb_int = (r << 16) | (g << 8) | b
                    arr[i]['rgb'] = np.float32(rgb_int)
        return arr.reshape((h, w))


if __name__ == '__main__':
    try:
        YOLOv8PoseDetectorSim()
    except rospy.ROSInterruptException:
        pass
