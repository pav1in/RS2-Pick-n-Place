#!/usr/bin/env python3
import os
import cv2
import numpy as np
np.float = np.float64  # safety alias for older APIs

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from ultralytics import YOLO
import ros2_numpy as rnp

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import (
    Image, CameraInfo, PointCloud2, PointField
)
from sensor_msgs.msg import Image as RosImage
from sensor_msgs_py import point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from image_geometry import PinholeCameraModel
from tf_transformations import quaternion_from_matrix
import tf2_ros

from message_filters import Subscriber, ApproximateTimeSynchronizer

class YOLOv8ObjectDetector(Node):
    def __init__(self):
        super().__init__('yolov8_object_detector')

        # Load YOLOv8 model
        pkg = get_package_share_directory('yolov8_object_detector')
        default_model = os.path.join(pkg, 'models', 'shapes_improved', 'best.onnx')
        self.declare_parameter('model_path', default_model)
        model_path = self.get_parameter('model_path').value
        self.get_logger().info(f"Loading YOLOv8 model from: {model_path}")
        self.model = YOLO(model_path, task='detect')

        # Internal state
        self.bridge       = CvBridge()
        self.cam_model    = PinholeCameraModel()
        self.depth_image  = None
        self.color_image  = None
        self.point_cloud  = None
        self.info_received = False
        self.depth_scale   = 0.001

        # Publishers
        self.pose_pub   = self.create_publisher(PoseStamped, '/detected_object_pose', 10)
        self.marker_pub = self.create_publisher(Marker,       '/roi_marker',           10)
        self.bbox_pub   = self.create_publisher(Marker,       '/obb_marker',           10)
        self.roi_pc_pub = self.create_publisher(PointCloud2,  '/segmented_roi',        10)
        self.img_pub    = self.create_publisher(RosImage,     '/yolo/image',           10)

        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # CameraInfo subscription
        self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info',
            self.cam_info_cb, 10
        )

        # Time‐synced subscribers
        self.color_sub = Subscriber(self, Image,      '/camera/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image,      '/camera/camera/aligned_depth_to_color/image_raw')
        self.pc_sub    = Subscriber(self, PointCloud2,'/camera/camera/depth/color/points')
        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.pc_sub],
            queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.synced_cb)

        self.get_logger().info("YOLOv8ObjectDetector ready.")

    def cam_info_cb(self, msg: CameraInfo):
        if not self.info_received:
            self.cam_model.fromCameraInfo(msg)
            self.img_height, self.img_width = msg.height, msg.width
            self.info_received = True

    def synced_cb(self, color_msg, depth_msg, pc_msg):
        if not self.info_received:
            return

        # 1) Convert color & depth
        try:
            frame = self.bridge.imgmsg_to_cv2(color_msg, 'bgr8')
            self.color_image = frame.copy()
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # 2) Build structured point-cloud array
        try:
            pc_dict = rnp.numpify(pc_msg)
            xyz = pc_dict.get('xyz', None)
            rgb = pc_dict.get('rgb', None)
            h, w = self.img_height, self.img_width

            if xyz is not None and xyz.ndim == 2 and xyz.shape == (h*w, 3):
                arr = np.zeros((h, w),
                               dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
                xyz3 = xyz.reshape((h, w, 3))
                arr['x'], arr['y'], arr['z'] = xyz3[:,:,0], xyz3[:,:,1], xyz3[:,:,2]
                if rgb is not None and rgb.shape[0] == h*w:
                    arr['rgb'] = rgb.reshape((h, w))
                self.point_cloud = arr
            else:
                self.point_cloud = self._reconstruct_pc()
        except Exception as e:
            self.get_logger().error(f"PointCloud conversion: {e}")
            self.point_cloud = self._reconstruct_pc()

        # 3) CLAHE for lighting normalization
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l,a,b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        frame_eq = cv2.cvtColor(
            cv2.merge((clahe.apply(l),a,b)), cv2.COLOR_LAB2BGR
        )

        # 4) YOLO inference
        res = self.model(frame_eq, conf=0.5, iou=0.45)[0]

        # 5) Clear old markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        self.marker_pub.publish(delete_all)
        self.bbox_pub.publish(delete_all)

        # 6) Loop detections
        for i, box in enumerate(res.boxes.xyxy):
            cls_id = int(res.boxes.cls[i])
            conf   = float(res.boxes.conf[i]) * 100.0
            x1, y1, x2, y2 = map(int, box)
            u, v = (x1 + x2)//2, (y1 + y2)//2
            Zc = float(self.depth_image[v, u]) * self.depth_scale
            if not (0.01 < Zc < 5.0):
                continue

            # Extract ROI
            h_img, w_img, _ = frame.shape
            x1c, x2c = np.clip([x1, x2], 0, w_img-1)
            y1c, y2c = np.clip([y1, y2], 0, h_img-1)
            roi = self.point_cloud[y1c:y2c, x1c:x2c]
            valid = roi[np.isfinite(roi['z'])]

            if valid.size:
                # Axis-aligned size
                xs, ys, zs = valid['x'], valid['y'], valid['z']
                size_aa = [
                    float(xs.max()  - xs.min()),
                    float(ys.max()  - ys.min()),
                    float(zs.max()  - zs.min())
                ]

                # PCA for OBB
                pts = np.vstack([xs, ys, zs]).T
                centroid = pts.mean(axis=0)
                cov = np.cov(pts.T)
                e_vals, e_vecs = np.linalg.eigh(cov)
                order = e_vals.argsort()[::-1]
                axes = e_vecs[:,order]
                projs = (pts - centroid) @ axes
                mn, mx = projs.min(axis=0), projs.max(axis=0)
                extents = [float(mx[j] - mn[j]) for j in range(3)]

                # Quaternion from matrix
                M = np.eye(4)
                M[:3,:3] = axes
                quat = quaternion_from_matrix(M)  # tuple of 4 floats

                # Build PointField list correctly
                fields = []
                for name, offset in [('x',0), ('y',4), ('z',8), ('rgb',12)]:
                    pf = PointField()
                    pf.name     = name
                    pf.offset   = offset
                    pf.datatype = PointField.FLOAT32
                    pf.count    = 1
                    fields.append(pf)

                pts_list = [
                    [float(xx), float(yy), float(zz), float(rgb)]
                    for xx,yy,zz,rgb in zip(xs, ys, zs, valid['rgb'])
                ]
                cloud = pc2.create_cloud(color_msg.header, fields, pts_list)
                self.roi_pc_pub.publish(cloud)

                cx, cy, cz = map(float, centroid)
            else:
                # fallback ray
                rx, ry, rz = self.cam_model.projectPixelTo3dRay((u, v))
                cx, cy, cz = float(rx*Zc), float(ry*Zc), float(rz*Zc)
                extents = [0.0, 0.0, 0.0]
                quat = (0.0, 0.0, 0.0, 1.0)
                size_aa = [0.0, 0.0, 0.0]

            # Broadcast TF
            tf = TransformStamped()
            tf.header = color_msg.header
            tf.child_frame_id = f"object_{i}"
            tf.transform.translation.x = cx
            tf.transform.translation.y = cy
            tf.transform.translation.z = cz
            tf.transform.rotation.x = float(quat[0])
            tf.transform.rotation.y = float(quat[1])
            tf.transform.rotation.z = float(quat[2])
            tf.transform.rotation.w = float(quat[3])
            self.tf_broadcaster.sendTransform(tf)

            # Publish PoseStamped
            ps = PoseStamped(header=color_msg.header)
            ps.pose.position.x = cx
            ps.pose.position.y = cy
            ps.pose.position.z = cz
            # identity orientation
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0
            self.pose_pub.publish(ps)

            # Sphere marker
            sph = Marker(header=ps.header, ns='det', id=i,
                         type=Marker.SPHERE, action=Marker.ADD)
            sph.pose = ps.pose
            sph.scale.x = sph.scale.y = sph.scale.z = 0.05
            sph.color.a = 1.0; sph.color.r = 1.0
            self.marker_pub.publish(sph)

            # OBB cube marker
            cub = Marker(header=ps.header, ns='obb', id=i,
                         type=Marker.CUBE, action=Marker.ADD)
            cub.pose.position.x = cx
            cub.pose.position.y = cy
            cub.pose.position.z = cz
            cub.pose.orientation.x = float(quat[0])
            cub.pose.orientation.y = float(quat[1])
            cub.pose.orientation.z = float(quat[2])
            cub.pose.orientation.w = float(quat[3])
            cub.scale.x = extents[0]
            cub.scale.y = extents[1]
            cub.scale.z = extents[2]
            cub.color.a = 0.3; cub.color.g = 1.0
            self.bbox_pub.publish(cub)

            # Annotate 2D image & log
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            lbl = f"{self.model.names[cls_id]} {conf:.0f}%"
            cv2.putText(frame, lbl, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
            self.get_logger().info(f"{lbl} AA={size_aa} OBB={extents}")

        # Publish annotated image + imshow
        out_img = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_img.header = color_msg.header
        self.img_pub.publish(out_img)
        cv2.imshow("YOLOv8 Detections", frame)
        cv2.waitKey(1)

    def _reconstruct_pc(self):
        """Fallback: organized cloud from depth + intrinsics."""
        h, w = self.img_height, self.img_width
        pc = np.zeros((h*w,),
                      dtype=[('x','f4'),('y','f4'),('z','f4'),('rgb','f4')])
        Zm = self.depth_image * self.depth_scale
        for idx in range(h*w):
            u, v = idx % w, idx // w
            Z = Zm[v, u]
            if Z > 0:
                ray = self.cam_model.projectPixelTo3dRay((u, v))
                pc[idx]['x'], pc[idx]['y'], pc[idx]['z'] = ray[0]*Z, ray[1]*Z, Z
                B, G, R = self.color_image[v, u]
                pc[idx]['rgb'] = float((int(R)<<16)|(int(G)<<8)|int(B))
        return pc.reshape((h, w))


def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8ObjectDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
