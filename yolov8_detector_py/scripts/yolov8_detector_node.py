#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import cv2
from ultralytics import YOLO  # Make sure ultralytics is installed (pip install ultralytics)

class YOLOv8Detector:
    def __init__(self):
        rospy.init_node('yolov8_detector_node', anonymous=True)
        self.bridge = CvBridge()
        # Load the YOLOv8 model (adjust the model path as needed)
        model_path = rospy.get_param('~model_path', '/home/megha/git/RS2/yolov8_detector_py/models/default/yolov8n.onnx')
        
        rospy.loginfo("Loading YOLOv8 model from: %s", model_path)
        self.model = YOLO(model_path)  # This loads the PyTorch model

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        # Publisher for detection results (example: a string message)
        self.detection_pub = rospy.Publisher('/yolov8_py/detections', String, queue_size=1)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        # Run YOLOv8 inference on the image
        results = self.model(cv_image)

        # Process the results (this is a placeholder - adjust as needed)
        detection_text = "Detections: " + str(len(results[0].boxes))
        rospy.loginfo(detection_text)
        self.detection_pub.publish(detection_text)

        # Optionally, display the image with detections drawn
        annotated_frame = results[0].plot()  # YOLOv8 results can plot detections
        cv2.imshow("YOLOv8 Detections", annotated_frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        detector = YOLOv8Detector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
