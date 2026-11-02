#!/usr/bin/python3
# -*- coding: utf-8 -*-

from sensor_msgs.msg import Image, CameraInfo
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
import cv2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np

class ArucoDetector(Node):
    """
    Example of a class that detects Aruco markers in an image.
    """

    def __init__(self, node_name="aruco_detector", namespace=""):
        super().__init__(node_name)
        
        # Define the QoS profile
        self.qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Keep only the last message
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT
        )
        self.image_sub = self.create_subscription(
            Image, "bottom_camera_sensor/image_raw", self.image_callback, self.qos_profile
        )
        self.camera_info_subs = self.create_subscription(
            CameraInfo, 'bottom_camera_sensor/camera_info', self.camera_info_callback, 10
        )
        
        self.timer = self.create_timer(0.1, self.timer_callback)

        
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        self.cv_image = np.zeros((480, 640, 3), np.uint8)

    
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d)
    
    def timer_callback(self):
        self.process_image(self.cv_image)
        # Publish the image
        # try:
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        # except CvBridgeError as e:
        #     print(e)
            
    def image_callback(self, msg):
        # print("Odom Callback")
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            
            
    def process_image(self, cv_image):  
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray =  gray[190:250,290:355]
        # gray = cv2.equalizeHist(gray)
        # gray = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
        # _, gray = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        print(ids)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                cv2.polylines(cv_image, [corners[i].astype(int)], isClosed=True, color=(0, 255, 0), thickness=2)
                # Put the Aruco marker ID text near the marker
                corner = corners[i][0][0]
                cv2.putText(cv_image, str(marker_id), (int(corner[0]), int(corner[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Image Detection', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
