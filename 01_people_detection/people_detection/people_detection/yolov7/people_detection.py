#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from deploy import Mapping, DeployYOLO, detect_frame
import numpy as np
import time
import threading

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node')
        self.bridge = CvBridge()
        self.pipeline = (
            "thetauvcsrc ! decodebin ! autovideoconvert ! video/x-raw,format=BGRx ! queue ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink"
        )
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            rospy.logerr("Error: Video capture could not be opened.")
            raise RuntimeError('Error: Video capture could not be opened.')

        self.mapping = Mapping(width=1920, shift= 1920 / 2)
        param_path =  rospy.get_param('~file_path') # 'catkin_ws/src/people_detection/yolov7/standard.pt' 
        self.detector = DeployYOLO(param_path, threshold=0.5, classes=['person'])
        self.frame_pub = rospy.Publisher('/camera_frame', Image, queue_size=10)
        self.angle_pub = rospy.Publisher('/desired_angles', Float64MultiArray, queue_size=10)

        self.frame_queue = []
        self.lock = threading.Lock()
        self.stop_thread = False

        rospy.on_shutdown(self.cleanup)

    def cleanup(self):
        self.stop_thread = True
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

    def capture_frames(self):
        while not rospy.is_shutdown() and not self.stop_thread:
            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("Warning: Could not read frame.")
                continue

            with self.lock:
                if len(self.frame_queue) < 10:
                    self.frame_queue.append(frame)

            # Add a small sleep time to give some breathing room
            time.sleep(0.01)

    def process_frames(self):
        while not rospy.is_shutdown() and not self.stop_thread:
            with self.lock:
                if self.frame_queue:
                    frame = self.frame_queue.pop(0)
                else:
                    frame = None

            if frame is not None:
                # Process frame
                frame, pt_1, pt_2, center, angle = detect_frame(self.detector, self.mapping, frame)
                print(self.mapping.shift)

                # Publish processed frame and angle
                self.frame_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
                if angle is not None:
                    angle_array = Float64MultiArray()
                    angle_array.data = angle
                    self.angle_pub.publish(angle_array)

            # Add a small sleep time to give some breathing room
            time.sleep(0.01)

    def run(self):
        capture_thread = threading.Thread(target=self.capture_frames)
        process_thread = threading.Thread(target=self.process_frames)

        capture_thread.start()
        process_thread.start()

        capture_thread.join()
        process_thread.join()

if __name__ == '__main__':
    try:
        node = CameraNode()
        node.run()
    except RuntimeError as e:
        rospy.logerr(e)
