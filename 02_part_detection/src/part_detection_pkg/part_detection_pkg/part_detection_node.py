#!/usr/bin/env python3 

import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from part_detection_interfaces.srv import DetectObjects, FindObject, FindObjectExact, SearchObject
from part_detection.detector import Detector
import numpy as np
from cv_bridge import CvBridge
import cv2
import json
from collections import deque


class PartDetection(Node):
    def __init__(self):
        super().__init__('part_detection')
        self.declare_parameter('depth_image_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/camera/camera/depth/camera_info')
        self.declare_parameter('rgb_image_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('rgb_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('depth_plane_threshold', 15)
        self.declare_parameter('depth_dilation', [20, 20])
        self.declare_parameter('topics', True)
        self.declare_parameter('services', True)
        self.declare_parameter('depth_area_threshold', 5000)
        self.declare_parameter('areas', json.dumps([(1, 8200, 50), (2, 22000, 50)]))
        self.declare_parameter('num_stack', 5)
        self.declare_parameter('detetion_distance_threshold', 325)
        self.declare_parameter('rgb_blob_max_pairing_distance', 50)
        self.declare_parameter('rgb_circle_max_pairing_distance', 50)
        self.declare_parameter('rgb_min_keypoint_size', 27)
        self.declare_parameter('rgb_blob_min_threshold', 10)
        self.declare_parameter('rgb_blob_max_threshold', 150)
        self.declare_parameter('rgb_blob_threshold_step', 10)
        self.declare_parameter('rgb_blob_min_area', 150)
        self.declare_parameter('rgb_blob_min_circularity', 0.6)
        self.declare_parameter('rgb_circle1_min_radius', 35)
        self.declare_parameter('rgb_circle1_max_radius', 60)
        self.declare_parameter('rgb_circle2_min_radius', 70)
        self.declare_parameter('rgb_circle2_max_radius', 110)
        self.declare_parameter('rgb_medianBlurKSize', 5)
        self.declare_parameter('rgb_cannyThreshold1', 20)
        self.declare_parameter('rgb_cannyThreshold2', 40)
        self.declare_parameter('rgb_gaussianBlurKSize', (5, 5))
        self.declare_parameter('rgb_houghDP', 1)
        self.declare_parameter('rgb_houghMinDist', 100)
        self.declare_parameter('rgb_houghParam1', 100)
        self.declare_parameter('rgb_houghParam2', 30)
        self.declare_parameter('depth2rgbx', 1.36)
        self.declare_parameter('depth2rgby', 0.98)
        self.declare_parameter('offsetx', 0.8)
        self.declare_parameter('offsety', 0.0)
        self.declare_parameter('home', [400.0, -280.0, 625.0, 0.0, 3.543, 0.0])
        self.declare_parameter('height_1', 475)
        self.declare_parameter('end_height', 350)
        self.declare_parameter('transformation', [37.2, 32.1, 0.0, 0.0, 0.401, 0.0])
        


        self.dept_image_subscription = self.create_subscription(
            Image, 
            self.get_parameter('depth_image_topic').value, 
            self.depth_image_callback, 
            10
        )

        self.dept_info_subscription = self.create_subscription(
            CameraInfo,
            self.get_parameter('depth_info_topic').value,
            self.dept_info_callback,
            10
        )

        self.rgb_image_subscription = self.create_subscription(
            Image, 
            self.get_parameter('rgb_image_topic').value, 
            self.rgb_image_callback, 
            10
        )

        self.rgb_info_subscription = self.create_subscription(
            CameraInfo,
            self.get_parameter('rgb_info_topic').value,
            self.rgb_info_callback,
            10
        )

        if self.get_parameter('topics').value:

            self.visual_result_publisher = self.create_publisher(
                Image,
                'part_detection/visual_results', 
                10
            )

            self.result_publisher = self.create_publisher(
                String,
                'part_detection/detection_results', 
                10
            )

            self.timer = self.create_timer(0.03, self.prepare_topics)

        if self.get_parameter('services').value:

            self.detect_service = self.create_service(
                DetectObjects,
                'part_detection/detect',
                self.detect_callback_service
            )

            self.find_object_= self.create_service(
                FindObject,
                'part_detection/find_object',
                self.handle_find_object
            )

            self.find_object_exact = self.create_service(
                FindObjectExact,
                'part_detection/find_object_exact',
                self.handle_find_object_exact
            )

            self.search_object = self.create_service(
                SearchObject,
                'part_detection/search_object',
                self.handle_search_object
            )


        self.depth_bridge = CvBridge()
        self.rgb_bridge = CvBridge()

        params = cv2.SimpleBlobDetector_Params()
        params.minThreshold = self.get_parameter('rgb_blob_min_threshold').value
        params.maxThreshold = self.get_parameter('rgb_blob_max_threshold').value
        params.thresholdStep = self.get_parameter('rgb_blob_threshold_step').value
        params.filterByArea = True
        params.minArea = self.get_parameter('rgb_blob_min_area').value
        params.filterByCircularity = True
        params.minCircularity = self.get_parameter('rgb_blob_min_circularity').value 

        rgb_circle_params = {
            'C1minRadius': self.get_parameter('rgb_circle1_min_radius').value,
            'C1maxRadius': self.get_parameter('rgb_circle1_max_radius').value,
            'C2minRadius': self.get_parameter('rgb_circle2_min_radius').value,
            'C2maxRadius': self.get_parameter('rgb_circle2_max_radius').value,
            'medianBlurKSize': self.get_parameter('rgb_medianBlurKSize').value,
            'cannyThreshold1': self.get_parameter('rgb_cannyThreshold1').value,
            'cannyThreshold2': self.get_parameter('rgb_cannyThreshold2').value,
            'gaussianBlurKSize': tuple(self.get_parameter('rgb_gaussianBlurKSize').value),
            'houghDP': self.get_parameter('rgb_houghDP').value,
            'houghMinDist': self.get_parameter('rgb_houghMinDist').value,
            'houghParam1': self.get_parameter('rgb_houghParam1').value,
            'houghParam2': self.get_parameter('rgb_houghParam2').value
        }

        dept_to_rgb_params = {
            'offsetx': self.get_parameter('offsetx').value,
            'offsety': self.get_parameter('offsety').value,
            'depth2rgbx': self.get_parameter('depth2rgbx').value,
            'depth2rgby': self.get_parameter('depth2rgby').value
        }

        self.detector = Detector(
            depth_area_threshold=self.get_parameter('depth_area_threshold').value,
            depth_dilation=tuple(self.get_parameter('depth_dilation').value),
            depth_areas=json.loads(self.get_parameter('areas').get_parameter_value().string_value),
            depth_plane_threshold=self.get_parameter('depth_plane_threshold').value,
            rgb_detection_params=params,
            rgb_min_keypoint_size=self.get_parameter('rgb_min_keypoint_size').value,
            rgb_circle_detection_params=rgb_circle_params,
            dept_to_rgb_params=dept_to_rgb_params,
            max_pairing_distance_blob=self.get_parameter('rgb_blob_max_pairing_distance').value,
            max_pairing_distance_circle=self.get_parameter('rgb_circle_max_pairing_distance').value,
            detection_distance_threshold=self.get_parameter('detetion_distance_threshold').value
            )
        
        self.part_detection_logic = PartDetectionLogic(
            transformation=self.get_parameter('transformation').value,
            home=self.get_parameter('home').value,
            height_1=self.get_parameter('height_1').value,
            end_height=self.get_parameter('end_height').value
        )

        self.depth_images = list()
        self.rgb_images = list() 
        self.rgb_fx = None
        self.rgb_fy = None
        self.depth_fx = None
        self.depth_fy = None
        self.camera_height = None

    def rgb_info_callback(self, msg):
        self.rgb_fx = msg.k[0]
        self.rgb_fy = msg.k[4]

    def dept_info_callback(self, msg):
        self.dept_fx = msg.k[0]
        self.dept_fy = msg.k[4]

    def rgb_image_callback(self, msg):
        rgb_image = self.rgb_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.rgb_images.append(rgb_image)
        if len(self.rgb_images) > self.get_parameter('num_stack').value:
            self.rgb_images.pop(0)
        # print("RGB Queue Length:", len(self.rgb_images))

    def depth_image_callback(self, msg):
        depth_image = self.depth_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        self.depth_images.append(depth_image)
        if len(self.depth_images) > self.get_parameter('num_stack').value:
            self.depth_images.pop(0)
        # print("Depth Queue Length:", len(self.depth_images))

    def detect_callback_service(self, request, response):
        if request.camera_height:
            camera_height = request.camera_height
        else:
            camera_height = self.camera_height
        objects = self.detect_objects(camera_height)

        results = {
            'labels': [label.item() if isinstance(label, np.generic) else label for label in objects['labels']],
            'deltas': [[delta.item() if isinstance(delta, np.generic) else delta for delta in pair] for pair in objects['deltas_mm']],
            'areas': [area.item() if isinstance(area, np.generic) else area for area in objects['areas']]
        }
        response.json_results = json.dumps(results)
        return response
    
    def handle_find_object(self, request, response):
        detected_objects = self.detect_objects(self.camera_height)
        adjusted_deltas, found = self.part_detection_logic.find_object(request.label, detected_objects)
        
        response.deltas = adjusted_deltas
        response.found = found
        return response

    def handle_find_object_exact(self, request, response):
        detected_objects = self.detect_objects(self.camera_height)
        adjusted_deltas, done = self.part_detection_logic.find_object_exact(detected_objects)
        
        response.deltas = adjusted_deltas
        response.done = done
        return response

    def handle_search_object(self, request, response):
        detected_objects = self.detect_objects(self.camera_height)
        deltas_list = self.part_detection_logic.search_object(detected_objects)
        
        # JSON-Dump der Deltas-Liste
        response.json_deltas = json.dumps(deltas_list)
        return response

    def detect_objects(self, camera_height=None):
        camera_height = camera_height if camera_height else self.camera_height

        objects = self.detector.predict_batch(self.rgb_images, self.depth_images, camera_height)
        objects['deltas_mm'] = self.calculate_delta_mm(
            objects['deltas'],
            objects['camera_height'],
            objects['object_heights']
        )
        return objects
    
    def prepare_topics(self):
        if len(self.depth_images) < self.get_parameter('num_stack').value and len(self.rgb_images) < self.get_parameter('num_stack').value:
            return
        objects = self.detect_objects()
        rgb_image = self.rgb_images[-1].copy()
        annotated_rgb_image = self.draw_detections(rgb_image, objects)
        self.send_detection_results_msg(objects)
        self.send_visual_result(annotated_rgb_image)
        self.depth_images = list()
        self.rgb_images = list()

    def calculate_delta_mm(self, deltas, height, object_heights):
        deltas_mm = []
        for pair, object_height in zip(deltas, object_heights):
            res = (
                round(float((height - object_height) * (pair[0] / self.rgb_fx)), 1),
                round(float((height - object_height) * (pair[1] / self.rgb_fy)), 1),
                round(float(height - object_height), 1)
            )
            deltas_mm.append(res)
        return deltas_mm
    

    def draw_detections(self, rgb_image, objects):
        img = rgb_image.copy()
        centers = objects['centers'][0] if len(objects['centers']) > 0  else list()
        blob_centers = objects['blob_centers']
        blob_radiuses = objects['blob_radiuses']
        circle_centers = objects['circle_centers']
        circle_radiuses = objects['circle_radiuses']
        areas = objects['areas']
        deltas = objects['deltas']
        labels = objects['labels']
        deltas_mm = objects['deltas_mm']

        #print('*****************')
        #print(objects)

        # Draw all detections in a single loop
        for i, (center, blob_center, blob_radius, circle_center, circle_radius, area, delta_mm, label) in enumerate(zip(centers, blob_centers, blob_radiuses, circle_centers, circle_radiuses, areas, deltas_mm, labels)):
            if center is not None:
                if label == 0:
                    color = (0, 0, 255)
                    text = 'unknown object'
                else:
                    color = (0, 255, 0)
                    text = f'Label: {label}'
                #print('area:', area)
                cv2.drawMarker(img, tuple(map(int, center)), color, markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2, line_type=cv2.LINE_AA)
                cv2.putText(img, text, (int(center[0]) - 20, int(center[1]) - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(img, f'Area: {int(area)}', (int(center[0]) - 20, int(center[1]) + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(img, f'Delta mm: ({int(delta_mm[0])}, {int(delta_mm[1])}, {int(delta_mm[2])})', (int(center[0]) - 20, int(center[1]) + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)            
            
            # Draw blob centers and radii
            if blob_center is not None and blob_radius is not None:
                cv2.circle(img, tuple(map(int, blob_center)), int(blob_radius), (0, 255, 255), 2)
                cv2.drawMarker(img, tuple(map(int, blob_center)), (0, 255, 255), markerType=cv2.MARKER_SQUARE, markerSize=20, thickness=2, line_type=cv2.LINE_AA)

            # Draw circle centers and radii
            if circle_center is not None and circle_radius is not None:
                cv2.circle(img, tuple(map(int, circle_center)), int(circle_radius), (255, 0, 0), 2)
                cv2.drawMarker(img, tuple(map(int, circle_center)), (255, 0, 0), markerType=cv2.MARKER_SQUARE, markerSize=20, thickness=2, line_type=cv2.LINE_AA)

        return img
        
    def send_visual_result(self, rgb_image):
        image_msg = self.rgb_bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")
        self.visual_result_publisher.publish(image_msg)

    def send_detection_results_msg(self, objects):
        results = {
            'labels': [label.item() if isinstance(label, np.generic) else label for label in objects['labels']],
            'deltas': [[delta.item() if isinstance(delta, np.generic) else delta for delta in pair] for pair in objects['deltas_mm']],
            'areas': [area.item() if isinstance(area, np.generic) else area for area in objects['areas']]
        }
        json_data = json.dumps(results)
        result_msg = String(data=json_data)
        self.result_publisher.publish(result_msg)


class PartDetectionLogic:
    def __init__(self, transformation, home, height_1, end_height, max_error=2):
        self.transformation = transformation  # [x, y, z, rx, ry, rz]
        self.home = home  # [x, y, z, rx, ry, rz]
        self.height_1 = height_1
        self.end_height = end_height
        self.max_error = max_error

    def calculate_delta_z(self, current_height, target_height):
        """Calculate the Z delta based on the current and target heights."""
        return target_height - current_height

    def find_object(self, label, detected_objects):
        """Find an object by label and return the corresponding deltas including rotations."""
        delta_z = self.calculate_delta_z(self.home[2], self.height_1)
        
        for i, obj_label in enumerate(detected_objects['labels']):
            if obj_label == label:
                deltas = detected_objects['deltas_mm'][i]
                adjusted_deltas = [
                    deltas[0],
                    deltas[1],
                    delta_z,
                    0,  # rx
                    0,  # ry
                    0   # rz
                ]
                return [float(value) for value in adjusted_deltas], True

        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False

    def find_object_exact(self, detected_objects):
        """Refine the object's position by selecting the object with the smallest average delta (rx + ry) / 2."""
        best_deltas = None
        min_average_delta = float('inf')

        # Durchlaufe alle gefundenen Objekte und finde das mit dem kleinsten durchschnittlichen Delta
        for i, deltas in enumerate(detected_objects['deltas_mm']):
            average_delta = (abs(deltas[0]) + abs(deltas[1])) / 2

            if average_delta < min_average_delta:
                min_average_delta = average_delta
                best_deltas = deltas

        # Wenn das beste Delta gefunden wurde, prüfe, ob es innerhalb der Fehlergrenze liegt
        if best_deltas:
            if min_average_delta <= self.max_error:
                # Position ist innerhalb der akzeptablen Fehlergrenzen
                delta_z = self.calculate_delta_z(self.height_1, self.end_height)
                adjusted_deltas = [
                    self.transformation[0],  # Apply transformation X
                    self.transformation[1],  # Apply transformation Y
                    delta_z,                 # Apply delta_z
                    self.transformation[3],  # rx transformation
                    self.transformation[4],  # ry transformation
                    self.transformation[5]   # rz transformation
                ]
                return [float(value) for value in adjusted_deltas], True
            else:
                # Position ist nicht innerhalb der Fehlergrenzen, gib die Deltas zurück mit delta_z = 0
                adjusted_deltas = [
                    best_deltas[0],
                    best_deltas[1],
                    0,  # delta_z is 0 because we stay on the same height
                    0,  # rx remains 0
                    0,  # ry remains 0
                    0   # rz remains 0
                ]
                # adjusted_deltas all float values
                return [float(value) for value in adjusted_deltas], False

        return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], False
    
    def search_object(self, detected_objects):
        """Search for all objects with label 0 and return a list of their deltas."""
        deltas_list = []

        for i, obj_label in enumerate(detected_objects['labels']):
            if obj_label == 0:
                deltas = detected_objects['deltas_mm'][i]
                adjusted_deltas = [
                    deltas[0],
                    deltas[1],
                    0,  # delta_z is 0 because we stay on the same height
                    0,  # rx remains 0
                    0,  # ry remains 0
                    0   # rz remains 0
                ]
                deltas_list.append(adjusted_deltas)

        return deltas_list if deltas_list else []


def main(args=None):
    rclpy.init(args=args)
    image_processor = PartDetection()
    rclpy.spin(image_processor)
    rclpy.shutdown()

if __name__ == '__main__':
    main() 