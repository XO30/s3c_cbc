import copy
import json
import os
import time

import cv2
import numpy as np
import state_representation as sr
from cv_bridge import CvBridge
from modulo_components.component import Component
from pyquaternion import Quaternion
from sensor_msgs.msg import CameraInfo, Image
from yaml import YAMLError, safe_load

from s3c_part_detection.detector import Detector
from s3c_part_detection.part_detection_logic import PartDetectionLogic


class PartDetector(Component):
    def __init__(self, node_name: str = "part_detector", *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)

        self._num_stack = sr.Parameter("num_stack", 5, sr.ParameterType.INT)
        self.add_parameter(
            "_num_stack", "Number of images to stack for averaging and reducing noice in image processing")
        self.add_parameter(sr.Parameter("config_file", sr.ParameterType.STRING),
                           "The configuration file for detection parameters")
        self.add_parameter(sr.Parameter("robot_base_frame", sr.ParameterType.STRING),
                           "The name of the robot base frame in the URDF")

        self.add_parameter(sr.Parameter("depth_image_topic", "~/depth_image",
                           sr.ParameterType.STRING), "Depth image topic")
        self.add_parameter(sr.Parameter("depth_info_topic", "~/depth_info",
                           sr.ParameterType.STRING), "Depth info topic")
        self.add_parameter(sr.Parameter("rgb_image_topic", "~/rgb_image", sr.ParameterType.STRING), "RGB image topic")
        self.add_parameter(sr.Parameter("rgb_info_topic", "~/rgb_info", sr.ParameterType.STRING), "RGB info topic")

        self.add_parameter(sr.Parameter("visual_results_topic", "~/visual_results",
                           sr.ParameterType.STRING), "Visual results topic")

        self._bridge = CvBridge()
        self._depth_images = []
        self._rgb_images = []
        self._depth_fx = None
        self._depth_fy = None
        self._rgb_fx = None
        self._rgb_fy = None
        self._detector: Detector
        self._logic: PartDetectionLogic
        self._current_label = None
        self._home = sr.CartesianPose()
        self._pose = sr.CartesianPose()

        self.add_static_tf_broadcaster()

        if not self.init():
            raise RuntimeError("Failed to initialize component")

        self._depth_image_sub = self.create_subscription(
            Image, self.get_parameter_value("depth_image_topic"),
            self._depth_image_cb, self.get_qos())
        self._depth_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter_value("depth_info_topic"),
            self._depth_info_cb, self.get_qos())
        self._rgb_image_sub = self.create_subscription(
            Image, self.get_parameter_value("rgb_image_topic"),
            self._rgb_image_cb, self.get_qos())
        self._rgb_info_sub = self.create_subscription(
            CameraInfo, self.get_parameter_value("rgb_info_topic"),
            self._rgb_info_cb, self.get_qos())

        self._visual_results_pub = self.create_publisher(
            Image, self.get_parameter_value("visual_results_topic"), self.get_qos())
        
        self.add_predicate("stage_found", False)
        self.add_predicate("stage_done", False)
        self.add_predicate("pickup_published", False)
        self.add_service("initial_detection", self._initial_detection)
        self.add_service("exact_detection", self._exact_detection)
        self.add_service("publish_pickup", self._publish_pickup)
        self.add_service("reset", self._reset)
    
    def _reset(self):
        self.set_predicate("stage_found", False)
        self.set_predicate("stage_done", False)
        self.set_predicate("pickup_published", False)
        self._pose = copy.deepcopy(self._home)
        return {"success": True, "message": ""}

    def init(self) -> bool:
        try:
            with open(self.get_parameter_value("config_file"), "r") as file:
                try:
                    config = safe_load(file)
                except YAMLError as e:
                    self.get_logger().error(
                        f"Failed to load config file from {self.get_parameter_value('config_file')}: {e}")
                    return False

            params = cv2.SimpleBlobDetector_Params()
            params.minThreshold = config["rgb_blob_min_threshold"]
            params.maxThreshold = config["rgb_blob_max_threshold"]
            params.thresholdStep = config["rgb_blob_threshold_step"]
            params.filterByArea = True
            params.minArea = config["rgb_blob_min_area"]
            params.filterByCircularity = True
            params.minCircularity = config["rgb_blob_min_circularity"]

            rgb_circle_params = {
                'C1minRadius': config["rgb_circle1_min_radius"],
                'C1maxRadius': config["rgb_circle1_max_radius"],
                'C2minRadius': config["rgb_circle2_min_radius"],
                'C2maxRadius': config["rgb_circle2_max_radius"],
                'medianBlurKSize': config["rgb_medianBlurKSize"],
                'cannyThreshold1': config["rgb_cannyThreshold1"],
                'cannyThreshold2': config["rgb_cannyThreshold2"],
                'gaussianBlurKSize': tuple(config["rgb_gaussianBlurKSize"]),
                'houghDP': config["rgb_houghDP"],
                'houghMinDist': config["rgb_houghMinDist"],
                'houghParam1': config["rgb_houghParam1"],
                'houghParam2': config["rgb_houghParam2"]
            }

            depth_to_rgb_params = {
                'offsetx': config["offsetx"],
                'offsety': config["offsety"],
                'depth2rgbx': config["depth2rgbx"],
                'depth2rgby': config["depth2rgby"]
            }

            self._detector = Detector(
                depth_area_threshold=config["depth_area_threshold"],
                depth_dilation=tuple(config["depth_dilation"]),
                depth_areas=json.loads(config["areas"]),
                depth_plane_threshold=config["depth_plane_threshold"],
                rgb_detection_params=params,
                rgb_min_keypoint_size=config["rgb_min_keypoint_size"],
                rgb_circle_detection_params=rgb_circle_params,
                dept_to_rgb_params=depth_to_rgb_params,
                max_pairing_distance_blob=config["rgb_blob_max_pairing_distance"],
                max_pairing_distance_circle=config["rgb_circle_max_pairing_distance"],
                detection_distance_threshold=config["detetion_distance_threshold"]
            )

            self._logic = PartDetectionLogic(
                transformation=config["transformation"],
                home=config["home"],
                height_1=config["height_1"],
                end_height=config["end_height"],
                max_error=config["max_error"]
            )

            self._home = sr.CartesianPose("home", self.get_parameter_value("robot_base_frame"))
            self._home.set_position([pos / 1000.0 for pos in config["home"][:3]])
            self._home.set_orientation(self.quat_from_rot_vec(config["home"][3:]))
            self._pose = copy.deepcopy(self._home)
            self.send_static_transform(self._home)

        except Exception as e:
            self.get_logger().error(f"Failed to create detector and logic: {e}")
            return False
        return True

    @staticmethod
    def quat_from_rot_vec(rot_vec):
        vec = np.asarray(rot_vec)
        norm = np.linalg.norm(vec)
        if norm == 0:
            return Quaternion()
        return Quaternion(axis=vec / norm, radians=norm)

    def _depth_image_cb(self, msg: Image):
        self._depth_images.append(self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").copy())
        if len(self._depth_images) > self._num_stack.get_value():
            self._depth_images.pop(0)
        self.get_logger().debug("Received new depth image", throttle_duration_sec=5.0)

    def _depth_info_cb(self, msg: CameraInfo):
        self._depth_fx = msg.k[0]
        self._depth_fy = msg.k[4]

    def _rgb_image_cb(self, msg: Image):
        self._rgb_images.append(self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").copy())
        if len(self._rgb_images) > self._num_stack.get_value():
            self._rgb_images.pop(0)
        self.get_logger().debug("Received new RGB image", throttle_duration_sec=5.0)

    def _rgb_info_cb(self, msg: CameraInfo):
        self._rgb_fx = msg.k[0]
        self._rgb_fy = msg.k[4]

    def _initial_detection(self, payload):
        if payload == "stage_2":
            label = 2
        elif payload == "stage_3":
            label = 1
        else:
            response = {"success": False, "message": "Invalid payload, only 'stage_2' or 'stage_3' supported"}
            return response

        if len(self._depth_images) < self._num_stack.get_value() or len(self._rgb_images) < self._num_stack.get_value():
            response = {"success": False, "message": "Not enough images to run detection"}
            return response

        objects = []
        indices = []
        for i in range(5):
            objects = self._detect_objects()
            self.get_logger().debug(f"{objects}")
            for j, obj_label in enumerate(objects["labels"]):
                if label == obj_label:
                    indices.append(j)
            if len(indices):
                break
            time.sleep(0.5)
        self._publish_detections(self._rgb_images[-1].copy(), objects)

        if not len(indices):
            response = {"success": False, "message": f"Could not find {payload} in current image"}
            return response
        elif len(indices) > 1:
            response = {"success": False, "message": f"Found several objects with label {payload} in current image"}
            return response

        deltas = self._logic.extract_initial_deltas(objects, indices[0])
        self._pose.set_position(self._pose.get_position() - np.asarray(deltas[:3]) / 1000.0)
        self._pose.set_name("initial_detection")
        self.send_static_transform(self._pose)
        self.set_predicate("stage_found", True)
        self._current_label = label
        return {"success": True, "message": f"{payload} successfully detected"}

    def _exact_detection(self):
        if self._current_label is None:
            response = {"success": False, "message": "Cannot perform exact detection, run initial detection first"}
            return response

        objects = []
        indices = []
        for i in range(5):
            objects = self._detect_objects()
            self.get_logger().debug(f"{objects}")
            for j, obj_label in enumerate(objects["labels"]):
                if self._current_label == obj_label:
                    indices.append(j)
            if len(indices):
                break
            time.sleep(0.5)
        self._publish_detections(self._rgb_images[-1].copy(), objects)

        if not len(indices):
            response = {"success": False, "message": f"Could not find stage in current image"}
            return response
        elif len(indices) > 1:
            response = {"success": False, "message": f"Found several objects in current image"}
            return response

        deltas, done = self._logic.extract_final_deltas(objects, indices[0])
        self.set_predicate("stage_done", done)
        if not done:
            self._pose.set_position(self._pose.get_position() - np.asarray(deltas[:3]) / 1000.0)
            self._pose.set_name("exact_detection")
            self.send_static_transform(self._pose)
            return {"success": True, "message": "Run exact detection again"}
        else:
            self._pose.set_position(self._pose.get_position() - np.asarray(deltas[:3]) / 1000.0)
            self._pose.set_orientation(self._pose.get_orientation() * self.quat_from_rot_vec(deltas[3:]))
            self._pose.set_name("exact_detection")
            self.send_static_transform(self._pose)
            return {"success": True, "message": "Final pose of stage detected"}

    def _publish_pickup(self, payload):
        if not self.get_predicate("stage_done"):
            return {"success": False, "message": "Cannot publish pickup before stage was detected"}
        position = self._pose.get_position()
        position[2] = float(payload)
        self._pose.set_position(position)
        self._pose.set_name("pickup")
        self.send_static_transform(self._pose)
        self.set_predicate("pickup_published", True)
        return {"success": True, "message": "Pickup frame published"}

    # def on_step_callback(self):
        # if len(self._depth_images) < self._num_stack.get_value() or len(self._rgb_images) < self._num_stack.get_value():
        #     self.get_logger().warn("Not enough images to run detection", throttle_duration_sec=1.0)
        #     return

        # objects = self._detect_objects()
        # self._publish_detections(self._rgb_images[-1].copy(), objects)

    def _detect_objects(self):
        objects = self._detector.predict_batch(self._rgb_images, self._depth_images)
        objects['deltas_mm'] = self._calculate_delta_mm(
            objects['deltas'],
            objects['camera_height'],
            objects['object_heights']
        )
        return objects

    def _calculate_delta_mm(self, deltas, height, object_heights):
        deltas_mm = []
        for pair, object_height in zip(deltas, object_heights):
            res = (
                round(float((height - object_height) * (pair[0] / self._rgb_fx)), 1),
                round(float((height - object_height) * (pair[1] / self._rgb_fy)), 1),
                round(float(height - object_height), 1)
            )
            deltas_mm.append(res)
        return deltas_mm

    def _publish_detections(self, img, objects):
        centers = objects['centers'][0] if len(objects['centers']) > 0 else list()
        blob_centers = objects['blob_centers']
        blob_radiuses = objects['blob_radiuses']
        circle_centers = objects['circle_centers']
        circle_radiuses = objects['circle_radiuses']
        areas = objects['areas']
        labels = objects['labels']
        deltas_mm = objects['deltas_mm']

        # Draw all detections in a single loop
        for i, (center, blob_center, blob_radius, circle_center, circle_radius, area, delta_mm, label) in enumerate(
                zip(centers, blob_centers, blob_radiuses, circle_centers, circle_radiuses, areas, deltas_mm, labels)):
            if center is not None:
                if label == 0:
                    color = (0, 0, 255)
                    text = 'unknown object'
                else:
                    color = (0, 255, 0)
                    text = f'Label: {label}'
                cv2.drawMarker(img, tuple(map(int, center)), color, markerType=cv2.MARKER_CROSS,
                               markerSize=20, thickness=2, line_type=cv2.LINE_AA)
                cv2.putText(img, text, (int(center[0]) - 20, int(center[1]) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(
                    img, f'Area: {int(area)}', (int(center[0]) - 20, int(center[1]) + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(
                    img, f'Delta mm: ({int(delta_mm[0])}, {int(delta_mm[1])}, {int(delta_mm[2])})',
                    (int(center[0]) - 20, int(center[1]) + 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # Draw blob centers and radii
            if blob_center is not None and blob_radius is not None:
                cv2.circle(img, tuple(map(int, blob_center)), int(blob_radius), (0, 255, 255), 2)
                cv2.drawMarker(img, tuple(map(int, blob_center)), (0, 255, 255),
                               markerType=cv2.MARKER_SQUARE, markerSize=20, thickness=2, line_type=cv2.LINE_AA)

            # Draw circle centers and radii
            if circle_center is not None and circle_radius is not None:
                cv2.circle(img, tuple(map(int, circle_center)), int(circle_radius), (255, 0, 0), 2)
                cv2.drawMarker(img, tuple(map(int, circle_center)), (255, 0, 0),
                               markerType=cv2.MARKER_SQUARE, markerSize=20, thickness=2, line_type=cv2.LINE_AA)

        self._visual_results_pub.publish(self._bridge.cv2_to_imgmsg(img, encoding="passthrough"))

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() in ["depth_image_topic", "depth_info_topic", "rgb_image_topic",
                                    "rgb_info_topic", "visual_results_topic"] and not parameter.get_value():
            self.get_logger().error(f"Provide a non empty value for parameter '{parameter.get_name()}'.")
            return False
        elif parameter.get_name() == "num_stack" and parameter.get_value() < 1:
            self.get_logger().error(f"Provide a value bigger than zero for parameter '{parameter.get_name()}'.")
            return False
        elif parameter.get_name() == "config_file":
            if not parameter:
                self.get_logger().error(f"Parameter '{parameter.get_name()}' cannot be empty.")
                return False
            if not os.path.exists(parameter.get_value()):
                self.get_logger().error(f"File '{parameter.get_value()}' does not exist.")
                return False
        elif parameter.get_name() == "robot_base_frame" and not parameter:
            self.get_logger().error(f"Parameter '{parameter.get_name()}' cannot be empty.")
            return False
        return True
