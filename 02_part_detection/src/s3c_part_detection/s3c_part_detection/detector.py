import numpy as np
import cv2
from skimage import exposure
from scipy.spatial import distance

class Detector:
    def __init__(
            self,
            depth_plane_threshold: int = 15,
            depth_area_threshold: int = 5000,
            depth_areas: list = None,
            depth_dilation: tuple = (20, 20),
            rgb_detection_params: dict = None,
            rgb_min_keypoint_size: int = 27,
            rgb_circle_detection_params: dict = None,
            dept_to_rgb_params: dict = None,
            max_pairing_distance_blob: int = 50,
            max_pairing_distance_circle: int = 50,
            detection_distance_threshold: int = 300
        ):
        self.depth_dilation = depth_dilation
        self.depth_area_threshold = depth_area_threshold
        self.depth_areas = depth_areas
        self.depth_plane_threshold = depth_plane_threshold
        self.rgb_detector = cv2.SimpleBlobDetector_create(rgb_detection_params)
        self.rgb_min_keypoint_size = rgb_min_keypoint_size
        self.rgb_circle_detection_params = rgb_circle_detection_params
        self.dept_to_rgb_params = dept_to_rgb_params
        self.max_pairing_distance_blob = max_pairing_distance_blob
        self.max_pairing_distance_circle = max_pairing_distance_circle
        self.detection_distance_threshold = detection_distance_threshold

    def _dilate(self, arr: np.ndarray) -> np.ndarray:
        """
        method to dilate an array
        :param arr: np.ndarray array to dilate
        :return: np.ndarray dilated array
        """
        kernel = np.ones(self.depth_dilation, np.uint8)
        return cv2.dilate(arr, kernel, iterations=1)

    def _cut_plane(self, arr: np.ndarray, threshold: int, plane: int or None = None, near_point_threshold = None) -> (np.ndarray, int):
        """
        method to filter the depth image according to a plane
        :param arr: np.ndarray array of the depth image
        :param threshold: int threshold to subtract from the plane
        :param plane: int or None plane, if plane is None, the median of the array is used
        :param near_point_threshold: int or None threshold to ignore points near the camera
        :return: np.ndarray filtered array, int plane
        """
        if plane is None:
            plane = np.median(arr)
        # set all values bigger and equal to the median to 0
        arr[arr >= plane - threshold] = 0
        # ignore points near the camera
        if near_point_threshold is not None:
            arr[arr <= near_point_threshold] = 0
        # set all non zero values to value - median
        arr[arr != 0] = plane - arr[arr != 0]
        return arr, plane
    
    def _project_depth_coordinates_onto_rgb(
            self,
            x: int, 
            y: int, 
            img_depth_shape: tuple, 
            img_shape: tuple, 
            offsetx: float = 0.8, 
            offsety: float = 0.0, 
            depth2rgbx: float = 1.4, 
            depth2rgby: float = 1.0
        ) -> tuple:
        """
        Method to project depth coordinates onto rgb image
        :param x: int x coordinate
        :param y: int y coordinate
        :param img_depth_shape: tuple shape of the depth image
        :param img_shape: tuple shape of the rgb image
        :param offsetx: float offset in x direction
        :param offsety: float offset in y direction
        :param depth2rgbx: float scale factor in x direction
        :param depth2rgby: float scale factor in y direction
        :return: tuple projected coordinates
        """
        # Ensure img_depth_shape and img_shape are tuples
        dx = offsetx * (img_depth_shape[1] - img_shape[1])
        dy = offsety * (img_depth_shape[0] - img_shape[0])

        # Scale depth coordinates to rgb image
        x = int((x - dx) * depth2rgbx)
        y = int((y - dy) * depth2rgby)

        return x, y
    

    def _detect_circles_in_grayscale(
            self,
            img_gray: np.ndarray,
            minRadius: int,
            maxRadius: int,
            median_blur_k_size: int = 5,
            canny_threshold1: int = 20,
            canny_threshold2: int = 40,
            gaussian_blur_k_size: tuple = (5, 5),
            hough_dp: float = 1,
            hough_minDist: int = 100,
            hough_param1: int = 100,
            hough_param2: int = 30
        ) -> (np.ndarray, np.ndarray):
        """
        Simple Hough circle detection in grayscale image.
        :param img_gray: np.ndarray grayscale image
        :param minRadius: int minimum radius of the circles
        :param maxRadius: int maximum radius of the circles
        :return: np.ndarray edges, np.ndarray circles
        """
        # Blur the image for better edge detection
        img_blur = cv2.medianBlur(img_gray, median_blur_k_size)
        
        # Canny Edge Detection
        edges = cv2.Canny(image=img_blur, threshold1=canny_threshold1, threshold2=canny_threshold2)
        edges = cv2.GaussianBlur(edges, gaussian_blur_k_size, 0)
        
        # Adjust HoughCircles parameters for better detection
        circles = cv2.HoughCircles(
            img_blur, 
            cv2.HOUGH_GRADIENT, 
            dp=hough_dp,  # Inverse ratio of the accumulator resolution to the image resolution
            minDist=hough_minDist,  # Minimum distance between the centers of the detected circles
            param1=hough_param1,  # Higher threshold for the internal Canny edge detector
            param2=hough_param2,  # Threshold for center detection
            minRadius=minRadius, 
            maxRadius=maxRadius
        )
                                
        return edges, circles

    
    def _process_rgb_image(self, rgb_image: np.ndarray) -> (list, list, list, list):
        """
        process rgb image
        :param rgb_image: np.ndarray rgb image
        :return: list blob_coordinates, list blob_radiuses, list circle_coordinates, list circle_radiuses
        """
        img = rgb_image.copy()
        img = cv2.bitwise_not(img)
        img = exposure.adjust_log(img, 5)
        img = cv2.bitwise_not(img)

        rgb_blob_keypoints = self.rgb_detector.detect(img)
        if self.rgb_min_keypoint_size is not None:
            rgb_blob_keypoints = [k for k in rgb_blob_keypoints if k.size > self.rgb_min_keypoint_size]
        blob_coordinates = []
        blob_radiuses = []

        for keypoint in rgb_blob_keypoints:
            x, y = keypoint.pt
            blob_coordinates.append((int(x), int(y)))
            blob_radiuses.append(int(keypoint.size // 2))

        gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY) 
        _, circles1 = self._detect_circles_in_grayscale(
            gray, self.rgb_circle_detection_params['C1minRadius'], 
            self.rgb_circle_detection_params['C1maxRadius'],
            self.rgb_circle_detection_params['medianBlurKSize'], 
            self.rgb_circle_detection_params['cannyThreshold1'],
            self.rgb_circle_detection_params['cannyThreshold2'], 
            self.rgb_circle_detection_params['gaussianBlurKSize'],
            self.rgb_circle_detection_params['houghDP'], 
            self.rgb_circle_detection_params['houghMinDist'],
            self.rgb_circle_detection_params['houghParam1'], 
            self.rgb_circle_detection_params['houghParam2']
            )
        _, circles2 = self._detect_circles_in_grayscale(
            gray, self.rgb_circle_detection_params['C2minRadius'], 
            self.rgb_circle_detection_params['C2maxRadius'],
            self.rgb_circle_detection_params['medianBlurKSize'],
            self.rgb_circle_detection_params['cannyThreshold1'],
            self.rgb_circle_detection_params['cannyThreshold2'],
            self.rgb_circle_detection_params['gaussianBlurKSize'],
            self.rgb_circle_detection_params['houghDP'],
            self.rgb_circle_detection_params['houghMinDist'],
            self.rgb_circle_detection_params['houghParam1'],
            self.rgb_circle_detection_params['houghParam2']
            )

        if circles1 is not None and circles2 is not None:
            circles1 = circles1[0]  # Unpack the first dimension
            circles2 = circles2[0]  # Unpack the first dimension
            circles = np.vstack((circles2, circles1))
        elif circles1 is not None:
            circles = circles1[0]  # Unpack the first dimension
        elif circles2 is not None:
            circles = circles2[0]  # Unpack the first dimension
        else:
            circles = None

        circle_coordinates = []
        circle_radiuses = []

        if circles is not None:
            for circle in circles:
                x, y, r = circle
                circle_coordinates.append((int(x), int(y)))
                circle_radiuses.append(int(r))

        return blob_coordinates, blob_radiuses, circle_coordinates, circle_radiuses


    def _process_depth_image(self, depth_image: np.ndarray, height: int or None = None) -> (list, list, int):
        """
        process depth image
        :param depth_image: np.ndarray depth image
        :param height: int or None height of the camera
        :return: list coordinates, list areas, int height
        """
        img, height = self._cut_plane(depth_image, self.depth_plane_threshold, height)
        dilatet_img = self._dilate(img)
        num_labels, labels = cv2.connectedComponents(dilatet_img.astype(np.uint8))

        coordinates = []
        areas = []

        for i in range(1, num_labels):
            ys, xs = np.where(labels == i)
            if not len(xs) or not len(ys):
                continue
            minx, maxx = np.min(xs), np.max(xs)
            miny, maxy = np.min(ys), np.max(ys)

            area = (maxx - minx + 1) * (maxy - miny + 1)
            if area < self.depth_area_threshold:
                continue

            x, y = (minx + maxx) // 2, (miny + maxy) // 2
            z = np.max(img[miny:maxy+1, minx:maxx+1])

            coordinates.append((x, y, z))
            areas.append(area)

        return coordinates, areas, height

    def predict(self, rgb_image: np.ndarray, depth_image: np.ndarray, height: int or None = None) -> dict:
        """
        predict method
        :param rgb_image: np.ndarray rgb image
        :param depth_image: np.ndarray depth image
        :param height: int or None height of the camera
        :return: dict objects_dict
        """
        rgb_blob_coordinates, rgb_blob_radiuses, rgb_circle_coordinates, rgb_circle_radiuses = self._process_rgb_image(rgb_image)
        depth_coordinates, areas, height = self._process_depth_image(depth_image, height)

        rgb_image_center = (rgb_image.shape[1] // 2, rgb_image.shape[0] // 2)

        objects_dict = {
            'blob_centers': [],
            'blob_radiuses': [],
            'circle_centers': [],
            'circle_radiuses': [],
            'depth_centers': [],
            'centers': [],
            'detected_on_rgb': [],
            'deltas': [],
            'object_heights': [],
            'areas': [],
            'labels': [],
            'camera_height': height
        }

        transformed_depth_coordinates = []
        for coordinate in depth_coordinates:
            x, y, z = coordinate
            x, y = self._project_depth_coordinates_onto_rgb(x, y, depth_image.shape, rgb_image.shape, **self.dept_to_rgb_params)
            transformed_depth_coordinates.append((x, y, z))

        objects_dict['depth_centers'] = transformed_depth_coordinates

        max_blob_distance = self.max_pairing_distance_blob
        max_circle_distance = self.max_pairing_distance_circle

                # Compare and fuse RGB and depth coordinates
        if transformed_depth_coordinates:
            depth_coords = np.array(transformed_depth_coordinates)
            
            # Prepare arrays to track assigned blob and circle coordinates
            assigned_blob_indices = set()
            assigned_circle_indices = set()
            
            # Match depth coordinates with blob and circle coordinates
            for depth_coord, depth_area in zip(depth_coords, areas):
                matched_blob_coord = None
                matched_circle_coord = None

                if rgb_blob_coordinates:
                    blob_coords = np.array(rgb_blob_coordinates)
                    blob_distances = distance.cdist([depth_coord[:2]], blob_coords[:, :2], 'euclidean').flatten()
                    for idx in np.argsort(blob_distances):
                        if idx not in assigned_blob_indices and blob_distances[idx] <= max_blob_distance:
                            matched_blob_coord = blob_coords[idx]
                            assigned_blob_indices.add(idx)
                            break

                if rgb_circle_coordinates:
                    circle_coords = np.array(rgb_circle_coordinates)
                    circle_distances = distance.cdist([depth_coord[:2]], circle_coords[:, :2], 'euclidean').flatten()
                    for idx in np.argsort(circle_distances):
                        if idx not in assigned_circle_indices and circle_distances[idx] <= max_circle_distance:
                            matched_circle_coord = circle_coords[idx]
                            assigned_circle_indices.add(idx)
                            break

                # Determine center
                if matched_blob_coord is not None:
                    center = matched_blob_coord[:2]
                    detected_on_rgb = True
                elif matched_circle_coord is not None:
                    center = matched_circle_coord[:2]
                    detected_on_rgb = True
                else:
                    center = depth_coord[:2]
                    detected_on_rgb = False

                # Store additional data
                objects_dict['centers'].append(center.tolist())
                objects_dict['deltas'].append((depth_coord[0] - rgb_image_center[0], depth_coord[1] - rgb_image_center[1]))
                objects_dict['object_heights'].append(depth_coord[2])
                objects_dict['areas'].append(depth_area)
                objects_dict['blob_centers'].append(matched_blob_coord[:2].tolist() if matched_blob_coord is not None else None)
                objects_dict['circle_centers'].append(matched_circle_coord[:2].tolist() if matched_circle_coord is not None else None)
                objects_dict['blob_radiuses'].append(rgb_blob_radiuses[blob_coords.tolist().index(matched_blob_coord.tolist())] if matched_blob_coord is not None else None)
                objects_dict['circle_radiuses'].append(rgb_circle_radiuses[circle_coords.tolist().index(matched_circle_coord.tolist())] if matched_circle_coord is not None else None)
                objects_dict['detected_on_rgb'].append(detected_on_rgb)
        
        # Area and label assignment
        if self.depth_areas is not None:
            for detected_area, detected_on_rgb in zip(objects_dict['areas'], objects_dict['detected_on_rgb']):
                differences = [(abs(detected_area - label_area), label, threshold) for label, label_area, threshold in self.depth_areas]
                differences.sort()  # Sort by smallest difference
                closest_match = differences[0]
                if detected_on_rgb:
                    if closest_match[2] is not None and closest_match[0] > closest_match[2] * detected_area / 100:
                        objects_dict['labels'].append(0)  # Threshold exceeded
                    else:
                        objects_dict['labels'].append(closest_match[1])  # Assign closest label
                else:
                    objects_dict['labels'].append(0)
        else:
            # Sorting objects by area in descending order and assigning numeric labels
            sorted_indices = np.argsort(objects_dict['areas'])[::-1]
            objects_dict['labels'] = list(range(1, len(sorted_indices) + 1))
            for key in objects_dict.keys():
                if key not in ['labels', 'camera_height']:  # 'labels' and 'camera_height' exempt from reordering
                    objects_dict[key] = [objects_dict[key][i] for i in sorted_indices]
                    
        return objects_dict
    
    def predict_batch(self, rgb_images: np.ndarray, depth_images: np.ndarray, height: int or None = None) -> dict:
        """
        predict_batch method
        :param rgb_images: np.ndarray rgb images
        :param depth_images: np.ndarray depth images
        :param heights: int or None height of the camera
        :return: dict aggregated_objects_dict
        """
        objects_dicts = []
        for rgb_image, depth_image in zip(rgb_images, depth_images):
            objects_dict = self.predict(rgb_image, depth_image, height)
            objects_dicts.append(objects_dict)

        # Find the median number of depth_centers
        num_depth_centers = [len(obj_dict['depth_centers']) for obj_dict in objects_dicts]
        median_length = np.median(num_depth_centers)

        # Filter frames that have more depth_centers than the median length
        filtered_objects_dicts = [obj_dict for obj_dict in objects_dicts if len(obj_dict['depth_centers']) == median_length]

        # Initialize aggregated objects dictionary with empty lists
        aggregated_objects_dict = {
            'blob_centers': [],
            'blob_radiuses': [],
            'circle_centers': [],
            'circle_radiuses': [],
            'depth_centers': [],
            'centers': [],
            'detected_on_rgb': [],
            'deltas': [],
            'object_heights': [],
            'areas': [],
            'labels': [],
            'camera_height': objects_dicts[-1]['camera_height'] if objects_dicts else 0
        }

        def filter_none_and_mean(values, axis=None):
            filtered_values = [v for v in values if v is not None]
            if not filtered_values:
                return None
            return np.mean(filtered_values, axis=axis)
        
        if objects_dicts[-1]['camera_height'] < self.detection_distance_threshold:
            return aggregated_objects_dict

        # Iterate over each depth_center by index
        for i in range(int(median_length)):
            # Initialize lists to store values for averaging
            blob_centers, blob_radiuses, circle_centers, circle_radiuses = [], [], [], []
            depth_centers, detected_on_rgb, = [], []
            object_heights, areas = [], []

            for obj_dict in filtered_objects_dicts:
                blob_centers.append(obj_dict['blob_centers'][i])
                blob_radiuses.append(obj_dict['blob_radiuses'][i])
                circle_centers.append(obj_dict['circle_centers'][i])
                circle_radiuses.append(obj_dict['circle_radiuses'][i])
                depth_centers.append(obj_dict['depth_centers'][i])
                detected_on_rgb.append(obj_dict['detected_on_rgb'][i])
                object_heights.append(obj_dict['object_heights'][i])
                areas.append(obj_dict['areas'][i])

            # Calculate the averages for each property
            aggregated_objects_dict['blob_centers'].append(filter_none_and_mean(blob_centers, axis=0))
            aggregated_objects_dict['blob_radiuses'].append(filter_none_and_mean(blob_radiuses))
            aggregated_objects_dict['circle_centers'].append(filter_none_and_mean(circle_centers, axis=0))
            aggregated_objects_dict['circle_radiuses'].append(filter_none_and_mean(circle_radiuses))
            aggregated_objects_dict['depth_centers'].append(filter_none_and_mean(depth_centers, axis=0))
            aggregated_objects_dict['detected_on_rgb'].append(any(detected_on_rgb))
            aggregated_objects_dict['object_heights'].append(filter_none_and_mean(object_heights))
            aggregated_objects_dict['areas'].append(filter_none_and_mean(areas))
            aggregated_objects_dict['labels'].append(0)

        if len(aggregated_objects_dict['depth_centers']) > 0:
            # Calculate centers based on given rules
            final_centers = []
            for dc, bc, cc in zip(aggregated_objects_dict['depth_centers'], aggregated_objects_dict['blob_centers'], aggregated_objects_dict['circle_centers']):
                if cc is not None:
                    final_centers.append(cc)
                elif bc is not None:
                    final_centers.append(bc)
                elif dc is not None:
                    final_centers.append(dc[:2])

            aggregated_objects_dict['centers'].append(final_centers)

            #print('final_centers:', final_centers)

            # Calculate deltas based on new centers
            rgb_image_center = (rgb_images[0].shape[1] // 2, rgb_images[0].shape[0] // 2)
            new_deltas = [
                (dc[0] - rgb_image_center[0], dc[1] - rgb_image_center[1])
                for dc in final_centers
            ]
            aggregated_objects_dict['deltas'] = new_deltas

            # Area and label assignment
            if self.depth_areas is not None:
                label_assignments = []
                for detected_area, detected_on_rgb_flag in zip(aggregated_objects_dict['areas'], aggregated_objects_dict['detected_on_rgb']):
                    if detected_area is not None:
                        differences = [(abs(detected_area - label_area), label, threshold) for label, label_area, threshold in self.depth_areas]
                        differences.sort()  # Sort by smallest difference
                        closest_match = differences[0]
                        if detected_on_rgb_flag:
                            if closest_match[2] is not None and closest_match[0] > closest_match[2] * detected_area / 100:
                                label_assignments.append(0)  # Threshold exceeded
                            else:
                                label_assignments.append(closest_match[1])  # Assign closest label
                        else:
                            label_assignments.append(0)
                aggregated_objects_dict['labels'] = label_assignments
            else:
                # Sorting objects by area in descending order and assigning numeric labels
                sorted_indices = np.argsort(aggregated_objects_dict['areas'])[::-1]
                sorted_labels = list(range(1, len(sorted_indices) + 1))
                aggregated_objects_dict['labels'] = sorted_labels
                for key in aggregated_objects_dict.keys():
                    if key not in ['labels', 'camera_height']:  # 'labels' and 'camera_height' exempt from reordering
                        aggregated_objects_dict[key] = [aggregated_objects_dict[key][i] for i in sorted_indices]

        #print('------------')
        #print(aggregated_objects_dict)
        return aggregated_objects_dict

