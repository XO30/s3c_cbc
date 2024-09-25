from hubconf import custom
import numpy as np
import pandas as pd
import cv2


class DeployYOLO:
    def __init__(self, path_or_model: str = 'last.pt', threshold: float = 0.25, classes: list = None):
        self.model = custom(path_or_model=path_or_model)
        self.threshold = threshold
        self.classes = classes


    def detect(self, image: np.ndarray) -> pd.DataFrame:
        """
        Detect objects in the given image
        :param image: np.ndarray: image to detect objects in
        :return: pd.DataFrame: detected objects in the image (BoundingBox, Confidence, Class, Name)
        """
        result = self.model(image).pandas().xyxy[0]
        result = result[result['confidence'] > self.threshold]
        if self.classes is not None:
            result = result[result['name'].isin(self.classes)]
        return result.reset_index(drop=True)


class Mapping:
    def __init__(self, width: int, shift: int = 0):
        self.width = width
        self.shift = shift % width

    def find_center(self, edges: list) -> np.ndarray or None:
        """
        Find the center of the given edges
        :param edges: list: list of vectors (np.ndarray) representing the edges
        :return: np.ndarray: center vector of the given edges
        """
        edges_count: int = len(edges)
        # make a sum of all edges result should be again an vector
        sum_edges = np.sum(edges, axis=0)
        if edges_count == 0:
            return None
        else:
            return sum_edges / edges_count

    def coordinates_to_degree(self, x: float) -> float:
        """
        Convert the given x coordinate to a degree
        :param x: float: x coordinate
        :return: float: degree
        """
        return 360 - (x + self.shift) / self.width * 360

    def map(self, edges: list) -> float or None:
        """
        Map the given edges to a degree
        :param edges: list: list of vectors (np.ndarray) representing the edges
        :return: float or None: degree
        """
        center = self.find_center(edges)
        if center is None:
            return None
        else:
            return self.coordinates_to_degree(float(center[0]))


def detect_frame(detector: DeployYOLO, mapping: Mapping, frame: np.ndarray):
    """
    function to detect objects in the given frame
    :param detector: DeployYOLO: detector to use
    :param frame: np.ndarray: frame to detect objects in
    :return: np.ndarray: frame with detected objects
    """
    result = detector.detect(frame)
    points_1 = list()
    points_2 = list()
    centers = list()
    angles = list()
    color = (255, 0, 0)  # Grün
    thickness = 4
    if result.empty:
        return frame, None, None, None, None
    for _, res in result.iterrows():
        # draw bounding box
        pt1 = (int(res['xmin']), int(res['ymin']))
        pt2 = (int(res['xmax']), int(res['ymax']))
        cv2.rectangle(frame, pt1, pt2, color, thickness)
        # draw center
        center = (int((res['xmin'] + res['xmax']) / 2), int((res['ymin'] + res['ymax']) / 2))
        cv2.circle(frame, center, 2, color, thickness)
        # draw degreeSS
        degree = mapping.map([np.array([res['xmin'], res['ymin']]), np.array([res['xmax'], res['ymax']])])
        cv2.putText(frame, f'{degree:.2f}', center, cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness)
        points_1.append(pt1)
        points_2.append(pt2)
        centers.append(center)
        angles.append(degree)
    return frame, points_1, points_2, centers, angles
