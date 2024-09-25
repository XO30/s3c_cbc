class PartDetectionLogic:
    def __init__(self, transformation, home, height_1, end_height, max_error=2):
        self.transformation = transformation  # [x, y, z, rx, ry, rz]
        self.home = home  # [x, y, z, rx, ry, rz]
        self.height_1 = height_1
        self.end_height = end_height
        self.max_error = max_error

    def extract_initial_deltas(self, detected_objects, index):
        deltas = detected_objects["deltas_mm"][index]
        return [deltas[1], deltas[0], self.home[2] - self.height_1, 0, 0, 0]

    def extract_final_deltas(self, detected_objects, index):
        deltas = detected_objects["deltas_mm"][index]
        average_delta = (abs(deltas[0]) + abs(deltas[1])) / 2

        if average_delta <= self.max_error:
            adjusted_deltas = [-t for t in self.transformation]
            adjusted_deltas[2] = self.height_1 - self.end_height
            return adjusted_deltas, True
        else:
            return [deltas[1], deltas[0], 0.0, 0.0, 0.0, 0.0], False
