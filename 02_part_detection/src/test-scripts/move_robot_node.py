import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from part_detection_interfaces.srv import DetectObjects
import json
import time

class UR10eNode(Node):
    def __init__(self):
        super().__init__('ur10e_node')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('transformation', [37.2, 32.1, 0]),
                ('home', [400, -280, 625]),
                ('height_1', 475),
                ('height_2', 475),
                ('end_height', 350) 
            ]
        )

        self.transformation = self.get_parameter('transformation').get_parameter_value().double_array_value
        self.home = self.get_parameter('home').get_parameter_value().double_array_value
        self.height_1 = self.get_parameter('height_1').get_parameter_value().double_value
        self.height_2 = self.get_parameter('height_2').get_parameter_value().double_value
        self.end_height = self.get_parameter('end_height').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(String, '/urscript_interface/script_command', 10)
        self.detect_objects_client = self.create_client(DetectObjects, '/part_detection/detect_new')

        self.create_service(Trigger, '/part_detection_go_home', self.handle_go_home)
        self.create_service(Trigger, '/part_detection/find_part1', self.handle_find_part1)
        self.create_service(Trigger, '/part_detection/find_part2', self.handle_find_part2)

        self.get_logger().info('UR10e Node started.')

    def calculate_position(self, origin, deltas):
        return [origin[0] - deltas[1], origin[1] - deltas[0], origin[2]]

    def calculate_position_with_transformation(self, origin, deltas, transformation):
        return [origin[0] - deltas[1] + transformation[0], origin[1] - deltas[0] + transformation[1], origin[2]]

    def send_ur_script_command(self, position):
        script_command = f"def my_prog():\nmovej(p[{position[0]/1000}, {position[1]/1000}, {position[2]/1000}, 0, 3.14, 0], a=1.2, v=0.25, r=0)\ntextmsg(\"motion finished\")\nend"
        msg = String()
        msg.data = script_command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent URScript command: {script_command}')

    def call_detect_objects_service(self):
        if not self.detect_objects_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /part_detection/detect not available.')
            return None
        
        request = DetectObjects.Request()
        future = self.detect_objects_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Service call succeeded.')
            try:
                result = json.loads(future.result().json_results)
                deltas = result.get('deltas', [])
                labels = result.get('labels', [])
                return deltas, labels
            except json.JSONDecodeError:
                self.get_logger().error('Failed to decode JSON response.')
                return None, None
        else:
            self.get_logger().error('Service call failed.')
            return None, None

    def handle_go_home(self, request, response):
        self.send_ur_script_command(self.home)
        time.sleep(5)
        response.success = True
        response.message = "Moved to home position."
        return response

    def handle_find_part(self, part_label):
        deltas, labels = self.call_detect_objects_service()
        if deltas is None or part_label not in labels:
            return False
        
        part_index = labels.index(part_label)
        deltas = deltas[part_index]
        position = self.calculate_position(self.home, deltas)
        position[2] = self.height_1
        self.send_ur_script_command(position)
        time.sleep(5)

        while True:
            deltas, labels = self.call_detect_objects_service()
            if deltas is None:
                return False
            
            if len(deltas) > 1:
                differences = []
                for delta in deltas:
                    # take the one with the smallest delta (delta x + delta y / 2)
                    differences.append(abs(delta[0]) + abs(delta[1]) / 2)
                min_index = differences.index(min(differences))
                deltas = deltas[min_index]

            deltas = deltas[0]  # Get the first detected part for fine adjustment
            if abs(deltas[0]) <= 5 and abs(deltas[1]) <= 5:
                break

            position = self.calculate_position(position, deltas)
            position[2] = self.height_2
            self.send_ur_script_command(position)
            time.sleep(2)

        position = self.calculate_position_with_transformation(position, [0, 0, 0], self.transformation)
        position[2] = self.end_height
        self.send_ur_script_command(position)
        return True

    def handle_find_part1(self, request, response):
        success = self.handle_find_part(1)
        response.success = success
        response.message = "Part 1 found and positioned." if success else "Failed to find or position part 1."
        return response

    def handle_find_part2(self, request, response):
        success = self.handle_find_part(2)
        response.success = success
        response.message = "Part 2 found and positioned." if success else "Failed to find or position part 2."
        return response

def main(args=None):
    rclpy.init(args=args)
    node = UR10eNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
