import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from part_detection_interfaces.srv import DetectObjects
import json
import time


TRANSFORMATION = [37.2, 32.1, 0] #437.2, -247.9
ry_home = 3.543
ry_grip = 3.142
HOME = [450, -280, 550] # [400, -280, 550]
HEIGHT_1 = 400
HEIGHT_2 = 400
END_HIGHT = 350
MODE = 'object'  # 'home' or 'object'

class UR10eNode(Node):
    def __init__(self):
        super().__init__('ur10e_node')
        self.publisher_ = self.create_publisher(String, '/urscript_interface/script_command', 10)
        self.detect_objects_client = self.create_client(DetectObjects, '/part_detection/detect')
        self.get_logger().info('UR10e Node started.')

    def calculate_position(self, origin, deltas):
        # math [x - dy, y - dx, z]
        return [origin[0] - deltas[1], origin[1] - deltas[0], origin[2]]

    def calculate_position_with_transformation(self, origin, deltas, transformation):
        # math [x - dy + tx, y - dx + ty, z]
        return [origin[0] - deltas[1] + transformation[0], origin[1] - deltas[0] + transformation[1], origin[2]]

    def send_ur_script_command(self, position, ry):
        script_command = f"def my_prog():\nmovej(p[{position[0]/1000}, {position[1]/1000}, {position[2]/1000}, 0, {ry}, 0], a=1.2, v=0.25, r=0)\ntextmsg(\"motion finished\")\nend"
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
                return deltas
            except json.JSONDecodeError:
                self.get_logger().error('Failed to decode JSON response.')
                return None
        else:
            self.get_logger().error('Service call failed.')
            return None

    def main_logic(self):
        if MODE == 'home':
            self.get_logger().info('Mode: home')
            self.send_ur_script_command(HOME, ry_home)
            time.sleep(5)
            deltas = self.call_detect_objects_service()
            if deltas is not None:
                self.get_logger().info(f'Detected deltas: {deltas[0]}')
            else:
                self.get_logger().info(f'no delatas')
            return
        elif MODE == 'object':
            self.get_logger().info('Mode: object')
            self.send_ur_script_command(HOME, ry_home)
            time.sleep(5)
            deltas = self.call_detect_objects_service()[0]
            self.get_logger().info(f'Deltas 0: {deltas}')
            position = self.calculate_position(HOME, deltas)
            position[2] = HEIGHT_1
            self.send_ur_script_command(position, ry_home)
            time.sleep(5)
            percentage = 1
            while True:
                deltas = self.call_detect_objects_service()[0]
                if abs(deltas[0]) <= 2 and abs(deltas[1]) <= 2:
                    print('finished')
                    break
                deltas[0] = percentage * deltas[0]
                deltas[1] = percentage * deltas[1]
                self.get_logger().info(f'Deltas 1: {deltas}')
                position = self.calculate_position(position, deltas)
                position[2] = HEIGHT_2
                self.send_ur_script_command(position, ry_home)
                time.sleep(2)

            #deltas_2 = self.call_detect_objects_service()[0]
            #self.get_logger().info(f'Deltas 2: {deltas_2}')
            position = self.calculate_position_with_transformation(position, [0, 0, 0], TRANSFORMATION)
            position[2] = END_HIGHT
            self.send_ur_script_command(position, ry_grip)
            return

def main(args=None):
    rclpy.init(args=args)
    node = UR10eNode()
    try:
        node.main_logic()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
