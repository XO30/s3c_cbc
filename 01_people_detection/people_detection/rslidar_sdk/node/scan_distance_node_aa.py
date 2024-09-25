#!/usr/bin/python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from threading import Timer
from aica_api.client import AICA
import requests

class LaserScanProcessor:
    def __init__(self):
        rospy.init_node('scan_distance_node', anonymous=True)
        self.aica = AICA()
        self.stopped = False
        self.full_speed = False
        self.marker_pub = rospy.Publisher('closest_scan_marker', Marker, queue_size=1)
        self.desired_angles = []
        self.angle_tolerance = 2.0
        self.scan_data = None
        self.delete_timer = None  
        self.last_n_scans = []
        self.n_scan_history = 7
        
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/desired_angles', Float64MultiArray, self.desired_angles_callback)
        rospy.spin()

    def __del__(self):
        self.play()

    def desired_angles_callback(self, angles_msg):
        self.desired_angles = angles_msg.data
        
        self.start_delete_timer()

    def start_delete_timer(self):
      
        if self.delete_timer is not None and self.delete_timer.is_alive():
            self.delete_timer.cancel()
        self.delete_timer = Timer(1.5, self.delete_markers)
        self.delete_timer.start()

    def delete_markers(self):
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        self.marker_pub.publish(delete_all_marker)


    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def calculate_moving_average(self, new_scan_ranges):
        self.last_n_scans.append(new_scan_ranges)

        if len(self.last_n_scans) > self.n_scan_history:
            self.last_n_scans.pop(0) 

        smoothed_ranges = [sum(col) / len(col) for col in zip(*self.last_n_scans)]
        return smoothed_ranges

    def scan_callback(self, scan):
        self.scan_data = scan
        closest_distance = math.inf

        smoothed_ranges = self.calculate_moving_average(scan.ranges)

        for i, desired_angle in enumerate(self.desired_angles):
            desired_angle_rad = self.normalize_angle(math.radians(desired_angle))

            min_range = float('inf')
            closest_angle = None
            for j, angle in enumerate(scan.angle_min + scan.angle_increment * j for j in range(len(scan.ranges))):
                angle = self.normalize_angle(angle)

                if abs(angle - desired_angle_rad) <= math.radians(self.angle_tolerance):
                    if smoothed_ranges[j] < min_range:
                        min_range = smoothed_ranges[j]
                        closest_angle = angle

            if closest_angle is not None:
                rospy.loginfo("Closest scan distance at %.1f degrees: %.2f meters" % (math.degrees(closest_angle), min_range))

                distance = -math.sin(closest_angle) * min_range
                if -2.356 < closest_angle < 0 and distance < closest_distance:
                    closest_distance = distance

                marker_color = self.get_marker_color(distance)

                origin_angle = math.atan2(0, 0)
                relative_angle = origin_angle + closest_angle

                marker = Marker()
                marker.header.frame_id = scan.header.frame_id
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = "package://rslidar_sdk/meshes/human3.obj"
                marker.action = Marker.ADD
                marker.pose.position.x = min_range * math.cos(closest_angle)
                marker.pose.position.y = min_range * math.sin(closest_angle)
                marker.pose.position.z = -1.3

                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = math.sin((relative_angle + math.radians(-90)) / 2.0)
                marker.pose.orientation.w = math.cos((relative_angle + math.radians(-90)) / 2.0)

                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = marker_color[0]
                marker.color.g = marker_color[1]
                marker.color.b = marker_color[2]

                marker.id = i

                marker.lifetime = rospy.Duration(0.2)

                self.marker_pub.publish(marker)
            else:
                rospy.loginfo("No scan found within the specified angle range for %.1f degrees." % desired_angle)
        
        if closest_distance < 0.5:
            self.stop()
        elif closest_distance < 1.0:
            self.modulate_speed(2 * (closest_distance - 0.5))
        else:
            self.play()
    
    def get_marker_color(self, min_range):
        if min_range < 0.5:
            return (1.0, 0.0, 0.0)  # Zone 1: Red
        elif 0.5 <= min_range < 1.0:
            return (1.0, 0.5, 0.0)  # Zone 2: Orange
        else:
            return (0.0, 1.0, 0.0)  # Zone 3: Green
    
    def stop(self):
        if self.stopped:
            return
        rospy.logerr("STOP")
        try:
            self.aica.set_component_parameter("orbiter", "speed_scale", "0.0")
            self.aica.set_controller_parameter("robot", "impedance", "max_linear_velocity", "0.0")
            self.aica.set_controller_parameter("robot", "impedance", "max_angular_velocity", "0.0")
            requests.put(self.aica._endpoint('application/hardware/robot/controller/dashboard/service/set_speed_fraction'), json={"payload": 0.0})
        except Exception:
            pass
        self.stopped = True
        self.full_speed = False
    
    def modulate_speed(self, factor):
        rospy.logerr(f"Modulating speed by factor {factor}")
        try:
            if self.stopped:
                self.aica.set_controller_parameter("robot", "impedance", "max_linear_velocity", "0.25")
                self.aica.set_controller_parameter("robot", "impedance", "max_angular_velocity", "0.5")
                self.stopped = False
            self.aica.set_component_parameter("orbiter", "speed_scale", f"{factor}")
            requests.put(self.aica._endpoint('application/hardware/robot/controller/dashboard/service/set_speed_fraction'), json={"payload": factor})
        except Exception:
            pass
        self.stopped = False
        self.full_speed = False

    def play(self):
        if self.full_speed:
            return
        rospy.logerr("PLAY")
        try:
            self.aica.set_component_parameter("orbiter", "speed_scale", "1.0")
            self.aica.set_controller_parameter("robot", "impedance", "max_linear_velocity", "0.25")
            self.aica.set_controller_parameter("robot", "impedance", "max_angular_velocity", "0.5")
            requests.put(self.aica._endpoint('application/hardware/robot/controller/dashboard/service/set_speed_fraction'), json={"payload": 1.0})
        except Exception:
            pass
        self.stopped = False
        self.full_speed = True


if __name__ == '__main__':
    LaserScanProcessor()
