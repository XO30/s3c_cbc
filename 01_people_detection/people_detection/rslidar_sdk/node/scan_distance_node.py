#!/usr/bin/python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from std_msgs.msg import Float64MultiArray
from aica_api.client import AICA

class LaserScanProcessor:
    def __init__(self):
        rospy.init_node('scan_distance_node', anonymous=True)
        self.aica = None
        no_aica = rospy.get_param('~no_aica')
        if not no_aica:
            self.aica = AICA()
            self.aica.check()
        self.stopped = False
        self.full_speed = False
        self.marker_pub = rospy.Publisher('closest_scan_marker', Marker, queue_size=1)
        self.desired_angles = []
        self.angle_tolerance = 2.0  # Set the angle tolerance for the desired angle range
        self.scan_data = None
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/desired_angles', Float64MultiArray, self.desired_angles_callback)
        rospy.spin()

    def desired_angles_callback(self, angles_msg):
        self.desired_angles = angles_msg.data

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def scan_callback(self, scan):
        # Store the scan data for use in desired angle processing
        self.scan_data = scan

        # Publish a marker with action set to DELETEALL to clear previous markers
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL
        self.marker_pub.publish(delete_all_marker)
        closest_distance = math.inf

        for i, desired_angle in enumerate(self.desired_angles):
            # Normalize the desired angle to be within the range of -π to π (radians)
            desired_angle_rad = self.normalize_angle(math.radians(desired_angle))

            # Find the closest scan point to the desired angle within the angle tolerance
            min_range = float('inf')
            closest_angle = None
            for j, angle in enumerate(scan.angle_min + scan.angle_increment * j for j in range(len(scan.ranges))):
                # Normalize the angle to be within the range of -π to π (radians)
                angle = self.normalize_angle(angle)

                if abs(angle - desired_angle_rad) <= math.radians(self.angle_tolerance):
                    if scan.ranges[j] < min_range:
                        min_range = scan.ranges[j]
                        closest_angle = angle

            if closest_angle is not None:
                rospy.loginfo("Closest scan distance at %.1f degrees: %.2f meters" % (math.degrees(closest_angle), min_range))

                distance = -math.sin(closest_angle) * min_range
                if -2.356 < closest_angle < 0 and distance < closest_distance:
                        closest_distance = distance

                # Calculate the angle between the closest scan and the origin
                origin_angle = math.atan2(0, 0)  # Calculate the angle towards the origin (x=0, y=0)
                relative_angle = origin_angle + closest_angle

                # Add 90 degrees clockwise rotation
                # relative_angle += math.radians(90)

                # Create a marker representing the closest scan
                marker = Marker()
                marker.header.frame_id = scan.header.frame_id
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = "package://rslidar_sdk/meshes/human3.obj"  # Path to the human model file
                marker.action = Marker.ADD
                marker.pose.position.x = min_range * math.cos(closest_angle)
                marker.pose.position.y = min_range * math.sin(closest_angle)
                marker.pose.position.z = 0.0

                # Set the orientation of the marker based on the relative angle
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = math.sin((relative_angle + math.radians(-90))/ 2.0)
                marker.pose.orientation.w = math.cos((relative_angle + math.radians(-90)) / 2.0)

                marker.scale.x = 0.02
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                # Assign a unique ID to each marker
                marker.id = i

                self.marker_pub.publish(marker)
            else:
                rospy.loginfo("No scan found within the specified angle range for %.1f degrees." % desired_angle)
        
        if closest_distance < 0.5:
            self.stop()
        elif closest_distance < 1.0:
            self.modulate_speed(2 * (closest_distance - 0.5))
        else:
            self.play()
    
    def stop(self):
        if self.stopped:
            return
        rospy.logerr("STOP")
        if self.aica:
            self.aica.set_component_parameter("orbiter", "speed_scale", "0.0")
            self.aica.switch_controllers("ur", deactivate=["impedance"])
        self.stopped = True
        self.full_speed = False
    
    def modulate_speed(self, factor):
        rospy.logerr(f"Modulating speed by factor {factor}")
        if self.aica:
            if self.stopped:
                self.aica.switch_controllers("ur", activate=["impedance"])
                self.stopped = False
            self.aica.set_component_parameter("orbiter", "speed_scale", f"{factor}")
        self.stopped = False
        self.full_speed = False

    def play(self):
        if self.full_speed:
            return
        rospy.logerr("PLAY")
        if self.aica:
            self.aica.set_component_parameter("orbiter", "speed_scale", "1.0")
            self.aica.switch_controllers("ur", activate=["impedance"])
        self.stopped = False
        self.full_speed = True


if __name__ == '__main__':
    LaserScanProcessor()
