#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def filter_callback(msg):
    filtered_points = []

    # Define the circular exclusion zone
    exclusion_radius = 0.5  # 0.5 meters

    for p in pc2.read_points(msg, skip_nans=True):
        # Calculate the Euclidean distance from the LiDAR
        distance = ((p[0] ** 2) + (p[1] ** 2)) ** 0.5

        # Exclude points within the circular exclusion zone
        if distance > exclusion_radius:
            filtered_points.append(p)

    # Create a new PointCloud2 message with the filtered points
    filtered_msg = pc2.create_cloud_xyz32(msg.header, filtered_points)

    # Publish the filtered point cloud
    pub.publish(filtered_msg)

if __name__ == '__main__':
    rospy.init_node('circular_filter')
    sub = rospy.Subscriber('/rslidar_points', PointCloud2, filter_callback)
    pub = rospy.Publisher('/filtered_point_cloud', PointCloud2, queue_size=10)
    rospy.spin()
