#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

def process_lidar(scan):
    ranges = scan.ranges
    angle_min = scan.angle_min
    angle_increment = scan.angle_increment

    min_distance = float('inf')
    min_angle = 0.0

    for i, distance in enumerate(ranges):
        if math.isfinite(distance) and distance < min_distance:
            min_distance = distance
            min_angle = angle_min + i * angle_increment

    if min_distance < float('inf'):
        angle_deg = math.degrees(min_angle)
        rospy.loginfo(f"Closest object: {min_distance:.3f} meters at {angle_deg:.1f}°")
    else:
        rospy.logwarn("No valid obstacle detected within range.")

def scan_callback(scan_msg):
    process_lidar(scan_msg)

def main():
    rospy.init_node('lidar_closest_distance_reader', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.loginfo("Listening to LIDAR scan... Press Ctrl+C to exit.")
    rospy.spin()

if __name__ == '__main__':
    main()
