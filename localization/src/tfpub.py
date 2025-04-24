#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros

def odom_callback(msg):
    # Extracting relevant data from the Odometry message
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation

    # Convert quaternion to Euler angles
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = euler_from_quaternion(quaternion)

    # Invert the yaw angle
    yaw = -euler[2]  # Negate the yaw

    # Convert back to quaternion
    inverted_quaternion = quaternion_from_euler(euler[0], euler[1], yaw)

    # Creating a TransformStamped message
    broadcaster = tf2_ros.TransformBroadcaster()
    odom_trans = TransformStamped()
    odom_trans.header.stamp = rospy.Time.now()
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"

    # Setting the translation
    odom_trans.transform.translation.x = position.x
    odom_trans.transform.translation.y = -position.y
    odom_trans.transform.translation.z = position.z

    # Setting the new inverted orientation
    odom_trans.transform.rotation.x = inverted_quaternion[0]
    odom_trans.transform.rotation.y = inverted_quaternion[1]
    odom_trans.transform.rotation.z = inverted_quaternion[2]
    odom_trans.transform.rotation.w = inverted_quaternion[3]

    # Publishing the TF transform
    broadcaster.sendTransform(odom_trans)

if __name__ == '__main__':
    rospy.init_node('odom_to_tf_publisher')
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

