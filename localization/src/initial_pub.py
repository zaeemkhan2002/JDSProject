#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion

def set_initial_pose():
    # Initialize the node
    rospy.init_node('set_initial_pose', anonymous=False)

    # Create a publisher object
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

    # Define the rate of publishing
    rate = rospy.Rate(1)  # 1 Hz, adjust as needed

    while not rospy.is_shutdown():
        # Create the PoseWithCovarianceStamped message
        initial_pose = PoseWithCovarianceStamped()

        # Set the header information
        initial_pose.header.stamp = rospy.Time.now()
        initial_pose.header.frame_id = "map"  # or "odom" based on your configuration

        # Set the pose (x, y, z, and orientation as quaternion)
        initial_pose.pose.pose.position.x = -8.0
        initial_pose.pose.pose.position.y = 5.5
        initial_pose.pose.pose.position.z = 0.0
        initial_pose.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))  
        # Set the covariance matrix (6x6)
        initial_pose.pose.covariance = [
            0.25, 0, 0, 0, 0, 0,  # Variance in x
            0, 0.25, 0, 0, 0, 0,  # Variance in y
            0, 0, 0, 0, 0, 0,  # Variance in z (ignore if on a flat plane)
            0, 0, 0, 0, 0, 0,  # Variance in roll (ignore if non-holonomic)
            0, 0, 0, 0, 0, 0,  # Variance in pitch (ignore)
            0, 0, 0, 0, 0, 0  # Variance in yaw
        ]

        # Publish the message
        pub.publish(initial_pose)
        # rospy.loginfo("Initial pose published.")

        # Sleep to maintain the publishing rate
        rate.sleep()

def quaternion_from_euler(roll, pitch, yaw):
    import tf
    a = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    return a
if __name__ == '__main__':
    try:
        set_initial_pose()
    except rospy.ROSInterruptException:
        pass
