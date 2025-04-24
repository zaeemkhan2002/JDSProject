#!/usr/bin/env python3
import rospy
import numpy as np
import math
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

# === Goal position ===
goal = np.array([0.0, 0.0])

# === Parameters ===
K_attr = 1.2
K_rep = 1.5
distance_tolerance = 0.3
max_linear_speed = 0.5
max_angular_speed_deg = 90

# === Robot state (from odom) ===
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0
laser_data = None

# === Callbacks ===
def quaternion_to_yaw(orientation):
    q = (orientation.x, orientation.y, orientation.z, orientation.w)
    return tf.transformations.euler_from_quaternion(q)[2]

def odom_callback(msg):
    global robot_x, robot_y, robot_theta
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    robot_theta = quaternion_to_yaw(msg.pose.pose.orientation)

def laser_callback(msg):
    global laser_data
    laser_data = msg

# === Force Computation ===
def compute_attractive_force(current_location):
    return K_attr * (goal - current_location)

def compute_repulsive_force(current_location):
    global laser_data
    repulsive_force = np.array([0.0, 0.0])

    if laser_data is None:
        return repulsive_force

    angle_min = laser_data.angle_min
    angle_increment = laser_data.angle_increment
    ranges = np.array(laser_data.ranges)

    valid_ranges = [r for r in ranges if not np.isnan(r)]
    if not valid_ranges:
        return repulsive_force

    closest_index, closest_distance = min(enumerate(valid_ranges), key=lambda x: x[1])
    angle = angle_min + closest_index * angle_increment

    obstacle_pos = np.array([
        robot_x + closest_distance * math.cos(angle + robot_theta),
        robot_y + closest_distance * math.sin(angle + robot_theta)
    ])
    d_obs = np.linalg.norm(np.array([robot_x, robot_y]) - obstacle_pos)

    if d_obs > 1e-3:
        rep = K_rep * (1 / (d_obs ** 1.5 + 1e-6))
        direction = (np.array([robot_x, robot_y]) - obstacle_pos) / (d_obs + 1e-6)
        repulsive_force = rep * direction

    return repulsive_force

# === Main Loop ===
def main():
    try:
        rospy.init_node("potential_field_controller", anonymous=True)

        # Subscribers
        rospy.Subscriber('/odom', Odometry, odom_callback)
        rospy.Subscriber('/scan', LaserScan, laser_callback)

        # Publisher
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)

        rospy.loginfo("Starting Potential Field Navigation...")

        while not rospy.is_shutdown():
            current_location = np.array([robot_x, robot_y])
            current_angle = robot_theta
            distance = np.linalg.norm(current_location - goal)

            if distance < distance_tolerance:
                rospy.loginfo("Goal reached!")
                cmd_pub.publish(Twist())
                break

            F_attr = compute_attractive_force(current_location)
            F_rep = compute_repulsive_force(current_location)
            F_net = F_attr - F_rep

            R_inv = np.array([[np.cos(current_angle), np.sin(current_angle)],
                            [-np.sin(current_angle), np.cos(current_angle)]])
            velocity = R_inv @ F_net
            linear_velocity, angular_velocity = velocity

            # Clip velocities
            linear_velocity = np.clip(linear_velocity, -max_linear_speed, max_linear_speed)
            angular_velocity = np.clip(math.degrees(angular_velocity), -max_angular_speed_deg, max_angular_speed_deg)

            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = math.radians(angular_velocity)
            cmd_pub.publish(twist)

            rate.sleep()

        cmd_pub.publish(Twist())  # final stop

    except:
        cmd_pub.publish(Twist())
if __name__ == "__main__":
    main()