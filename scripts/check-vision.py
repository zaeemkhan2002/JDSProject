#!/usr/bin/env python3
import cv2
import numpy as np
import time
import json
import math
import rospy
import tf
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge

# === Globals ===
bridge = CvBridge()
latest_frame = None
laser_data = None
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

# === Callbacks ===
def image_callback(msg):
    global latest_frame
    latest_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

def odom_callback(msg):
    global robot_x, robot_y, robot_theta
    q = msg.pose.pose.orientation
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    robot_theta = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

def lidar_callback(msg):
    global laser_data
    laser_data = msg

# === Object Detection ===
def get_object_frame_and_hsv(json_path="image-response/response.json"):
    with open(json_path, "r") as f:
        data = json.load(f)
    for i in range(3):
        frame_name = f"frame_{i}.png"
        info = data.get(frame_name, {})
        if info.get("is_object") == "Yes":
            return i * 120, np.array(info["lower_hsv"]), np.array(info["upper_hsv"])
    return 0, np.array([0, 0, 0]), np.array([0, 0, 0])

def detect_object(image, lower_hsv, upper_hsv):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    object_x, box_width = None, None
    if contours:
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        object_x = x + w // 2
        box_width = w
    return object_x, box_width, mask

def rotate_to_center(cmd_pub, object_x, frame_width, pixel_tolerance=6):
    center_x = frame_width // 2
    offset = object_x - center_x
    if abs(offset) > pixel_tolerance:
        twist = Twist()
        twist.angular.z = 0.3 if offset > 0 else -0.3
        cmd_pub.publish(twist)
        time.sleep(0.4)
        cmd_pub.publish(Twist())
        return True
    return False

def wait_until_close_lidar(cmd_pub, lower_hsv, upper_hsv, gripper_pub, arm_pub):
    print("Using LIDAR to approach the object...")
    ALIGNMENT_INTERVAL = 2
    last_align_time = time.time()
    SAFE_DISTANCE = 0.25
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if laser_data is None or latest_frame is None:
            rate.sleep()
            continue

        ranges = np.array(laser_data.ranges)
        angle_min = laser_data.angle_min
        angle_increment = laser_data.angle_increment

        min_angle = math.radians(-150)
        max_angle = math.radians(-135)
        min_distance = float("inf")

        for i, d in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if min_angle <= angle <= max_angle and math.isfinite(d):
                min_distance = min(min_distance, d)

        if time.time() - last_align_time > ALIGNMENT_INTERVAL:
            frame = latest_frame.copy()
            object_x, _, _ = detect_object(frame, lower_hsv, upper_hsv)
            if object_x is not None:
                rotate_to_center(cmd_pub, object_x, frame.shape[1])
            last_align_time = time.time()

        twist = Twist()
        if not math.isfinite(min_distance):
            twist.linear.x = 0.1
        elif min_distance < SAFE_DISTANCE:
            twist.linear.x = 0.0
            cmd_pub.publish(twist)
            break
        else:
            twist.linear.x = 0.1
        cmd_pub.publish(twist)
        rate.sleep()

    print("Picking up object...")
    gripper_pub.publish("open")
    arm_pub.publish("down")
    time.sleep(1)
    gripper_pub.publish("close")
    time.sleep(1)
    arm_pub.publish("lift")
    time.sleep(1)

    twist = Twist()
    twist.linear.x = -0.2
    cmd_pub.publish(twist)
    time.sleep(1)
    cmd_pub.publish(Twist())

    arm_pub.publish("down")
    time.sleep(1)
    gripper_pub.publish("open")
    time.sleep(1)

# === Main ===
def main():
    rospy.init_node("vision_pick_and_place", anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    rospy.Subscriber("/ep_camera/image_raw", Image, image_callback)

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    gripper_pub = rospy.Publisher("/gripper_cmd", String, queue_size=10)
    arm_pub = rospy.Publisher("/arm_cmd", String, queue_size=10)

    angle_to_rotate, lower_hsv, upper_hsv = get_object_frame_and_hsv()
    print(f"[HSV] Lower: {lower_hsv}, Upper: {upper_hsv}")

    last_alignment_time = time.time()
    aligned = False

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if latest_frame is None:
            rate.sleep()
            continue

        frame = latest_frame.copy()
        object_x, box_width, _ = detect_object(frame, lower_hsv, upper_hsv)

        if object_x is not None:
            if not aligned or time.time() - last_alignment_time > 2:
                aligned = rotate_to_center(cmd_pub, object_x, frame.shape[1])
                last_alignment_time = time.time()
                continue

            print(f"[INFO] Object aligned. Width: {box_width}")
            wait_until_close_lidar(cmd_pub, lower_hsv, upper_hsv, gripper_pub, arm_pub)
            print("Done.")
            break

        rate.sleep()

if __name__ == "__main__":
    main()