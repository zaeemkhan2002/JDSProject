#!/usr/bin/env python3
import rospy
import math
import tf
import numpy as np
from robomaster import robot
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from std_msgs.msg import String
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2



class RoboMasterController:
    def __init__(self):
        rospy.init_node("robomaster_controller")

        # SDK Init
        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="rndis")
        self.ep_chassis = self.ep_robot.chassis
        self.ep_arm = self.ep_robot.robotic_arm
        self.ep_gripper = self.ep_robot.gripper
        self.ep_camera = self.ep_robot.camera

        self.ep_chassis.sub_position(freq=10, callback=self.update_position)
        self.ep_chassis.sub_attitude(freq=10, callback=self.update_attitude)

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_time = rospy.Time.now().to_sec()

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0

        self.x_vel = 0.0
        self.y_vel = 0.0
        self.angular_vel = 0.0

        # Publishers and subscribers
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        rospy.Subscriber("/gripper_cmd", String, self.gripper_callback)
        rospy.Subscriber("/arm_cmd", String, self.arm_callback)
        # In __init__:
        self.ep_camera.start_video_stream(display=False)
        
        self.image_pub = rospy.Publisher("/ep_camera/image_raw", Image, queue_size=1)
        self.bridge = CvBridge()
        
        self.rate = rospy.Rate(10)

    def update_position(self, pos):
        # SDK returns (x, y, z)
        self.x, self.y, _ = pos

    def update_attitude(self, att):
        # SDK returns (yaw, pitch, roll)
        yaw, _, _ = att
        self.theta = math.radians(yaw)

    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular_deg = math.degrees(msg.angular.z)
        self.ep_chassis.drive_speed(x=linear, y=0, z=angular_deg)

    def gripper_callback(self, msg):
        if msg.data == "open":
            self.ep_gripper.open()
        elif msg.data == "close":
            self.ep_gripper.close()

    def arm_callback(self, msg):
        if msg.data == "down":
            self.ep_arm.moveto(x=100, y=120).wait_for_completed()
        elif msg.data == "lift":
            self.ep_arm.moveto(x=50, y=150).wait_for_completed()

    def publish_odometry(self):
        current_time = rospy.Time.now()
        dt = current_time.to_sec() - self.prev_time
        self.prev_time = current_time.to_sec()

        self.x_vel = (self.x - self.prev_x) / dt
        self.y_vel = (self.y - self.prev_y) / dt
        self.angular_vel = (self.theta - self.prev_theta) / dt

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.position = Point(self.x, self.y, 0)
        odom.pose.pose.orientation = Quaternion(*quat)
        odom.twist.twist.linear.x = self.x_vel
        odom.twist.twist.linear.y = self.y_vel
        odom.twist.twist.angular.z = self.angular_vel

        self.odom_pub.publish(odom)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_odometry()
            frame = self.ep_camera.read_cv2_image(strategy="newest")
            if frame is not None:
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(image_msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        controller = RoboMasterController()
        controller.run()
    except rospy.ROSInterruptException:
        pass