import time
import cv2
import subprocess
import os
import rospy
import tf
from nav_msgs.msg import Odometry
from robomaster import robot
from robomaster import camera

# === Odometry globals ===
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

def quaternion_to_yaw(orientation):
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2]

def odom_callback(msg):
    global robot_x, robot_y, robot_theta
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y
    robot_theta = quaternion_to_yaw(msg.pose.pose.orientation)

def capture_images_during_rotation(output_dir="image-feed", num_pics=3):
    rospy.init_node("robomaster_image_capture_node", anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize robot and camera
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis

    ep_camera.start_video_stream(display=True, resolution=camera.STREAM_360P)
    time.sleep(4)

    degrees_per_shot = 360 / num_pics

    for i in range(num_pics):
        if i != 0:
            print(f"Rotating {degrees_per_shot}°...")
            ep_chassis.move(x=0, y=0, z=degrees_per_shot, z_speed=30).wait_for_completed()
            time.sleep(1.5)  # Allow stabilization

        frame = ep_camera.read_cv2_image()
        if frame is not None:
            image_path = os.path.join(output_dir, f"frame_{i}.png")
            cv2.imwrite(image_path, frame)
            print(f"Saved {image_path} | Pose=({robot_x:.2f}, {robot_y:.2f}, {robot_theta:.2f} rad)")
        else:
            print(f"Failed to capture frame {i}")

    # Optional: return to original orientation
    ep_chassis.move(x=0, y=0, z=degrees_per_shot, z_speed=30).wait_for_completed()

    ep_camera.stop_video_stream()
    ep_robot.close()

def run_gemini_captioning():
    try:
        print("Forcing pyenv environment activation...")

        command = """
        export PATH="$HOME/.pyenv/bin:$PATH"
        eval "$(pyenv init --path)"
        eval "$(pyenv init -)"
        eval "$(pyenv virtualenv-init -)"
        cd newpython
        pyenv shell ros-free-env
        python gemini_vlm.py
        """
        subprocess.run(["bash", "-c", command], check=True)
        print("Gemini script executed in correct env.")
    except subprocess.CalledProcessError as e:
        print(f"Error running Gemini script: {e}")

if __name__ == "__main__":
    capture_images_during_rotation()
    run_gemini_captioning()