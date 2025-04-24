import cv2
import numpy as np
from robomaster import robot
import time
import json
import os

def get_object_frame_and_hsv(json_path="image-response/response.json"):
    with open(json_path, "r") as f:
        data = json.load(f)

    for i in range(3):
        frame_name = f"frame_{i}.png"
        info = data.get(frame_name, {})
        if info.get("is_object") == "Yes":
            return i * 120, np.array(info["lower_hsv"]), np.array(info["upper_hsv"])
    return 0, np.array([0, 0, 0]), np.array([0, 0, 0])



def detect_and_label(image, lower_hsv, upper_hsv):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = image.copy()

    if contours:
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) > 300:  # avoid noise
            x, y, w, h = cv2.boundingRect(largest)
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(result, "Tracking", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.circle(result, (x + w // 2, y + h // 2), 4, (0, 0, 255), -1)

    return mask, result

def main():
    # Define your HSV color range (tweak based on object color)
    angle_to_rotate, lower_hsv, upper_hsv = get_object_frame_and_hsv()

    print(f"[INFO] Rotating {angle_to_rotate}° to face object.")
    if angle_to_rotate > 0:
        ep_chassis.drive_speed(x=0, y=0, z=20)
        time.sleep(angle_to_rotate / 60)  # ~60°/sec assumed
        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(1)


    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")
    ep_camera = ep_robot.camera

    ep_camera.start_video_stream(display=False)
    time.sleep(1)

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest")
            mask, labeled = detect_and_label(frame, lower_hsv, upper_hsv)

            cv2.imshow("Segmentation Mask", mask)
            cv2.imshow("Object Tracking View", labeled)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
