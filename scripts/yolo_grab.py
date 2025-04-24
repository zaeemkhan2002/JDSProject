import time
import cv2
import numpy as np
from robomaster import robot
from ultralytics import YOLO

# Load YOLOv8 model (nano for Jetson-friendly inference)
model = YOLO("yolov8n.pt")

# Parameters
TARGET_OBJECT_NAME = "bottle"  # change this to "stool" if YOLO recognizes it
KNOWN_WIDTH = 0.1  # meters (adjust this for your target object width)
FOCAL_LENGTH = 800  # assumed/calibrated
MIN_DISTANCE_THRESHOLD = 0.35  # meters

def detect_with_yolo(image, object_name="chair"):
    results = model(image)
    for r in results:
        boxes = r.boxes
        for box in boxes:
            cls_id = int(box.cls[0])
            name = model.names[cls_id]
            if object_name.lower() in name.lower():
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                w = x2 - x1
                x_center = (x1 + x2) // 2

                # Draw bounding box for visualization
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, name, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                return (x_center, y1, w, y2 - y1), image
    return None, image

def estimate_distance(width, known_width=KNOWN_WIDTH, focal_length=FOCAL_LENGTH):
    return (known_width * focal_length) / width

def compute_angle(x, frame_width, fov=120):
    center_x = frame_width / 2
    angle = ((x - center_x) / center_x) * (fov / 2)
    return angle

def move_robot(ep_chassis, distance, angle):
    # Rotate first if needed
    if abs(angle) > 5:
        ep_chassis.move(x=0, y=0, z=angle, z_speed=30).wait_for_completed()
    if distance > 0.05:
        ep_chassis.move(x=distance, y=0, z=0, xy_speed=0.2).wait_for_completed()

def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")

    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    ep_camera.start_video_stream(display=False)
    time.sleep(1)

    try:
        while True:
            frame = ep_camera.read_cv2_image(strategy="newest")
            result = frame.copy()

            detection, result = detect_with_yolo(frame, TARGET_OBJECT_NAME)

            cv2.imshow("YOLO Detection", result)

            if detection:
                x, y, w, h = detection
                distance = estimate_distance(w)
                angle = compute_angle(x, frame.shape[1])

                print(f"Detected {TARGET_OBJECT_NAME} | Distance: {distance:.2f}m | Angle: {angle:.2f}°")

                if distance > MIN_DISTANCE_THRESHOLD:
                    move_robot(ep_chassis, min(distance * 0.25, 0.2), angle)
                else:
                    print("Object close enough to grab.")
                    break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.4)

        # Approach and pick up the object
        ep_chassis.move(x=0.075, y=0, z=0, xy_speed=0.2).wait_for_completed()
        time.sleep(2)
        ep_gripper.close()
        time.sleep(2)

        ep_arm.move(x=0, y=50).wait_for_completed()
        ep_chassis.move(x=-0.5, y=0, z=0, xy_speed=0.2).wait_for_completed()
        ep_gripper.open()
        ep_arm.move(x=0, y=-50).wait_for_completed()

    finally:
        ep_camera.stop_video_stream()
        ep_robot.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
