import cv2
import mediapipe as mp
import numpy as np
import serial
import time

# Configure Serial
arduino = serial.Serial('COM5', 115200)
time.sleep(2)

# Mediapipe setup
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.5)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

# Servo practical ranges
BASE_MIN, BASE_MAX = 0, 180
SHOULDER_MIN, SHOULDER_MAX = 30, 150
ELBOW_MIN, ELBOW_MAX = 30, 150
GRAB_MIN, GRAB_MAX = 10, 120

def map_value(value, in_min, in_max, out_min, out_max):
    return int((value - in_min)*(out_max - out_min)/(in_max - in_min)+out_min)

def distance(p1, p2):
    return np.linalg.norm(np.array(p1)-np.array(p2))

def smooth(current, target, factor=0.2):
    return current + (target - current) * factor

# Initialize angles
base_angle = 90
shoulder_angle = 90
elbow_angle = 90
grab_angle = GRAB_MAX

# For drawing path
hand_path = []
#hand handiling
try:
    while True:
        ret, img = cap.read()
        if not ret:
            break
        img = cv2.flip(img, 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        res_hands = hands.process(img_rgb)

        if res_hands.multi_hand_landmarks:
            hl = res_hands.multi_hand_landmarks[0]
            # Get normalized hand center
            cx = np.mean([lm.x for lm in hl.landmark])
            cy = np.mean([lm.y for lm in hl.landmark])
            # Convert to pixel coordinates
            px = int(cx * img.shape[1])
            py = int(cy * img.shape[0])

            # Append to path for drawing
            hand_path.append((px, py))
            if len(hand_path) > 50:  # Limit path length
                hand_path.pop(0)

            # Draw circle on hand center
            cv2.circle(img, (px, py), 15, (0, 0, 255), cv2.FILLED)
            # Draw path
            for i in range(1, len(hand_path)):
                cv2.line(img, hand_path[i-1], hand_path[i], (255, 0, 0), 2)

            # Base angle (move left/right) - flipped mapping
            target_base = map_value(cx, 0, 1, BASE_MAX, BASE_MIN)
            base_angle = smooth(base_angle, target_base)

            # Distance from camera (approx z-axis)
            wrist = hl.landmark[0]
            index_tip = hl.landmark[8]
            z_distance = distance([wrist.x, wrist.y], [index_tip.x, index_tip.y])
            # Shoulder/elbow mapping
            target_shoulder = map_value(z_distance, 0.02, 0.2, SHOULDER_MAX, SHOULDER_MIN)
            target_elbow = map_value(z_distance, 0.02, 0.2, ELBOW_MIN, ELBOW_MAX)
            shoulder_angle = smooth(shoulder_angle, target_shoulder)
            elbow_angle = smooth(elbow_angle, target_elbow)

            # Grab (thumb-index distance)
            thumb = [hl.landmark[4].x, hl.landmark[4].y]
            index = [hl.landmark[8].x, hl.landmark[8].y]
            target_grab = GRAB_MIN if distance(thumb, index)<0.05 else GRAB_MAX
            grab_angle = smooth(grab_angle, target_grab, factor=0.3)

        # Send angles to Arduino
        command = f"{int(base_angle)},{int(shoulder_angle)},{int(elbow_angle)},{int(grab_angle)}\n"
        arduino.write(command.encode())

        cv2.imshow("Arm Control", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    hands.close()
    arduino.close()