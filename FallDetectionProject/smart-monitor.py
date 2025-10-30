import cv2
import mediapipe as mp
import serial
import threading
def monitor_sensors():
    try:
        arduino = serial.Serial('COM3', 9600) 
        while True:
            line = arduino.readline().decode('utf-8').strip()
            if ',' in line: 
                pulse, temp = line.split(',')
                pulse = int(pulse)
                temp = float(temp)
                print(f"Heart Rate: {pulse}, Temperature: {temp:.1f}C")

                if pulse > 100:
                    print("⚠️ Abnormal heart rate detected!")
                if temp > 38:
                    print("⚠️ High temperature detected!")
    except Exception as e:
        print("Sensor monitoring error:", e)
sensor_thread = threading.Thread(target=monitor_sensors, daemon=True)
sensor_thread.start()
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
cap = cv2.VideoCapture(0)
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = pose.process(rgb)

    if result.pose_landmarks:
        mp_drawing.draw_landmarks(frame, result.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        landmarks = result.pose_landmarks.landmark

        left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
        left_hip = landmarks[mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = landmarks[mp_pose.PoseLandmark.RIGHT_HIP]

        shoulder_y = (left_shoulder.y + right_shoulder.y)/2
        hip_y = (left_hip.y + right_hip.y)/2
        vertical_ratio = abs(hip_y - shoulder_y)

        if vertical_ratio < 0.15:
            cv2.putText(frame, "⚠️ FALL DETECTED!", (50, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)

    cv2.imshow("Smart Health Monitoring", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()