import cv2
import face_recognition
import pickle
import numpy as np
import serial
import time
import os  # For using `say` on macOS

# === SERIAL SETUP ===
try:
    arduino = serial.Serial('/dev/cu.usbserial-1130', 9600, timeout=1)  # Updated port
    time.sleep(2)
    print("✅ Arduino connected on /dev/cu.usbserial-110")
    arduino.write(b'Connected to Raspberry Pi\n')
except Exception as e:
    print(f"❌ Failed to connect to Arduino: {e}")
    arduino = None

# === LOAD ENCODINGS ===
with open("encodings.pickle", "rb") as f:
    data = pickle.load(f)

# === START CAMERA ===
cap = cv2.VideoCapture(4)  # Use 4 if that is your working webcam index
if not cap.isOpened():
    print("❌ Camera error.")
    exit()

print("✅ Starting face recognition. Press 'q' to quit.")

last_sent_name = ""

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame.")
        break

    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    face_locations = face_recognition.face_locations(rgb_small_frame)
    face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    names_in_frame = []

    for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
        matches = face_recognition.compare_faces(data["encodings"], face_encoding)
        name = "Unknown"

        face_distances = face_recognition.face_distance(data["encodings"], face_encoding)
        if len(face_distances) > 0:
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = data["names"][best_match_index]

        names_in_frame.append(name)

        # Scale back up
        top *= 4
        right *= 4
        bottom *= 4
        left *= 4

        # Draw box and label
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
        cv2.putText(frame, name, (left + 6, bottom - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    # === Send to Arduino and Speak only if name has changed ===
    if arduino and names_in_frame:
        first_name = names_in_frame[0]
        if first_name != last_sent_name:
            try:
                message = first_name + "$"
                arduino.write(message.encode())
                print(f"📤 Sent to Arduino: {message}")

                # Speak
                if first_name == "Unknown":
                    os.system("say 'Hello unknown. Please register your face'")
                else:
                    os.system(f"say 'Hello {first_name}'")

                last_sent_name = first_name
            except Exception as e:
                print(f"❌ Error sending to Arduino: {e}")

    cv2.imshow("Face Recognition", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
    print("🔌 Arduino connection closed.")