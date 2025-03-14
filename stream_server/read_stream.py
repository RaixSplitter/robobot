import cv2
import numpy as np

cap = cv2.VideoCapture('http://192.168.2.211:7123/stream.mjpg')

if not cap.isOpened():
    cap.release()
    cv2.destroyAllWindows()
    print("Failed to open video stream")
    exit(1)

while cap.isOpened():
    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
