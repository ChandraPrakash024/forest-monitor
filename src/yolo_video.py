from ultralytics import YOLO
import cv2
from picamera2 import Picamera2
import numpy as np

# Load YOLOv8 model
model = YOLO('yolov8n.pt')
print("Model loaded.")

# Initialize the Picamera2 library
picam2 = Picamera2()
picam2.start()

print("Starting live detection. Press 'q' to quit.")

while True:
    # Capture frame from Raspberry Pi camera
    frame = picam2.capture_array()

    # Convert the frame to BGR format (3 channels)
    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

    # Perform inference using YOLOv8
    results = model(frame_bgr, conf=0.6)

    # Annotate the frame with results
    annotated_frame = results[0].plot()

    # Show the frame with annotations
    cv2.imshow("YOLOv8 Live Detection", annotated_frame)

    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

picam2.stop()
cv2.destroyAllWindows()
print("Live detection ended.")
