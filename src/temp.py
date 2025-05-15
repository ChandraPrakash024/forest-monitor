import torch
import cv2

# Load the YOLOv5 model (small model for faster performance)
model = torch.hub.load('ultralytics/yolov5', 'yolov5n')  # You can also try 'yolov5s', 'yolov5m', etc.

# Initialize video capture (0 for the default webcam)
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # Perform inference on the current frame
    results = model(frame)

    # Print results (optional, can be used for debugging or logging)
    results.print()

    # Render results (this adds the bounding boxes to the frame)
    results.render()

    # Display the frame with detections
    cv2.imshow("YOLOv5 Object Detection", frame)

    # Press 'q' to quit the live feed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close the window
cap.release()
cv2.destroyAllWindows()
