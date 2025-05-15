import time
import subprocess
import torch
import RPi.GPIO as GPIO

# Initialize PIR sensor (change GPIO pin number if needed)
PIR_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_PIN, GPIO.IN)

# Load YOLOv5 model (small version for faster inference)
model = torch.hub.load('ultralytics/yolov5', 'yolov5n')  # Load small model

def capture_image(output_file="image.jpg"):
    # Using libcamera to capture an image
    subprocess.run(['libcamera-still', '-o', output_file])

def detect_objects(image_file):
    # Load image and make predictions with YOLO
    results = model(image_file)
    results.print()  # Print out the results (labels and confidence)
    results.show()   # Display the image with predictions

def main():
    try:
        while True:
            if GPIO.input(PIR_PIN):  # Motion detected
                print("Motion detected! Capturing image...")
                capture_image("motion_image.jpg")  # Capture image
                
                # Run object detection on the captured image
                detect_objects("motion_image.jpg")
                time.sleep(1)  # Small delay to avoid multiple detections
            time.sleep(0.1)  # Small delay to avoid maxing out CPU
    except KeyboardInterrupt:
        print("Program exited.")
    finally:
        GPIO.cleanup()

# Start the program
if __name__ == "__main__":
    main()
