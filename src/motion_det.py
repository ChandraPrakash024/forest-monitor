import time
from gpiozero import MotionSensor
from picamera2 import Picamera2
from datetime import datetime
import os

# === Configuration ===
PIR_PIN = 17  # GPIO pin where PIR sensor's OUT is connected
SAVE_DIR = "/home/pi/Pictures/motion_captures"  # Directory to save captured images
CAPTURE_DELAY = 1  # Delay (in seconds) after capturing to avoid multiple triggers

# === Setup PIR Sensor ===
pir = MotionSensor(PIR_PIN)

# === Setup Camera ===
picam2 = Picamera2()
picam2.configure(picam2.create_still_configuration())
picam2.start()

# === Create Save Directory If Not Exists ===
os.makedirs(SAVE_DIR, exist_ok=True)

print("Motion detection system ready. Waiting for movement...")

try:
    while True:
        pir.wait_for_motion()
        print("⚠️ Motion detected!")

        # Generate timestamp and image filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        image_path = os.path.join(SAVE_DIR, f"motion_{timestamp}.jpg")

        # Capture photo
        picam2.capture_file(image_path)
        print(f"✅ Image saved to: {image_path}")

        # Wait to avoid multiple captures for same event
        time.sleep(CAPTURE_DELAY)

except KeyboardInterrupt:
    print("\n Program stopped by user.")
finally:
    picam2.stop()
    print("Camera stopped. Goodbye!")
