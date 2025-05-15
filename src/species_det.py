  import time
import os
import cv2
import torch
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from datetime import datetime
from pathlib import Path

# ==== SETTINGS ====
PIR_PIN = 17
VIDEO_DURATION = 5  # seconds
SAVE_DIR = Path("/home/pi/detections")
FRAME_INTERVAL = 10  # analyze 1 out of every 10 frames
YOLO_MODEL = 'yolov5n'

# ==== SETUP ====
SAVE_DIR.mkdir(exist_ok=True)

# Setup PIR
GPIO.setmode(GPIO.BCM)
GPIO.setup(PIR_PIN, GPIO.IN)

# Setup PiCamera2
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 480)}))

# Load YOLO model
model = torch.hub.load('ultralytics/yolov5', YOLO_MODEL)

# ==== MOTION HANDLER ====
def record_video(filename, duration):
    picam2.start_recording(f"{filename}.h264")
    time.sleep(duration)
    picam2.stop_recording()
    
    # Convert to MP4
    os.system(f"ffmpeg -y -i {filename}.h264 -c copy {filename}.mp4")
    os.remove(f"{filename}.h264")

def extract_frames(video_path, every_n=10):
    cap = cv2.VideoCapture(video_path)
    frames = []
    count = 0
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        if count % every_n == 0:
            frames.append(frame)
        count += 1
    cap.release()
    return frames

def get_direction(boxes):
    # Takes list of bounding box x-centers and estimates direction
    if len(boxes) < 2:
        return "Unknown"
    start = boxes[0]
    end = boxes[-1]
    if end > start:
        return "Left ➡ Right"
    elif end < start:
        return "Right ➡ Left"
    else:
        return "Stationary"

# ==== MAIN LOOP ====
print("System ready. Waiting for motion...")

try:
    while True:
        if GPIO.input(PIR_PIN):
            print("Motion detected!")

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            video_file = SAVE_DIR / f"motion_{timestamp}"
            record_video(str(video_file), VIDEO_DURATION)

            # Analyze
            print("Extracting frames...")
            frames = extract_frames(f"{video_file}.mp4", every_n=FRAME_INTERVAL)

            print("Running object detection...")
            species_seen = set()
            direction_track = []

            for frame in frames:
                results = model(frame)
                labels, coords = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
                names = results.names
                if labels.numel() == 0:
                    continue

                for *xyxy, conf, cls in results.pred[0]:
                    x_center = (xyxy[0] + xyxy[2]) / 2
                    direction_track.append(x_center.item())
                    species_seen.add(names[int(cls)])

            # Save snapshot
            if frames:
                snapshot_path = SAVE_DIR / f"snapshot_{timestamp}.jpg"
                cv2.imwrite(str(snapshot_path), frames[0])
                print(f"Saved snapshot: {snapshot_path}")

            # Determine direction
            direction = get_direction(direction_track)

            # Save log
            log_path = SAVE_DIR / f"log_{timestamp}.txt"
            with open(log_path, "w") as f:
                f.write(f"Species detected: {', '.join(species_seen) or 'None'}\n")
                f.write(f"Direction: {direction}\n")
                f.write(f"Video: {video_file}.mp4\n")
                f.write(f"Snapshot: {snapshot_path.name if frames else 'N/A'}\n")
            print(f"Detection logged at {log_path}")

            print("Waiting for next motion...\n")
            time.sleep(2)  # Debounce time

except KeyboardInterrupt:
    print("Shutting down...")
    GPIO.cleanup()
