# import spidev
# import RPi.GPIO as GPIO
# import time
# from picamera2 import Picamera2
# from PIL import Image
# import io
# import numpy as np

# # LoRa module settings
# LORA_CS_PIN = 8
# LORA_RESET_PIN = 25

# # SPI setup
# spi = spidev.SpiDev(0, 0)
# spi.max_speed_hz = 5000000

# # GPIO setup
# GPIO.setmode(GPIO.BCM)
# GPIO.setup(LORA_RESET_PIN, GPIO.OUT)

# def lora_reset():
#     GPIO.output(LORA_RESET_PIN, GPIO.LOW)
#     time.sleep(0.01)
#     GPIO.output(LORA_RESET_PIN, GPIO.HIGH)
#     time.sleep(0.01)

# def lora_send(data):
#     spi.xfer2(data)

# def capture_image(camera):
#     # Capture the image as a NumPy array
#     frame = camera.capture_array()
    
#     # Convert to JPEG
#     stream = io.BytesIO()
#     img = Image.fromarray(frame)
#     img.save(stream, format='JPEG', quality=40)  # Reduce quality for smaller size
#     stream.seek(0)
#     return stream.read()

# try:
#     lora_reset()
#     print("LoRa Module Ready")
    
#     # Initialize the camera **only once**
#     camera = Picamera2()
#     config = camera.create_still_configuration(main={"size": (160, 120)})
#     camera.configure(config)
#     camera.start()
#     print("Camera initialized")

#     while True:
#         # Capture image and convert to bytes
#         image_data = capture_image(camera)
#         print(f"Captured Image Size: {len(image_data)} bytes")
        
#         # Break the image into chunks for LoRa transmission
#         chunk_size = 250
#         chunks = [image_data[i:i+chunk_size] for i in range(0, len(image_data), chunk_size)]
        
#         for chunk in chunks:
#             # Prepend each chunk with a start byte (0x00)
#             lora_send([0x00] + list(chunk))
#             time.sleep(0.05)  # Allow time for processing on the receiver side
            
#         print("Frame sent")
#         time.sleep(0.5)

# except KeyboardInterrupt:
#     print("Transmission stopped")
# finally:
#     camera.stop()  # Properly release the camera
#     spi.close()
#     GPIO.cleanup()
#     print("Resources released")
import RPi.GPIO as GPIO
import time
from picamera2 import Picamera2
from PIL import Image
import io
import numpy as np
import pyLoRa  # Correct import


# LoRa module settings
LORA_FREQUENCY = 433E6
LORA_SPREADING_FACTOR = 7
LORA_BANDWIDTH = 125E3
LORA_TX_POWER = 20

# GPIO setup
LORA_RESET_PIN = 25
GPIO.setmode(GPIO.BCM)
GPIO.setup(LORA_RESET_PIN, GPIO.OUT)

def lora_reset():
    GPIO.output(LORA_RESET_PIN, GPIO.LOW)
    time.sleep(0.01)
    GPIO.output(LORA_RESET_PIN, GPIO.HIGH)
    time.sleep(0.01)

def capture_image(camera):
    # Capture the image as a NumPy array
    frame = camera.capture_array()
    
    # Convert to JPEG
    stream = io.BytesIO()
    img = Image.fromarray(frame)
    img.save(stream, format='JPEG', quality=40)  # Reduce quality for smaller size
    stream.seek(0)
    return stream.read()

try:
    lora_reset()
    print("LoRa Module Reset")

    # Initialize LoRa with pyLora
    pyLora.init()
    pyLora.set_frequency(LORA_FREQUENCY)
    pyLora.set_spreading_factor(LORA_SPREADING_FACTOR)
    pyLora.set_bandwidth(LORA_BANDWIDTH)
    pyLora.set_tx_power(LORA_TX_POWER)
    print("LoRa Module Initialized")

    # Initialize the camera **only once**
    camera = Picamera2()
    config = camera.create_still_configuration(main={"size": (160, 120)})
    camera.configure(config)
    camera.start()
    print("Camera initialized")

    while True:
        # Capture image and convert to bytes
        image_data = capture_image(camera)
        print(f"Captured Image Size: {len(image_data)} bytes")
        
        # Break the image into chunks for LoRa transmission
        chunk_size = 250
        chunks = [image_data[i:i+chunk_size] for i in range(0, len(image_data), chunk_size)]
        
        for chunk in chunks:
            # Send the chunk over LoRa
            pyLora.send_packet(bytes([0x00]) + chunk)
            time.sleep(0.05)  # Allow time for processing on the receiver side
            
        print("Frame sent")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Transmission stopped")
finally:
    pyLora.close()  # Properly close the LoRa module
    GPIO.cleanup()
    print("Resources released")
