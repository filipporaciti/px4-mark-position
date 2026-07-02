import numpy as np
from cv2 import aruco
import time
import math
import asyncio
from picamera2 import Picamera2

from VisualOdometry import VisualOdometry
from DroneMavlink import DroneMavlink

WIDTH = 640
HEIGHT = 480
FOCAL_LENGTH = (WIDTH/2)/math.tan(math.radians(31.1))

drone_address = "serial:///dev/serial0:921600"
marker_type = aruco.DICT_4X4_50
camera_matrix = np.array([
    [FOCAL_LENGTH, 0.0, (WIDTH/2)],
    [0.0, FOCAL_LENGTH, (HEIGHT/2)], 
    [0.0, 0.0, 1.0]], 
    dtype=np.float32)
visual_odometry = VisualOdometry(marker_type, camera_matrix, show_video=False, show_terminal=False,  marker_info_path="src/marker_info/aruco_floor.json")
droneMavlink = DroneMavlink(drone_address)

print("Camera initialize...")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    sensor={"output_size": (3280, 2464)}, 
    main={"size": (WIDTH, HEIGHT), "format": "YUV420"}
))

print("Camera starting...")
picam2.start()

async def run_async():
    await droneMavlink.connect()
    try:
        while True:
            yuv = picam2.capture_array()
            gray = yuv[:HEIGHT, :WIDTH]
            ids, corners = visual_odometry.process_frame(gray, grayConvert=False)
            coordinates, angles, cov_matrix = visual_odometry.get_position(gray, corners, ids)

            timestamp_us = int(time.time() * 1e6) # Seconds to microseconds

            await droneMavlink.update_position(timestamp_us, coordinates, angles, cov_matrix)
    finally:
        picam2.stop()

if __name__ == "__main__":
    asyncio.run(run_async())
