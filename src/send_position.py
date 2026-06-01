import numpy as np
from cv2 import aruco
import cv2
import time
import asyncio
from picamera2 import Picamera2

from VisualOdometry import VisualOdometry


marker_type = aruco.DICT_4X4_50
camera_matrix = np.array([
    [530.5, 0.0, 320.0],
    [0.0, 530.5, 240.0], 
    [0.0, 0.0, 1.0]], 
    dtype=np.float32)
visual_odometry = VisualOdometry(marker_type, camera_matrix, show_video=False, marker_info_path="src/marker_info/aruco_floor.json")

print("Camera initialize...")
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    sensor={"output_size": (3280, 2464)}, 
    main={"size": (640, 480), "format": "RGB888"}
))

print("Camera starting...")
picam2.start()

async def run_async():
    try:
        while True:

            start_t = time.time()

            frame = picam2.capture_array()
            ids, corners = visual_odometry.process_frame(frame)
            coordinates, angles, cov_matrix = visual_odometry.get_position(frame, corners, ids)
    
            print("Time: ", time.time() - start_t)
    finally:
        picam2.stop()

if __name__ == "__main__":
    asyncio.run(run_async())
