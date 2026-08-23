import numpy as np
from cv2 import aruco
import time
import math
import asyncio
from picamera2 import Picamera2

from VisualOdometry import VisualOdometry
from DroneMavlink import DroneMavlink

import gc
import os

# High priority for the process to reduce latency in the main loop. Execute as root
try:
    os.nice(-20)
except PermissionError:
    pass

async def run_send_position(drone_mavlink: DroneMavlink, visual_odometry: VisualOdometry, height: int, width: int):

    print("Camera initialize...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        sensor={"output_size": (3280, 2464)}, 
        main={"size": (width, height), "format": "YUV420"},
        buffer_count=2
    ))

    print("Camera starting...")
    picam2.start()
    picam2.set_controls({
        "ExposureTime": 5000, 
        "FrameDurationLimits": (66666, 66666)
        })

    await drone_mavlink.connect()
    try:
        while True:

            # ===== Get frame and metadata =====
            request = picam2.capture_request()

            yuv = request.make_array("main")
            gray = yuv[:height, :width]

            metadata = request.get_metadata()
            sensor_timestamp_us = metadata["SensorTimestamp"] // 1000

            request.release()
            # ==================================

            ids, corners = visual_odometry.process_frame(gray, grayConvert=False)
            coordinates, angles, cov_matrix = visual_odometry.get_position(gray, corners, ids)

            await drone_mavlink.update_position(sensor_timestamp_us, coordinates, angles, cov_matrix)
    finally:
        picam2.stop()

if __name__ == "__main__":

    WIDTH = 640
    HEIGHT = 480

    DRONE_ADDRESS = "serial:///dev/serial0:921600"

    marker_type = aruco.DICT_4X4_50
    camera_matrix = np.array(
        [[507.69726421,    0,         323.50083348 ],
        [  0,         507.66044937, 230.86122221],
        [  0,           0,           1        ]], 
        dtype=np.float32)
    dist_coeff = np.array([ 0.15549911, -0.08546357, -0.00459428,  0.00295946, -0.75765939], dtype=np.float32)
    visual_odometry = VisualOdometry(marker_type, camera_matrix, show_video=False, show_terminal=False,  marker_info_path="src/marker_info/aruco_floor.json", dist_coeff=dist_coeff)
    drone_mavlink = DroneMavlink(DRONE_ADDRESS)

    asyncio.run(run_send_position(drone_mavlink, visual_odometry, HEIGHT, WIDTH))
