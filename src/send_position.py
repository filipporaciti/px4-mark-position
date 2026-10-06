from picamera2 import Picamera2
from cv2 import aruco
import numpy as np
import asyncio
import time
import os

from VisualOdometry import VisualOdometry
from DroneMavlink import DroneMavlink

# Set the process priority to high (requires root privileges)
try:
    os.nice(-20)
except PermissionError:
    pass

async def run_send_position(drone_mavlink: DroneMavlink, visual_odometry: VisualOdometry, height: int, width: int):

    print("Camera initialize...")
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={
            "size": (width, height), 
            "format": "YUV420"
            },
        buffer_count=2,
        controls={
            "ExposureTime": 1500,
            "FrameDurationLimits": (16666, 16666) # 60 fps
            }
    ))

    print("Camera starting...")
    picam2.start()
    time.sleep(1)

    await drone_mavlink.connect()

    loop = asyncio.get_running_loop()

    def frame_analyzer(job):
        request = picam2.wait(job)

        yuv = request.make_array("main")
        gray = yuv[:height, :width]

        metadata = request.get_metadata()
        sensor_timestamp_us = metadata["SensorTimestamp"] // 1000

        request.release()
        # =======

        ids, corners = visual_odometry.process_frame(gray, grayConvert=False)
        coordinates, angles, cov_matrix = visual_odometry.get_position(gray, corners, ids)
        
        asyncio.run_coroutine_threadsafe(
            drone_mavlink.update_position(sensor_timestamp_us, coordinates, angles, cov_matrix),
            loop
        )


    try:
        while True:
            picam2.capture_request(signal_function=frame_analyzer)
            await asyncio.sleep(0.01) # Allow other tasks to run
    finally:
        picam2.stop()

if __name__ == "__main__":

    WIDTH = 640
    HEIGHT = 480

    DRONE_ADDRESS = None

    marker_type = aruco.DICT_4X4_50
    camera_matrix = np.array(
        [[663.84860945,   0,         324.23725676],
         [  0,         663.64022013, 244.3655193 ],
         [  0,           0,           1        ]],
 
        dtype=np.float32)
    dist_coeff = np.array([-3.86314231e-01,  4.79606396e-01, -1.66928451e-03,  3.68820236e-04, -7.73846826e-01], dtype=np.float32)
    
    visual_odometry = VisualOdometry(marker_type, camera_matrix, show_video=False, show_terminal=False,  marker_info_path="src/marker_info/aruco_floor.json", dist_coeff=dist_coeff)
    drone_mavlink = DroneMavlink(DRONE_ADDRESS)

    asyncio.run(run_send_position(drone_mavlink, visual_odometry, HEIGHT, WIDTH))