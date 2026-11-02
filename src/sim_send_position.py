from cv2 import aruco
import cv2
import numpy as np
import asyncio
import time
import math

from VisualOdometry import VisualOdometry
from DroneMavlink import DroneMavlink


WIDTH = 1280
HEIGHT = 960
FOV = 100       # degrees
FOCAL_LENGTH = (WIDTH/2)/math.tan(math.radians(FOV/2))

drone_address = "udpin://0.0.0.0:14540"
marker_type = aruco.DICT_4X4_50
camera_matrix = np.array([
    [FOCAL_LENGTH, 0.0, (WIDTH/2)],
    [0.0, FOCAL_LENGTH, (HEIGHT/2)], 
    [0.0, 0.0, 1.0]], 
    dtype=np.float32)
visual_odometry = VisualOdometry(marker_type, camera_matrix)
droneMavlink = DroneMavlink(drone_address)

pipeline = (
    'udpsrc port=5600 '
    'caps="application/x-rtp,media=video,'
    'clock-rate=90000,encoding-name=H264" '
    '! rtph264depay '
    '! avdec_h264 '
    '! videoconvert '
    '! video/x-raw,format=BGR '
    '! appsink drop=true sync=false'
)
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


if not cap.isOpened():
    raise Exception(f"Error: Unable to connect to the video stream")


async def run_async():
    await droneMavlink.connect()
    while True:

        _, frame = cap.read()
        ids, corners = visual_odometry.process_frame(frame)
        coordinates, angles, cov_matrix = visual_odometry.get_position(frame, corners, ids)
        
        timestamp_us = int(time.time() * 1e6) # Seconds to microseconds

        await droneMavlink.update_position(timestamp_us, coordinates, angles, cov_matrix)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    asyncio.run(run_async())