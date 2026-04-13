import numpy as np
from cv2 import aruco
import cv2
import time
import asyncio

from VisualOdometry import VisualOdometry
from PIDVisualizer import PIDVisualizer
from DroneMavlink import DroneMavlink


drone_address = "udpin://0.0.0.0:14540"
video_url = "udp://127.0.0.1:5001?fifo_size=0&overrun_nonfatal=1"
marker_type = aruco.DICT_4X4_50
camera_matrix = np.array([
    [537.0, 0.0, 640.0], 
    [0.0, 537.0, 480.0], 
    [0.0, 0.0, 1.0]], 
    dtype=np.float32)
visual_odometry = VisualOdometry(video_url, marker_type, camera_matrix)
pid_visualizer = PIDVisualizer()
droneMavlink = DroneMavlink(drone_address)


async def run_async():
    await droneMavlink.connect()
    while True:
        timestamp_us = int(time.time() * 1e6) # Seconds to microseconds

        frame = visual_odometry.get_frame()
        ids, corners = visual_odometry.process_frame(frame)
        coordinates, yaw = visual_odometry.get_position(frame, corners, ids)
        
        if coordinates is not None and yaw is not None:
            pid_visualizer.update(coordinates[0], coordinates[1], coordinates[2], target_x=0.0, target_y=0.0, target_z=-2.0)

        await droneMavlink.update_position(timestamp_us, coordinates, yaw)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


if __name__ == "__main__":
    asyncio.run(run_async())