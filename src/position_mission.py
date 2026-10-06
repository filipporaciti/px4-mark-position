import numpy as np
from cv2 import aruco
import math
import asyncio
import json
import sys

from VisualOdometry import VisualOdometry
from DroneMavlink import DroneMavlink

from send_position import run_send_position
from mission import run_mission

import gc
import os

# Set the process priority to high (requires root privileges)
try:
    os.nice(-20)
except PermissionError:
    pass


async def main():

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
    
    visual_odometry = VisualOdometry(marker_type, camera_matrix, show_video=False, show_terminal=True,  marker_info_path="src/marker_info/aruco_floor.json", dist_coeff=dist_coeff)
    drone_mavlink = DroneMavlink(DRONE_ADDRESS)


    if len(sys.argv) != 2:
        print("Usage: python3 mission.py <mission_file.json>")
        sys.exit(1)

    mission_file = sys.argv[1]
    mission = json.load(open(mission_file, "r"))

    await asyncio.gather(
        run_send_position(drone_mavlink, visual_odometry, HEIGHT, WIDTH),
        run_mission(drone_mavlink, mission)
    )


if __name__ == "__main__":
    asyncio.run(main())
