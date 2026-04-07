import asyncio
import math
import time

from cv2 import aruco
import cv2
from VisualOdometry import VisualOdometry

from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, Covariance, AngleBody, PositionBody


class DroneMavlink:

    def __init__(self, drone_address: str, visual_odometry: VisualOdometry):
        self.drone_address = drone_address
        self.drone = System()
        self.visual_odometry = visual_odometry

    
    def get_covariance_matrix(self, dev_xy: float, dev_z: float, dev_yaw: float):
        v_x = dev_xy**2
        v_y = dev_xy**2
        v_z = dev_z**2
        v_roll = dev_yaw**2
        v_pitch = dev_yaw**2
        v_yaw = dev_yaw**2

        cov_matrix = [
            v_x, 0, 0, 0, 0, 0,
                v_y, 0, 0, 0, 0,
                    v_z, 0, 0, 0,
                        v_roll, 0, 0,
                            v_pitch, 0,
                                v_yaw
        ]
        return cov_matrix


    async def run(self):
        await self.drone.connect(system_address=self.drone_address)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break
            
        old_coordinates = [0.0, 0.0, 0.0]
        old_yaw = 0.0

        while True:
            timestamp_us = int(time.time() * 1e6) # Seconds to microseconds

            frame = self.visual_odometry.get_frame()
            ids, corners = self.visual_odometry.process_frame(frame)
            coordinates, yaw = self.visual_odometry.get_position(frame, corners, ids)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if coordinates is None or yaw is None:
                coordinates = old_coordinates
                yaw = old_yaw
            old_coordinates = coordinates
            old_yaw = yaw

            current_pitch = 0
            current_roll = 0
            async for attitude in self.drone.telemetry.attitude_euler():
                current_roll = math.radians(attitude.roll_deg)
                current_pitch = math.radians(attitude.pitch_deg)
                break

            cov_matrix = self.get_covariance_matrix(0.5, coordinates[2] * 0.1, math.radians(2))

            await self.drone.mocap.set_vision_position_estimate(VisionPositionEstimate(
                timestamp_us,
                PositionBody(coordinates[1], coordinates[0], -coordinates[2]),
                AngleBody(current_roll, -current_pitch, -yaw),
                Covariance(cov_matrix)
                ))
                

if __name__ == "__main__":
    drone_address = "udpin://0.0.0.0:14540"
    video_url = "udp://127.0.0.1:5001?fifo_size=0&overrun_nonfatal=1"
    marker_type = aruco.DICT_4X4_50
    visual_odometry = VisualOdometry(video_url, marker_type)
    droneMavlink = DroneMavlink(drone_address, visual_odometry)
    asyncio.run(droneMavlink.run())