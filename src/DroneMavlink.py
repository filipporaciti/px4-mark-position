import asyncio
import math
import time
import cv2

from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, Covariance, AngleBody, PositionBody


class DroneMavlink:

    def __init__(self, drone_address: str):
        self.drone_address = drone_address
        self.drone = System()

        self.__old_coordinates = [0.0, 0.0, 0.0]
        self.__old_yaw = 0.0

    
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

    async def get_roll_pitch(self):
        roll = 0.0
        pitch = 0.0
        async for attitude in self.drone.telemetry.attitude_euler():
            roll = math.radians(attitude.roll_deg)
            pitch = math.radians(attitude.pitch_deg)
            break
        return pitch, roll
    
    async def connect(self):
        await self.drone.connect(system_address=self.drone_address)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

    async def update_position(self, timestamp_us, coordinates, yaw):
        if coordinates is None or yaw is None:
            coordinates = self.__old_coordinates
            yaw = self.__old_yaw
        self.__old_coordinates = coordinates
        self.__old_yaw = yaw

        pitch, roll = await self.get_roll_pitch()

        cov_matrix = self.get_covariance_matrix(0.05, coordinates[2] * 0.05, math.radians(1))

        await self.drone.mocap.set_vision_position_estimate(VisionPositionEstimate(
            timestamp_us,
            PositionBody(coordinates[0], coordinates[1], coordinates[2]),
            AngleBody(roll, pitch, yaw),
            Covariance(cov_matrix)
            ))         


if __name__ == "__main__":
    drone_address = "udpin://0.0.0.0:14540"
    droneMavlink = DroneMavlink(drone_address)

    async def run_async():
        await droneMavlink.connect()
        while True:
            timestamp_us = int(time.time() * 1e6) # Seconds to microseconds

            coordinates = [0.0, 0.0, 0.0]
            yaw = 0.0

            await droneMavlink.update_position(timestamp_us, coordinates, yaw)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    asyncio.run(run_async())