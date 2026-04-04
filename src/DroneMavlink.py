import asyncio
import time

from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, Covariance, AngleBody, PositionBody


class DroneMavlink:

    def __init__(self, drone_address: str):
        self.drone_address = drone_address
        self.drone = System()


    async def run(self):
        await self.drone.connect(system_address=self.drone_address)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break
            

        while True:
            timestamp_us = int(time.time() * 1e6) # Seconds to microseconds
            coordinates = [0, 0, 0]
            angle_body = [0, 0, 0]
            cov_matrix = [float('NaN')] * 21

            print(f"Sending vision position estimate: timestamp={timestamp_us}, x={coordinates[0]:.2f}, y={coordinates[1]:.2f}, z={coordinates[2]:.2f}, roll={angle_body[0]:.1f}°, pitch={angle_body[1]:.1f}°, yaw={angle_body[2]:.1f}°")

            await self.drone.mocap.set_vision_position_estimate(VisionPositionEstimate(
                timestamp_us,
                PositionBody(coordinates[0], coordinates[1], -coordinates[2]),
                AngleBody(angle_body[0], angle_body[1], angle_body[2]),
                Covariance(cov_matrix)
                ))
            

if __name__ == "__main__":
    drone_address = "udpin://0.0.0.0:14540"
    droneMavlink = DroneMavlink(drone_address)
    asyncio.run(droneMavlink.run())