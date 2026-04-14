import asyncio
import math
import time
import cv2

from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, Covariance, AngleBody, PositionBody
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.telemetry import LandedState

class DroneMavlink:

    def __init__(self, drone_address: str):
        self.drone_address = drone_address
        self.drone = System()

        self.__old_coordinates = [0.0, 0.0, 0.0]
        self.__old_yaw = 0.0

        self.OFFBOARD_XY_TOLERANCE = 0.1
        self.OFFBOARD_Z_TOLERANCE = 0.05
        self.OFFBOARD_XY_VEL_TOLERANCE = 0.05
        self.OFFBOARD_Z_VEL_TOLERANCE = 0.01
        self.OFFBOARD_YAW_TOLERANCE = 0.1

    async def start_mission(self, mission: dict):
        await self.connect()
        success = await self.start_offboard()
        if not success:
            return

        await self.health_check()
        await self.arm()

        for target in mission["targets"]:
            await self.move_to(target["north_m"], target["east_m"], target["down_m"], target["yaw_deg"])

        await self.land()
        await self.disarm()

    async def move_to(self, x: float, y: float, z: float, yaw: float):
        print(f"Moving to: x={x} y={y} z={z} yaw={yaw}")
        await self.drone.offboard.set_position_ned(PositionNedYaw(x, y, z, yaw))

        async for pos in self.drone.telemetry.position_velocity_ned():
            print(f"Pos: {pos.position.north_m: .4f}, {pos.position.east_m: .4f}, {pos.position.down_m: .4f} | Vel: {pos.velocity.north_m_s: .4f}, {pos.velocity.east_m_s: .4f}, {pos.velocity.down_m_s: .4f}")
            if abs(pos.position.north_m - x) < self.OFFBOARD_XY_TOLERANCE and abs(pos.position.east_m - y) < self.OFFBOARD_XY_TOLERANCE and abs(pos.position.down_m - z) < self.OFFBOARD_Z_TOLERANCE and abs(pos.velocity.north_m_s) < self.OFFBOARD_XY_VEL_TOLERANCE and abs(pos.velocity.east_m_s) < self.OFFBOARD_XY_VEL_TOLERANCE and abs(pos.velocity.down_m_s) < self.OFFBOARD_Z_VEL_TOLERANCE:
                break

        async for angle in self.drone.telemetry.attitude_euler():
            print(f"Angle: {angle.yaw_deg}")
            if abs(angle.yaw_deg - yaw) < self.OFFBOARD_YAW_TOLERANCE:
                break
        
        await asyncio.sleep(1)


    async def arm(self):
        print("-- Arming")
        await self.drone.action.arm()

    async def disarm(self):
        print("-- Disarming")
        await self.drone.action.disarm()

    async def land(self):
        print("-- Landing")
        await self.drone.action.land()

        async for state in self.drone.telemetry.landed_state():
            print(f"Landed State: {state}")
            if state == LandedState.ON_GROUND:
                break

    async def start_offboard(self):
        print("-- Setting initial setpoint")
        await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

        print("-- Starting offboard")
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: {error._result.result}")
            print("-- Disarming")
            await self.drone.action.disarm()
            return False
        return True

    async def health_check(self):

        async for health in self.drone.telemetry.health():
            print()
            print("Chech: | ", end="")

            if health.is_gyrometer_calibration_ok:
                print("gyrometer | ", end="")
            else:
                continue
            
            if health.is_accelerometer_calibration_ok:
                print("accelerometer | ", end="")
            else:
                continue

            if health.is_magnetometer_calibration_ok:
                print("magnetometer | ", end="")
            else:
                continue

            if health.is_local_position_ok:
                print("local position | ", end="")
            else:
                continue

            if health.is_armable:
                print("armable | ", end="")
            else:
                continue
            
            print()
            break

    
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