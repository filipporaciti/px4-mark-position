from typing import NamedTuple
import asyncio
import time
import cv2

from mavsdk import System
from mavsdk.mocap import VisionPositionEstimate, Covariance, AngleBody, PositionBody
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
from mavsdk.telemetry import LandedState

class Target(NamedTuple):
    north_m: float
    east_m: float
    down_m: float
    yaw_deg: float = 0.0
    hover_time_ms: int = 0
    expiration_time_s: float = None
    vel_m_s: float = None

class DroneMavlink:

    def __init__(self, drone_address: str):
        self.drone_address = drone_address
        
        if self.drone_address is None:
            self.drone = System(mavsdk_server_address="127.0.0.1", port=50051)
        else:
            self.drone = System()

        self.__old_coordinates = [0.0, 0.0, 0.0]
        self.__old_angles = [0.0, 0.0, 0.0]

        self.__old_target = Target(0.0, 0.0, -1.4, 0.0, 1000)

        self.OFFBOARD_XY_TOLERANCE = 0.05
        self.OFFBOARD_Z_TOLERANCE = 0.02
        self.OFFBOARD_XY_VEL_TOLERANCE = 0.05
        self.OFFBOARD_Z_VEL_TOLERANCE = 0.02
        self.OFFBOARD_YAW_DEGREE_TOLERANCE = 5

    async def start_mission(self, mission: dict):
        await self.connect()
        await self.health_check()
        await self.arm()

        success = await self.start_offboard()
        if not success:
            await self.land()
            await self.disarm()

        await self.move_to(self.__old_target, self.__old_target)

        for t in mission["targets"]:
            target = Target(**t)
            await self.move_to(target, self.__old_target)

            self.__old_target = target

        await self.land()

    async def move_to(self, target: Target, old_target: Target):
        print(f"Moving to: x={target.north_m} y={target.east_m} z={target.down_m} yaw={target.yaw_deg}")
        await self.drone.offboard.set_position_ned(PositionNedYaw(target.north_m, target.east_m, target.down_m, target.yaw_deg))

        if target.expiration_time_s is not None or target.vel_m_s is not None:
            expiration_time_s = target.expiration_time_s
            distance_m = ((target.north_m - old_target.north_m) ** 2 + (target.east_m - old_target.east_m) ** 2 + (target.down_m - old_target.down_m) ** 2) ** 0.5
            if distance_m == 0:
                return

            if target.expiration_time_s is not None:
                north_m_s = (target.north_m - old_target.north_m) / target.expiration_time_s
                east_m_s = (target.east_m - old_target.east_m) / target.expiration_time_s
                down_m_s = (target.down_m - old_target.down_m) / target.expiration_time_s
            else:
                north_m_s = (target.north_m - old_target.north_m) / distance_m * target.vel_m_s
                east_m_s = (target.east_m - old_target.east_m) / distance_m * target.vel_m_s
                down_m_s = (target.down_m - old_target.down_m) / distance_m * target.vel_m_s
                expiration_time_s = distance_m / target.vel_m_s
            yaw_deg_s = (target.yaw_deg - old_target.yaw_deg) / expiration_time_s

            print(f"Distance: {distance_m} m, Expiration Time: {expiration_time_s} s, Speed: north_m_s={north_m_s} east_m_s={east_m_s} down_m_s={down_m_s} yaw_deg_s={yaw_deg_s}")

            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(north_m_s, east_m_s, down_m_s, yaw_deg_s))
            await asyncio.sleep(expiration_time_s)
        else:
            async for pos in self.drone.telemetry.position_velocity_ned():
                print(f"Pos: {pos.position.north_m: .4f}, {pos.position.east_m: .4f}, {pos.position.down_m: .4f} | Vel: {pos.velocity.north_m_s: .4f}, {pos.velocity.east_m_s: .4f}, {pos.velocity.down_m_s: .4f}")
                if abs(pos.position.north_m - target.north_m) < self.OFFBOARD_XY_TOLERANCE and abs(pos.position.east_m - target.east_m) < self.OFFBOARD_XY_TOLERANCE and abs(pos.position.down_m - target.down_m) < self.OFFBOARD_Z_TOLERANCE and abs(pos.velocity.north_m_s) < self.OFFBOARD_XY_VEL_TOLERANCE and abs(pos.velocity.east_m_s) < self.OFFBOARD_XY_VEL_TOLERANCE and abs(pos.velocity.down_m_s) < self.OFFBOARD_Z_VEL_TOLERANCE:
                    break

            async for angle in self.drone.telemetry.attitude_euler():
                print(f"Angle: {angle.yaw_deg}")
                if abs(((angle.yaw_deg + 360) % 360) - ((target.yaw_deg + 360) % 360)) < self.OFFBOARD_YAW_DEGREE_TOLERANCE or abs(((angle.yaw_deg + 360) % 360) - ((target.yaw_deg + 360) % 360)) > (360 - self.OFFBOARD_YAW_DEGREE_TOLERANCE):
                    break

        await asyncio.sleep(target.hover_time_ms / 1000)


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
    
    async def connect(self):
        if self.drone_address is None:
            await self.drone.connect(system_address="grpc://127.0.0.1:50051")
        else:
            await self.drone.connect(system_address=self.drone_address)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break

    async def update_position(self, timestamp_us, coordinates, angles, cov_matrix):
        if coordinates is None or angles is None:
            coordinates = self.__old_coordinates
            angles = self.__old_angles
            cov_matrix = [float('nan')] * 21
        self.__old_coordinates = coordinates
        self.__old_angles = angles

        await self.drone.mocap.set_vision_position_estimate(VisionPositionEstimate(
            timestamp_us,
            PositionBody(coordinates[0], coordinates[1], coordinates[2]),
            AngleBody(angles[0], angles[1], angles[2]),
            Covariance(cov_matrix),
            0
            ))


if __name__ == "__main__":
    drone_address = "udpin://0.0.0.0:14540"
    droneMavlink = DroneMavlink(drone_address)

    async def run_async():
        await droneMavlink.connect()
        while True:
            timestamp_us = int(time.time() * 1e6) # Seconds to microseconds

            coordinates = [0.0, 0.0, 0.0]
            angles = [0.0, 0.0, 0.0]
            cov_matrix = [0.0] * 21

            await droneMavlink.update_position(timestamp_us, coordinates, angles, cov_matrix)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    asyncio.run(run_async())