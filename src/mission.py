import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

drone = System()

async def health_check():

    async for health in drone.telemetry.health():
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

async def run():
    
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone!")
            break


    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(
            f"Starting offboard mode failed \
                with error code: {error._result.result}"
        )
        print("-- Disarming")
        await drone.action.disarm()
        return
    
    await health_check()
        
    print("-- Arming")
    await drone.action.arm()

    print("TAKEOFF")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 90.0))
    await asyncio.sleep(10)

    print("turn 0")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(5)

    print("x=4 y=0 z=-2")
    await drone.offboard.set_position_ned(PositionNedYaw(4.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(20)

    print("x=0 y=3 z=-2")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 3.0, -2.0, 0.0))
    await asyncio.sleep(20)

    print("x=6 y=6 z=-2")
    await drone.offboard.set_position_ned(PositionNedYaw(6.0, 6.0, -2.0, 0.0))
    await asyncio.sleep(20)

    print("x=0 y=0 z=-2")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(20)

    print("LANDING")
    await drone.action.land()
    await asyncio.sleep(10)

    print("-- Disarming")
    await drone.action.disarm()



if __name__ == "__main__":
    asyncio.run(run())
