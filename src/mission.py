import asyncio
from mavsdk.offboard import PositionNedYaw

from DroneMavlink import DroneMavlink

drone_address = "udpin://0.0.0.0:14540"
droneMavlink = DroneMavlink(drone_address)

async def run():
    
    await droneMavlink.connect()

    success = await droneMavlink.start_offboard()
    if not success:
        return

    await droneMavlink.health_check()
        
    await droneMavlink.arm()

    await droneMavlink.move_to(0.0, 0.0, -2.0, 0.0)

    await droneMavlink.move_to(4.0, 0.0, -2.0, 0.0)

    await droneMavlink.move_to(4.0, 0.0, -2.0, 0.0)

    await droneMavlink.move_to(0.0, 3.0, -2.0, 0.0)

    await droneMavlink.move_to(6.0, 6.0, -2.0, 0.0)

    await droneMavlink.move_to(0.0, 0.0, -2.0, 0.0)

    await droneMavlink.land()

    await droneMavlink.disarm()


if __name__ == "__main__":
    asyncio.run(run())
