import asyncio
import json

from DroneMavlink import DroneMavlink

drone_address = "udpin://0.0.0.0:14540"
droneMavlink = DroneMavlink(drone_address)

mission = json.load(open("missions/mission1.json", "r"))

async def run():
    
    await droneMavlink.connect()

    success = await droneMavlink.start_offboard()
    if not success:
        return

    await droneMavlink.health_check()
        
    await droneMavlink.arm()

    for target in mission["targets"]:
        await droneMavlink.move_to(target["north_m"], target["east_m"], target["down_m"], target["yaw_deg"])

    await droneMavlink.land()

    await droneMavlink.disarm()


if __name__ == "__main__":
    asyncio.run(run())
