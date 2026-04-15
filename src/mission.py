import asyncio
import json

from DroneMavlink import DroneMavlink

drone_address = "udpin://0.0.0.0:14540"
droneMavlink = DroneMavlink(drone_address)

mission = json.load(open("src/missions/mission1.json", "r"))

async def run():
    await droneMavlink.start_mission(mission)


if __name__ == "__main__":
    asyncio.run(run())
