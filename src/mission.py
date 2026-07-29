import asyncio
import json
import sys

from DroneMavlink import DroneMavlink


async def run_mission(drone_mavlink: DroneMavlink, mission: dict):
    await drone_mavlink.start_mission(mission)
    

if __name__ == "__main__":

    DRONE_ADDRESS = "udpin://0.0.0.0:14540"
    drone_mavlink = DroneMavlink(DRONE_ADDRESS)

    if len(sys.argv) != 2:
        print("Usage: python3 mission.py <mission_file.json>")
        sys.exit(1)

    mission_file = sys.argv[1]
    mission = json.load(open(mission_file, "r"))

    asyncio.run(run_mission(drone_mavlink, mission))
