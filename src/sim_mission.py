import asyncio
import json
import sys

from DroneMavlink import DroneMavlink

drone_address = "udpin://0.0.0.0:14540"
droneMavlink = DroneMavlink(drone_address)

if len(sys.argv) != 2:
    print("Usage: python mission.py <mission_file.json>")
    sys.exit(1)

mission_file = sys.argv[1]
mission = json.load(open(mission_file, "r"))

async def run():
    await droneMavlink.start_mission(mission)


if __name__ == "__main__":
    asyncio.run(run())
