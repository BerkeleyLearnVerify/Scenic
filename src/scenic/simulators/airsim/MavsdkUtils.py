from mavsdk import System
import asyncio

async def connect():
    drone = System()
    await drone.connect(system_address="udp://:14550")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    async for health in drone.telemetry.health():
        print(health)
        break

    return drone


async def takeoff(drone):
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    print("done")


async def move(drone,location):
    print("moving")
    await drone.action.goto_location(location[0], location[1], location[2], 0)
    print("done moving")


    await asyncio.sleep(10)