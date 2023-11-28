from mavsdk import System
import asyncio

# import airsim


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
