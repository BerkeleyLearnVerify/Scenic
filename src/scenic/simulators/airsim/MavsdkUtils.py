import mavsdk
from mavsdk import System
from mavsdk.offboard import VelocityBodyYawspeed


async def connect():
    # CODE SRC: https://github.com/mavlink/MAVSDK-Python/blob/main/examples/offboard_velocity_body.py
    drone = System()
    await drone.connect(system_address="udp://:14550")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()
    await drone.offboard.start()

    return drone


async def takeoff(drone):
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    print("done")


async def move(drone, location):
    print("moving")
    await drone.action.goto_location(location[0], location[1], location[2], 0)
    print("done moving")


async def moveByVelocity(drone, velocity):
    print("Moving by velocity")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(velocity[0], velocity[1], velocity[2], 3)
    )
    print("Done moving by velocity")
