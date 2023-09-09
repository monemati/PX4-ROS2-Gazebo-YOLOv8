import asyncio
import random
from mavsdk import System
import KeyPressModule as kp

# Test set of manual inputs. Format: [roll, pitch, throttle, yaw]

kp.init()
drone = System()

roll, pitch, throttle, yaw = 0, 0, 0.5, 0
async def getKeyboardInput(my_drone):
    global roll, pitch, throttle, yaw
    while True:
        roll, pitch, throttle, yaw = 0, 0, 0.5, 0
        value = 0.5
        if kp.getKey("LEFT"):
            pitch = -value
        elif kp.getKey("RIGHT"):
            pitch = value
        if kp.getKey("UP"):
            roll = value
        elif kp.getKey("DOWN"):
            roll = -value
        if kp.getKey("w"):
            throttle = 1
        elif kp.getKey("s"):
            throttle = 0
        if kp.getKey("a"):
            yaw = -value
        elif kp.getKey("d"):
            yaw = value
        elif kp.getKey("i"):
            asyncio.ensure_future(print_flight_mode(my_drone))
        elif kp.getKey("r") and my_drone.telemetry.landed_state():
            await my_drone.action.arm()
        elif kp.getKey("l") and my_drone.telemetry.in_air():
            await my_drone.action.land()
        # print(roll, pitch, throttle, yaw)
        await asyncio.sleep(0.1)

async def print_flight_mode(my_drone):
    async for flight_mode in my_drone.telemetry.flight_mode():
        print("FlightMode:", flight_mode)
        # return flight_mode

async def manual_control_drone(my_drone):
    global roll, pitch, throttle, yaw
    while True:
        print(roll, pitch, throttle, yaw)
        await my_drone.manual_control.set_manual_control_input(roll, pitch, throttle, yaw)
        await asyncio.sleep(0.1)

async def run_drone():
    asyncio.ensure_future(getKeyboardInput(drone))
    await drone.connect(system_address="udp://:14540")
    # This waits till a mavlink based drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
    # Checking if Global Position Estimate is ok
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break
    asyncio.ensure_future(manual_control_drone(drone))


async def run():
    global roll, pitch, throttle, yaw
    """Main function to connect to the drone and input manual controls"""
    await asyncio.gather(run_drone())

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
