import asyncio
import cv2
import numpy as np
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading

shutdown_event = threading.Event()
# Global frame for camera
bridge = CvBridge()
latest_frame = None


def image_callback(msg):
    global latest_frame
    latest_frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


async def run_interactive_mission():
    drone = System()
    print("Connecting to drone...")
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Connected!")
            break

    print("Waiting for global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Set initial hover setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.5, 0.0))

    try:
        print("-- Starting offboard mode")
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Offboard start failed: {error._result.result}")
        await drone.action.disarm()
        return

    await asyncio.sleep(3)

    current_position = [0.0, 0.0, -1.5, 0.0]  # Start position

    while True:
        # Keep hovering at current position
        await drone.offboard.set_position_ned(PositionNedYaw(*current_position))

        user_input = input("\nEnter next waypoint as: N E D YAW (or 'done' to land): ").strip()
        if user_input.lower() == 'done':
            print("-- Landing")
            await drone.action.land()
            Shutdown_event.set()
            break

        try:
            n, e, d, yaw = map(float, user_input.split())
            print(f"-- Going to waypoint: N={n} E={e} D={d} Yaw={yaw}")
            await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, yaw))
            current_position = [n, e, d, yaw]
            await asyncio.sleep(10)  # Hover and wait for next waypoint
        except ValueError:
            print("Invalid input. Enter 4 numbers separated by space.")


def ros_image_visualizer():
    global latest_frame
    rclpy.init()
    node = rclpy.create_node('camera_viewer')
    sub = node.create_subscription(Image, '/camera/image_raw', image_callback, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest_frame is not None:
                cv2.imshow('Drone Camera', latest_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    shutdown_event.set()
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    import threading

    # Start camera visualization
    ros_thread = threading.Thread(target=ros_image_visualizer)
    ros_thread.start()

    # Start drone control loop
    asyncio.run(run_interactive_mission())

