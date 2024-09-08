from math import floor
from threading import Lock, Thread
from time import sleep
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

BATTERY_HIGH = 0.95
BATTERY_LOW = 0.2
BATTERY_CRITICAL = 0.1

class BatteryMonitor(Node):
    def __init__(self, lock):
        super().__init__('battery_monitor')
        self.lock = lock
        self.battery_percent = None
        self.battery_state_subscriber = self.create_subscription(
            BatteryState, 
            'battery_state', 
            self.battery_state_callback,
            qos_profile_sensor_data
        )

    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage

    def thread_function(self):
        executor = SingleThreadedExecutor()
        executor.add_node(self)
        executor.spin()

def main(args=None):
    rclpy.init(args=args)
    lock = Lock()
    battery_monitor = BatteryMonitor(lock)

    thread = Thread(target=battery_monitor.thread_function, daemon=True)
    thread.start()

    navigator = TurtleBot4Navigator()
    goal_poses = navigator.createPath()

    if not navigator.getDockedStatus():
        navigator.info('Docking before initializing pose')
        navigator.dock()

    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    navigator.undock()
    result = navigator.getResult()

    while True:
        with lock:
            battery_percent = battery_monitor.battery_percent

        if battery_percent is not None:
            navigator.info(f'Battery is at {(battery_percent * 100):.2f}% charge')

            if battery_percent < BATTERY_CRITICAL:
                navigator.error('Battery critically low. Charge or power down')
                break
            elif battery_percent < BATTERY_LOW:
                navigator.info('Docking for charge')
                navigator.startToPose(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))
                navigator.dock()

                if not navigator.getDockedStatus():
                    navigator.error('Robot failed to dock')
                    break

                navigator.info('Charging...')
                prev_battery_percent = 0
                while battery_percent < BATTERY_HIGH:
                    sleep(15)
                    prev_battery_percent = floor(battery_percent * 100) / 100
                    with lock:
                        battery_percent = battery_monitor.battery_percent

                    if battery_percent > (prev_battery_percent + 0.01):
                        navigator.info(f'Battery is at {(battery_percent * 100):.2f}% charge')

                navigator.undock()
            else:
                result = navigator.startFollowWaypoints(goal_poses)
                if result == TaskResult.SUCCEEDED:
                    navigator.startFollowWaypoints(list(reversed(goal_poses)))
                break

    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()