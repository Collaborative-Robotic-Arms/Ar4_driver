#!/usr/bin/env python3
from functools import partial

import rclpy
import select
import sys
import termios
import threading
import tty
import time
from geometry_msgs.msg import Point, Pose, Quaternion, TwistStamped
from moveit_msgs.srv import ServoCommandType
from pymoveit2 import MoveIt2
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class JoystickServo(Node):

    def __init__(self):
        super().__init__("joystick_servo")

        # Constants
        self.input_to_velocity_linear_scaling = 0.04
        self.input_to_velocity_angular_scaling = 0.3
        self.reset_pose = Pose(
            position=Point(x=0.0, y=-0.35, z=0.30),
            orientation=Quaternion(x=0.0, y=1.0, z=0.0, w=0.0),
        )

        # State variables
        # _last_joy_msg is kept for compatibility but keyboard input will drive commands
        self._last_joy_msg: Joy | None = None
        self._command_enabled: bool = False
        self._resetting = False

        # Keyboard input handler (replaces joystick)
        try:
            self._keyboard = KeyboardHandler(self.get_logger())
            self.get_logger().info("Keyboard control enabled")
        except Exception as e:
            self.get_logger().warn(f"Keyboard control disabled: {e}")
            self._keyboard = None
        self.servo_pub = self.create_publisher(TwistStamped,
                                               "/servo_node/delta_twist_cmds",
                                               10)

        self.switch_command_type_client = self.create_client(
            ServoCommandType,
            "/servo_node/switch_command_type",
        )
        self.pause_client = self.create_client(
            SetBool,
            "/servo_node/pause_servo",
        )

        while not self.switch_command_type_client.wait_for_service(
                timeout_sec=1.0):
            self.get_logger().info(
                "Switch command type service not available, waiting again...")

        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Pause servo service not available, waiting again...")

        self.arm_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=self.arm_joint_names,
            base_link_name="base_link",
            end_effector_name="link_6",
            group_name="ar_manipulator",
            callback_group=ReentrantCallbackGroup(),
        )
        self.moveit2.planner_id = "RRTConnectkConfigDefault"

        self.timer = self.create_timer(
            0.05,
            self.timer_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        # Switch to twist command type
        future = self.switch_command_type_client.call_async(
            ServoCommandType.Request(
                command_type=ServoCommandType.Request.TWIST))
        future.add_done_callback(
            partial(
                self.service_callback,
                service_name=self.switch_command_type_client.srv_name,
            ))

        self._command_enabled = True
        self.get_logger().info("Joystick Servo Node has been started.")

    def joy_callback(self, msg):
        self._last_joy_msg = msg
 

    def reset_arm(self):
        self._resetting = True
        self._logger.info("Disabling servo control")
        response = self.pause_client.call(SetBool.Request(data=True))
        if not response.success:
            self.get_logger().error("Disable servo failed")
            self._resetting = False
            return

        self._logger.info("Resetting the arm")
        self.moveit2.move_to_pose(self.reset_pose)
        self.moveit2.wait_until_executed()
        
        self._logger.info("Enabling servo control")
        response = self.pause_client.call(SetBool.Request(data=False))
        if not response.success:
            self.get_logger().error("Enable servo failed")
            self._resetting = False
            return

        self._resetting = False

    def timer_callback(self):
        # If keyboard not available, do nothing
        if self._keyboard is None:
            return

        # Process one-shot events (toggles, reset, gripper commands)
        events = self._keyboard.get_events()
        for ch, _ in events:
            # Reset arm (z)
            if ch == "z":
                if not self._resetting:
                    self.reset_arm()
            # Toggle enable/disable (space)
            elif ch == " ":
                if self._command_enabled:
                    self.get_logger().info("Disabling servo control")
                    future = self.pause_client.call_async(
                        SetBool.Request(data=True))
                    future.add_done_callback(
                        partial(self.service_callback,
                                service_name=self.pause_client.srv_name))
                    self._command_enabled = False
                else:
                    self.get_logger().info("Enabling servo control")
                    future = self.pause_client.call_async(
                        SetBool.Request(data=False))
                    future.add_done_callback(
                        partial(self.service_callback,
                                service_name=self.pause_client.srv_name))
                    self._command_enabled = True
        if not self._command_enabled:
            return

        # Build twist based on held keys. We consider a key 'held' if it was
        # pressed within the last hold_time seconds. This works with OS auto-repeat
        # or repeated key taps.
        s = self.input_to_velocity_linear_scaling
        ang_s = self.input_to_velocity_angular_scaling

        lx = 0.0
        ly = 0.0
        lz = 0.0
        ax = 0.0
        ay = 0.0
        az = 0.0

        # Linear controls
        if self._keyboard.is_held("r"):
            lx += s
        if self._keyboard.is_held("f"):
            lx -= s
        if self._keyboard.is_held("a"):
            ly += s
        if self._keyboard.is_held("d"):
            ly -= s
        # Up / down
        if self._keyboard.is_held("w"):
            lz += s
        if self._keyboard.is_held("s"):
            lz -= s

        # Angular controls
        if self._keyboard.is_held("u"):
            ax += ang_s
        if self._keyboard.is_held("j"):
            ax -= ang_s
        if self._keyboard.is_held("h"):
            ay += ang_s
        if self._keyboard.is_held("k"):
            ay -= ang_s
        # yaw
        if self._keyboard.is_held("q"):
            az += ang_s
        if self._keyboard.is_held("e"):
            az -= ang_s

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"

        twist.twist.linear.x = lx
        twist.twist.linear.y = ly
        twist.twist.linear.z = lz

        twist.twist.angular.x = ax
        twist.twist.angular.y = ay
        twist.twist.angular.z = az

        self.servo_pub.publish(twist)

    def service_callback(self, future, service_name: str = ""):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"{service_name} service call succeeded")
            else:
                self.get_logger().error(f"{service_name} service call failed")
        except Exception as e:
            self.get_logger().error(f"{service_name} service call failed: {e}")


class KeyboardHandler:
    """Simple non-blocking keyboard reader.

    - Spawns a thread and reads single characters from stdin in raw mode.
    - Keeps timestamps of last-pressed keys to implement key-hold behavior.
    - Queues raw key events for one-shot actions (toggles, reset, gripper cmds).
    """

    def __init__(self, logger=None, hold_time=0.3):
        self.logger = logger
        self.hold_time = hold_time
        self._old_settings = None
        self._running = True
        self._lock = threading.Lock()
        self._last_pressed = {}  # key -> last pressed time
        self._events = []

        # Make stdin non-blocking raw
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY")

        self._old_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def _read_loop(self):
        while self._running:
            try:
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if r:
                    ch = sys.stdin.read(1)
                    if ch:
                        with self._lock:
                            self._last_pressed[ch] = time.time()
                            self._events.append((ch, time.time()))
            except Exception:
                # swallow IO errors while shutting down
                pass

    def get_events(self):
        with self._lock:
            ev = list(self._events)
            self._events.clear()
            return ev

    def is_held(self, key):
        """Return True if key was pressed within hold_time seconds."""
        with self._lock:
            t = self._last_pressed.get(key, 0)
        return (time.time() - t) < self.hold_time

    def shutdown(self):
        self._running = False
        try:
            if self._old_settings is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN,
                                  self._old_settings)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = JoystickServo()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()