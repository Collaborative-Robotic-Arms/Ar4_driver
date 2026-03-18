#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import serial
import time

class SimpleServoSmartSweep(Node):
    def __init__(self):
        super().__init__('simple_servo_smart_sweep')

        # --- Configuration ---
        self.serial_port = '/dev/ttyUSB0' # CHECK THIS
        self.baud_rate = 115200
        self.servo_index = 0
        
        # --- State Tracking ---
        self.current_angle = 0          # To remember where the servo is
        self.target_angle_true = 25     # Target when True (Open)
        self.target_angle_false = 3     # Target when False (Close)
        self.step_delay = 0.05          # Seconds per degree (0.05s * 30deg = 1.5s move)

        # --- Setup Serial ---
        self.ser = None
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            time.sleep(2) # Wait for Arduino reset
            self.get_logger().info(f'Connected to Arduino on {self.serial_port}')
            
            # Initialize servo to 0 gently on startup
            self.send_serial_command(0)
            
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial: {e}')

        # --- Service Server ---
        # Matches the client in ar4_task_server.cpp: "ar4_gripper/set"
        self.srv = self.create_service(
            SetBool,
            'ar4_gripper/set',
            self.gripper_service_callback
        )
        self.get_logger().info('AR4 Gripper Service Ready.')

    def send_serial_command(self, angle):
        """Helper to send the raw command string"""
        command_str = f"SV{self.servo_index}P{angle}\n"
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(command_str.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Serial Error: {e}')

    def perform_sweep(self, target):
        """Sweeps from current_angle to target_angle slowly"""
        if self.current_angle == target:
            self.get_logger().info('Servo already at target angle.')
            return True

        self.get_logger().info(f'Sweeping from {self.current_angle} to {target}...')

        # Determine direction (+1 for Up, -1 for Down)
        step = 1 if target > self.current_angle else -1
        
        # Range arguments: start, stop (exclusive), step
        # We add 'step' to the stop value so the loop includes the target
        for angle in range(self.current_angle, target + step, step):
            self.send_serial_command(angle)
            time.sleep(self.step_delay) # Pause to prevent brownout
            
        # Update our memory of where the servo is
        self.current_angle = target
        self.get_logger().info(f'Reached {target} degrees.')
        return True

    def gripper_service_callback(self, request, response):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("Cannot execute command: Serial connection is down.")
            response.success = False
            response.message = "Serial connection down."
            return response

        if request.data:
            # === TRUE received: OPEN gripper (Sweep UP to 30) ===
            self.get_logger().info('Service called: OPEN Gripper')
            self.perform_sweep(self.target_angle_true)
            response.message = "Gripper opened successfully."
        else:
            # === FALSE received: CLOSE gripper (Sweep DOWN to 0) ===
            self.get_logger().info('Service called: CLOSE Gripper')
            self.perform_sweep(self.target_angle_false)
            response.message = "Gripper closed successfully."

        response.success = True
        return response

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleServoSmartSweep()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()