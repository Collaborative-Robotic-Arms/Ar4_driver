import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
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
        self.last_trigger_state = None  # To detect True/False changes
        self.current_angle = 0          # To remember where the servo is
        self.target_angle_true = 30     # Target when True
        self.target_angle_false = 0     # Target when False
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

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            Bool,
            '/servo_trigger',
            self.listener_callback,
            10
        )

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
            return

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

    def listener_callback(self, msg):
        if self.ser is None:
            return

        # Edge Detection: Only act if the boolean value CHANGES
        if msg.data == self.last_trigger_state:
            return
        
        self.last_trigger_state = msg.data

        if msg.data:
            # === TRUE received: Sweep UP to 30 ===
            self.perform_sweep(self.target_angle_true)
        else:
            # === FALSE received: Sweep DOWN to 0 ===
            self.perform_sweep(self.target_angle_false)

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