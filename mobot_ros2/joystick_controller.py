import rclpy
from rclpy.node import Node
import lgpio
from sensor_msgs.msg import Joy
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # GPIO Setup
        self.chip = lgpio.gpiochip_open(4)  # Open GPIO chip 0

        # Motor 1 GPIO Pins (BCM GPIO numbers)
        self.ENA1, self.IN1, self.IN2 = 13, 6, 5  # Motor 1
        lgpio.gpio_claim_output(self.chip, self.ENA1)
        lgpio.gpio_claim_output(self.chip, self.IN1)
        lgpio.gpio_claim_output(self.chip, self.IN2)

        # Motor 2 GPIO Pins (BCM GPIO numbers)
        self.ENA2, self.IN3, self.IN4 = 12, 24, 23  # Motor 2
        lgpio.gpio_claim_output(self.chip, self.ENA2)
        lgpio.gpio_claim_output(self.chip, self.IN3)
        lgpio.gpio_claim_output(self.chip, self.IN4)

        # Subscriber to joystick messages
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.get_logger().info('MotorController node initialized.')

    def joy_callback(self, msg):
        """
        Callback for the Joy topic. Maps joystick axes to motor speed and direction.
        """
        # Map joystick axes to motor speeds
        left_motor_speed = int((msg.axes[1] + msg.axes[0]) * 250)  # Adjust scaling as necessary
        right_motor_speed = int((msg.axes[1] - msg.axes[0]) * 250)

        self.get_logger().info(f'Left motor speed: {left_motor_speed}, Right motor speed: {right_motor_speed}')

        # Control both motors
        self.control_motor(self.ENA1, self.IN1, self.IN2, left_motor_speed, flip_direction=False)
        self.control_motor(self.ENA2, self.IN3, self.IN4, right_motor_speed, flip_direction=True)

    def control_motor(self, ena, in1, in2, speed, flip_direction=False):
        """
        Controls a motor's direction and speed using GPIO pins.
        If flip_direction is True, the direction logic is reversed.
        """
        if flip_direction:
            # Reverse the direction logic
            if speed > 0:
                lgpio.gpio_write(self.chip, in1, 0)
                lgpio.gpio_write(self.chip, in2, 1)
            elif speed < 0:
                lgpio.gpio_write(self.chip, in1, 1)
                lgpio.gpio_write(self.chip, in2, 0)
            else:
                lgpio.gpio_write(self.chip, in1, 0)
                lgpio.gpio_write(self.chip, in2, 0)
        else:
            # Standard direction logic
            if speed > 0:
                lgpio.gpio_write(self.chip, in1, 1)
                lgpio.gpio_write(self.chip, in2, 0)
            elif speed < 0:
                lgpio.gpio_write(self.chip, in1, 0)
                lgpio.gpio_write(self.chip, in2, 1)
            else:
                lgpio.gpio_write(self.chip, in1, 0)
                lgpio.gpio_write(self.chip, in2, 0)

        # Clamp duty_cycle between 0 and 100
        duty_cycle = min(max(abs(speed), 0), 100)
        self.simulate_pwm(ena, duty_cycle)

    def simulate_pwm(self, ena, duty_cycle):
        """
        Simulates PWM by toggling the GPIO pin at a specific duty cycle.
        """
        pwm_period = 0.01  # 10ms period
        high_time = (duty_cycle / 100) * pwm_period
        low_time = pwm_period - high_time

        # Toggle the GPIO pin for a short duration to simulate PWM
        lgpio.gpio_write(self.chip, ena, 1)
        time.sleep(high_time)
        lgpio.gpio_write(self.chip, ena, 0)
        time.sleep(low_time)

    def destroy_node(self):
        """
        Cleans up GPIO resources when the node is destroyed.
        """
        lgpio.gpiochip_close(self.chip)
        self.get_logger().info('GPIO resources released.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

