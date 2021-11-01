import rclpy
from rclpy.node import Node
from gpiozero import Servo
from gpiozero.tools import sin_values
from gpiozero.pins.pigpio import PiGPIOFactory as fac

# from time import sleep
from rmf_door_msgs.msg import DoorMode


class Swipe(Node):
    def __init__(self):
        super().__init__("swipe")
        self.fac = fac()
        self.servo = Servo(
            17,
            pin_factory=self.fac,
            min_pulse_width=1.0 / 1000,
            max_pulse_width=2.0 / 1000,
        )
        # print(self.servo.source)
        self.subscription = self.create_subscription(
            DoorMode, "smart_card_trigger", self.trigger_callback, 1
        )
        self.subscription  # prevent unused variable warning

    def trigger_callback(self, msg):
        self.servo.source = None
        if msg.value == 2:
            self.servo.source = sin_values()
            self.servo.source_delay = 0.01  # controls swiping speed
            self.get_logger().info("Swiping back-and forth")
        elif msg.value == 1:
            self.servo.min()  # returns servo to minimum rotation
            self.get_logger().info("Returning to default pos")


def main(args=None):
    rclpy.init(args=args)
    swipe = Swipe()
    rclpy.spin(swipe)
    swipe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
