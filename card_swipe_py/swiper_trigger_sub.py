import math
from time import sleep
from threading import Thread, Lock
import rclpy
from rclpy.node import Node
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory as fac

# from time import sleep
from rmf_door_msgs.msg import DoorMode
from rmf_door_msgs.msg import DoorState
from rmf_door_msgs.msg import DoorRequest

# -------------------------------------------------
# ros2 topic info /door_states
# Type: rmf_door_msgs/msg/DoorState

# door_time:
#   sec: 378
#   nanosec: 9999999
# door_name: icu1_sliding_door
# current_mode:
#   value: 0 - closed / 1 - moving / 2 - opened

# -------------------------------------------------
# ros2 topic info /door_requests 
# Type: rmf_door_msgs/msg/DoorRequest

# request_time:
#   sec: 1641833352
#   nanosec: 139427038
# requester_id: door_supervisor
# door_name: icu1_sliding_door
# requested_mode:
#   value: 0 - close / 2 - open

class Swipe(Node):
    def __init__(self):
        super().__init__("icu1_sliding_door_controller")
        self.fac = fac()
        self.servo = Servo(
            17,
            pin_factory=self.fac,
            min_pulse_width=1.0 / 1000,
            max_pulse_width=2.0 / 1000,
        )

        self.servo.min()
        
        self.get_logger().info('Initializing ...')

        # Initialize mutex for safe management of door msgs
        self.door_state_mutex = Lock()
        self.door_request_mutex = Lock()

        # Initialize the door state
        self.door_state_msg = DoorState()
        self.door_state_msg.door_name = "icu1_sliding_door"
        self.door_state_msg.current_mode = DoorMode()
        self.door_state_msg.current_mode.value = 0

        # Initialize the door mode request
        self.door_request_msg = DoorRequest()

        # Set up publishers, subscribers, etc
        self.door_state_pub = self.create_publisher(DoorState, 'door_states', 1)
        self.door_state_timer = self.create_timer(0.5, self.door_state_timer_callback)
        self.door_request_sub = self.create_subscription(DoorRequest, "door_requests", self.door_requests_callback, 1)

        # Set up the servo control thread
        self.get_logger().info('Starting the servo control thread ...')
        self.servo_control_thread = Thread(target=self.servo_control_loop)
        self.servo_control_thread.start()

        self.get_logger().info('The door opener is good to go')

    def set_door_state(self, state):
        self.door_state_mutex.acquire()
        self.door_state_msg.current_mode.value = state
        self.door_state_mutex.release()

    def get_door_state(self):
        return self.door_state_msg.current_mode.value

    def open_the_door(self):
        return self.door_request_msg.requested_mode.value == 2

    def close_the_door(self):
        return self.door_request_msg.requested_mode.value == 0

    def door_requests_callback(self, msg):
        self.door_request_mutex.acquire()
        self.door_request_msg = msg
        self.door_request_mutex.release()

    def door_state_timer_callback(self):
        self.door_state_mutex.acquire()
        self.door_state_msg.door_time = self.get_clock().now().to_msg()
        self.get_logger().debug('Publishing: "{0}"'.format(self.door_state_msg))
        self.door_state_pub.publish(self.door_state_msg)
        self.door_state_mutex.release()

    def servo_control_loop(self):
        while rclpy.ok():
            if self.open_the_door() and self.get_door_state() == 0:
                self.get_logger().info("Opening the door ...")
                self.set_door_state(1)

                # Swipe the RFID servo
                for i in range(0, 360):
                    self.servo.value = math.cos(math.radians(i+180))
                    sleep(0.01)

                # Give the door sime time to actually open up
                sleep(3)
                self.set_door_state(2)
                self.get_logger().info("The door is open")

            elif self.close_the_door() and self.get_door_state() == 2:
                # Do nothing, the door actually closes automatically
                self.get_logger().info("Closing the door ...")
                self.set_door_state(1)
                sleep(4)
                self.set_door_state(0)
                self.get_logger().info("The door is closed")
            
            sleep(0.5)

def main(args=None):
    rclpy.init(args=args)
    swipe = Swipe()
    rclpy.spin(swipe)
    rclpy.shutdown()
    swipe.servo_control_thread.join()
    swipe.destroy_node()

if __name__ == "__main__":
    main()
