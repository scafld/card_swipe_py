

import rclpy
from rclpy.node import Node

#from std_msgs.msg import Bool
from rmf_door_msgs.msg import DoorMode


class TriggerPublisher(Node):

    def __init__(self):
        super().__init__('trigger_publisher')
        self.publisher_ = self.create_publisher(DoorMode, 'smart_card_trigger', 1)
        timer_period = 4 #s
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = DoorMode()
        if self.i % 2:
            msg.value = 2
        else:
            msg.value = 1
        self.publisher_.publish(msg)
        logmsg = 'Publishing' + str(msg.value)
        self.get_logger().info(logmsg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    trigger_publisher = TriggerPublisher()
    rclpy.spin(trigger_publisher)
    trigger_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
