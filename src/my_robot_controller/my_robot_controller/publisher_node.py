import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Publisher(Node):

    def __init__(self):
        super().__init__("publisher_node")
        self.counter_ = 0
        self.publisher_ = self.create_publisher(String, "/myTopic", 10)
        self.timers_ = self.create_timer(1.0, self.publisher)
        self.get_logger().info("publisher node is started")

    def publisher(self):
        msg = String()
        msg.data = "Hello " + str(self.counter_)
        self.counter_ += 1
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

