import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TurtlebotControlNode(Node):
    def __init__(self):
        super().__init__('turtlebot_control_node')
        self.subscription = self.create_subscription(String, 'openai_response', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def listener_callback(self, msg):
        self.get_logger().info(f"Command: {msg.data}")
        twist = Twist()

        if "forward" in msg.data:
            twist.linear.x = 0.2
        elif "backward" in msg.data:
            twist.linear.x = -0.2
        elif "left" in msg.data:
            twist.angular.z = 0.5
        elif "right" in msg.data:
            twist.angular.z = -0.5
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()