import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai

class OpenAINode(Node):
    def __init__(self):
        super().__init__('openai_node')
        self.subscription = self.create_subscription(String, 'speech_text', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'openai_response', 10)

        openai.api_key = 'your-openai-api-key'

    def listener_callback(self, msg):
        self.get_logger().info(f"Received text: {msg.data}")
        response = self.query_openai(msg.data)
        self.get_logger().info(f"Response: {response}")
        
        response_msg = String()
        response_msg.data = response
        self.publisher_.publish(response_msg)

    def query_openai(self, text):
        try:
            response = openai.Completion.create(
                engine="text-davinci-003",
                prompt=text,
                max_tokens=150
            )
            return response.choices[0].text.strip()
        except Exception as e:
            self.get_logger().error(f"OpenAI API error: {e}")
            return "Sorry, I couldn't process that."

def main(args=None):
    rclpy.init(args=args)
    node = OpenAINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()