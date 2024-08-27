import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.timer = self.create_timer(5.0, self.listen_and_publish)

    def listen_and_publish(self):
        with self.microphone as source:
            audio = self.recognizer.listen(source)

        try:
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"Recognized: {text}")
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results; {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.publisher_ = self.create_publisher(String, 'speech_text', 10)
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        self.timer = self.create_timer(5.0, self.listen_and_publish)

    def listen_and_publish(self):
        with self.microphone as source:
            audio = self.recognizer.listen(source)

        try:
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"Recognized: {text}")
            msg = String()
            msg.data = text
            self.publisher_.publish(msg)
        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
        except sr.RequestError as e:
            self.get_logger().error(f"Could not request results; {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()