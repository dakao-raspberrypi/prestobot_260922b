import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rclpy.serialization import serialize_message
# import rosbag2_py

class TopicRelay(Node):
    """
    A generic ROS 2 topic relay node in Python.
    """
    def __init__(self):
        super().__init__('topic_relay')

        # Declare parameters for input topic, output topic, and message type
        self.declare_parameter('input_topic', '/default_input')
        self.declare_parameter('output_topic', '/default_output')
        self.declare_parameter('message_type', 'std_msgs/msg/String')

        # Get the parameter values
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        message_type_str = self.get_parameter('message_type').get_parameter_value().string_value

        # Dynamically import the message type
        try:
            # Split the message type into package and message name
            parts = message_type_str.split('/')
            if len(parts) == 3: # Format is package/interface/message
                module_name = f"{parts[0]}.{parts[1]}"
                msg_name = parts[2]
            else: # Fallback for format package/message
                module_name = f"{parts[0]}.msg"
                msg_name = parts[1]

            message_module = __import__(module_name, fromlist=[msg_name])
            self.message_type = getattr(message_module, msg_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Failed to import message type '{message_type_str}': {e}")
            return

        # Create the publisher and subscriber
        self.publisher = self.create_publisher(self.message_type, output_topic, 10)
        self.subscription = self.create_subscription(
            self.message_type,
            input_topic,
            self.listener_callback,
            10)
        self.get_logger().info(f"Relaying from '{input_topic}' to '{output_topic}' with message type '{message_type_str}'")

    def listener_callback(self, msg):
        """
        Callback function for the subscriber.
        This function receives a message and republishes it.
        """
        # self.get_logger().info(f"Relaying message: {msg}")
        self.publisher.publish(msg)

def main(args=None):
    """
    Main function to initialize and run the node.
    """
    rclpy.init(args=args)
    topic_relay = TopicRelay()
    rclpy.spin(topic_relay)
    topic_relay.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()