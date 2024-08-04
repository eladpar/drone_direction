#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from diagnostic_updater import Updater, DiagnosticStatusWrapper

class HeuristicModelNode(Node):
    """
    HeuristicModelNode class implements a ROS2 node that subscribes to the 'accurate_direction'
    topic and publishes heuristic direction data to the 'heuristic_direction' topic.

    The node operates with two main callbacks:
    1. listener_callback: Handles incoming messages from the 'accurate_direction' topic.
    2. publish_heuristic_direction: Periodically publishes heuristic direction data based on the latest received accurate direction.

    The default callback group is used, ensuring that only one callback is executed at a time
    (MutuallyExclusiveCallbackGroup), which prevents race conditions.
    """
    def __init__(self):
        super().__init__('heuristic_model')
        # Declare parameters with default values
        self.declare_parameter('heuristic_multiplier', 1.05)
        self.declare_parameter('publish_frequency', 5.0)

        # Retrieve parameter values
        self.heuristic_multiplier = self.get_parameter('heuristic_multiplier').get_parameter_value().double_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Create a subscription to the 'accurate_direction' topic
        self.subscription = self.create_subscription(
            Float64,
            'accurate_direction',
            self.listener_callback,
            10
        )

        # Create a publisher for the 'heuristic_direction' topic
        self.publisher_ = self.create_publisher(Float64, 'heuristic_direction', 10)

        # Initialize last accurate value as None
        self.last_accurate_value = None

        # Create a timer to periodically publish heuristic direction data
        # Timer interval is determined by publish_frequency parameter
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_heuristic_direction)

        # Initialize diagnostic updater
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID("heuristic_model_node")
        self.diagnostic_updater.add("Heuristic Model Status", self.produce_diagnostics)

        # Create a timer to periodically update diagnostics
        self.diagnostic_timer = self.create_timer(1.0, self.diagnostic_updater.force_update)

    def listener_callback(self, msg):
        """
        Callback function that handles incoming messages from the 'accurate_direction' topic.
        Updates the last received accurate direction value.

        :param msg: The message containing the accurate direction data.
        """
        self.last_accurate_value = msg.data
        self.get_logger().info(f'Received accurate direction: {msg.data}')

    def publish_heuristic_direction(self):
        """
        Callback function that publishes heuristic direction data.
        The heuristic direction is calculated using the last received accurate direction value
        and the heuristic_multiplier parameter.

        Note: This is stub logic that multiplies the last accurate value with a multiplier.
        In a real implementation, this would be a model inference.
        """
        if self.last_accurate_value is not None:
            heuristic_value = self.last_accurate_value * self.heuristic_multiplier  # Stub logic instead of model
            msg = Float64()
            msg.data = heuristic_value
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing heuristic direction: {msg.data}')

    def produce_diagnostics(self, stat):
        """
        Produces diagnostic information for the diagnostics topic.
        Updates the diagnostic status with the latest information.

        :param stat: DiagnosticStatusWrapper to update with diagnostic information.
        :return: Updated DiagnosticStatusWrapper.
        """
        if self.last_accurate_value is not None:
            stat.summary(DiagnosticStatusWrapper.OK, "Node is running smoothly")
            stat.add("Latest Accurate Direction", str(self.last_accurate_value))
            stat.add("Heuristic Multiplier", str(self.heuristic_multiplier))
        else:
            stat.summary(DiagnosticStatusWrapper.WARN, "No accurate direction received yet")
        return stat
def main(args=None):
    """
    Main function to initialize the ROS2 node and start spinning.
    """
    rclpy.init(args=args)
    node = HeuristicModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
