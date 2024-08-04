#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random
from diagnostic_updater import DiagnosticTask, Updater, DiagnosticStatusWrapper

class AccurateModelNode(Node):
    """
    AccurateModelNode class implements a ROS2 node that publishes accurate direction data.
    It includes a dummy subscriber that simulates receiving sensor data, which will be used in model inference in the future.

    Parameters:
    - direction_min: The minimum value for the generated direction.
    - direction_max: The maximum value for the generated direction.
    - publish_frequency: The frequency at which to publish the direction data.
    """
    def __init__(self):
        super().__init__('accurate_model')

        # Declare parameters with default values
        self.declare_parameter('direction_min', -1.0)
        self.declare_parameter('direction_max', 1.0)
        self.declare_parameter('publish_frequency', 1.0)

        # Retrieve parameter values
        self.direction_min = self.get_parameter('direction_min').get_parameter_value().double_value
        self.direction_max = self.get_parameter('direction_max').get_parameter_value().double_value
        self.publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value

        # Create a publisher for the 'accurate_direction' topic
        self.publisher_ = self.create_publisher(Float64, 'accurate_direction', 10)

        # Create a dummy subscriber for sensor data
        self.sensor_subscription = self.create_subscription(
            Float64,
            'sensor_data',
            self.sensor_data_callback,
            10
        )

        # Create a timer to periodically publish accurate direction data
        # Timer interval is determined by publish_frequency parameter
        self.timer = self.create_timer(1.0 / self.publish_frequency, self.publish_direction)

        # Initialize a variable to store the latest sensor data
        self.latest_sensor_data = 0.0

        # Initialize diagnostic updater
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID("accurate_model_node")
        self.diagnostic_updater.add("Accurate Model Status", self.produce_diagnostics)

        # Create a timer to periodically update diagnostics
        self.diagnostic_timer = self.create_timer(1.0, self.diagnostic_updater.force_update)

    def publish_direction(self):
        """
        Callback function that generates and publishes accurate direction data.
        The direction is randomly generated within the range [direction_min, direction_max].

        Note: The use of random data is stub logic for demonstration purposes.
        In a real application, this would be replaced by actual model output.
        """
        msg = Float64()
        msg.data = random.uniform(self.direction_min, self.direction_max)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing accurate direction: {msg.data}')

    def sensor_data_callback(self, msg):
        """
        Callback function that handles incoming sensor data messages.
        Updates the latest sensor data, which will be used for model inference.

        Depending on the sensor data and topic frequency, we might want to use different callback groups for
        sensor and publish callbacks.

        :param msg: The message containing the sensor data.
        """
        self.latest_sensor_data = msg.data
        self.get_logger().info(f'Received sensor data: {self.latest_sensor_data}')

    def produce_diagnostics(self, stat):
        """
        Produces diagnostic information for the diagnostics topic.
        Updates the diagnostic status with the latest information.

        :param stat: DiagnosticStatusWrapper to update with diagnostic information.
        :return: Updated DiagnosticStatusWrapper.
        """
        if self.latest_sensor_data is not None:
            stat.summary(DiagnosticStatusWrapper.OK, "Node is running smoothly")
            stat.add("Latest Sensor Data", str(self.latest_sensor_data))
        else:
            stat.summary(DiagnosticStatusWrapper.WARN, "No sensor data received yet")
        return stat

def main(args=None):
    """
    Main function to initialize the ROS2 node and start spinning.
    """
    rclpy.init(args=args)
    node = AccurateModelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
