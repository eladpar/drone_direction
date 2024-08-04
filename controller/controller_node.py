#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from diagnostic_updater import Updater, DiagnosticStatusWrapper

class ControllerNode(Node):
    """
    ControllerNode class implements a ROS2 node that subscribes to the 'heuristic_direction'
    topic and sends control commands to the drone using MAVROS.

    This node listens to heuristic direction data and converts it into velocity commands
    for the drone, publishing to the appropriate MAVROS topic.

    Attributes:
    - cmd_topic: The MAVROS topic to which velocity commands are published.
    """
    def __init__(self):
        super().__init__('controller_node')
        
        # Declare parameters with default values
        self.declare_parameter('cmd_topic', '/mavros/setpoint_velocity/cmd_vel')
        self.declare_parameter('linear_velocity', 0.1)

        # Retrieve parameter values from the parameter server
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.linear_velocity = self.get_parameter('linear_velocity').get_parameter_value().double_value

        # Create a subscription to the 'heuristic_direction' topic
        self.subscription = self.create_subscription(
            Float64,
            'heuristic_direction',
            self.listener_callback,
            10
        )

        # Create a publisher for the MAVROS command topic
        self.cmd_publisher = self.create_publisher(Twist, self.cmd_topic, 10)

        # Initialize diagnostic updater
        self.diagnostic_updater = Updater(self)
        self.diagnostic_updater.setHardwareID("controller_node")
        self.diagnostic_updater.add("Controller Status", self.produce_diagnostics)

        # Initialize the latest received heuristic direction
        self.latest_heuristic_direction = None

        # Create a timer to periodically update diagnostics
        self.diagnostic_timer = self.create_timer(1.0, self.diagnostic_updater.force_update)

    def listener_callback(self, msg):
        """
        Callback function that handles incoming messages from the 'heuristic_direction' topic.
        Converts the received direction data into a velocity command and publishes it.

        :param msg: The message containing the heuristic direction data.
        """
        # Update the latest heuristic direction
        self.latest_heuristic_direction = msg.data

        # Convert heuristic direction data into a velocity command
        angular_velocity = msg.data 

        # Create a Twist message for the MAVROS topic
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_velocity  # Move at a constant linear speed
        twist_msg.linear.y = 0.0                   # No lateral movement
        twist_msg.linear.z = 0.0                   # No vertical movement
        twist_msg.angular.z = angular_velocity     # Rotate based on heuristic direction

        # Publish the command
        self.cmd_publisher.publish(twist_msg)
        self.get_logger().info(f'Sent velocity command: linear.x={self.linear_velocity}, angular.z={angular_velocity}')

    def produce_diagnostics(self, stat):
        """
        Produces diagnostic information for the diagnostics topic.
        Updates the diagnostic status with the latest information.

        :param stat: DiagnosticStatusWrapper to update with diagnostic information.
        :return: Updated DiagnosticStatusWrapper.
        """
        if self.latest_heuristic_direction is not None:
            stat.summary(DiagnosticStatusWrapper.OK, "Node is running smoothly")
            stat.add("Latest Heuristic Direction", str(self.latest_heuristic_direction))
            stat.add("Linear Velocity", str(self.linear_velocity))
        else:
            stat.summary(DiagnosticStatusWrapper.WARN, "No heuristic direction received yet")
        return stat

def main(args=None):
    """
    Main function to initialize the ROS2 node and start spinning.
    """
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
