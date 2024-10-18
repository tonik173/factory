#!/usr/bin/env python3
# The line above is required for mixed language packages

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_position_controller/controller_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info('trajectory_subscriber inited')

    def listener_callback(self, msg):
        self.process_message(msg)

    def process_message(self, msg):
        joint_names = msg.joint_names

        index = 0
        while index < len(joint_names):
            self.get_logger().info(f"{joint_names[index]} = {msg.reference.positions[index]}")
            index = index + 1




def main(args=None):
    rclpy.init(args=args)

    trajectory_subscriber = TrajectorySubscriber()

    rclpy.spin(trajectory_subscriber)

    trajectory_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()