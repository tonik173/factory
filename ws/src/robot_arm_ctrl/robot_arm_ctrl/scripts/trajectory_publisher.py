#!/usr/bin/env python3
# The line above is required for mixed language packages

import os
import array as arr
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState

from sp_node import SparkplugNode

class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')

        #brokerHost = os.environ['MQTT_BROKER_HOST']
        #brokerPort = os.environ['MQTT_BROKER_PORT']

        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/joint_trajectory_position_controller/controller_state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.edgeNode = SparkplugNode('172.16.1.10', 1885, 'factory', 'robot_arm', self.command_callback)
        self.get_logger().info('trajectory_subscriber inited')

    def listener_callback(self, msg):
        self.process_message(msg)

    def process_message(self, msg):
        joint_names = msg.joint_names

        n = len(joint_names)
        z = arr.array("d", [0]*n) # create array with as many zeros as joint names
        index = 0
        while index < n:

            #print("*****************************")
            # gets the values from the joints. adds the zero array to the end to make sure we always have at least n numbers in the array
            positions = [(msg.reference.positions + z)[index], (msg.feedback.positions + z)[index], (msg.error.positions + z)[index], (msg.output.positions + z)[index]]
            #print(positions)
            velocities = [(msg.reference.velocities + z)[index], (msg.feedback.velocities + z)[index], (msg.error.velocities + z)[index], (msg.output.velocities + z)[index]] 
            #print(velocities)
            accelerations = [(msg.reference.accelerations + z)[index], (msg.feedback.accelerations + z)[index], (msg.error.accelerations + z)[index], (msg.output.accelerations + z)[index]]
            #print(accelerations)
            effort = [(msg.reference.effort + z)[index], (msg.feedback.effort + z)[index], (msg.error.effort + z)[index], (msg.output.effort + z)[index]] 
            #print(effort)

            #self.edgeNode.set_trajectory_move(joint_names[index], "revolute", positions, velocities, accelerations, effort)  
            #self.edgeNode.send()                      

            index = index + 1

    def command_callback(self, cmd):
        print("TrajectorySubscriber received CMD: %s" % str(cmd))

def main(args=None):
    rclpy.init(args=args)

    trajectory_subscriber = TrajectorySubscriber()

    rclpy.spin(trajectory_subscriber)

    trajectory_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()