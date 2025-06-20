#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/r6bot_controller/joint_trajectory', 10)

        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_positions = [0.0] * 6
        self.step = 0.05  # Adjust sensitivity

    def joy_callback(self, msg):
        if len(msg.axes) >= 6:
            self.joint_positions[0] += self.step * msg.axes[0]
            self.joint_positions[1] += self.step * msg.axes[1]
            self.joint_positions[2] += self.step * msg.axes[3]
            self.joint_positions[3] += self.step * msg.axes[4]
            self.joint_positions[4] += self.step * msg.axes[2]
            self.joint_positions[5] += self.step * msg.axes[5]

        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point1 = JointTrajectoryPoint()
        point1.positions = self.joint_positions.copy()
        point1.velocities = [0.0] * 6
        point1.time_from_start.sec = 0

        point2 = JointTrajectoryPoint()
        point2.positions = self.joint_positions.copy()
        point2.velocities = [0.0] * 6
        point2.time_from_start.sec = 1

        traj.points = [point1, point2]
        self.publisher.publish(traj)

def main():
    rclpy.init()
    node = ArmTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
