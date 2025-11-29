#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class ZeroPoseController(Node):
    """Simple controller that commands all joints to zero position"""
    
    def __init__(self):
        super().__init__('zero_pose_controller')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # Define all joint names (29 DOF)
        self.joint_names = [
            # Legs
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            # Torso
            'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
            # Arms
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
            'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
            'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
        ]
        
        # Create timer to publish commands at 50Hz
        self.timer = self.create_timer(0.02, self.publish_zero_pose)
        
        self.get_logger().info('Zero Pose Controller started - commanding all joints to zero')
    
    def publish_zero_pose(self):
        """Publish zero position for all joints"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = ZeroPoseController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
