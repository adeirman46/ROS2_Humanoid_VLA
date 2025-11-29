#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class StandController(Node):
    """Controller that commands robot to standing pose"""
    
    def __init__(self):
        super().__init__('stand_controller')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # Define standing pose (proper values for upright humanoid standing)
        self.standing_pose = {
            # Legs - nearly straight for standing, slight bend for stability
            'left_hip_pitch_joint': -0.15,
            'left_hip_roll_joint': 0.0,
            'left_hip_yaw_joint': 0.0,
            'left_knee_joint': 0.3,
            'left_ankle_pitch_joint': -0.15,
            'left_ankle_roll_joint': 0.0,
            'right_hip_pitch_joint': -0.15,
            'right_hip_roll_joint': 0.0,
            'right_hip_yaw_joint': 0.0,
            'right_knee_joint': 0.3,
            'right_ankle_pitch_joint': -0.15,
            'right_ankle_roll_joint': 0.0,
            # Torso - upright
            'waist_yaw_joint': 0.0,
            'waist_roll_joint': 0.0,
            'waist_pitch_joint': 0.0,
            # Arms - relaxed at sides
            'left_shoulder_pitch_joint': 0.2,
            'left_shoulder_roll_joint': 0.1,
            'left_shoulder_yaw_joint': 0.0,
            'left_elbow_joint': 0.4,
            'left_wrist_roll_joint': 0.0,
            'left_wrist_pitch_joint': 0.0,
            'left_wrist_yaw_joint': 0.0,
            'right_shoulder_pitch_joint': 0.2,
            'right_shoulder_roll_joint': -0.1,
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 0.4,
            'right_wrist_roll_joint': 0.0,
            'right_wrist_pitch_joint': 0.0,
            'right_wrist_yaw_joint': 0.0,
        }
        
        # Create timer to publish commands at 50Hz
        self.timer = self.create_timer(0.02, self.publish_standing_pose)
        
        self.get_logger().info('Stand Controller started - commanding robot to standing pose')
    
    def publish_standing_pose(self):
        """Publish standing position for all joints"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.standing_pose.keys())
        msg.position = list(self.standing_pose.values())
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    controller = StandController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
