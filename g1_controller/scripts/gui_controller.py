#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tkinter as tk
from tkinter import ttk
import threading
import math


class GUIController(Node):
    """GUI Controller for G1 robot with motion commands"""
    
    def __init__(self):
        super().__init__('gui_controller')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )
        
        # Define joint names
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
        
        # Motion state
        self.current_motion = 'stand'
        self.walk_phase = 0.0
        
        # Create timer for continuous publishing
        self.timer = self.create_timer(0.02, self.publish_joint_commands)  # 50Hz
        
        self.get_logger().info('GUI Controller initialized')
    
    def get_standing_pose(self):
        """Return standing pose joint positions"""
        return {
            # Legs - nearly straight for standing, slight bend for stability
            'left_hip_pitch_joint': -0.15,    # Slight forward lean
            'left_hip_roll_joint': 0.0,        # No sideways tilt
            'left_hip_yaw_joint': 0.0,         # No rotation
            'left_knee_joint': 0.3,            # Slight knee bend for balance
            'left_ankle_pitch_joint': -0.15,   # Counter hip pitch
            'left_ankle_roll_joint': 0.0,      # No roll
            'right_hip_pitch_joint': -0.15,    # Same as left
            'right_hip_roll_joint': 0.0,
            'right_hip_yaw_joint': 0.0,
            'right_knee_joint': 0.3,           # Same as left
            'right_ankle_pitch_joint': -0.15,  # Same as left
            'right_ankle_roll_joint': 0.0,
            # Torso - upright
            'waist_yaw_joint': 0.0,
            'waist_roll_joint': 0.0,
            'waist_pitch_joint': 0.0,
            # Arms - relaxed at sides
            'left_shoulder_pitch_joint': 0.2,   # Slightly forward
            'left_shoulder_roll_joint': 0.1,    # Slightly outward
            'left_shoulder_yaw_joint': 0.0,
            'left_elbow_joint': 0.4,            # Slight bend
            'left_wrist_roll_joint': 0.0,
            'left_wrist_pitch_joint': 0.0,
            'left_wrist_yaw_joint': 0.0,
            'right_shoulder_pitch_joint': 0.2,  # Same as left
            'right_shoulder_roll_joint': -0.1,  # Mirror of left
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 0.4,           # Same as left
            'right_wrist_roll_joint': 0.0,
            'right_wrist_pitch_joint': 0.0,
            'right_wrist_yaw_joint': 0.0,
        }
    
    def get_walk_forward_pose(self):
        """Return walking forward pose (alternating leg swing)"""
        pose = self.get_standing_pose()
        
        # Simple gait: swing legs alternately
        swing = 0.3 * math.sin(self.walk_phase)
        
        pose['left_hip_pitch_joint'] = -swing
        pose['right_hip_pitch_joint'] = swing
        pose['left_knee_joint'] = max(0, swing)
        pose['right_knee_joint'] = max(0, -swing)
        
        # Arm swing (opposite to legs)
        pose['left_shoulder_pitch_joint'] = swing * 0.5
        pose['right_shoulder_pitch_joint'] = -swing * 0.5
        
        self.walk_phase += 0.1  # Increment phase
        
        return pose
    
    def get_turn_clockwise_pose(self):
        """Return turning clockwise pose"""
        pose = self.get_standing_pose()
        pose['waist_yaw_joint'] = -0.3 * math.sin(self.walk_phase)
        
        # Alternate leg stepping
        swing = 0.2 * math.sin(self.walk_phase)
        pose['left_hip_yaw_joint'] = -swing
        pose['right_hip_yaw_joint'] = swing
        
        self.walk_phase += 0.05
        
        return pose
    
    def get_turn_counterclockwise_pose(self):
        """Return turning counterclockwise pose"""
        pose = self.get_standing_pose()
        pose['waist_yaw_joint'] = 0.3 * math.sin(self.walk_phase)
        
        # Alternate leg stepping
        swing = 0.2 * math.sin(self.walk_phase)
        pose['left_hip_yaw_joint'] = swing
        pose['right_hip_yaw_joint'] = -swing
        
        self.walk_phase += 0.05
        
        return pose
    
    def publish_joint_commands(self):
        """Publish joint commands based on current motion"""
        # Get pose based on motion
        if self.current_motion == 'stand':
            pose = self.get_standing_pose()
        elif self.current_motion == 'walk_forward':
            pose = self.get_walk_forward_pose()
        elif self.current_motion == 'turn_cw':
            pose = self.get_turn_clockwise_pose()
        elif self.current_motion == 'turn_ccw':
            pose = self.get_turn_counterclockwise_pose()
        else:
            pose = self.get_standing_pose()
        
        # Create and publish message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(pose.keys())
        msg.position = list(pose.values())
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        
        self.joint_pub.publish(msg)
    
    def set_motion(self, motion):
        """Set the current motion"""
        self.current_motion = motion
        self.walk_phase = 0.0  # Reset phase
        self.get_logger().info(f'Motion changed to: {motion}')


def create_gui(controller):
    """Create the GUI window"""
    root = tk.Tk()
    root.title("G1 Robot Controller")
    root.geometry("400x300")
    
    # Title
    title = tk.Label(root, text="Unitree G1 Controller", font=("Arial", 16, "bold"))
    title.pack(pady=10)
    
    # Status
    status_label = tk.Label(root, text="Current Motion: Stand", font=("Arial", 12))
    status_label.pack(pady=5)
    
    # Button frame
    button_frame = ttk.Frame(root, padding="10")
    button_frame.pack(pady=10)
    
    def update_motion(motion, label):
        controller.set_motion(motion)
        status_label.config(text=f"Current Motion: {label}")
    
    # Create buttons
    ttk.Button(
        button_frame,
        text="🧍 Stand",
        command=lambda: update_motion('stand', 'Stand'),
        width=20
    ).grid(row=0, column=0, columnspan=2, pady=5)
    
    ttk.Button(
        button_frame,
        text="🚶 Walk Forward",
        command=lambda: update_motion('walk_forward', 'Walk Forward'),
        width=20
    ).grid(row=1, column=0, columnspan=2, pady=5)
    
    ttk.Button(
        button_frame,
        text="↻ Turn Clockwise",
        command=lambda: update_motion('turn_cw', 'Turn Clockwise'),
        width=20
    ).grid(row=2, column=0, columnspan=2, pady=5)
    
    ttk.Button(
        button_frame,
        text="↺ Turn Counter-Clockwise",
        command=lambda: update_motion('turn_ccw', 'Turn Counter-Clockwise'),
        width=20
    ).grid(row=3, column=0, columnspan=2, pady=5)
    
    # Quit button
    ttk.Button(
        root,
        text="Quit",
        command=root.quit,
        width=20
    ).pack(pady=10)
    
    # Info label
    info = tk.Label(
        root,
        text="Click buttons to change robot motion",
        font=("Arial", 9),
        fg="gray"
    )
    info.pack(pady=5)
    
    return root


def main(args=None):
    rclpy.init(args=args)
    controller = GUIController()
    
    # Create GUI in separate thread
    root = create_gui(controller)
    
    # Run ROS in thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(controller), daemon=True)
    ros_thread.start()
    
    # Run GUI (blocks)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
