#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import tkinter as tk
from tkinter import ttk
import threading


class ROS2ControlGUIController(Node):
    """GUI Controller using ROS2 Control (JointTrajectory actions)"""
    
    def __init__(self):
        super().__init__('ros2_control_gui_controller')
        
        # Action client for trajectory controller
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/position_trajectory_controller/follow_joint_trajectory'
        )
        
        # Define joint names
        self.joint_names = [
            'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint',
            'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
            'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
            'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
        ]
        
        self.get_logger().info('ROS2 Control GUI Controller initialized')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')
    
    def get_standing_pose(self):
        """Return standing pose joint positions"""
        pose = {}
        for joint in self.joint_names:
            if 'hip_pitch' in joint:
                pose[joint] = -0.15
            elif 'knee' in joint:
                pose[joint] = 0.3
            elif 'ankle_pitch' in joint:
                pose[joint] = -0.15
            elif 'shoulder_pitch' in joint:
                pose[joint] = 0.2
            elif 'shoulder_roll' in joint:
                pose[joint] = 0.1 if 'left' in joint else -0.1
            elif 'elbow' in joint:
                pose[joint] = 0.4
            else:
                pose[joint] = 0.0
        return pose
    
    def get_walk_forward_pose(self, phase):
        """Return walking forward pose based on phase (0-2*pi)"""
        import math
        pose = self.get_standing_pose()
        
        # Simple alternating leg swing
        swing = 0.2 * math.sin(phase)
        
        pose['left_hip_pitch_joint'] = -0.15 + swing
        pose['right_hip_pitch_joint'] = -0.15 - swing
        pose['left_knee_joint'] = 0.3 + max(0, swing)
        pose['right_knee_joint'] = 0.3 + max(0, -swing)
        
        # Arm swing opposite to legs
        pose['left_shoulder_pitch_joint'] = 0.2 - swing * 0.5
        pose['right_shoulder_pitch_joint'] = 0.2 + swing * 0.5
        
        return pose
    
    def get_turn_clockwise_pose(self, phase):
        """Return turning clockwise pose"""
        import math
        pose = self.get_standing_pose()
        
        # Rotate torso
        pose['waist_yaw_joint'] = -0.2 * math.sin(phase)
        
        # Alternating leg movement
        swing = 0.15 * math.sin(phase)
        pose['left_hip_yaw_joint'] = -swing
        pose['right_hip_yaw_joint'] = swing
        
        return pose
    
    def get_turn_counterclockwise_pose(self, phase):
        """Return turning counter-clockwise pose"""
        import math
        pose = self.get_standing_pose()
        
        # Rotate torso
        pose['waist_yaw_joint'] = 0.2 * math.sin(phase)
        
        # Alternating leg movement
        swing = 0.15 * math.sin(phase)
        pose['left_hip_yaw_joint'] = swing
        pose['right_hip_yaw_joint'] = -swing
        
        return pose
    
    def send_trajectory(self, joint_positions, duration=2.0):
        """Send trajectory goal to controller"""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Create trajectory
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Create waypoint
        point = JointTrajectoryPoint()
        point.positions = [joint_positions.get(name, 0.0) for name in self.joint_names]
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        goal_msg.trajectory.points = [point]
        
        # Send goal
        self.get_logger().info(f'Sending trajectory goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code}')
    
    def command_stand(self):
        """Command standing pose"""
        self.get_logger().info('Commanding STAND pose')
        pose = self.get_standing_pose()
        self.send_trajectory(pose, duration=3.0)
    
    def command_zero(self):
        """Command zero pose"""
        self.get_logger().info('Commanding ZERO pose')
        pose = {joint: 0.0 for joint in self.joint_names}
        self.send_trajectory(pose, duration=3.0)
    
    def command_walk_forward(self):
        """Command walk forward (single step)"""
        self.get_logger().info('Commanding WALK FORWARD')
        import math
        pose = self.get_walk_forward_pose(math.pi / 2)  # One step
        self.send_trajectory(pose, duration=2.0)
    
    def command_turn_cw(self):
        """Command turn clockwise"""
        self.get_logger().info('Commanding TURN CLOCKWISE')
        import math
        pose = self.get_turn_clockwise_pose(math.pi / 2)
        self.send_trajectory(pose, duration=2.0)
    
    def command_turn_ccw(self):
        """Command turn counter-clockwise"""
        self.get_logger().info('Commanding TURN COUNTER-CLOCKWISE')
        import math
        pose = self.get_turn_counterclockwise_pose(math.pi / 2)
        self.send_trajectory(pose, duration=2.0)


def create_gui(controller):
    """Create the GUI window"""
    root = tk.Tk()
    root.title("G1 Robot - ROS2 Control")
    root.geometry("400x350")
    
    # Title
    title = tk.Label(root, text="Unitree G1 - ROS2 Control", font=("Arial", 16, "bold"))
    title.pack(pady=10)
    
    # Status
    status_label = tk.Label(root, text="Using JointTrajectory Actions", font=("Arial", 10), fg="green")
    status_label.pack(pady=5)
    
    # Button frame
    button_frame = ttk.Frame(root, padding="10")
    button_frame.pack(pady=10)
    
    # Create buttons
    ttk.Button(
        button_frame,
        text="🧍 Stand Pose",
        command=controller.command_stand,
        width=25
    ).grid(row=0, column=0, pady=5)
    
    ttk.Button(
        button_frame,
        text="⭕ Zero Pose",
        command=controller.command_zero,
        width=25
    ).grid(row=1, column=0, pady=5)
    
    ttk.Button(
        button_frame,
        text="🚶 Walk Forward",
        command=controller.command_walk_forward,
        width=25
    ).grid(row=2, column=0, pady=5)
    
    ttk.Button(
        button_frame,
        text="↻ Turn Clockwise",
        command=controller.command_turn_cw,
        width=25
    ).grid(row=3, column=0, pady=5)
    
    ttk.Button(
        button_frame,
        text="↺ Turn Counter-Clockwise",
        command=controller.command_turn_ccw,
        width=25
    ).grid(row=4, column=0, pady=5)
    
    # Quit button
    ttk.Button(
        root,
        text="Quit",
        command=root.quit,
        width=25
    ).pack(pady=10)
    
    # Info label
    info = tk.Label(
        root,
        text="Click buttons to command robot poses\\n(Commands sent via ROS2 Control)",
        font=("Arial", 9),
        fg="gray"
    )
    info.pack(pady=5)
    
    return root


def main(args=None):
    rclpy.init(args=args)
    controller = ROS2ControlGUIController()
    
    # Create GUI
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
