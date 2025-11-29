#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
import threading
import time


class WASDController(Node):
    """WASD Keyboard Controller using ROS2 Control (JointTrajectory actions)"""
    
    def __init__(self):
        super().__init__('wasd_controller')
        
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
        
        # Publisher for base velocity to make robot actually move in Gazebo
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Current motion state
        self.current_motion = None
        self.motion_phase = 0.0
        self.running = True
        
        self.get_logger().info('WASD Controller initialized')
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')
        
        # Start motion loop
        self.motion_thread = threading.Thread(target=self.motion_loop, daemon=True)
        self.motion_thread.start()
        
        # Start keyboard listener
        self.start_keyboard_listener()
    
    def get_standing_pose(self):
        """Return standing pose joint positions - adjusted for proper standing"""
        pose = {}
        for joint in self.joint_names:
            if 'hip_pitch' in joint:
                pose[joint] = -0.4  # More hip flexion for upright stance
            elif 'knee' in joint:
                pose[joint] = 0.8  # Significant knee bend to support standing
            elif 'ankle_pitch' in joint:
                pose[joint] = -0.4  # Balance the knee bend
            elif 'ankle_roll' in joint:
                pose[joint] = 0.0  # Keep ankles straight - no spinning!
            elif 'shoulder_pitch' in joint:
                pose[joint] = 0.3  # Arms slightly forward
            elif 'shoulder_roll' in joint:
                pose[joint] = 0.15 if 'left' in joint else -0.15  # Arms at sides
            elif 'elbow' in joint:
                pose[joint] = 0.5  # Slight elbow bend
            else:
                pose[joint] = 0.0
        return pose
    
    def get_walk_forward_pose(self, phase):
        """Return walking forward pose based on phase (0-2*pi)"""
        import math
        pose = self.get_standing_pose()
        
        # MUCH larger alternating leg swing for very visible movement
        swing = 1.2 * math.sin(phase)  # Tripled from 0.4
        
        pose['left_hip_pitch_joint'] = -0.4 + swing
        pose['right_hip_pitch_joint'] = -0.4 - swing
        pose['left_knee_joint'] = 0.8 + max(0, swing * 0.5)
        pose['right_knee_joint'] = 0.8 + max(0, -swing * 0.5)
        
        # Much larger arm swing opposite to legs
        pose['left_shoulder_pitch_joint'] = 0.3 - swing * 2.5  # Tripled
        pose['right_shoulder_pitch_joint'] = 0.3 + swing * 2.5
        
        return pose
    
    def get_walk_backward_pose(self, phase):
        """Return walking backward pose (reverse of forward)"""
        import math
        pose = self.get_standing_pose()
        
        # MUCH larger reverse leg swing
        swing = 1.2 * math.sin(phase)  # Tripled from 0.4
        
        pose['left_hip_pitch_joint'] = -0.4 - swing
        pose['right_hip_pitch_joint'] = -0.4 + swing
        pose['left_knee_joint'] = 0.8 + max(0, -swing * 0.5)
        pose['right_knee_joint'] = 0.8 + max(0, swing * 0.5)
        
        # Much larger arm swing
        pose['left_shoulder_pitch_joint'] = 0.3 + swing * 2.5  # Tripled
        pose['right_shoulder_pitch_joint'] = 0.3 - swing * 2.5
        
        return pose
    
    def get_turn_clockwise_pose(self, phase):
        """Return turning clockwise pose"""
        import math
        pose = self.get_standing_pose()
        
        # MUCH larger torso rotation for very visible turning
        pose['waist_yaw_joint'] = -1.5 * math.sin(phase)  # Tripled from 0.5
        
        # Much larger alternating leg movement
        swing = 0.9 * math.sin(phase)  # Tripled from 0.3
        pose['left_hip_yaw_joint'] = -swing
        pose['right_hip_yaw_joint'] = swing
        
        return pose
    
    def get_turn_counterclockwise_pose(self, phase):
        """Return turning counter-clockwise pose"""
        import math
        pose = self.get_standing_pose()
        
        # MUCH larger torso rotation for very visible turning
        pose['waist_yaw_joint'] = 1.5 * math.sin(phase)  # Tripled from 0.5
        
        # Much larger alternating leg movement
        swing = 0.9 * math.sin(phase)  # Tripled from 0.3
        pose['left_hip_yaw_joint'] = swing
        pose['right_hip_yaw_joint'] = -swing
        
        return pose
    
    def send_trajectory(self, joint_positions, duration=1.0):
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
        
        # Send goal (non-blocking)
        self._action_client.send_goal_async(goal_msg)
    
    def publish_base_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command to move robot base in Gazebo"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)
    
    def motion_loop(self):
        """Continuous motion loop that updates robot pose based on current motion"""
        import math
        
        while self.running:
            if self.current_motion:
                # Increment phase
                self.motion_phase += 0.1
                if self.motion_phase > 2 * math.pi:
                    self.motion_phase = 0.0
                
                # Get pose based on current motion
                if self.current_motion == 'forward':
                    pose = self.get_walk_forward_pose(self.motion_phase)
                    self.publish_base_velocity(linear_x=0.3)  # Move forward!
                elif self.current_motion == 'backward':
                    pose = self.get_walk_backward_pose(self.motion_phase)
                    self.publish_base_velocity(linear_x=-0.3)  # Move backward!
                elif self.current_motion == 'left':
                    pose = self.get_turn_counterclockwise_pose(self.motion_phase)
                    self.publish_base_velocity(angular_z=0.5)  # Turn left!
                elif self.current_motion == 'right':
                    pose = self.get_turn_clockwise_pose(self.motion_phase)
                    self.publish_base_velocity(angular_z=-0.5)  # Turn right!
                elif self.current_motion == 'stand':
                    pose = self.get_standing_pose()
                    self.publish_base_velocity()  # Stop
                    self.current_motion = None  # One-shot command
                else:
                    pose = self.get_standing_pose()
                    self.publish_base_velocity()  # Stop
                
                # Send trajectory
                self.send_trajectory(pose, duration=0.8)
            else:
                # No motion - stop base
                self.publish_base_velocity()
            
            time.sleep(0.5)
    
    def start_keyboard_listener(self):
        """Start listening for keyboard input"""
        try:
            from pynput import keyboard
        except ImportError:
            self.get_logger().error('pynput not installed. Install with: pip install pynput')
            return
        
        def on_press(key):
            try:
                # Handle character keys
                if hasattr(key, 'char'):
                    if key.char == 'w':
                        self.current_motion = 'forward'
                        self.motion_phase = 0.0
                        self.get_logger().info('🚶 Walking FORWARD')
                    elif key.char == 's':
                        self.current_motion = 'backward'
                        self.motion_phase = 0.0
                        self.get_logger().info('🚶 Walking BACKWARD')
                    elif key.char == 'a':
                        self.current_motion = 'left'
                        self.motion_phase = 0.0
                        self.get_logger().info('↺ Turning COUNTERCLOCKWISE (LEFT)')
                    elif key.char == 'd':
                        self.current_motion = 'right'
                        self.motion_phase = 0.0
                        self.get_logger().info('↻ Turning CLOCKWISE (RIGHT)')
                # Handle special keys
                elif key == keyboard.Key.space:
                    self.current_motion = 'stand'
                    self.motion_phase = 0.0
                    self.get_logger().info('🧍 Returning to STAND pose')
                elif key == keyboard.Key.esc:
                    self.get_logger().info('ESC pressed - Shutting down...')
                    self.running = False
                    return False  # Stop listener
            except AttributeError:
                pass
        
        def on_release(key):
            # Stop motion when key is released
            if hasattr(key, 'char') and key.char in ['w', 'a', 's', 'd']:
                self.current_motion = None
                self.get_logger().info('⏸️  Motion stopped')
        
        # Print instructions
        print("\n" + "="*50)
        print("  WASD Controller - Keyboard Controls")
        print("="*50)
        print("\n  W - Walk Forward")
        print("  S - Walk Backward")
        print("  A - Turn Left (Counterclockwise)")
        print("  D - Turn Right (Clockwise)")
        print("  SPACE - Stand Pose")
        print("  ESC - Quit")
        print("\n" + "="*50)
        print("Press and hold keys to move the robot")
        print("="*50 + "\n")
        
        # Start listener
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()


def main(args=None):
    rclpy.init(args=args)
    controller = WASDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.running = False
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
