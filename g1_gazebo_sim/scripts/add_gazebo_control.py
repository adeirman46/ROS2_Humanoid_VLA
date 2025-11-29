#!/usr/bin/env python3

"""
Script to add ROS2 control (gazebo_ros2_control) to the G1 URDF
WITHOUT xml declaration (causes lxml issues)
"""

import xml.etree.ElementTree as ET

def add_ros2_control_to_urdf(input_file, output_file):
    """Add ros2_control and transmissions to URDF"""
    
    # Parse the URDF
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # List of all controllable joints
    controlled_joints = [
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
    
    # Create ros2_control tag
    ros2_control = ET.SubElement(root, 'ros2_control', name='GazeboSystem', type='system')
    
    # Add hardware
    hardware = ET.SubElement(ros2_control, 'hardware')
    plugin = ET.SubElement(hardware, 'plugin')
    plugin.text = 'gazebo_ros2_control/GazeboSystem'
    
    # Add each joint with proper initial values for standing
    initial_positions = {
        'left_hip_pitch_joint': -0.15,
        'left_knee_joint': 0.3,
        'left_ankle_pitch_joint': -0.15,
        'right_hip_pitch_joint': -0.15,
        'right_knee_joint': 0.3,
        'right_ankle_pitch_joint': -0.15,
        'left_shoulder_pitch_joint': 0.2,
        'left_shoulder_roll_joint': 0.1,
        'left_elbow_joint': 0.4,
        'right_shoulder_pitch_joint': 0.2,
        'right_shoulder_roll_joint': -0.1,
        'right_elbow_joint': 0.4,
    }
    
    for joint_name in controlled_joints:
        joint_elem = ET.SubElement(ros2_control, 'joint', name=joint_name)
        
        # Command interface (position control) with initial value
        command_interface = ET.SubElement(joint_elem, 'command_interface', name='position')
        initial_value = ET.SubElement(command_interface, 'param', name='initial_value')
        initial_value.text = str(initial_positions.get(joint_name, 0.0))
        
        # State interfaces
        state_position = ET.SubElement(joint_elem, 'state_interface', name='position')
        ET.SubElement(state_position, 'param', name='initial_value').text = str(initial_positions.get(joint_name, 0.0))
        state_velocity = ET.SubElement(joint_elem, 'state_interface', name='velocity')
        state_effort = ET.SubElement(joint_elem, 'state_interface', name='effort')
    
    # Add Gazebo plugin for ros2_control
    gazebo_elem = ET.SubElement(root, 'gazebo')
    gazebo_plugin = ET.SubElement(gazebo_elem, 'plugin',
                                  filename='libgazebo_ros2_control.so',
                                  name='gazebo_ros2_control')
    
    # Robot description parameter
    parameters = ET.SubElement(gazebo_plugin, 'parameters')
    parameters.text = '$(find g1_controller)/config/g1_controllers.yaml'
    
    # Write the modified URDF WITHOUT xml declaration
    tree.write(output_file, encoding='unicode', xml_declaration=False)
    print(f"✓ Added ros2_control to {output_file}")

if __name__ == '__main__':
    input_urdf = '/home/irman/ROS2_Humanoid_VLA/g1_package/urdf/g1_29dof.urdf'
    output_urdf = '/home/irman/ROS2_Humanoid_VLA/g1_package/urdf/g1_29dof_ros2_control.urdf'
    
    add_ros2_control_to_urdf(input_urdf, output_urdf)
    print(f"✓ Created ROS2 control URDF (no XML declaration): {output_urdf}")
