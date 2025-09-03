import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation as R

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QGroupBox, QTabWidget,
                             QPushButton, QGridLayout, QDoubleSpinBox, QFrame,
                             QSizePolicy)
from PyQt5.QtCore import Qt, QTimer
import math

class PandaControlNode(Node):
    def __init__(self):
        super().__init__('panda_control_node')
        self.left_publisher = self.create_publisher(JointState, '/Left/joint_command', 10)
        self.right_publisher = self.create_publisher(JointState, '/right/joint_command', 10)
        
        # Joint limits for Panda arm (in radians)
        self.joint_limits = [
            (-2.8973, 2.8973),   # joint1
            (-1.7628, 1.7628),   # joint2
            (-2.8973, 2.8973),   # joint3
            (-3.0718, -0.0698),  # joint4
            (-2.8973, 2.8973),   # joint5
            (-0.0175, 3.7525),   # joint6
            (-2.8973, 2.8973),   # joint7
            (0.0, 0.04)          # finger_joint (in meters)
        ]
        
        # DH parameters for Panda arm
        self.dh_params = [
            [0.0, 0.333, 0.0, 0.0],
            [0.0, 0.0, 0.0, -math.pi/2],
            [0.0, 0.316, 0.0, math.pi/2],
            [0.0825, 0.0, 0.0, math.pi/2],
            [-0.0825, 0.384, 0.0, -math.pi/2],
            [0.0, 0.0, 0.0, math.pi/2],
            [0.088, 0.0, 0.0, math.pi/2]
        ]
        
        # Forward kinematics transformation matrices
        self.T_base_ee = np.eye(4)  # Base to end effector
        
    def publish_left_joints(self, positions):
        msg = JointState()
        msg.name = [f'panda_joint{i+1}' for i in range(7)] + ['panda_finger_joint1']
        msg.position = positions
        self.left_publisher.publish(msg)
        
    def publish_right_joints(self, positions):
        msg = JointState()
        msg.name = [f'panda_joint{i+1}' for i in range(7)] + ['panda_finger_joint1']
        msg.position = positions
        self.right_publisher.publish(msg)
    
    def forward_kinematics(self, joints):
        """Calculate forward kinematics for given joint angles"""
        T = np.eye(4)
        
        for i in range(7):
            a, d, alpha, theta_offset = self.dh_params[i]
            theta = joints[i] + theta_offset
            
            # DH transformation matrix
            T_i = np.array([
                [math.cos(theta), -math.sin(theta)*math.cos(alpha), math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
                [math.sin(theta), math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
                [0, math.sin(alpha), math.cos(alpha), d],
                [0, 0, 0, 1]
            ])
            
            T = np.dot(T, T_i)
        
        self.T_base_ee = T
        return T
    
    def inverse_kinematics(self, target_pose, initial_joints):
        """Solve inverse kinematics using optimization"""
        def cost_function(joints):
            current_pose = self.forward_kinematics(joints)
            
            # Position error
            pos_error = np.linalg.norm(current_pose[:3, 3] - target_pose[:3, 3])
            
            # Orientation error (using quaternion difference)
            R_current = current_pose[:3, :3]
            R_target = target_pose[:3, :3]
            
            # Convert to quaternions and calculate difference
            q_current = R.from_matrix(R_current).as_quat()
            q_target = R.from_matrix(R_target).as_quat()
            
            # Ensure consistent quaternion signs
            if np.dot(q_current, q_target) < 0:
                q_current = -q_current
            
            # Quaternion dot product (cosine of half the angle)
            dot_product = np.dot(q_current, q_target)
            orient_error = 1.0 - abs(dot_product)
            
            # Joint limit penalty
            joint_penalty = 0.0
            for i, joint in enumerate(joints):
                if joint < self.joint_limits[i][0]:
                    joint_penalty += 100.0 * (self.joint_limits[i][0] - joint)**2
                elif joint > self.joint_limits[i][1]:
                    joint_penalty += 100.0 * (joint - self.joint_limits[i][1])**2
            
            return pos_error * 10.0 + orient_error * 2.0 + joint_penalty
        
        # Solve using optimization
        result = minimize(cost_function, initial_joints, method='SLSQP', 
                         bounds=self.joint_limits[:7],
                         options={'maxiter': 200, 'ftol': 1e-6, 'disp': False})
        
        if result.success:
            return result.x
        else:
            # Fallback: use the initial joints if IK fails
            self.get_logger().warn(f"IK failed: {result.message}")
            return initial_joints

class PoseSlider(QWidget):
    def __init__(self, label, min_val, max_val, default_val, units, precision=3):
        super().__init__()
        self.value = default_val
        
        layout = QVBoxLayout()
        
        # Label showing current value
        self.value_label = QLabel(f"{label}: {default_val:.{precision}f} {units}")
        layout.addWidget(self.value_label)
        
        # Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setValue(self.value_to_slider(default_val, min_val, max_val))
        self.slider.valueChanged.connect(lambda value: self.slider_changed(value, min_val, max_val, units, precision))
        layout.addWidget(self.slider)
        
        # Min/Max labels
        range_layout = QHBoxLayout()
        range_layout.addWidget(QLabel(f"{min_val:.{precision}f}"))
        range_layout.addStretch()
        range_layout.addWidget(QLabel(f"{max_val:.{precision}f}"))
        layout.addLayout(range_layout)
        
        self.setLayout(layout)
    
    def value_to_slider(self, value, min_val, max_val):
        return int((value - min_val) / (max_val - min_val) * 1000)
    
    def slider_to_value(self, slider_pos, min_val, max_val):
        return min_val + (slider_pos / 1000.0) * (max_val - min_val)
    
    def slider_changed(self, value, min_val, max_val, units, precision):
        self.value = self.slider_to_value(value, min_val, max_val)
        label_text = self.value_label.text().split(":")[0]  # Keep the label part
        self.value_label.setText(f"{label_text}: {self.value:.{precision}f} {units}")

class IKControlWidget(QWidget):
    def __init__(self, arm_name, node, joint_sliders):
        super().__init__()
        self.arm_name = arm_name
        self.node = node
        self.joint_sliders = joint_sliders
        self.ik_enabled = True
        self.updating_from_ik = False
        
        layout = QVBoxLayout()
        
        # Position sliders
        pos_group = QGroupBox("Position (meters)")
        pos_layout = QGridLayout()
        
        self.x_slider = PoseSlider("X", -1.0, 1.0, 0.5, "m")
        self.y_slider = PoseSlider("Y", -1.0, 1.0, 0.0, "m")
        self.z_slider = PoseSlider("Z", 0.1, 1.0, 0.5, "m")
        
        pos_layout.addWidget(self.x_slider, 0, 0)
        pos_layout.addWidget(self.y_slider, 1, 0)
        pos_layout.addWidget(self.z_slider, 2, 0)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        # Orientation sliders
        orient_group = QGroupBox("Orientation (degrees)")
        orient_layout = QGridLayout()
        
        self.roll_slider = PoseSlider("Roll", -180, 180, 0, "°", 1)
        self.pitch_slider = PoseSlider("Pitch", -180, 180, 0, "°", 1)
        self.yaw_slider = PoseSlider("Yaw", -180, 180, 0, "°", 1)
        
        orient_layout.addWidget(self.roll_slider, 0, 0)
        orient_layout.addWidget(self.pitch_slider, 1, 0)
        orient_layout.addWidget(self.yaw_slider, 2, 0)
        
        orient_group.setLayout(orient_layout)
        layout.addWidget(orient_group)
        
        # Connect slider changes to IK solving
        for slider in [self.x_slider, self.y_slider, self.z_slider, 
                      self.roll_slider, self.pitch_slider, self.yaw_slider]:
            slider.slider.valueChanged.connect(self.solve_ik)
        
        # Current pose display
        current_group = QGroupBox("Current End Effector Pose")
        current_layout = QVBoxLayout()
        self.pose_label = QLabel("Calculating...")
        self.pose_label.setWordWrap(True)
        current_layout.addWidget(self.pose_label)
        current_group.setLayout(current_layout)
        layout.addWidget(current_group)
        
        # IK control buttons
        button_layout = QHBoxLayout()
        
        self.ik_toggle = QPushButton("Disable IK")
        self.ik_toggle.setCheckable(True)
        self.ik_toggle.clicked.connect(self.toggle_ik)
        button_layout.addWidget(self.ik_toggle)
        
        reset_btn = QPushButton("Reset to Current")
        reset_btn.clicked.connect(self.reset_to_current)
        button_layout.addWidget(reset_btn)
        
        layout.addLayout(button_layout)
        
        self.setLayout(layout)
        
        # Timer to update current pose display
        self.pose_timer = QTimer()
        self.pose_timer.timeout.connect(self.update_current_pose_display)
        self.pose_timer.start(100)  # Update every 100ms
    
    def toggle_ik(self, checked):
        self.ik_enabled = not checked
        self.ik_toggle.setText("Enable IK" if checked else "Disable IK")
    
    def reset_to_current(self):
        # Get current joint positions
        current_joints = [slider.value for slider in self.joint_sliders[:7]]
        
        # Calculate forward kinematics
        current_pose = self.node.forward_kinematics(current_joints)
        
        # Extract position
        position = current_pose[:3, 3]
        
        # Extract orientation (convert to Euler angles)
        rotation = R.from_matrix(current_pose[:3, :3])
        euler_angles = rotation.as_euler('xyz', degrees=True)
        
        # Update sliders without triggering IK
        self.updating_from_ik = True
        
        self.x_slider.slider.setValue(self.x_slider.value_to_slider(position[0], -1.0, 1.0))
        self.y_slider.slider.setValue(self.y_slider.value_to_slider(position[1], -1.0, 1.0))
        self.z_slider.slider.setValue(self.z_slider.value_to_slider(position[2], 0.1, 1.0))
        
        self.roll_slider.slider.setValue(self.roll_slider.value_to_slider(euler_angles[0], -180, 180))
        self.pitch_slider.slider.setValue(self.pitch_slider.value_to_slider(euler_angles[1], -180, 180))
        self.yaw_slider.slider.setValue(self.yaw_slider.value_to_slider(euler_angles[2], -180, 180))
        
        self.updating_from_ik = False
    
    def solve_ik(self):
        if not self.ik_enabled or self.updating_from_ik:
            return
        
        # Get target position
        target_pos = np.array([
            self.x_slider.value,
            self.y_slider.value,
            self.z_slider.value
        ])
        
        # Get target orientation (convert from degrees to radians)
        roll = math.radians(self.roll_slider.value)
        pitch = math.radians(self.pitch_slider.value)
        yaw = math.radians(self.yaw_slider.value)
        
        # Create rotation matrix from Euler angles (XYZ convention)
        rotation = R.from_euler('xyz', [roll, pitch, yaw])
        R_target = rotation.as_matrix()
        
        # Create target pose matrix
        target_pose = np.eye(4)
        target_pose[:3, :3] = R_target
        target_pose[:3, 3] = target_pos
        
        # Get current joint positions as initial guess
        initial_joints = [slider.value for slider in self.joint_sliders[:7]]
        
        # Solve inverse kinematics
        solved_joints = self.node.inverse_kinematics(target_pose, initial_joints)
        
        # Update joint sliders
        for i, joint_value in enumerate(solved_joints):
            self.joint_sliders[i].slider.setValue(
                self.joint_sliders[i].value_to_slider(joint_value)
            )
    
    def update_current_pose_display(self):
        # Get current joint positions
        current_joints = [slider.value for slider in self.joint_sliders[:7]]
        
        # Calculate forward kinematics
        current_pose = self.node.forward_kinematics(current_joints)
        
        # Extract position
        position = current_pose[:3, 3]
        
        # Extract orientation (convert to Euler angles)
        rotation = R.from_matrix(current_pose[:3, :3])
        euler_angles = rotation.as_euler('xyz', degrees=True)
        
        # Update display
        self.pose_label.setText(
            f"Position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] m\n"
            f"Orientation: [{euler_angles[0]:.1f}, {euler_angles[1]:.1f}, {euler_angles[2]:.1f}]°"
        )

class JointSlider(QWidget):
    def __init__(self, joint_id, joint_limits, initial_value=0.0, is_finger=False):
        super().__init__()
        self.joint_id = joint_id
        self.joint_limits = joint_limits
        self.value = initial_value
        self.is_finger = is_finger
        
        layout = QVBoxLayout()
        
        # Joint label
        unit = "m" if is_finger else "rad"
        joint_name = f"Finger Joint" if is_finger else f"Joint {joint_id+1}"
        self.label = QLabel(f"{joint_name}: {initial_value:.3f} {unit}")
        layout.addWidget(self.label)
        
        # Slider
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 1000)
        self.slider.setValue(self.value_to_slider(initial_value))
        self.slider.valueChanged.connect(self.slider_changed)
        layout.addWidget(self.slider)
        
        # Min/Max labels
        range_layout = QHBoxLayout()
        range_layout.addWidget(QLabel(f"{joint_limits[0]:.2f}"))
        range_layout.addStretch()
        range_layout.addWidget(QLabel(f"{joint_limits[1]:.2f}"))
        layout.addLayout(range_layout)
        
        self.setLayout(layout)
    
    def value_to_slider(self, value):
        min_val, max_val = self.joint_limits
        return int((value - min_val) / (max_val - min_val) * 1000)
    
    def slider_to_value(self, slider_pos):
        min_val, max_val = self.joint_limits
        return min_val + (slider_pos / 1000.0) * (max_val - min_val)
    
    def slider_changed(self, value):
        self.value = self.slider_to_value(value)
        unit = "m" if self.is_finger else "rad"
        joint_name = f"Finger Joint" if self.is_finger else f"Joint {self.joint_id+1}"
        self.label.setText(f"{joint_name}: {self.value:.3f} {unit}")

class ArmControlWidget(QWidget):
    def __init__(self, arm_name, joint_limits, node, publisher_callback):
        super().__init__()
        self.arm_name = arm_name
        self.joint_limits = joint_limits
        self.node = node
        self.publisher_callback = publisher_callback
        
        # Initial joint positions
        self.initial_positions = [0.012, -0.5686, 0.0, -2.8106, 0.0, 3.0367, 0.741, 0.0]
        self.joint_sliders = []
        
        # Create main layout with tabs
        main_tabs = QTabWidget()
        
        # Direct joint control tab
        joint_tab = QWidget()
        joint_layout = QHBoxLayout()
        
        # Left side: Arm joints
        arm_group = QGroupBox("Arm Joints")
        arm_layout = QVBoxLayout()
        
        for i in range(7):
            slider = JointSlider(i, joint_limits[i], self.initial_positions[i])
            slider.slider.valueChanged.connect(self.update_joints)
            self.joint_sliders.append(slider)
            arm_layout.addWidget(slider)
        
        arm_group.setLayout(arm_layout)
        
        # Right side: Finger joint and preset buttons
        right_group = QGroupBox("Gripper Control")
        right_layout = QVBoxLayout()
        
        # Finger joint slider
        finger_slider = JointSlider(7, joint_limits[7], self.initial_positions[7], is_finger=True)
        finger_slider.slider.valueChanged.connect(self.update_joints)
        self.joint_sliders.append(finger_slider)
        right_layout.addWidget(finger_slider)
        
        # Preset buttons
        presets_group = QGroupBox("Preset Poses")
        presets_layout = QGridLayout()
        
        home_btn = QPushButton("Home Position")
        home_btn.clicked.connect(lambda: self.set_preset([0.012, -0.5686, 0.0, -2.8106, 0.0, 3.0367, 0.741, 0.0]))
        presets_layout.addWidget(home_btn, 0, 0)
        
        extended_btn = QPushButton("Extended Position")
        extended_btn.clicked.connect(lambda: self.set_preset([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78, 0.0]))
        presets_layout.addWidget(extended_btn, 0, 1)
        
        open_btn = QPushButton("Open Gripper")
        open_btn.clicked.connect(lambda: self.set_gripper(0.04))
        presets_layout.addWidget(open_btn, 1, 0)
        
        close_btn = QPushButton("Close Gripper")
        close_btn.clicked.connect(lambda: self.set_gripper(0.0))
        presets_layout.addWidget(close_btn, 1, 1)
        
        presets_group.setLayout(presets_layout)
        right_layout.addWidget(presets_group)
        
        right_group.setLayout(right_layout)
        
        joint_layout.addWidget(arm_group, 3)
        joint_layout.addWidget(right_group, 1)
        joint_tab.setLayout(joint_layout)
        
        # Inverse kinematics tab
        ik_tab = IKControlWidget(arm_name, node, self.joint_sliders)
        
        # Add tabs
        main_tabs.addTab(joint_tab, "Joint Control")
        main_tabs.addTab(ik_tab, "Inverse Kinematics")
        
        main_layout = QVBoxLayout()
        main_layout.addWidget(main_tabs)
        self.setLayout(main_layout)
    
    def update_joints(self):
        positions = [slider.value for slider in self.joint_sliders]
        self.publisher_callback(positions)
    
    def set_preset(self, positions):
        for i, position in enumerate(positions):
            self.joint_sliders[i].slider.setValue(
                self.joint_sliders[i].value_to_slider(position)
            )
    
    def set_gripper(self, position):
        self.joint_sliders[7].slider.setValue(
            self.joint_sliders[7].value_to_slider(position)
        )

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Dual Panda Arm Controller with IK")
        self.setGeometry(100, 100, 1200, 800)
        
        # Create tab widget
        tabs = QTabWidget()
        
        # Left arm tab
        left_widget = ArmControlWidget(
            "Left Arm", 
            node.joint_limits, 
            node,
            node.publish_left_joints
        )
        tabs.addTab(left_widget, "Left Arm")
        
        # Right arm tab
        right_widget = ArmControlWidget(
            "Right Arm", 
            node.joint_limits, 
            node,
            node.publish_right_joints
        )
        tabs.addTab(right_widget, "Right Arm")
        
        self.setCentralWidget(tabs)

def main():
    rclpy.init()
    node = PandaControlNode()
    
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    
    result = app.exec_()
    
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)

if __name__ == '__main__':
    main()