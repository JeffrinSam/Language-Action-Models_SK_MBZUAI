import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSlider, QLabel, QGroupBox, QTabWidget,
                             QPushButton, QGridLayout)
from PyQt5.QtCore import Qt
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
        self.slider.setRange(0, 1000)  # Use integer range for smoother sliding
        self.slider.setValue(self.value_to_slider(initial_value))
        self.slider.valueChanged.connect(self.slider_changed)
        layout.addWidget(self.slider)
        
        self.setLayout(layout)
    
    def value_to_slider(self, value):
        # Convert from joint value to slider position
        min_val, max_val = self.joint_limits
        return int((value - min_val) / (max_val - min_val) * 1000)
    
    def slider_to_value(self, slider_pos):
        # Convert from slider position to joint value
        min_val, max_val = self.joint_limits
        return min_val + (slider_pos / 1000.0) * (max_val - min_val)
    
    def slider_changed(self, value):
        self.value = self.slider_to_value(value)
        unit = "m" if self.is_finger else "rad"
        joint_name = f"Finger Joint" if self.is_finger else f"Joint {self.joint_id+1}"
        self.label.setText(f"{joint_name}: {self.value:.3f} {unit}")
        return self.value

class ArmControlWidget(QWidget):
    def __init__(self, arm_name, joint_limits, node, publisher_callback):
        super().__init__()
        self.arm_name = arm_name
        self.joint_limits = joint_limits
        self.node = node
        self.publisher_callback = publisher_callback
        
        # Initial joint positions (from your example)
        self.initial_positions = [0.012, -0.5686, 0.0, -2.8106, 0.0, 3.0367, 0.741, 0.0]
        self.joint_sliders = []
        
        # Create main layout
        main_layout = QHBoxLayout()
        
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
        
        # Home position button
        home_btn = QPushButton("Home Position")
        home_btn.clicked.connect(lambda: self.set_preset([0.012, -0.5686, 0.0, -2.8106, 0.0, 3.0367, 0.741, 0.0]))
        presets_layout.addWidget(home_btn, 0, 0)
        
        # Extended position button
        extended_btn = QPushButton("Extended Position")
        extended_btn.clicked.connect(lambda: self.set_preset([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78, 0.0]))
        presets_layout.addWidget(extended_btn, 0, 1)
        
        # Gripper open button
        open_btn = QPushButton("Open Gripper")
        open_btn.clicked.connect(lambda: self.set_gripper(0.04))
        presets_layout.addWidget(open_btn, 1, 0)
        
        # Gripper close button
        close_btn = QPushButton("Close Gripper")
        close_btn.clicked.connect(lambda: self.set_gripper(0.0))
        presets_layout.addWidget(close_btn, 1, 1)
        
        presets_group.setLayout(presets_layout)
        right_layout.addWidget(presets_group)
        
        right_group.setLayout(right_layout)
        
        # Add both groups to main layout
        main_layout.addWidget(arm_group, 3)  # 3 parts for arm
        main_layout.addWidget(right_group, 1)  # 1 part for gripper
        
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
        # Set only the gripper position (joint 7)
        self.joint_sliders[7].slider.setValue(
            self.joint_sliders[7].value_to_slider(position)
        )

class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Dual Panda Arm Controller")
        self.setGeometry(100, 100, 1000, 700)
        
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
    # Initialize ROS2
    rclpy.init()
    node = PandaControlNode()
    
    # Create Qt application
    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    
    # Start the application
    result = app.exec_()
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(result)

if __name__ == '__main__':
    main()