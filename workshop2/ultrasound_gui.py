import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasoundNode(Node):
    def __init__(self):
        super().__init__('ultrasound_gui_node')
        self.range_data = None
        self.subscription = self.create_subscription(
            Range,
            '/ultrasound',  # เปลี่ยนชื่อ topic ตามต้องการ
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.range_data = msg.range

class UltrasoundWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("ROS2 Tutor1: Ultrasound Range Viewer")
        self.resize(400,300)
        self.ros_node = ros_node

        self.label = QLabel("Waiting for Topic data...", self)
        self.label.setStyleSheet("font-size: 24px;")

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_label)
        self.timer.start(100)  # อัปเดตทุก 100ms

    def update_label(self):
        if self.ros_node.range_data is not None:
            self.label.setText(f"Distance: {self.ros_node.range_data:.2f} m")
        else:
            self.label.setText("Waiting for data...")

def main(args=None):
    rclpy.init(args=args)

    ros_node = UltrasoundNode()

    app = QApplication(sys.argv)
    window = UltrasoundWindow(ros_node)
    window.show()

    # ใช้ QTimer เพื่อหมุน rclpy.spin_once
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)  # เรียก ROS2 ทุก 10ms

    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

