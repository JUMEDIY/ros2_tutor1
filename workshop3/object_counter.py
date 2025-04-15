import sys
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasoundNode(Node):
    def __init__(self):
        super().__init__('ultrasound_counter_node')
        self.range_data = None
        self.subscription = self.create_subscription(
            Range,
            '/ultrasound',  # ชื่อ topic ที่ subscribe
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.range_data = msg.range

class UltrasoundWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Ultrasound Object Counter")
        self.resize(400, 300)  # ขนาดหน้าต่าง

        self.ros_node = ros_node
        self.object_count = 0
        self.object_in_range = False  # ใช้สำหรับตรวจจับ transition (เข้า -> ออก)

        # GUI Labels
        self.label_distance = QLabel("Waiting for data...", self)
        self.label_count = QLabel("Object Count: 0", self)

        self.label_distance.setStyleSheet("font-size: 24px;")
        self.label_count.setStyleSheet("font-size: 24px; color: green;")

        layout = QVBoxLayout()
        layout.addWidget(self.label_distance)
        layout.addWidget(self.label_count)
        self.setLayout(layout)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_label)
        self.timer.start(100)  # อัปเดตทุก 100ms

    def update_label(self):
        if self.ros_node.range_data is not None:
            distance = self.ros_node.range_data
            self.label_distance.setText(f"Distance: {distance:.2f} m")

            if distance < 0.3:  # < 30 cm
                if not self.object_in_range:
                    self.object_count += 1
                    self.object_in_range = True
                    self.label_count.setText(f"Object Count: {self.object_count}")
            else:
                self.object_in_range = False
        else:
            self.label_distance.setText("Waiting for data...")

def main(args=None):
    rclpy.init(args=args)

    ros_node = UltrasoundNode()

    app = QApplication(sys.argv)
    window = UltrasoundWindow(ros_node)
    window.show()

    # หมุน ROS2
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
