import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QPainter, QColor, QBrush, QFont
from PyQt5.QtCore import QTimer, Qt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32


class UltrasoundNode(Node):
    def __init__(self):
        super().__init__('ultrasound_led_node')
        self.range_data = None
        self.last_status = 0

        # Subscriber: ultrasound
        self.subscription = self.create_subscription(
            Range,
            '/ultrasound',
            self.listener_callback,
            10
        )

        # Publisher: status
        self.status_publisher = self.create_publisher(Int32, '/status', 10)

    def listener_callback(self, msg):
        self.range_data = msg.range
        status_msg = Int32()
        if msg.range < 0.15:
            status_msg.data = 1
        elif 0.15 <= msg.range < 0.25:
            status_msg.data = 2
        elif msg.range > 0.30:
            status_msg.data = 3
        else:
            status_msg.data = 0  # ไม่เข้าเงื่อนไขใดๆ

        if status_msg.data != self.last_status:
            self.status_publisher.publish(status_msg)
            self.last_status = status_msg.data


class LedDisplay(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Ultrasound LED Indicator")
        self.resize(400, 300)

        self.ros_node = ros_node
        self.led_color = QColor('gray')
        self.distance_cm = 0.0

        # Label แสดงระยะ
        self.label = QLabel("Distance: -- cm", self)
        self.label.setAlignment(Qt.AlignCenter)
        self.label.setFont(QFont("Arial", 14))

        # Layout
        self.layout = QVBoxLayout()
        self.setLayout(self.layout)
        self.layout.addStretch()
        self.layout.addWidget(self.label)
        self.layout.setSpacing(10)

        # Timer อัปเดต GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(100)

    def update_display(self):
        if self.ros_node.range_data is not None:
            distance = self.ros_node.range_data
            self.distance_cm = distance * 100.0
            self.label.setText(f"Distance: {self.distance_cm:.1f} cm")

            if distance < 0.15:
                self.led_color = QColor('red')
            elif 0.15 <= distance < 0.25:
                self.led_color = QColor('yellow')
            elif distance > 0.30:
                self.led_color = QColor('green')
            else:
                self.led_color = QColor('gray')
        else:
            self.label.setText("Distance: -- cm")
            self.led_color = QColor('gray')

        self.update()

    def paintEvent(self, event):
        qp = QPainter(self)
        qp.setRenderHint(QPainter.Antialiasing)

        center_x = self.width() // 2
        center_y = self.height() // 2 - 40
        radius = 80

        qp.setBrush(QBrush(self.led_color, Qt.SolidPattern))
        qp.setPen(Qt.NoPen)
        qp.drawEllipse(center_x - radius, center_y - radius, radius * 2, radius * 2)


def main(args=None):
    rclpy.init(args=args)

    ros_node = UltrasoundNode()

    app = QApplication(sys.argv)
    window = LedDisplay(ros_node)
    window.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    sys.exit(app.exec_())

    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
