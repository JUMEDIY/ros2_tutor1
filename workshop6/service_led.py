import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QLabel
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ESP32ServiceClient(Node):
    def __init__(self):
        super().__init__('esp32_service_client')
        self.cli = self.create_client(SetBool, 'service_onoff_led')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = SetBool.Request()

    def send_request(self, data=True):
        self.req.data = data
        future = self.cli.call_async(self.req)
        return future

class ESP32GUI(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("ESP32 Connect GUI")
        self.setFixedSize(400, 400)

        self.label = QLabel("Click to Connect ESP32", self)
        self.buttonON = QPushButton("TURN LED ON", self)
        self.buttonON.clicked.connect(self.on_btn_click)
        
        self.buttonOFF = QPushButton("TURN LED OFF", self)
        self.buttonOFF.clicked.connect(self.off_btn_click)

        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.buttonON)
        layout.addWidget(self.buttonOFF)
        self.setLayout(layout)

    def on_btn_click(self):
        future = self.ros_node.send_request(True)
        future.add_done_callback(self.handle_response)
    
    def off_btn_click(self):
        future = self.ros_node.send_request(False)
        future.add_done_callback(self.handle_response)    

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.label.setText("Connected: " + response.message)
            else:
                self.label.setText("Failed: " + response.message)
        except Exception as e:
            self.label.setText(f"Service call failed: {str(e)}")

def main():
    rclpy.init()
    ros_node = ESP32ServiceClient()

    app = QApplication(sys.argv)
    gui = ESP32GUI(ros_node)
    gui.show()

    timer = ros_node.create_timer(0.1, lambda: None)

    def spin_ros():
        rclpy.spin_once(ros_node, timeout_sec=0.01)

    qt_timer = gui.startTimer(10)
    gui.timerEvent = lambda event: spin_ros()

    sys.exit(app.exec_())

    gui.killTimer(qt_timer)
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

