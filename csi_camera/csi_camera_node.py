import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from jetcam.csi_camera import CSICamera
import numpy as np

class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')

        self.declare_parameter('output_topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('get_frame_freq_hz', 30.0)
        self.declare_parameter('publish_freq_hz', 3.0)
        self.declare_parameter('verbose', 1)

        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.get_frame_freq_hz = self.get_parameter('get_frame_freq_hz').get_parameter_value().double_value
        self.publish_freq_hz = self.get_parameter('publish_freq_hz').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value

        self.camera = CSICamera(width=self.width, height=self.height, capture_fps=self.get_frame_freq_hz)
        self.camera.running = True

        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.get_frame_timer = self.create_timer(1.0/self.get_frame_freq_hz, self._get_frame_callback)
        self.pub_timer = self.create_timer(1.0/self.publish_freq_hz, self._publish_callback)

        self.frame = None

        if self.verbose >= 1:
            self.get_logger().info(f'CSI-Camera frame shape (h, w, c): ({self.height}, {self.width}, 3)')

    def _get_frame_callback(self):
        self.frame = self.camera.read()
        self.frame = np.random.rand(self.height, self.width, 3).astype(np.float32)
        self.frame = np.clip(self.frame * 255.0, 0, 255).astype(np.uint8)

        if self.verbose >= 2:
            self.get_logger().info(f'Frame recieved')

    def _publish_callback(self):
        if self.frame is None:
            return
        self.get_logger().info(self.frame)
        # self.pub.publish(msg)

        if self.verbose >= 2:
            self.get_logger().info(f'Frame sent')

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down (Ctrl-C)')
    finally:
        try:
            node.camera.running = False
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
