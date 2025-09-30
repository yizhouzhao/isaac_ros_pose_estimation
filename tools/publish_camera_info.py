import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class TestCameraPublisher(Node):
    def __init__(self, frame_rate=2.0):
        super().__init__('test_camera_publisher')
        self.bridge = CvBridge()

        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/rgb/image_rect_color2', 10)
        self.depth_pub = self.create_publisher(Image, '/depth_image2', 10)
        self.seg_pub = self.create_publisher(Image, '/segmentation2', 10)
        self.cinfo_pub = self.create_publisher(CameraInfo, '/rgb/camera_info2', 10)

        # Load images
        self.rgb_image = cv2.imread('rgb.png', cv2.IMREAD_COLOR)  # uint8 BGR
        depth_mm = cv2.imread('depth.png', cv2.IMREAD_UNCHANGED)   # uint16 in mm
        self.seg_image = cv2.imread('segmentation.png', cv2.IMREAD_GRAYSCALE)  # 8-bit


        if self.rgb_image is None:
            self.get_logger().error("Failed to load rgb.png")
            return
        if depth_mm is None:
            self.get_logger().error("Failed to load depth.png")
            return

        # Convert depth from mm → meters float32
        self.depth_image = (depth_mm.astype(np.float32) / 1000.0)
        

        # Camera info (fixed)
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "tf_camera"
        self.camera_info_msg.height = 480
        self.camera_info_msg.width = 640
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.k = [
            319.5820007324219, 0.0, 320.21498476769557,
            0.0, 417.1186828613281, 244.34866808710467,
            0.0, 0.0, 1.0
        ]
        self.camera_info_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        self.camera_info_msg.p = [
            319.5820007324219, 0.0, 320.21498476769557, 0.0,
            0.0, 417.1186828613281, 244.34866808710467, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        self.camera_info_msg.binning_x = 0
        self.camera_info_msg.binning_y = 0
        self.camera_info_msg.roi.x_offset = 0
        self.camera_info_msg.roi.y_offset = 0
        self.camera_info_msg.roi.height = 0
        self.camera_info_msg.roi.width = 0
        self.camera_info_msg.roi.do_rectify = False

        # Timer
        self.timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Update headers
        stamp = self.get_clock().now().to_msg()

        # RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(self.rgb_image, encoding='bgr8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = "tf_camera"
        self.rgb_pub.publish(rgb_msg)

        # Depth
        depth_msg = self.bridge.cv2_to_imgmsg(self.depth_image, encoding='32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "tf_camera"
        self.depth_pub.publish(depth_msg)

        # Segmentation
        seg_msg = self.bridge.cv2_to_imgmsg(self.seg_image, encoding='mono8')
        seg_msg.header.stamp = stamp
        seg_msg.header.frame_id = "tf_camera"
        self.seg_pub.publish(seg_msg)

        # CameraInfo
        self.camera_info_msg.header.stamp = stamp
        self.cinfo_pub.publish(self.camera_info_msg)

        self.get_logger().info(f"Published frame {self.counter}")
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestCameraPublisher(frame_rate=2.0)  # adjust frame rate here
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
