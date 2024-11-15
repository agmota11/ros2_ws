import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoPublisher(Node):
    """
    Este nodo imita el comportamiento que tendrían las camaras (solo la imagen).
    A partir de un video publica los frames

    Args:
        Node (Node): Nodo de ros
    """

    def __init__(self):
        super().__init__("video_publisher")
        """
        Publisher de las imagenes del video.
        """
        self.publisher = self.create_publisher(
            msg_type=Image,
            topic="/zed/zed_node/rgb_raw/image_raw_color",
            qos_profile=10,
        )

        """
        Frecuencia de publicacion
        """
        self.timer = self.create_timer(0.1, self.publish_image)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture("/home/agmota/Downloads/Video-conos-demo.mp4")

    def publish_image(self):
        """
        Publica la imagen en el topicº
        """
        ret, frame = self.cap.read()

        if ret:
            # Convert the OpenCV image to a ROS Image message
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")

            self.get_logger().info("Publishing image...")
            self.publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()

    rclpy.spin(video_publisher)

    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
