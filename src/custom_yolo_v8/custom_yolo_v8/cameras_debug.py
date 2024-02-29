#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cameras_msgs.msg import ImageWithBoundingBox
from cameras_msgs.msg import BoundingBox
import cv2
from cv_bridge import CvBridge
from custom_msgs.msg import ConeArray


class CamerasDebugNode(Node):
    """
    Nodo encargado de recibir una imagen, usar el modelo para generar
    las bounding boxes y enviarsela al nodo de camaras.

    Args:
        Node (Node): Ros2 node
    """

    def __init__(self):
        super().__init__("cameras_debug")
        self.get_logger().info(f"Starting cameras_debug...")

        """
        Subscriptor de las bounding boxes generadas por el modelo.
        """
        self.bounding_box_subscriber = self.create_subscription(
            msg_type=ImageWithBoundingBox,
            topic="/yolov8/results",
            callback=self.yolo_results_callback,
            qos_profile=1,
        )

        """
        Subscriptor de las bounding boxes de personas generadas por el modelo.
        """
        self.bounding_box_subscriber = self.create_subscription(
            msg_type=ImageWithBoundingBox,
            topic="/yolov8/results/personas",
            callback=self.yolo_personas_callback,
            qos_profile=1,
        )

        self.bounding_box_subscriber = self.create_subscription(
            msg_type=ConeArray,
            topic="/CamConeList",
            callback=self.cone_list_callback,
            qos_profile=1,
        )

        self.bounding_boxes = []
        self.image = None
        self.bridge = CvBridge()
        self.cone_list = []
        self.bounding_boxes_personas = []
        # self.loop()
        # self.create_timer(0.1, self.draw_image)
        self.colors = {
            "orange_cone": (244, 88, 4),
            "large_orange_cone": (255, 0, 0),
            "blue_cone": (50, 200, 180),
            "yellow_cone": (240, 240, 0),
            "person": (0, 0, 0),
        }

    def cone_list_callback(self, msg: ConeArray):
        self.cone_list = msg.cones
        # self.get_logger().info(f"Number of cones {msg.cone_number}")

    def yolo_results_callback(self, msg: ImageWithBoundingBox):
        self.bounding_boxes = msg.bounding_boxes
        self.image = self.bridge.imgmsg_to_cv2(msg.rgb_image, desired_encoding="rgb8")
        self.draw_image()

    def yolo_personas_callback(self, msg: ImageWithBoundingBox):
        self.bounding_boxes_personas = msg.bounding_boxes
        self.get_logger().info(f"Drawing {len(self.bounding_boxes_personas)} people")

    def draw_image(self):
        if self.image is None:
            return

        img = self.image
        for bb in self.bounding_boxes:
            self.draw_rect(img, bb)

        for bb in self.bounding_boxes_personas:
            self.draw_rect(img, bb)

        self.draw_cones_dist(img)

        # Display the image using OpenCV
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.resize(img, (1920 // 2, 1080 // 2))
        cv2.imshow("ROS2 Image Display", img)
        cv2.waitKey(1)

    def draw_rect(self, image, box):
        top_left = (box.xmin, box.ymin)
        bottom_right = (box.xmax, box.ymax)

        cv2.rectangle(image, top_left, bottom_right, self.colors[box.class_id], 1)
        self.draw_center_point(image, box)

    def draw_center_point(self, image, box):
        x = (box.xmax - box.xmin) // 2 + box.xmin
        y = (box.ymax - box.ymin) // 2 + box.ymin
        cv2.circle(image, (x, y), 1, (0, 255, 0), cv2.FILLED)

    def draw_cones_dist(self, img):
        x, y = 10, 10
        cv2.rectangle(
            img,
            (0, 0),
            (100, 20 * (len(self.cone_list) + 1)),
            (250, 250, 250),
            cv2.FILLED,
        )
        for cone in self.cone_list:
            text = f"({cone.x_pos:.2f}, {cone.y_pos:.2f}, {cone.z_pos:.2f})"
            cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 0, 0))
            cv2.rectangle(
                img,
                (3, y - 5),
                (6, y),
                self.colors[cone.color],
                cv2.FILLED,
            )
            y += 20
        cv2.putText(
            img,
            f"Total {len(self.cone_list)}",
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.25,
            (0, 0, 0),
        )


def main(args=None):
    rclpy.init(args=args)
    node = CamerasDebugNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
