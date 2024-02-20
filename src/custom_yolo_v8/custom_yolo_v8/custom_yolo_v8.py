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


class YoloNode(Node):
    """
    Nodo encargado de recibir una imagen, usar el modelo para generar
    las bounding boxes y enviarsela al nodo de camaras.

    Args:
        Node (Node): Ros2 node
    """

    def __init__(self):
        super().__init__("Yolo_v8")

        """
        Parámetro que define la confinza con la cual el yolo
        define si crea una bounding box o no
        """
        CONF_PARAM_NAME = "conf"
        self.declare_parameter(CONF_PARAM_NAME, 0.7)
        self.conf = (
            self.get_parameter(CONF_PARAM_NAME).get_parameter_value().double_value
        )

        """
        Parámetro al path donde se encuentran los pesos del modelo.
        """
        WEIGHTS_PATH_PARAM_NAME = "weights_path"
        self.declare_parameter(
            WEIGHTS_PATH_PARAM_NAME, "/home/agmota/Downloads/best.pt"
        )
        weights_path = (
            self.get_parameter(WEIGHTS_PATH_PARAM_NAME)
            .get_parameter_value()
            .string_value
        )

        self.model = YOLO(weights_path)

        """       
        Publicador de las bounding boxes generadas por el modelo.
        """
        self.bounding_box_publisher = self.create_publisher(
            msg_type=ImageWithBoundingBox, topic="/yolov8/results", qos_profile=1
        )

        """
        Subscriptor a las imágenes enviadas por el zed wrapper de las cámaras.
        """
        self.images_subscriber = self.create_subscription(
            msg_type=Image,
            topic="/zed/zed_node/rgb_raw/image_raw_color",
            callback=self.image_received_callback,
            qos_profile=1,
        )
        self.image = None

        """
        Subscriptor a las imágenes de profundidad enviadas por el zed wrapper de las cámaras.
        """
        self.depth_subscriber = self.create_subscription(
            msg_type=Image,
            topic="/zed/zed_node/depth/depth_registered",
            callback=self.depth_received_callback,
            qos_profile=1,
        )
        self.depth_image = None
        self.get_logger().info(f"Weights path: {weights_path}, Conf: {self.conf}")
        self.bridge = CvBridge()

    def image_received_callback(self, msg: Image):
        """
        Función que se ejecuta cuando se recibe una imagen de las cámaras.

            1. Pasar de array de una dimensión a dos
            2. Hacer predicción
            3. Obtener bounding boxes
            4. Enviar bounding boxes

        Args:
            msg (Image): Imagen enviada por las cámaras
        """
        self.image = msg
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

        results = self.model.predict(img, conf=self.conf)
        bounding_boxes = self.get_bounding_boxes_from_results(results)

        self.publish_bounding_boxes(bounding_boxes)

    def depth_received_callback(self, msg: Image):
        self.depth_image = msg

    def draw_rect(self, image, box):
        top_left = (box.xmin, box.ymin)
        bottom_right = (box.xmax, box.ymax)

        cv2.rectangle(image, top_left, bottom_right, (255, 0, 0), 4)

    def get_bounding_boxes_from_results(self, results):
        """
        Obtiene las bounding boxes de los resultados de la yolo

        xmin, ymin
            +-----------------+
            |        /\       |
            |       /  \      |
            |      /    \     |
            |     /      \    |
            |    /        \   |
            |   /          \  |
            |  /    cono    \ |
            | /              \|
            +-----------------+
                            xmax, ymax

        Args:
            results (yolo prediction): Predicción de la yolo

        Returns:
            list: lista de bounding boxes
        """
        boxes = []
        for r in results:
            for box, cone_class in zip(
                r.boxes.xyxy.cpu().numpy(), r.boxes.cls.cpu().numpy()
            ):
                bounding_box = BoundingBox()
                bounding_box.id = int(cone_class)
                bounding_box.class_id = r.names[cone_class]
                bounding_box.xmin = int(box[0])
                bounding_box.ymin = int(box[1])
                bounding_box.xmax = int(box[2])
                bounding_box.ymax = int(box[3])
                boxes.append(bounding_box)
        return boxes

    def publish_bounding_boxes(self, bounding_boxes):
        """
        Publica el mensaje con la bounding boxes

        Args:
            bounding_boxes (list): lista de BoundingBox
        """
        if self.image is None or self.depth_image is None:
            return

        msg = ImageWithBoundingBox()
        msg.bounding_boxes = bounding_boxes
        msg.rgb_image = self.image
        msg.depth_image = self.depth_image

        self.get_logger().info(f"Publishing {len(bounding_boxes)} bounding boxes")
        self.bounding_box_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
