from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="custom_yolo_v8",
                executable="custom_yolo_v8",
                name="custom_yolo",
            ),
            Node(
                package="custom_yolo_v8",
                executable="yolov8_personas",
                name="custom_yolo_personas",
            ),
            Node(package="cameras", executable="cam_cones", name="cam_cones"),
            Node(
                package="custom_yolo_v8",
                executable="cameras_debug",
                name="cam_debug",
            ),
            Node(
                package="custom_yolo_v8",
                executable="video_publisher",
                name="video_publisher",
            ),
        ]
    )
