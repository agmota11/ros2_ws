from setuptools import setup

package_name = "custom_yolo_v8"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="agmota",
    maintainer_email="agmota@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "custom_yolo_v8 = custom_yolo_v8.custom_yolo_v8:main",
            "video_publisher = custom_yolo_v8.VideoPublisher:main",
            "cameras_debug = custom_yolo_v8.cameras_debug:main",
            "yolov8_personas = custom_yolo_v8.yolov8_personas:main",
        ],
    },
)
