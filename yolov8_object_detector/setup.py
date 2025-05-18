from setuptools import setup

package_name = 'yolov8_object_detector'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models/shapes',
         ['models/shapes/best.onnx']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Megha',
    maintainer_email='you@example.com',
    description='ROS 2 node: YOLOv8 ONNX → 3D object detection, segmentation, TF + markers + annotated image',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'object_detector = yolov8_object_detector.object_detector:main',
        ],
    },
)
