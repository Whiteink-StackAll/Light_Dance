from setuptools import setup
import os

package_name = 'light_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python'],
    zip_safe=False,  # 关键：禁用zip安全模式，确保生成实体文件
    maintainer='whiteink',
    maintainer_email='whiteink@example.com',
    description='YOLO-based light detection for ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # 明确指定模块路径
            'detector_node = light_detector.detector_node:main',
        ],
    },
)