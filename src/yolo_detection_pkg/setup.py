from setuptools import setup
import os
from glob import glob

package_name = 'yolo_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # <-- this line installs launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quocthang',
    maintainer_email='quocthang@todo.todo',
    description='YOLOv8 Drone Detection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detection_node = yolo_detection_pkg.yolo_detection_node:main'
        ],
    },
)
