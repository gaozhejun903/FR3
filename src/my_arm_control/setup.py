from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_arm_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join(os.path.dirname(__file__), 'launch', '*.launch.py'))),
        (os.path.join('lib', package_name),
            glob(os.path.join('lib', package_name, '*'))),
    ],
    install_requires=['setuptools', 'pyserial', 'tf-transformations'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='双臂控制与虚拟视觉节点包',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'virtual_vision_node = my_arm_control.virtual_vision_node:main',
            'arm_task_node = my_arm_control.arm_task_manager:main',
            'fake_vision_node = my_arm_control.fake_vision_node:main',
            'mock_robo_ctrl_node = my_arm_control.mock_robo_ctrl_node:main',
            'virtual_vision_task1 = my_arm_control.virtual_vision_task1:main',
            'task1_pour_service = my_arm_control.task1_pour_service:main',
        ],
    },
)
