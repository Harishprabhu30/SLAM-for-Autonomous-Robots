from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jetbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # shows the ekf.yaml file 
        # ('share/jetbot_control/config', ['config/ekf.yaml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ✅ install the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vgtu',
    maintainer_email='augustinas.stas@gmail.com',
    description='JetBot control package',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_pub_py = jetbot_control.cmd_vel_pub_py:main',
            'odom_with_cov_pub_py = jetbot_control.odom_with_cov_pub_py:main',
            'tf_broadcaster_sim = jetbot_control.tf_broadcaster_sim:main',
            'camera_sub_py = jetbot_control.camera_sub_py:main',
            'visual_odom_py = jetbot_control.visual_odom_py:main',
        ],
    },
)

