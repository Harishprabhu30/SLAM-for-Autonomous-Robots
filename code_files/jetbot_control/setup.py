print("Develop branch version")


from setuptools import find_packages, setup
<<<<<<< HEAD
import os
from glob import glob
=======
>>>>>>> main

package_name = 'jetbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
<<<<<<< HEAD
        # shows the ekf.yaml file 
        # ('share/jetbot_control/config', ['config/ekf.yaml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # ✅ install the launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
=======
        ('share/ament_index/resource_index/packages', ['resource/jetbot_control']),
    	('share/jetbot_control/launch', ['launch/jetbot_all.launch.py']),
>>>>>>> main
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vgtu',
    maintainer_email='augustinas.stas@gmail.com',
<<<<<<< HEAD
    description='JetBot control package',
=======
    description='TODO: Package description',
>>>>>>> main
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
<<<<<<< HEAD
            'cmd_vel_pub_py = jetbot_control.cmd_vel_pub_py:main',
            'odom_with_cov_pub_py = jetbot_control.odom_with_cov_pub_py:main',
#            'camera_sub_py = jetbot_control.camera_sub_py:main',
        ],
    },
)

=======
        'cmd_vel_pub_py = jetbot_control.cmd_vel_pub_py:main',
        'odom_sub_py = jetbot_control.odom_sub_py:main',
        'camera_sub_py = jetbot_control.camera_sub_py:main',
        ],
    },
)
>>>>>>> main
