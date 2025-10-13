from setuptools import find_packages, setup

package_name = 'jetbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vgtu',
    maintainer_email='augustinas.stas@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'cmd_vel_pub_py = jetbot_control.cmd_vel_pub_py:main',
        'odom_sub_py = jetbot_control.odom_sub_py:main',
        ],
    },
)
