from setuptools import setup, find_packages
package_name = 'uav_signal_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Pose conversion and decentralized signal optimization controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_to_pose = uav_signal_tools.odom_to_pose:main',
            'signal_ascent_controller = uav_signal_tools.signal_ascent_controller:main',
        ],
    },
)
