from setuptools import find_packages, setup

package_name = 'meshtastic_ros_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'meshtastic==2.7.3',  # pinned for compatibility
        'pypubsub'            # pip install pypubsub
    ],
    zip_safe=True,
    maintainer='Gemini',
    maintainer_email='gemini@google.com',
    description='A ROS 2 bridge for Meshtastic devices to publish node data.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_bridge_node = meshtastic_ros_bridge.tracker_bridge_node:main',
        ],
    },
)