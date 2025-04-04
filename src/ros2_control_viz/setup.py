from setuptools import find_packages, setup

package_name = 'ros2_control_viz'

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
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'graph_publisher = ros2_control_viz.graph_publisher:main',
            'turtle_graph_publisher = ros2_control_viz.turtle_graph_publisher:main',
            'graph_subscriber = ros2_control_viz.graph_subscriber:main',
            'web_socket_graph = ros2_control_viz.websocket_server:main',
        ],
    },
)
