from setuptools import setup

package_name = 'my_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],  # 沒有真正的 Python package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gazebo_clean.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='wenjia',
    maintainer_email='wenjia@example.com',
    description='My Gazebo ROS2 package',
    license='MIT',
    entry_points={},
)

