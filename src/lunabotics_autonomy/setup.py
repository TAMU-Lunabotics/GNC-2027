from setuptools import setup

setup(
    name='lunabotics_autonomy',
    version='0.2.2',
    packages=['autonomy'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lunabotics_autonomy']),
        ('share/lunabotics_autonomy', ['package.xml', 'README.md']),
        ('share/lunabotics_autonomy/launch', ['launch/autonomy.launch.py']),
    ],
    entry_points={'console_scripts': [
        'mission_node = autonomy.ros_mission:main',
        'fiducial_node = autonomy.ros_fiducials:main',
        'berm_node = autonomy.ros_berm:main',
    ]},
)
