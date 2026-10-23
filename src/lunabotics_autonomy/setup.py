from setuptools import setup

setup(
    name='lunabotics_autonomy',
    version='0.2.2',
    packages=['autonomy'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lunabotics_autonomy']),
        ('share/lunabotics_autonomy', ['package.xml', 'README.md']),
        ('share/lunabotics_autonomy/launch', ['launch/autonomy.launch.py']),
        ('share/lunabotics_autonomy/launch', ['launch/graph_sitl.launch.py']),
    ],
    entry_points={'console_scripts': [
        'mission_node = autonomy.ros_mission:main',
        'localization_node = autonomy.ros_localization:main',
        'graph_sitl_node = autonomy.ros_graph_sitl:main',
        'graph_acceptance = autonomy.ros_graph_acceptance:main',
        'fiducial_node = autonomy.ros_fiducials:main',
        'berm_node = autonomy.ros_berm:main',
    ]},
)
