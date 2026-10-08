"""Disarmed GNC bringup; GNC publishes logical commands only."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'use_sim_time':'false',
        'armed':'false', 'geometry_confirmed':'false',
        'hopper_limit_l':'0.0','bulk_density_kg_m3':'0.0',
        'target_l':'25.0','per_cycle_l':'14.0',
        'minimum_battery_v':'20.0',
        'arena_width':'7.9','arena_height':'4.4','robot_radius':'0.48',
        'dig_x':'6.9','dig_y':'2.2','dig_yaw':'0.0',
        'dump_x':'1.1','dump_y':'2.2','dump_yaw':'3.141592653589793',
        'require_depth':'false',
        'camera_name':'camera',
        'marker_size_m':'0.0','tag_map_json':'{}',
        'base_camera_transform_json':'[]',
        'roi_x_min':'0.0','roi_y_min':'0.0',
        'roi_x_max':'0.0','roi_y_max':'0.0',
    }
    mission_keys = ('armed','geometry_confirmed','hopper_limit_l',
        'bulk_density_kg_m3','target_l','per_cycle_l','minimum_battery_v',
        'arena_width','arena_height','robot_radius',
        'dig_x','dig_y','dig_yaw','dump_x','dump_y','dump_yaw','require_depth',
        'camera_name')
    camera_keys = ('marker_size_m','tag_map_json','base_camera_transform_json',
                   'camera_name')
    berm_keys = ('roi_x_min','roi_y_min','roi_x_max','roi_y_max','camera_name')
    strings = {'tag_map_json','base_camera_transform_json','camera_name'}
    booleans = {'armed','geometry_confirmed','require_depth'}

    def params(keys):
        return {k:ParameterValue(L(k),value_type=(str if k in strings else
            bool if k in booleans else float)) for k in keys}

    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        Node(package='lunabotics_autonomy',executable='mission_node',
             parameters=[params(mission_keys),{'use_sim_time':ParameterValue(
                 L('use_sim_time'),value_type=bool)}],output='screen'),
        Node(package='lunabotics_autonomy',executable='fiducial_node',
             parameters=[params(camera_keys),{'use_sim_time':ParameterValue(
                 L('use_sim_time'),value_type=bool)}],output='screen'),
        Node(package='lunabotics_autonomy',executable='berm_node',
             parameters=[params(berm_keys),{'use_sim_time':ParameterValue(
                 L('use_sim_time'),value_type=bool)}],output='screen'),
    ])
