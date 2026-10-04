"""Generate matching world/config then bring up the full ROS baseline."""
import os
import tempfile
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory as share
import xacro
from aperturenav.config import load
from aperturenav.world import generate

def launch_nodes(context):
    config=LaunchConfiguration('config').perform(context)
    folder=Path(tempfile.mkdtemp(prefix='aperturenav_'));world=folder/'world.sdf';generate(load(config),world)
    params={'use_sim_time':True,'config_file':config}
    truth=LaunchConfiguration('ground_truth_aperture').perform(context).lower()=='true'
    urdf=xacro.process_file(share('drone_description')+'/urdf/drone.urdf.xacro').toxml()
    nodes=[IncludeLaunchDescription(PythonLaunchDescriptionSource(share('ros_gz_sim')+'/launch/gz_sim.launch.py'),launch_arguments={'gz_args':f'-r {world}'}.items()),
           Node(package='ros_gz_bridge',executable='parameter_bridge',parameters=[{'config_file':share('drone_gazebo')+'/config/bridge.yaml'}]),
           Node(package='ros_gz_bridge',executable='parameter_bridge',arguments=['/world/aperture/control@ros_gz_interfaces/srv/ControlWorld']),
           Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':urdf}]),
           Node(package='aperture_perception',executable='aperture_perception',parameters=[params],output='screen'),
           Node(package='aperture_localization',executable='aperture_localization',parameters=[params],output='screen'),
           Node(package='mission_manager',executable='mission_manager',parameters=[params,{'ground_truth_aperture':truth,'results_directory':LaunchConfiguration('results').perform(context)}],output='screen')]
    for parent,child in [('world','map'),('map','odom')]:
        nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--frame-id',parent,'--child-frame-id',child],parameters=[{'use_sim_time':True}]))
    if LaunchConfiguration('rviz').perform(context).lower()=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',share('drone_bringup')+'/rviz/aperturenav.rviz'],parameters=[{'use_sim_time':True}]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config',default_value=share('drone_bringup')+'/config/default.yaml'),
        DeclareLaunchArgument('rviz',default_value='true'),DeclareLaunchArgument('ground_truth_aperture',default_value='false'),
        DeclareLaunchArgument('results',default_value=os.path.abspath('experiments/results')),OpaqueFunction(function=launch_nodes)])
