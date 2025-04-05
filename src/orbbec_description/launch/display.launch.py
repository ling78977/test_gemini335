import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description(): 
	# 声明包名、文件名，方便以后换文件
    package_name = 'orbbec_description'
    urdf_name = 'test_gemini_335.urdf.xacro'
    # rviz_name = 'display.rviz'

    ld = LaunchDescription()
	# 获取功能包路径（注意，这个路径是在工作空间的install文件夹里
    pkg_description = get_package_share_directory(package_name)
	# 声明文件路径，os.path.join将口号内的str用\连接，组成路径
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('orbbec_description'), 'urdf', 'test_gemini_335.urdf.xacro')])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'publish_frequency': 1000.0
        }]
    )
    robot_point = Node(
        package='gemini_335',
        executable='gemini_335_node',
        
    )
    mapping=Node(
        package='test_mapping',
        executable='test_mapping_node'
    )
    
    ld.add_action(robot_state_publisher)
    ld.add_action(robot_point)
    ld.add_action(mapping)

    return ld
