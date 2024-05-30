from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_param_builder import load_xacro 
from os.path import join 
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess

def generate_launch_description():


    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")    
    gazebo_sim = IncludeLaunchDescription(path, launch_arguments=[("gz_args", "empty.sdf")])

    robot = ExecuteProcess(
        cmd="ros2 run ros_gz_sim create -topic robot_description -z 1.0".split(),
        name="spawn robot",
        output="both"
    )

    robot_file = join(get_package_share_directory("maciv2"), "robot_description", "maciv2.urdf.xacro")
    robot_xml = load_xacro(Path(robot_file))


    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description':robot_xml, 
                     'use_sim_time':True}],
        #namespace="/maciv2"
    )

    return LaunchDescription([gazebo_sim, robot, robot_state_publisher])