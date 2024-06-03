from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os.path import join
from launch.substitutions import Command


"""
Basic gazebo world loading. 
"""
def generate_launch_description():

    # Start a simulation with the cafe world
    path = join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")    
    coke_world = join(get_package_share_directory("maciv2"), "models", "cokeworld.sdf")
    gazebo_sim = IncludeLaunchDescription(path, launch_arguments=[("gz_args",  coke_world)])

    # Gazebo Bridge: This brings data (sensors/clock) out of gazebo into ROS.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/realsense/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                   ],
        output='screen'
        )

    maciv2 = IncludeLaunchDescription(join(get_package_share_directory("maciv2"), "launch","spawn_maciv2.launch.py"))
    moveit = IncludeLaunchDescription(join(get_package_share_directory("maciv2"), "launch","moveit.launch.py"))


    return LaunchDescription([gazebo_sim, bridge, maciv2, moveit])