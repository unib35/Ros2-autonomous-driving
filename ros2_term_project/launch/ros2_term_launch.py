from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file_name = 'gazebo_ros_map_demo.world'
    world = os.path.join(get_package_share_directory('ros2_term_project'),
                         'worlds', world_file_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

   # py_follower_node = Node(
      #  package='ros2_term_project',
      #  namespace='/',
       # executable='follower',
       # name='follower',
       # remappings=[
       #     ('cmd_vel', 'start/cmd_PR001'),
     #   ]
   # )

 #   py_spawn_node = Node(
 #       package='ros2_term_project',
 #       namespace='/',
  #      executable='box_spawn',
  #      name='box_spawn',
   # )

    return LaunchDescription([
        gazebo_node,
    #    py_follower_node,
    #    py_spawn_node,
    ])
