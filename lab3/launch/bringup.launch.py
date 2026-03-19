import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def launch_setup(context):
    # Отримуємо шлях до пакету динамічно
    pkg_share = get_package_share_directory('lab3')
    
    robot_world = os.path.join(pkg_share, 'worlds', 'robot.sdf')
    
    rviz_config = os.path.join(pkg_share, 'rviz', 'trajectory.rviz')
    use_rviz = LaunchConfiguration('rviz').perform(context) == 'true'

    # Запуск Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', robot_world],
        output='screen'
    )

    # Міст для команд руху
    bridge_cmd_vel = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.Twist'],
        output='screen'
    )
    bridge_tf = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/model/vehicle_purple/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        output='screen'
)

    # Міст для одометрії 
    bridge_odom = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/model/vehicle_purple/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
        output='screen'
    )

    # паблішер траєкторії
    odom_path = ExecuteProcess(
        cmd=['ros2', 'run', 'lab3', 'odom_path_publisher'],
        output='screen'
    )

    actions = [gazebo, bridge_cmd_vel, bridge_odom, odom_path]
    
    if use_rviz:
        rviz = ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
        actions.append(rviz)

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('lab3')
    

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(pkg_share, 'models'), ':', os.path.join(pkg_share, 'worlds')]
    )

    return LaunchDescription([
        set_gz_resource_path,
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        OpaqueFunction(function=launch_setup),
    ])