import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import xacro

# define the package name
packageName = 'demo_control'

# relative path of the xacro file with respect to the package path
xacroRelativePath = 'model/model.xacro' # 'model/example_robot.urdf.xacro' #'model/model.xacro'

# RViz config file path with respect to the package path
rvizRelativePath = 'config/config.rviz'

# relative path of the ros2_control configuration file with respect to the package folder
ros2controlRelativePath = 'config/robot_controller.yaml'

def generate_launch_description():
    # absolute package path
    pkgPath = launch_ros.substitutions.FindPackageShare(package=packageName).find(packageName)
    # absolute xacro model path
    xacroModelPath = os.path.join(pkgPath, xacroRelativePath)
    # absolute rviz config file path
    rvizConfigPath = os.path.join(pkgPath, rvizRelativePath)
    # absolute ros2_control config file path
    ros2controlPath = os.path.join(pkgPath, ros2controlRelativePath)

    # here, for verification, print the xacro model path
    print(xacroModelPath)
    # get the robot description from the xacro model file
    robot_desc = xacro.process_file(xacroModelPath).toxml()
    robot_description = {'robot_description': robot_desc}

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        launch.actions.DeclareLaunchArgument(
            name="gui",
            default_value="true",
            description="Start the RViz2 GUI."
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")

    # for starting Gazebo
    gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "rviz3 empty.sdf"}.items(),
        condition=launch.conditions.IfCondition(gui)
    )

    gazebo_headless = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_ros.substitutions.FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "--headless-rendering -s -rviz3 empty.sdf"}.items(),
        condition=launch.conditions.UnlessCondition(gui)
    )

    # Gazebo bridge
    gazebo_bridge = launch_ros.actions.Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]"],
        output="screen"
    )

    gz_spawn_entity = launch_ros.actions.Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "robot_system_position",
            "-allow_renaming", "true"
        ]
    )

    # joint state publisher node (needed for revolute joints)
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # robot state publisher node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # rviz node
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

    # ros2 control node
    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2controlPath],
        output="both"
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--param-file", ros2controlPath],
    )

    nodeList = [
        gazebo,
        gazebo_headless,
        gazebo_bridge,
        gz_spawn_entity,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        control_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner
    ]

    return launch.LaunchDescription(declared_arguments + nodeList)
