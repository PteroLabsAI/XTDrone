"""Both followers of the fixed-wing formation on one leader, each a fw_formation_follower in its slot.

    ros2 launch fixed_wing_formation fw_formation.launch.py leader:=standard_vtol_0 \
        follower1:=standard_vtol_1 follower2:=standard_vtol_2 formation_type:=1

The names are the vehicles' PX4_UXRCE_DDS_NS. The figure changes in flight on /fw_formation/type (std_msgs/UInt8).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

FOLLOWERS = (1, 2)  # the law has a slot for each


def generate_launch_description():
    arguments = [DeclareLaunchArgument(name) for name in ["leader", "formation_type"] + ["follower%d" % i for i in FOLLOWERS]]
    followers = [
        Node(package="fixed_wing_formation", executable="fw_formation_follower", name="fw_formation_follower_%d" % i,
             output="screen",
             parameters=[{"vehicle_ns": LaunchConfiguration("follower%d" % i),
                          "leader_ns": LaunchConfiguration("leader"),
                          "follower_index": i,
                          "formation_type": ParameterValue(LaunchConfiguration("formation_type"), value_type=int)}])
        for i in FOLLOWERS]
    return LaunchDescription(arguments + followers)
