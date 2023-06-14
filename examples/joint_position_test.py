#!/usr/bin/env python3
"""
Code for moving the Franka Panda robot's arm to three different joint positions.
"""

import rclpy
from rclpy.node import Node
from threading import Thread

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


"""
Example of moving to multiple joint configurations.
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameters for joint configurations
    joint_positions1 = [
        0.0,
        0.0,
        0.0,
        -0.7853981633974483,
        0.0,
        1.5707963267948966,
        0.7853981633974483,
    ]
    joint_positions2 = [
        1.57,
        -1.57,
        0.0,
        -1.57,
        0.0,
        1.57,
        0.7854,
    ]
    joint_positions3 = [
        -1.3962634016,
        -1.5882496193,
        0.3839724354,
        -1.4660765717,
        1.343903524,
        0.872664626,
        0.8203047484,
    ]
    node.declare_parameter("joint_positions1", joint_positions1)
    node.declare_parameter("joint_positions2", joint_positions2)
    node.declare_parameter("joint_positions3", joint_positions3)

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name=panda.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameter values for each joint configuration
    joint_positions1 = (
        node.get_parameter("joint_positions1").get_parameter_value().double_array_value
    )
    joint_positions2 = (
        node.get_parameter("joint_positions2").get_parameter_value().double_array_value
    )
    joint_positions3 = (
        node.get_parameter("joint_positions3").get_parameter_value().double_array_value
    )

    # Move to each joint configuration
    node.get_logger().info(f"Moving to joint configuration 1: {list(joint_positions1)}")
    moveit2.move_to_configuration(joint_positions1)
    moveit2.wait_until_executed()

    node.get_logger().info(f"Moving to joint configuration 2: {list(joint_positions2)}")
    moveit2.move_to_configuration(joint_positions2)
    moveit2.wait_until_executed()

    node.get_logger().info(f"Moving to joint configuration 3: {list(joint_positions3)}")
    moveit2.move_to_configuration(joint_positions3)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

    rclpy.init()

    # Create node for this example
    node = Node("move_arm")

    # Declare parameters for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            [-0.0872664626, -0.1396263402, 0.0698131701, -1.7976891296, 0.0698131701, 1.6057029118, 2.3736477827],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.7976891296, -1.6057029118, -2.3736477827, 0.0872664626, 0.1396263402, -0.0698131701, -0.0698131701]
        ],
    )

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=panda.joint_names(),
        base_link_name=panda.base_link_name(),
        end_effector_name=panda.end_effector_name(),
        group_name=panda.MOVE_GROUP_ARM,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Move to the first joint configuration
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value[0]
    )
    node.get_logger().info(f"Moving to joint positions: {list(joint_positions)}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    # Move to the second joint configuration
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value[1]
    )
    node.get_logger().info(f"Moving to joint positions: {list(joint_positions)}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    # Move to the third joint configuration
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value[2]
    )
    node.get_logger().info(f"Moving to joint positions: {list(joint_positions)}")
    moveit2.move_to_configuration(joint_positions)
    moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

