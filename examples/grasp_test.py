#!/usr/bin/env python3
"""
Code for moving grasping the object.
"""

from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2 import GripperCommand
from pymoveit2.robots import panda


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_joint_goal")

    # Declare parameters for joint configurations
    joint_positions1 = [
        1.308996939,
        -0.7853981634,
        0.0,
        -2.3561944902,
        0.0,
        1.5707963268,
        0.7853981634,
    ]
    joint_positions2 = [
        1.4311699866,
        -0.2967059728 ,
        0.0,
        -1.8325957146,
        0.0,
        1.5707963268,
        1.0122909662,
    ]
    joint_positions3 = [
        0.837758041,
        0.5061454831,
        0.2617993878,
        -0.5235987756,
        -0.1570796327,
        1.0122909662,
        1.745329252,
    ]
    joint_positions4 = [
        0.7330382858,
        0.1745329252,
        0.2792526803,
        -1.3962634016,
        -0.0523598776,
        1.5533430343,
        1.745329252,
    ]
    joint_positions5 = [
        0.5934119457,
        0.0174532925,
        0.1919862177,
        -1.0646508437,
        -0.0698131701,
        1.0646508437,
        1.5533430343,
    ]
    node.declare_parameter("joint_positions1", joint_positions1)
    node.declare_parameter("joint_positions2", joint_positions2)
    node.declare_parameter("joint_positions3", joint_positions3)
    node.declare_parameter("joint_positions4", joint_positions4)
    node.declare_parameter("joint_positions5", joint_positions5)
    
    # Declare parameter for gripper action
    node.declare_parameter("action", "close")

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
    
    # Create MoveIt 2 gripper interface
    moveit2_gripper = GripperCommand(
        node=node,
        gripper_joint_names=panda.gripper_joint_names(),
        open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
        closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
        max_effort=10.0,
        ignore_new_calls_while_executing=True,
        callback_group=callback_group,
        gripper_command_action_name="/panda_hand_controller/gripper_cmd",
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
    joint_positions4 = (
        node.get_parameter("joint_positions4").get_parameter_value().double_array_value
    )
    joint_positions5 = (
        node.get_parameter("joint_positions5").get_parameter_value().double_array_value
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
    
    node.get_logger().info(f"Moving to joint configuration 4: {list(joint_positions4)}")
    moveit2.move_to_configuration(joint_positions4)
    moveit2.wait_until_executed()
    
    # Perform gripper action
    action = node.get_parameter("action").get_parameter_value().string_value
    node.get_logger().info(f'Performing gripper action "{action}"')
    if "open" == action:
        moveit2_gripper.open()
        moveit2_gripper.wait_until_executed()
    elif "close" == action:
        moveit2_gripper.close()
        moveit2_gripper.wait_until_executed()
    else:
        period_s = 1.0
        rate = node.create_rate(1 / period_s)
        while rclpy.ok():
            moveit2_gripper()
            moveit2_gripper.wait_until_executed()
            rate.sleep()
            
    # Move to last joint configuration
    node.get_logger().info(f"Moving to joint configuration 5: {list(joint_positions5)}")
    moveit2.move_to_configuration(joint_positions5)
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

