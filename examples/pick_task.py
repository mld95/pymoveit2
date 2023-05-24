#!/usr/bin/env python3
"""
Combined code for creating a collision object, moving the Franka Panda robot's arm, and commanding the gripper.
"""

import rclpy
from os import path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from threading import Thread

from pymoveit2 import MoveIt2, GripperCommand
from pymoveit2.robots import panda

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "flaskrack.stl"
)


# Function to create the collision object
def create_collision_object(node, moveit2):
    filepath = node.get_parameter("filepath").get_parameter_value().string_value
    position = node.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = node.get_parameter("quat_xyzw").get_parameter_value().double_array_value

    # Use the default example mesh if invalid
    if not filepath:
        node.get_logger().info("Using the default example mesh file")
        filepath = DEFAULT_EXAMPLE_MESH

    # Make sure the mesh file exists
    if not path.exists(filepath):
        node.get_logger().error(f"File '{filepath}' does not exist")
        rclpy.shutdown()
        exit(1)

    # Determine ID of the collision mesh
    mesh_id = path.basename(filepath).split(".")[0]

    # Add collision mesh
    node.get_logger().info(
        f"Adding collision mesh '{filepath}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
    )
    moveit2.add_collision_mesh(
        filepath=filepath, id=mesh_id, position=position, quat_xyzw=quat_xyzw
    )

    return mesh_id


def main():
    rclpy.init()

    # Create node for this example
    node = Node("pick_task")

    # Declare parameters for collision object
    node.declare_parameter("filepath", "")
    node.declare_parameter("position", [0.5, 0.0, 0.5])
    node.declare_parameter("quat_xyzw", [0.0, 0.0, -0.707, 0.707])

    # Declare parameters for joint positions
    node.declare_parameter(
        "joint_positions",
        [
            0.0,
            0.0,
            0.0,
            -0.7853981633974483,
            0.0,
            1.5707963267948966,
            0.7853981633974483,
        ],
    )

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
    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Create collision object
    mesh_id = create_collision_object(node, moveit2)

    # Move to joint configuration
    joint_positions = (
        node.get_parameter("joint_positions").get_parameter_value().double_array_value
    )
    node.get_logger().info(f"Moving to {{joint_positions: {list(joint_positions)}}}")
    moveit2.move_to_configuration(joint_positions)
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

    # Remove collision object
    node.get_logger().info(f"Removing collision mesh with ID '{mesh_id}'")
    moveit2.remove_collision_mesh(id=mesh_id)

    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()

