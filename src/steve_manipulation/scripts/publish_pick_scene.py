#!/usr/bin/env python3
"""Publish the pick stand and cube into the MoveIt planning scene.

Gazebo already spawns the SDF models, but RViz's MotionPlanning plugin only
shows geometry that lives in move_group's planning scene. This node adds the
same boxes there.

The cube is visible but allowed to collide, so the gripper can close on it.
The stand stays as a real obstacle so the arm will not plan through it.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA


STAND_SIZE = (0.25, 0.25, 0.8)
CUBE_SIZE = (0.04, 0.04, 0.04)
STAND_COLOR = (0.35, 0.35, 0.38, 1.0)
CUBE_COLOR = (0.85, 0.15, 0.10, 1.0)


def as_bool(value, default=True) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    return str(value).lower() in ("1", "true", "yes", "on")


def box_object(object_id, frame, xyz, size) -> CollisionObject:
    obj = CollisionObject()
    obj.id = object_id
    obj.header.frame_id = frame
    obj.operation = CollisionObject.ADD

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = list(size)

    pose = Pose()
    pose.position.x = xyz[0]
    pose.position.y = xyz[1]
    pose.position.z = xyz[2]
    pose.orientation.w = 1.0

    obj.primitives.append(primitive)
    obj.primitive_poses.append(pose)
    return obj


def object_color(object_id, rgba) -> ObjectColor:
    color = ObjectColor()
    color.id = object_id
    color.color = ColorRGBA(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])
    return color


class PickScenePublisher(Node):
    def __init__(self):
        super().__init__("publish_pick_scene")
        self.declare_parameter("x", -0.20)
        self.declare_parameter("y", 0.55)
        self.declare_parameter("z", 0.82)
        self.declare_parameter("frame", "world")
        self.declare_parameter("robot_yaw", 3.14159)
        self.declare_parameter("spawn_stand", True)
        self.declare_parameter("spawn_cube", True)

        gx = float(self.get_parameter("x").value)
        gy = float(self.get_parameter("y").value)
        self.z = float(self.get_parameter("z").value)
        self.frame = str(self.get_parameter("frame").value)
        yaw = float(self.get_parameter("robot_yaw").value)
        # Gazebo spawn XY is world. MoveIt's SRDF world is fixed to base_link,
        # so convert through the robot spawn yaw (pi in simulation.launch.py).
        c, s = math.cos(yaw), math.sin(yaw)
        self.x = c * gx + s * gy
        self.y = -s * gx + c * gy
        self.spawn_stand = as_bool(self.get_parameter("spawn_stand").value)
        self.spawn_cube = as_bool(self.get_parameter("spawn_cube").value)

        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def build_scene(self) -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        if self.spawn_stand:
            # SDF visual/collision origin is at the box center (z = 0.4 m).
            scene.world.collision_objects.append(
                box_object(
                    "pick_stand",
                    self.frame,
                    (self.x, self.y, STAND_SIZE[2] / 2.0),
                    STAND_SIZE,
                )
            )
            scene.object_colors.append(object_color("pick_stand", STAND_COLOR))
        if self.spawn_cube:
            scene.world.collision_objects.append(
                box_object("pick_cube", self.frame, (self.x, self.y, self.z), CUBE_SIZE)
            )
            scene.object_colors.append(object_color("pick_cube", CUBE_COLOR))
            # Let the gripper (and the stand) touch the cube so a grasp is legal.
            scene.allowed_collision_matrix.default_entry_names.append("pick_cube")
            scene.allowed_collision_matrix.default_entry_values.append(True)
        return scene

    def apply(self, timeout=60.0) -> bool:
        self.get_logger().info("Waiting for /apply_planning_scene (move_group)...")
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            if self.client.wait_for_service(timeout_sec=1.0):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.client.service_is_ready():
            self.get_logger().error("move_group is not running; cannot add pick scene")
            return False

        scene = self.build_scene()
        if not scene.world.collision_objects:
            self.get_logger().warn("Neither stand nor cube enabled; nothing to publish")
            return True

        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error("ApplyPlanningScene failed")
            return False

        ids = ", ".join(obj.id for obj in scene.world.collision_objects)
        self.get_logger().info(
            f"Added {ids} at ({self.x:.3f}, {self.y:.3f}) in {self.frame}"
        )
        return True


def main(argv=None):
    rclpy.init(args=argv)
    node = PickScenePublisher()
    try:
        ok = node.apply()
        return 0 if ok else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
