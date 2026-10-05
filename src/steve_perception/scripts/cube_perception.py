#!/usr/bin/env python3
"""Identify the red cube and publish its pose.

A red blob with a finite depth is a point on the cube. The pose is that point
shifted 2 cm along the ray, to the cube center, then transformed with TF.

Either camera is enough. A pose is published from the pan camera or the
wrist camera, whichever has a red blob with depth. When both agree, the
closer wrist measurement is the one the arm grasps from.

/cube_pose is the center in the map frame, for the base.
/cube_pose_base is the same point in base_link, for the arm.
/cube_source is "pan depth" or "wrist depth".

A missing depth image does not publish a pose. The stand height is not used
as a stand-in for the range.
"""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

CUBE_Z = 0.82
CUBE_Z_MIN = 0.55
CUBE_Z_MAX = 1.15
# A 40 mm cube is about 12 px across in the 640-wide pan image at the far
# spawn, and much larger in the wrist image once the base is beside the stand.
MIN_BLOB = 6
RED_PIXELS = 6
# Half the cube, added along the camera ray so the pose is the center
# rather than the near face.
CUBE_HALF = 0.02
# Red far walls and furniture show up as blobs too. The cube is in front
# of the base and within a few meters.
MIN_RANGE = 0.25
MAX_RANGE = 2.5
# Pan and wrist agree within this, so the closer wrist fix is used.
AGREE_M = 0.06


def red_mask(rgb):
    red = rgb[..., 0].astype(np.int16)
    green = rgb[..., 1].astype(np.int16)
    blue = rgb[..., 2].astype(np.int16)
    return (red > 70) & (green < 120) & (blue < 120) & (red > green + 40) & (red > blue + 40)


def close_mask(mask, radius=2):
    def dilate(image):
        grown = image.copy()
        grown[1:, :] |= image[:-1, :]
        grown[:-1, :] |= image[1:, :]
        grown[:, 1:] |= image[:, :-1]
        grown[:, :-1] |= image[:, 1:]
        return grown

    closed = mask
    for _ in range(radius):
        closed = dilate(closed)
    opened = ~closed
    for _ in range(radius):
        opened = dilate(opened)
    return ~opened


def red_blobs(mask):
    ys, xs = np.nonzero(mask)
    if len(xs) == 0 or len(xs) > 20000:
        return []
    points = list(zip(ys.tolist(), xs.tolist()))
    index = {point: i for i, point in enumerate(points)}
    parent = list(range(len(points)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    for i, (y, x) in enumerate(points):
        for neighbor in ((y + 1, x), (y, x + 1)):
            j = index.get(neighbor)
            if j is not None:
                union(i, j)

    groups = {}
    for i, point in enumerate(points):
        groups.setdefault(find(i), []).append(point)

    blobs = []
    for pixels in groups.values():
        if len(pixels) < MIN_BLOB:
            continue
        uu = [p[1] for p in pixels]
        vv = [p[0] for p in pixels]
        blobs.append((len(pixels), int(np.median(uu)), int(np.median(vv)), pixels))
    blobs.sort(key=lambda item: item[0], reverse=True)
    return blobs


def image_rgb(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        return None
    row = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step == msg.width * 3:
        rgb = row.reshape(msg.height, msg.width, 3)
    else:
        rgb = row.reshape(msg.height, msg.step)[:, : msg.width * 3]
        rgb = rgb.reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        rgb = rgb[..., ::-1]
    return rgb


def _rows(msg, channels, dtype):
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    width_bytes = msg.width * channels
    rows = raw.reshape(msg.height, msg.step)[:, :width_bytes]
    return rows.copy().view(dtype).reshape(msg.height, msg.width)


def depth_meters(msg):
    if msg is None:
        return None
    if msg.encoding == "32FC1":
        return _rows(msg, 4, np.float32)
    if msg.encoding == "16UC1":
        return _rows(msg, 2, np.uint16).astype(np.float32) * 0.001
    return None


def _finite_ranges(values):
    values = np.asarray(values, dtype=np.float32).ravel()
    values = values[np.isfinite(values)]
    values = values[(values > 0.08) & (values < 8.0)]
    if len(values) == 0:
        return None
    return values


def range_for_blob(depth, pixels):
    """Median range of the red pixels themselves.

    Nearby pixels are not used. Those often belong to the stand, which would
    place the cube at the wrong distance.
    """
    ys = [p[0] for p in pixels]
    xs = [p[1] for p in pixels]
    on_blob = _finite_ranges(depth[ys, xs])
    if on_blob is None or len(on_blob) < 4:
        return None
    return float(np.median(on_blob))


def rays_from_pixel(u, v, depth, info):
    fx, fy = info.k[0], info.k[4]
    cx, cy = info.k[2], info.k[5]
    if fx == 0.0 or fy == 0.0 or not math.isfinite(depth) or depth <= 0.05:
        return []
    right = (u - cx) / fx * depth
    down = (v - cy) / fy * depth
    return [
        (right, down, depth),
        (depth, -right, -down),
    ]


class CameraStream:
    def __init__(self, node, name, color_topics, depth_topics, info_topics):
        self.node = node
        self.name = name
        self.color = None
        self.depth = None
        self.info = None
        self._logged = set()
        for topic in color_topics:
            node.create_subscription(Image, topic, self._on_color, 10)
        for topic in depth_topics:
            node.create_subscription(Image, topic, self._on_depth, 10)
        for topic in info_topics:
            node.create_subscription(CameraInfo, topic, self._on_info, 10)

    def _note(self, kind, msg):
        key = (kind, msg.width, msg.height, msg.encoding, msg.header.frame_id)
        if key in self._logged:
            return
        self._logged.add(key)
        self.node.get_logger().info(
            f"{self.name} {kind} {msg.width}x{msg.height} {msg.encoding} "
            f"frame={msg.header.frame_id}"
        )

    def _on_color(self, msg):
        self._note("color", msg)
        self.color = msg

    def _on_depth(self, msg):
        self._note("depth", msg)
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg


class CubePerception(Node):
    def __init__(self):
        super().__init__("cube_perception")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # gazebo_ros_camera publishes /<namespace>/<camera_name>/<remap>.
        # Both layouts are subscribed so a plugin prefix change still works.
        self.wrist = CameraStream(
            self,
            "wrist",
            [
                "/wrist_camera/wrist_camera/color/image_raw",
                "/wrist_camera/color/image_raw",
            ],
            [
                "/wrist_camera/wrist_camera/aligned_depth_to_color/image_raw",
                "/wrist_camera/aligned_depth_to_color/image_raw",
            ],
            [
                "/wrist_camera/wrist_camera/color/camera_info",
                "/wrist_camera/color/camera_info",
            ],
        )
        self.pan = CameraStream(
            self,
            "pan",
            [
                "/pan_tilt_camera/pan_tilt_camera/color/image_raw",
                "/pan_tilt_camera/color/image_raw",
            ],
            [
                "/pan_tilt_camera/pan_tilt_camera/aligned_depth_to_color/image_raw",
                "/pan_tilt_camera/aligned_depth_to_color/image_raw",
            ],
            [
                "/pan_tilt_camera/pan_tilt_camera/color/camera_info",
                "/pan_tilt_camera/color/camera_info",
            ],
        )
        self.map_pub = self.create_publisher(PoseStamped, "/cube_pose", 10)
        self.base_pub = self.create_publisher(PoseStamped, "/cube_pose_base", 10)
        self.source_pub = self.create_publisher(String, "/cube_source", 10)
        self.create_timer(0.1, self._update)
        self._last_log = ""

    def _lookup(self, target, source, stamp=None):
        # The pan head and the arm move. TF at the image time keeps the
        # ray where the camera was when the frame was taken.
        if stamp is not None:
            try:
                return self.tf_buffer.lookup_transform(target, source, Time.from_msg(stamp))
            except Exception:
                pass
        return self.tf_buffer.lookup_transform(target, source, Time())

    def _to_frame(self, xyz, frame_id, stamp, target):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = stamp
        point.point.x, point.point.y, point.point.z = xyz
        transformed = do_transform_point(point, self._lookup(target, frame_id, stamp))
        return (
            transformed.point.x,
            transformed.point.y,
            transformed.point.z,
        )

    def _blobs(self, camera):
        if camera.color is None or camera.info is None:
            return []
        rgb = image_rgb(camera.color)
        if rgb is None:
            return []
        raw = red_mask(rgb)
        if int(np.count_nonzero(raw)) < RED_PIXELS:
            return []
        frame_id = camera.color.header.frame_id or camera.info.header.frame_id
        stamp = camera.color.header.stamp
        found = []
        for count, u, v, pixels in red_blobs(close_mask(raw, radius=1)):
            found.append((count, u, v, pixels, frame_id, stamp))
        return found

    def _from_depth(self, camera, blob):
        depth = depth_meters(camera.depth)
        if depth is None:
            return None
        _count, u, v, pixels, frame_id, stamp = blob
        if depth.shape[0] != camera.color.height or depth.shape[1] != camera.color.width:
            return None
        distance = range_for_blob(depth, pixels)
        if distance is None:
            return None
        # The ray hits the near face. The center sits one half-edge further.
        distance += CUBE_HALF
        best = None
        for ray in rays_from_pixel(u, v, distance, camera.info):
            try:
                point = self._to_frame(ray, frame_id, stamp, "base_link")
            except Exception:
                continue
            if not (CUBE_Z_MIN <= point[2] <= CUBE_Z_MAX):
                continue
            reach = math.hypot(point[0], point[1])
            if point[0] < 0.1 or not (MIN_RANGE <= reach <= MAX_RANGE):
                continue
            error = abs(point[2] - CUBE_Z)
            if best is None or error < best[0]:
                best = (error, point)
        if best is None:
            return None
        return best[1]

    def _locate(self, camera):
        blobs = self._blobs(camera)
        best = None
        for blob in blobs:
            point = self._from_depth(camera, blob)
            if point is None:
                continue
            reach = math.hypot(point[0], point[1])
            if best is None or reach < best[0]:
                best = (reach, point)
        if blobs and best is None:
            self.get_logger().warn(
                f"{camera.name} sees red pixels, but depth did not become a base_link pose",
                throttle_duration_sec=5.0,
            )
        if best is None:
            return None
        return best[1]

    def _select(self, wrist_point, pan_point):
        """Use whichever camera has a pose.

        When both do and they agree, the wrist is closer and wins. When they
        disagree, the one whose height matches the stand top wins.
        """
        if wrist_point is None and pan_point is None:
            return None, None
        if wrist_point is None:
            return pan_point, "pan"
        if pan_point is None:
            return wrist_point, "wrist"
        if math.dist(wrist_point, pan_point) <= AGREE_M:
            return wrist_point, "wrist"
        if abs(wrist_point[2] - CUBE_Z) + 0.02 < abs(pan_point[2] - CUBE_Z):
            return wrist_point, "wrist"
        return pan_point, "pan"

    def _base_to_map(self, point):
        transform = self._lookup("map", "base_link")
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
        pose.pose.orientation.w = 1.0
        mapped = do_transform_point(
            self._as_point(pose),
            transform,
        )
        return (mapped.point.x, mapped.point.y, mapped.point.z)

    @staticmethod
    def _as_point(pose):
        point = PointStamped()
        point.header = pose.header
        point.point = pose.pose.position
        return point

    def _publish(self, base_point, source):
        stamp = self.get_clock().now().to_msg()
        base = PoseStamped()
        base.header.frame_id = "base_link"
        base.header.stamp = stamp
        base.pose.position.x, base.pose.position.y, base.pose.position.z = base_point
        base.pose.orientation.w = 1.0
        self.base_pub.publish(base)
        self.source_pub.publish(String(data=source))
        map_text = "map unavailable"
        try:
            map_point = self._base_to_map(base_point)
        except Exception as exc:
            self.get_logger().warn(
                f"Cube is in base_link but not in the map ({exc})",
                throttle_duration_sec=5.0,
            )
        else:
            world = PoseStamped()
            world.header.frame_id = "map"
            world.header.stamp = stamp
            world.pose.position.x, world.pose.position.y, world.pose.position.z = map_point
            world.pose.orientation.w = 1.0
            self.map_pub.publish(world)
            map_text = f"map ({map_point[0]:.2f}, {map_point[1]:.2f}, {map_point[2]:.2f})"
        text = (
            f"{source}: {map_text} "
            f"base_link ({base_point[0]:.2f}, {base_point[1]:.2f}, {base_point[2]:.2f})"
        )
        if text != self._last_log:
            self._last_log = text
            self.get_logger().info(text)

    def _update(self):
        point, camera_name = self._select(self._locate(self.wrist), self._locate(self.pan))
        if point is None:
            return
        self._publish(point, f"{camera_name} depth")


def main():
    rclpy.init()
    node = CubePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
