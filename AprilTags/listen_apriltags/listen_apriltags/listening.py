"""

ConnectS the camera frame and the robot frame through AprilTags.

PUBLISHERS:
  + paint_loc - the location of the paint brushes and palette

SUBSCRIBERS:
  +  NONE

SERVICES:
  +  NONE

PARAMS:
  +  NONE

"""


import rclpy
from rclpy.node import Node
from listen_apriltags_interface.msg import Loc
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from enum import Enum, auto


class State(Enum):
    """

    Current state of the apriltags system.

    Determines what the main timer function should be doing on each iteration.

    """

    CALIBRATING = (auto(),)
    TRANSFORMING = (auto(),)


class listener(Node):
    """Publishe transforms bw the camera, robot, paintbrush, and palette."""

    def __init__(self):
        super().__init__("listener")

        # create publihsers for apriltag positions
        self.pub_paint = self.create_publisher(Loc, "paint_loc", 10)

        # initialized transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.camera_to_tag = None
        self.base_to_tag = [-0.4751, 0.21998, -0.1240]

        self.brush1_tag_offset_x = 0.083
        self.brush2_tag_offset_x = self.brush1_tag_offset_x + 0.058
        self.brush_tag_offset_y = 0.0
        self.bw_brush_inc_y = 0.0

        self.robot_to_tag = TransformStamped()
        self.robot_to_tag.header.frame_id = "calibration"
        self.robot_to_tag.child_frame_id = "panda_link0"
        self.robot_to_tag.transform.translation.x = self.base_to_tag[0]
        self.robot_to_tag.transform.translation.y = self.base_to_tag[1]
        self.robot_to_tag.transform.translation.z = self.base_to_tag[2]
        self.robot_to_tag.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.robot_to_tag)

        self.state = State.CALIBRATING

    def timer_callback(self):
        """Send the transform from camera to paint brushes and palette."""
        if self.state == State.CALIBRATING:
            camera_frame = "camera_color_optical_frame"
            tag_frame6 = "calibration"

            # robot to camera transform
            try:
                t = self.tf_buffer.lookup_transform(
                    camera_frame, tag_frame6, rclpy.time.Time()
                )
                self.camera_to_tag = [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ]

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {tag_frame6} to {camera_frame}: {ex}"
                )

            if (self.camera_to_tag is not None) and (self.base_to_tag is not None):
                self.get_logger().info("Got transform from camera to robot base")
                x = self.base_to_tag[0]
                y = self.base_to_tag[1]
                z = self.base_to_tag[2]
                self.get_logger().info(f"Base: x={x}, y={y}, z={z}")
                x = self.camera_to_tag[0]
                y = self.camera_to_tag[1]
                z = self.camera_to_tag[2]
                self.get_logger().info(f"Camera: x={x}, y={y}, z={z}")
                self.state = State.TRANSFORMING

        if self.state == State.TRANSFORMING:
            camera_frame = "panda_link0"
            tag_frame1 = "brush"
            tag_frame5 = "paint"

            paint = Loc()

            # paint brush rack transform
            try:
                t = self.tf_buffer.lookup_transform(
                    tag_frame1, camera_frame, rclpy.time.Time()
                )
                self.robot_to_brush = [
                    -t.transform.translation.x,
                    -t.transform.translation.y,
                    t.transform.translation.z,
                ]

                paint.blue = [
                    self.robot_to_brush[0] + self.brush1_tag_offset_x,
                    self.robot_to_brush[1] + self.brush_tag_offset_y,
                    self.robot_to_brush[2],
                ]
                paint.red = [
                    self.robot_to_brush[0] + self.brush2_tag_offset_x,
                    self.robot_to_brush[1] + self.brush_tag_offset_y,
                    self.robot_to_brush[2],
                ]

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {camera_frame} to {tag_frame1}: {ex}"
                )
                paint.red = []
                paint.orange = []
                paint.yellow = []
                paint.green = []
                paint.blue = []
                paint.purple = []

            # paint palete transform
            try:
                t = self.tf_buffer.lookup_transform(
                    tag_frame5, camera_frame, rclpy.time.Time()
                )
                self.robot_to_palete = [
                    -t.transform.translation.x,
                    -t.transform.translation.y,
                    t.transform.translation.z,
                ]

                paint.palete = self.robot_to_palete

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {camera_frame} to {tag_frame5}: {ex}"
                )
                paint.palete = []

            # publish to topic
            self.pub_paint.publish(paint)


def main(args=None):
    """Run node function."""
    rclpy.init(args=args)
    listen_node = listener()
    rclpy.spin(listen_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
