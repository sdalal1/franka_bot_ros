import rclpy
from rclpy.node import Node
from listen_apriltags_interface.srv import Location
from listen_apriltags_interface.msg import Loc
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from enum import Enum, auto
import numpy as np


class State(Enum):

    CALIBRATING = auto(),
    TRANSFORMING = auto(),


class listener(Node):

    def __init__(self):
        super().__init__('listener')

        # # create service for apriltag positions
        # self.paintbrush_service = self.create_service(Location, 'pb_location', self.pb_callback)

        # create publihsers for apriltag positions
        self.pub_paint = self.create_publisher(Loc, 'paint_loc', 10)

        # initialized transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)
        # self.broadcaster = TransformBroadcaster(self, qos=10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.camera_to_tag = None
        # self.base_to_tag = [0.5017, -0.2306, 0.0168]
        self.base_to_tag = [-0.4751, 0.21998, -0.1240]

        self.brush1_tag_offset_x = 0.083  # old 0.1
        self.brush2_tag_offset_x = self.brush1_tag_offset_x + 0.058   # old 0.04
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
        """Print the transform from platform to brick."""

        if self.state == State.CALIBRATING:

            camera_frame = "camera_color_optical_frame"
            tag_frame6 = "calibration"

            # robot to camera transform
            try:
                t = self.tf_buffer.lookup_transform(
                    camera_frame, tag_frame6, rclpy.time.Time())
                self.camera_to_tag = [
                    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,]

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {tag_frame6} to {camera_frame}: {ex}")

            if (self.camera_to_tag is not None) and (self.base_to_tag is not None):
                self.get_logger().info('Got transform from camera to robot base')
                self.get_logger().info(
                    f'Base: x={self.base_to_tag[0]}, y={self.base_to_tag[1]}, z={self.base_to_tag[2]}')
                self.get_logger().info(
                    f'Camera: x={self.camera_to_tag[0]}, y={self.camera_to_tag[1]}, z={self.camera_to_tag[2]}')
                self.state = State.TRANSFORMING

        if self.state == State.TRANSFORMING:
            camera_frame = "panda_link0"
            tag_frame1 = "brush"
            tag_frame5 = "paint"

            # initialize message types
            paint = Loc()

            # paint brush rack transform
            try:
                t = self.tf_buffer.lookup_transform(
                    tag_frame1, camera_frame, rclpy.time.Time())
                # t = self.tf_buffer.lookup_transform(camera_frame, tag_frame1, rclpy.time.Time())
                self.robot_to_brush = [-t.transform.translation.x, -
                                       t.transform.translation.y, t.transform.translation.z]

                # individual paints:
                # hard code the axes of april tag, so circles along y axis and1* centered on x axis
                # paint.red = [self.robot_to_brush[0], self.robot_to_brush[1], self.robot_to_brush[2]]

                paint.red = [self.robot_to_brush[0] + self.brush1_tag_offset_x,
                             self.robot_to_brush[1] + self.brush_tag_offset_y, self.robot_to_brush[2]]
                # paint.orange = [paint.red[0], paint.red[1] + self.bw_brush_inc_y, paint.red[2]]
                paint.yellow = [self.robot_to_brush[0] + self.brush2_tag_offset_x,
                                self.robot_to_brush[1] + self.brush_tag_offset_y, self.robot_to_brush[2]]
                # paint.green = [paint.yellow[0], paint.yellow[1] + self.bw_brush_inc_y, paint.yellow[2]]
                # paint.blue = [paint.green[0], paint.green[1] + self.bw_brush_inc_y, paint.green[2]]
                # paint.red = [paint.blue[0], paint.blue[1] + self.bw_brush_inc_y, paint.blue[2]]

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {camera_frame} to {tag_frame1}: {ex}")
                paint.red = []
                paint.orange = []
                paint.yellow = []
                paint.green = []
                paint.blue = []
                paint.purple = []

            # paint palete transform
            try:
                t = self.tf_buffer.lookup_transform(
                    tag_frame5, camera_frame, rclpy.time.Time())
                # t = self.tf_buffer.lookup_transform(camera_frame, tag_frame5, rclpy.time.Time())
                self.robot_to_palete = [-t.transform.translation.x, -
                                        t.transform.translation.y, t.transform.translation.z,]

                paint.palete = self.robot_to_palete

            except TransformException as ex:
                self.get_logger().info(
                    f"Could not transform {camera_frame} to {tag_frame5}: {ex}")
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
