"""

Detects color on the palette placed in Franka's Workspace.

Publishers:
  + None

Service:
  + None

Subscriptions:
  + image_raw Image - Topic provides the raw image from the realsense camera
  + camera_info CamerInfo - Infomation regarding the camera

Parameters
----------
  + None

"""


import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import pyrealsense2 as rs2
import cv2

if not hasattr(rs2, "intrinsics"):
    import pyrealsense2.pyrealsense2 as rs2
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class ImageListener(Node):
    def __init__(self):
        super().__init__("imagelistener")
        self.camera_rgb_subscriber = self.create_subscription(
            Image, "/camera/color/image_raw", self.camera_rgb_callback, 10
        )
        self.bridge = CvBridge()
        self.timer = self.create_timer(1 / 100, self.timer_callback)
        self.last_image = None

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.camera_info = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.camera_info_cb, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cx_purple = None
        self.cy_purple = None
        self.cx_yellow = None
        self.cy_yellow = None
        self.cx_red = None
        self.cy_red = None
        self.cx_blue = None
        self.cy_blue = None
        self.cx_green = None
        self.cy_green = None
        self.cx_orange = None
        self.cy_orange = None

        self.intrinsics = None

    def camera_info_cb(self, msg):
        """
        Receive the camera intrinsincs values for pixel to distance calculations.

        Args:
            msg: msg returns camera intrinsic infomation

        Returns
        -------
            An empty response

        """
        value = msg.k
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.coeffs = msg.d
        self.intrinsics.fx = value[0]
        self.intrinsics.fy = value[4]
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = value[2]
        self.intrinsics.ppy = value[5]
        self.intrinsics.width = msg.width

    def camera_rgb_callback(self, msg):
        """
        Receive the video captured by the camera.

        Args:
            msg: msg returns last image from the camera

        Returns
        -------
            An empty response

        """
        self.last_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="bgr8")

    def taglistener(self):
        """Listen for the location of the paint palette april-tag location."""
        try:
            trans = self.buffer.lookup_transform(
                "camera_color_optical_frame", "paint", rclpy.time.Time()
            )
            paint_tag_x = trans.transform.translation.x
            paint_tag_y = trans.transform.translation.y
            paint_tag_z = trans.transform.translation.z
            tag_log = [paint_tag_x, paint_tag_y, paint_tag_z]
            return tag_log
        except tf2_ros.LookupException as e:
            self.get_logger().info(f"Lookup exception: {e}")

    def masking(self, image, tagpos):
        """
        Maskes the camera stream to the location of the palette inthe workspace.

        Args:
            image: Image captured by the camera
            tagpos: Position of the palete april-tag location

        Returns
        -------
            masked: masked image

        """
        mask = np.zeros(image.shape[:2], dtype="uint8")
        tagposx = tagpos[0]
        tagposy = tagpos[1]
        tagposz = tagpos[2]
        if self.intrinsics is not None:
            yoffset = 0.25
            yoffset1 = 0.18
            xoffset1 = 0.08
            xoffset2 = 0.36
            rect1 = rs2.rs2_project_point_to_pixel(
                self.intrinsics, [tagposx - xoffset1,
                                  tagposy + yoffset, tagposz]
            )
            rect2 = rs2.rs2_project_point_to_pixel(
                self.intrinsics, [tagposx - xoffset1,
                                  tagposy - yoffset1, tagposz]
            )
            rect3 = rs2.rs2_project_point_to_pixel(
                self.intrinsics, [tagposx - xoffset2,
                                  tagposy - yoffset1, tagposz]
            )
            rect4 = rs2.rs2_project_point_to_pixel(
                self.intrinsics, [tagposx - xoffset2,
                                  tagposy + yoffset, tagposz]
            )
            rect = np.array([rect1, rect2, rect3, rect4], np.int32)
            cv2.fillConvexPoly(mask, rect, 1)
            masked = cv2.bitwise_and(image, image, mask=mask)
            return masked

    def colordetenction(self, masked):
        """
        Perform Computer Vision to detect color in the masked region.

        Args:
            masked: masked image from the function

        Returns
        -------
            An empty response

        """
        hsv = cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)

        lower_purple = np.array([114, 20, 86], np.uint8)
        upper_purple = np.array([149, 149, 165], np.uint8)
        purple_mask = cv2.inRange(hsv, lower_purple, upper_purple)

        lower_yellow = np.array([6, 101, 163], np.uint8)
        upper_yellow = np.array([42, 170, 255], np.uint8)

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        lower_red = np.array([113, 114, 155], np.uint8)
        upper_red = np.array([179, 163, 200], np.uint8)

        red_mask = cv2.inRange(hsv, lower_red, upper_red)

        lower_blue = np.array([100, 68, 101], np.uint8)
        upper_blue = np.array([117, 110, 167], np.uint8)

        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        lower_green = np.array([49, 33, 97], np.uint8)
        upper_green = np.array([94, 103, 188], np.uint8)

        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        lower_orange = np.array([0, 54, 181], np.uint8)
        upper_orange = np.array([14, 168, 255], np.uint8)

        orange_mask = cv2.inRange(hsv, lower_orange, upper_orange)

        kernel_size = (5, 5)
        kernelo = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_size)
        kernel_sizec = (20, 20)
        kernelc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, kernel_sizec)
        img_open = cv2.morphologyEx(purple_mask, cv2.MORPH_OPEN, kernelo)
        img_open1 = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernelo)
        img_open2 = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernelo)
        img_open3 = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernelo)
        img_close_purple = cv2.morphologyEx(img_open, cv2.MORPH_CLOSE, kernelc)
        img_close_yellow = cv2.morphologyEx(
            img_open1, cv2.MORPH_CLOSE, kernelc)
        img_close_red = cv2.morphologyEx(img_open2, cv2.MORPH_CLOSE, kernelc)
        img_close_blue = cv2.morphologyEx(img_open3, cv2.MORPH_CLOSE, kernelc)

        contours, hierarchy = cv2.findContours(
            img_close_purple, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_contour = 0
        if len(contours) > 0:
            biggest_contour = contours[0]
            for a in range(0, len(contours)):
                if cv2.contourArea(contours[a]) > cv2.contourArea(biggest_contour):
                    biggest_contour = contours[a]

            M = cv2.moments(biggest_contour)
            try:
                self.cx_purple = int(M["m10"] / M["m00"])
                self.cy_purple = int(M["m01"] / M["m00"])

                cv2.circle(
                    masked, (self.cx_purple,
                             self.cy_purple), 3, (255, 255, 255), -1
                )
                cv2.putText(
                    masked,
                    "purple_centroid",
                    (self.cx_purple - 10, self.cy_purple - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                self.get_logger().info("purple not detected")

        contours, hierarchy = cv2.findContours(
            img_close_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_contour = 0
        if len(contours) > 0:
            biggest_contour = contours[0]
            for a in range(0, len(contours)):
                if cv2.contourArea(contours[a]) > cv2.contourArea(biggest_contour):
                    biggest_contour = contours[a]
            M = cv2.moments(biggest_contour)
            try:
                self.cx_yellow = int(M["m10"] / M["m00"])
                self.cy_yellow = int(M["m01"] / M["m00"])

                cv2.circle(
                    masked, (self.cx_yellow,
                             self.cy_yellow), 3, (255, 255, 255), -1
                )
                cv2.putText(
                    masked,
                    "yellow_centroid",
                    (self.cx_yellow - 10, self.cy_yellow - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                self.get_logger().info("yellow not detected")

        contours, hierarchy = cv2.findContours(
            img_close_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_contour = 0
        if len(contours) > 0:
            biggest_contour = contours[0]
            for a in range(0, len(contours)):
                if cv2.contourArea(contours[a]) > cv2.contourArea(biggest_contour):
                    biggest_contour = contours[a]

            M = cv2.moments(biggest_contour)
            try:
                self.cx_red = int(M["m10"] / M["m00"])
                self.cy_red = int(M["m01"] / M["m00"])
                cv2.circle(masked, (self.cx_red, self.cy_red),
                           3, (255, 255, 255), -1)
                cv2.putText(
                    masked,
                    "red_centroid",
                    (self.cx_red - 10, self.cy_red - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                self.get_logger().info("red not detected")

        contours, hierarchy = cv2.findContours(
            img_close_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_contour = 0
        if len(contours) > 0:
            biggest_contour = contours[0]
            for a in range(0, len(contours)):
                if cv2.contourArea(contours[a]) > cv2.contourArea(biggest_contour):
                    biggest_contour = contours[a]

            M = cv2.moments(biggest_contour)
            try:
                self.cx_blue = int(M["m10"] / M["m00"])
                self.cy_blue = int(M["m01"] / M["m00"])

                cv2.circle(masked, (self.cx_blue, self.cy_blue),
                           3, (255, 255, 255), -1)
                cv2.putText(
                    masked,
                    "blue_centroid",
                    (self.cx_blue - 10, self.cy_blue - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                self.get_logger().info("blue not detected")

        contours, hierarchy = cv2.findContours(
            green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_contour = 0
        if len(contours) > 0:
            biggest_contour = contours[0]
            for a in range(0, len(contours)):
                if cv2.contourArea(contours[a]) > cv2.contourArea(biggest_contour):
                    biggest_contour = contours[a]

            M = cv2.moments(biggest_contour)
            try:
                self.cx_green = int(M["m10"] / M["m00"])
                self.cy_green = int(M["m01"] / M["m00"])

                cv2.circle(
                    masked, (self.cx_green,
                             self.cy_green), 3, (255, 255, 255), -1
                )
                cv2.putText(
                    masked,
                    "green_centroid",
                    (self.cx_green - 10, self.cy_green - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                self.get_logger().info("green not detected")

        contours, hierarchy = cv2.findContours(
            orange_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_contour = 0
        if len(contours) > 0:
            biggest_contour = contours[0]
            for a in range(0, len(contours)):
                if cv2.contourArea(contours[a]) > cv2.contourArea(biggest_contour):
                    biggest_contour = contours[a]

            M = cv2.moments(biggest_contour)
            try:
                self.cx_orange = int(M["m10"] / M["m00"])
                self.cy_orange = int(M["m01"] / M["m00"])

                cv2.circle(
                    masked, (self.cx_orange,
                             self.cy_orange), 3, (255, 255, 255), -1
                )
                cv2.putText(
                    masked,
                    "orange_centroid",
                    (self.cx_orange - 10, self.cy_orange - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            except Exception:
                self.get_logger().info("orange not detected")

        cv2.waitKey(1)

    def broadcaster(self, tagpos):
        """
        Broadcast location of each paint with respect to panda_link_0.

        Args:
            tagpos: Positon of the palette apriltag.

        Returns
        -------
            An Empty response.

        """
        depth = tagpos[2]
        if self.cx_yellow is not None and self.cy_yellow is not None:
            yellow_pos = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [self.cx_yellow, self.cy_yellow], depth
            )
            r = TransformStamped()

            r.header.stamp = self.get_clock().now().to_msg()
            r.header.frame_id = "camera_color_optical_frame"
            r.child_frame_id = "yellow_color"

            r.transform.translation.x = yellow_pos[0]
            r.transform.translation.y = yellow_pos[1]
            r.transform.translation.z = yellow_pos[2]

            self.tf_broadcaster.sendTransform(r)

        if self.cx_purple is not None and self.cy_purple is not None:
            purple_pos = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [self.cx_purple, self.cy_purple], depth
            )
            b = TransformStamped()

            b.header.stamp = self.get_clock().now().to_msg()
            b.header.frame_id = "camera_color_optical_frame"
            b.child_frame_id = "purple_color"

            b.transform.translation.x = purple_pos[0]
            b.transform.translation.y = purple_pos[1]
            b.transform.translation.z = purple_pos[2]

            self.tf_broadcaster.sendTransform(b)

        if self.cx_red is not None and self.cy_red is not None:
            red_pos = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [self.cx_red, self.cy_red], depth
            )
            g = TransformStamped()

            g.header.stamp = self.get_clock().now().to_msg()
            g.header.frame_id = "camera_color_optical_frame"
            g.child_frame_id = "red_color"

            g.transform.translation.x = red_pos[0]
            g.transform.translation.y = red_pos[1]
            g.transform.translation.z = red_pos[2]

            self.tf_broadcaster.sendTransform(g)

        if self.cx_blue is not None and self.cy_blue is not None:
            blue_pos = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [self.cx_blue, self.cy_blue], depth
            )
            y = TransformStamped()

            y.header.stamp = self.get_clock().now().to_msg()
            y.header.frame_id = "camera_color_optical_frame"
            y.child_frame_id = "blue_color"

            y.transform.translation.x = blue_pos[0]
            y.transform.translation.y = blue_pos[1]
            y.transform.translation.z = blue_pos[2]

            self.tf_broadcaster.sendTransform(y)

        if self.cx_green is not None and self.cy_green is not None:
            green_pos = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [self.cx_green, self.cy_green], depth
            )
            o = TransformStamped()

            o.header.stamp = self.get_clock().now().to_msg()
            o.header.frame_id = "camera_color_optical_frame"
            o.child_frame_id = "green_color"

            o.transform.translation.x = green_pos[0]
            o.transform.translation.y = green_pos[1]
            o.transform.translation.z = green_pos[2]

            self.tf_broadcaster.sendTransform(o)

        if self.cx_orange is not None and self.cy_orange is not None:
            orange_pos = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [self.cx_orange, self.cy_orange], depth
            )
            p = TransformStamped()

            p.header.stamp = self.get_clock().now().to_msg()
            p.header.frame_id = "camera_color_optical_frame"
            p.child_frame_id = "orange_color"

            p.transform.translation.x = orange_pos[0]
            p.transform.translation.y = orange_pos[1]
            p.transform.translation.z = orange_pos[2]

            self.tf_broadcaster.sendTransform(p)

    def timer_callback(self):
        """Run at a set frequency."""
        try:
            tagloc = self.taglistener()
        except tf2_ros.LookupException as e:
            self.get_logger().info(f"Lookup exception: {e}")

        if self.last_image is not None and tagloc is not None:
            self.get_logger().info("Video Received", once=True)
            masked = self.masking(self.last_image, tagloc)
            self.colordetenction(masked)
            self.broadcaster(tagloc)
        else:
            self.get_logger().info("Waiting")


def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    rclpy.shutdown()
