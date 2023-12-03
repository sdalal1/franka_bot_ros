"""This node is used to test the functionality of the frankastein package."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Pose
from .submodules.frankastein import Wrapper, Gripper, FRANKA
from enum import Enum, auto
import numpy as np
import csv
from listen_apriltags_interface.msg import Loc
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import pandas as pd
import json


class State(Enum):
    """

    Current state of the system.

    Determines what the main timer function should be doing on each iteration.

    """

    NEUTRAL = auto()
    DONE = auto()
    INITIALIZE = auto()
    WAYPOINT1 = auto()
    WAYPOINT2 = auto()
    GRIPPERCLOSE = auto()
    GRIPPEROPEN = auto()
    PLANNING = auto()
    PLANNING1 = auto()
    EXECUTING = auto()
    EXECUTING1 = auto()
    PICKUP = auto()
    GRIPPERWAYPT = auto()
    PLANNING_GRIPPER = auto()
    PICKBRUSH = auto()
    UP = auto()
    EXEC = auto()
    UPPLAN = auto()
    TEST = auto()
    START = auto()
    PLANHOME = auto()
    EXECUTING2 = auto()
    CHANGE_COLOR = auto()
    FINISHED = auto()


class ILikeToMoveItMoveIt(Node):
    """
    Test Node. Commented code includes functions that work with interbotix.

    Tried with different positions and orientations.

    """

    def __init__(self):
        super().__init__("iliketomoveitmoveit")
        self.KingJulien = Wrapper(self, robot_type='panda_manipulator')
        self.Mort = FRANKA
        self.con1 = 0
        self.grasping = Gripper(self)
        self.con = 0
        self.orientation = Quaternion(
            x=0.96791, y=-0.24773, z=0.017813, w=0.038285)

        self.orientation1 = self.orientation
        self.state = State.START

        self.timer = self.create_timer(1/100, callback=self.timer_callback)
        self.apriltagsub = self.create_subscription(
            Loc, "paint_loc", self.apriltagloc_cb, 10)

        file_name = 'swarthmore.json'
        f = open(file_name)
        image = json.load(f)
        self.color_list = list(image.keys())
        self.waypoints = []

        for color in self.color_list:
            points = image[color]
            self.waypoints.append(points)

        # self.waypoints[0] = self.waypoints[0][:]
        # self.waypoints[1] = self.waypoints[1][183:]
        f.close()
        self.brushlocs = {}

        self.current_color_idx = 0
        self.current_color = self.color_list[self.current_color_idx]

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.zoffset = 0.067

        self.z_brush_standoff = 0.3 + self.zoffset + 0.1
        self.z_paint_standoff = 0.4 + self.zoffset
        self.z_brush_dot = 0.125 + .045 + 0.012  # 0.01
        self.z_paint_dip = 0.14 + .055
        self.z_brush_dip = 0.16 + self.zoffset + 0.011  # 0.012
        self.z_dot_standoff = 0.25

        # # paint standoff location
        # self.paint_location_standoff = Pose()
        # self.paint_location_standoff.position.x = 0.40275
        # self.paint_location_standoff.position.y = 0.43162
        # self.paint_location_standoff.position.z = self.z_paint_standoff
        # self.paint_location_standoff.orientation = self.orientation

        # # paint dip location
        # self.paint_location_dip = Pose()
        # self.paint_location_dip.position.x = 0.40275
        # self.paint_location_dip.position.y = 0.43162
        # # NOTE: we were subtracting 0.05 from the z value and keeping max_step = 0.1 and it was working but once we just decreased max_step, it also worked??
        # self.paint_location_dip.position.z = self.z_paint_dip
        # self.paint_location_dip.orientation = self.orientation

        # paintbrush location
        # brush standoff pose
        self.pickup_loc = Pose()
        self.pickup_loc.position.x = 0.43337
        self.pickup_loc.position.y = 0.244664
        self.pickup_loc.position.z = self.z_brush_standoff
        self.pickup_loc.orientation = self.orientation

        self.point_count = 0

        # brush pickup location
        self.pickup_dip = Pose()
        self.pickup_dip.position.x = self.pickup_loc.position.x
        self.pickup_dip.position.y = self.pickup_loc.position.y
        self.pickup_dip.position.z = self.z_brush_dip
        self.pickup_dip.orientation = self.orientation

        self.pick_msg_wpts = []
        self.pick_msg_wpts.append(self.pickup_loc)
        self.pick_msg_wpts.append(self.pickup_dip)

        self.visited = []
        self.count = 0
        self.paintx = 0.0
        self.painty = 0.0
        self.paintz = 0.0

        self.current_waypoints = []

        self.count_brush = 0

    def apriltagloc_cb(self, msg: Loc):
        # message type for the paint brush locations
        # self.get_logger().info(f'IN APRILTAGLOC_CB {msg}')
        if self.count_brush == 0:
            try:
                self.brushlocs["purple"] = msg.purple
                self.brushlocs["yellow"] = msg.yellow
                self.brushlocs["red"] = msg.red
                self.brushlocs["blue"] = msg.blue
                self.brushlocs["green"] = msg.green
                self.brushlocs["orange"] = msg.orange

                self.brushlocs["palete"] = msg.palete
                self.set_PaintLocs()
                self.count_brush = 1
            except:
                self.get_logger().info('Brush Locations Not Initlaized Yet in April Tag Callback')

    def set_PaintLocs(self):
        try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
            # print("current_color", self.current_color+"_color")
            trans = self.buffer.lookup_transform(
                "panda_link0", self.current_color + "_color", rclpy.time.Time())
            self.current_paint_x = trans.transform.translation.x
            self.current_paint_y = trans.transform.translation.y
            self.current_paint_z = trans.transform.translation.z
            # self.get_logger().info(f"Tag Location: {self.paintx,self.painty,self.paintz}")

        except tf2_ros.LookupException as e:
            self.get_logger().info(f"Lookup exception: {e}")

        self.paint_location_standoff = Pose()
        self.paint_location_dip = Pose()

        self.paint_location_standoff.position.z = self.z_paint_standoff
        self.paint_location_standoff.orientation = self.orientation

        self.paint_location_dip.position.z = self.z_paint_dip
        self.paint_location_dip.orientation = self.orientation

        self.paint_location_standoff.position.x = self.current_paint_x
        self.paint_location_dip.position.x = self.current_paint_x
        self.paint_location_standoff.position.y = self.current_paint_y
        self.paint_location_dip.position.y = self.current_paint_y

    def timer_callback(self):

        if self.state == State.START:
            self.get_logger().info("Making sure we start here all the time")
            start = input("Enter s to begin: ")
            try:
                self.set_PaintLocs()
            except:
                pass

            if start == "s":
                self.state = State.PICKUP
            else:
                self.get_logger().info("need to input a command to start")

        if self.state == State.TEST:
            try:
                # get the latest transform between left and right
                # (rclpy.time.Time() means get the latest information)
                trans = self.buffer.lookup_transform(
                    "panda_link0", "brush", rclpy.time.Time())
                self.paintx = trans.transform.translation.x
                self.painty = trans.transform.translation.y
                self.paintz = trans.transform.translation.z
                # self.get_logger().info(
                # f"Tag Location: {self.paintx,self.painty,self.paintz}")

            except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().info(f"Lookup exception: {e}")

        if self.state == State.PICKUP:
            # go to paint brush location and dip down
            # # this is the position of the paint pallete
            try:
                # standoff pose
                # print("in the 1st try")
                self.get_logger().info(
                    f'Current Color PICKUP: {self.current_color}')
                self.pickup_loc = Pose()
                self.get_logger().info(
                    f'BRUSH LOCATION: {self.brushlocs}')
                self.pickup_loc.position.x = self.brushlocs[self.current_color][0]
                self.pickup_loc.position.y = self.brushlocs[self.current_color][1]
                self.pickup_loc.position.z = self.z_brush_standoff
                self.pickup_loc.orientation = self.orientation
                self.get_logger().info(
                    f'BRUSH LOCATION: {self.pickup_loc.position.x, self.pickup_loc.position.y}')

                self.pickup_dip = Pose()
                self.pickup_dip.position.x = self.pickup_loc.position.x
                self.pickup_dip.position.y = self.pickup_loc.position.y
                self.get_logger().info(
                    f'PICKUP DIP: {self.pickup_dip.position.x, self.pickup_dip.position.y}')
                self.pickup_dip.position.z = self.z_brush_dip
                self.pickup_dip.orientation = self.orientation
                self.pick_msg_wpts = [self.pickup_loc, self.pickup_dip]
                self.get_logger().info("Line before planner in PICKUP")
                self.KingJulien.plan_path_cartesian(self.pick_msg_wpts)

                self.state = State.PLANNING_GRIPPER
                # self.get_logger().info(f'BRUSH LOCATION: {self.pickup_loc.position.x, self.pickup_loc.position.y}')
                # self.get_logger().info(f'BRUSH LOCATION: {self.pickup_loc.position.x, self.pickup_loc.position.y}')

            except:
                self.get_logger().info('Brush Locations in timer not initialized yet in PICKUP')

        elif self.state == State.PLANNING_GRIPPER:
            # self.get_logger().info('IN PLANNING GRIPPER', once=True)
            if self.KingJulien.state == self.Mort.EXECUTING:
                # self.get_logger().info('EXECUTING', once=True)
                self.state = State.EXECUTING1

        elif self.state == State.EXECUTING1:
            if self.KingJulien.state == self.Mort.DONE:
                self.state = State.GRIPPERCLOSE

        elif self.state == State.GRIPPERCLOSE:
            self.get_logger().info('\n\tGripper Closing', once=True)
            if self.con1 == 0:
                self.grasp_close_goal = self.grasping.create_close_grasp_msg()
                self.con1 = 1
            if self.grasping.state == self.Mort.CLOSE:
                # needs to go to standoff position of paintbursh to pick it up
                self.state = State.PICKBRUSH

        elif self.state == State.PICKBRUSH:
            # this is the position of the paint brush; must be taken in as a list
            # self.pickup = [0.44337, 0.244664, 0.25]
            try:
                # standoff pose
                # self.pickup = [self.brushlocs[self.current_color][0], self.brushlocs[self.current_color][1], self.z_brush_standoff]
                # self.pickup_brush = Pose()
                # self.pickup_brush.position.x = self.brushlocs[self.current_color][0]
                # self.pickup_brush.position.y = self.brushlocs[self.current_color][1]
                # self.pickup_brush.position.z = self.z_brush_standoff
                # self.pickup_brush.orientation = self.orientation
                # self.KingJulien.plan_path_cartesian([self.pickup_brush])
                self.KingJulien.plan_path_cartesian([self.pickup_loc])
                # self.KingJulien.plan_path_to_position_orientation(self.pickup, self.orientation)
                self.state = State.UP

            except:
                self.get_logger().info('Brush Locations in state PICKBRUSH not found')

        elif self.state == State.UP:
            if self.KingJulien.state == self.Mort.EXECUTING:
                self.state = State.EXEC
        elif self.state == State.EXEC:
            if self.KingJulien.state == self.Mort.DONE:
                self.current_waypoints = self.waypoints[self.current_color_idx]
                self.state = State.INITIALIZE

        elif self.state == State.INITIALIZE:
            # self.get_logger().info('IN INITIALIZE', once=True)
            msg_waypoints = []

            if not self.current_waypoints:
                self.get_logger().info("self.curent_waypoints is Empty")
                self.home = input(
                    "Return paintbrush back and change color? (y/n): ")
                # self.waypoints.pop(0)
                self.state = State.DONE

            else:
                if self.count % 5 == 0 or self.count == 0:
                    # Updating pallete location from the dictionary to the message we are sending.
                    self.set_PaintLocs()
                    msg_waypoints.append(self.paint_location_standoff)
                    msg_waypoints.append(self.paint_location_dip)
                    msg_waypoints.append(self.paint_location_standoff)

                standoff = Pose()
                standoff.position.x = self.current_waypoints[0][0]
                standoff.position.y = self.current_waypoints[0][1]
                standoff.position.z = self.z_dot_standoff
                standoff.orientation = self.orientation1

                dot_pos = Pose()
                dot_pos.position.x = standoff.position.x
                dot_pos.position.y = standoff.position.y
                dot_pos.position.z = self.z_brush_dot
                dot_pos.orientation = self.orientation1
                for _ in range(100):
                    msg_waypoints.append(standoff)
                for _ in range(100):
                    msg_waypoints.append(dot_pos)
                for _ in range(100):
                    msg_waypoints.append(standoff)

                self.state = State.EXECUTING

                self.visited.append(self.waypoints[0])
                self.current_waypoints.pop(0)
                self.point_count += 1
                self.get_logger().info(f"point count: {self.point_count}")
                # self.get_logger().info(f"current color {self.current_color}")

                self.KingJulien.plan_path_cartesian(msg_waypoints)
                self.state = State.PLANNING

        elif self.state == State.PLANNING:
            if self.KingJulien.state == self.Mort.EXECUTING:
                self.state = State.EXECUTING

        elif self.state == State.EXECUTING:
            # self.get_logger().info('Waiting for Execution to be done', once=True)
            if self.KingJulien.state == self.Mort.DONE:
                self.count += 1
                self.state = State.INITIALIZE

        elif self.state == State.DONE:
            if self.home == 'y':
                self.get_logger().info('DONE full painting', once=True)
                self.pickup_loc = Pose()
                self.pickup_loc.position.x = self.brushlocs[self.current_color][0]
                self.pickup_loc.position.y = self.brushlocs[self.current_color][1]
                self.pickup_loc.position.z = self.z_brush_standoff
                self.pickup_loc.orientation = self.orientation1

                self.pickup_dip = Pose()
                self.pickup_dip.position.x = self.pickup_loc.position.x
                self.pickup_dip.position.y = self.pickup_loc.position.y
                self.pickup_dip.position.z = self.z_brush_dip
                self.pickup_dip.orientation = self.orientation1
                self.pick_msg_wpts = [self.pickup_loc, self.pickup_dip]
                self.KingJulien.plan_path_cartesian(self.pick_msg_wpts)
                self.state = State.PLANHOME
            elif self.home == 'n':
                print("Not returning paintbrush and beginning at start")
                self.state = State.START

        elif self.state == State.PLANHOME:
            # self.get_logger().info('in planning back to paintbrush location', once=True)
            if self.KingJulien.state == self.Mort.EXECUTING:
                self.get_logger().info('EXECUTING', once=True)
                self.state = State.EXECUTING2

        elif self.state == State.EXECUTING2:
            # self.get_logger().info('Waiting for Execution to be done for execute 2', once=True)
            if self.KingJulien.state == self.Mort.DONE:
                # self.get_logger().info("going to open the gripper")
                self.state = State.GRIPPEROPEN

        elif self.state == State.GRIPPEROPEN:
            # self.get_logger().info('\n\tNOTE: ILikeToMoveItMoveItGRIPPEROPENING', once=True)
            if self.con == 0:
                self.grasp_open_goal = self.grasping.create_open_grasp_msg()
                self.con = 1
            if self.grasping.state == self.Mort.OPEN:
                # self.state = State.START
                if len(self.waypoints) != 0:
                    self.get_logger().info("Changing Color")
                    self.state = State.CHANGE_COLOR
                else:
                    self.state = State.FINISHED
                    # finished = input("Enter f to finish: ")

        elif self.state == State.CHANGE_COLOR:
            self.current_color_idx += 1
            self.con = 0
            self.con1 = 0
            self.count = 0  # reset so when it gets 2nd color, it gets paint first before dotting

            self.current_color = self.color_list[self.current_color_idx]
            # self.get_logger().info(
            # f'Current Color: {self.current_color}', once=True)
            self.state = State.PICKUP

        elif self.state == State.FINISHED:
            self.get_logger().info("Robot Has Completed All Colors")


def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = ILikeToMoveItMoveIt()
    rclpy.spin(node)
    rclpy.shutdown()
