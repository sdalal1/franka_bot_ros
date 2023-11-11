"""This node is used to test the functionality of the frankastein package."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Pose
from .submodules.frankastein import Wrapper, Gripper, FRANKA
from enum import Enum, auto
import numpy as np
import csv


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
    WAYPOINT3 = auto()
    GRIPPERCLOSE = auto()
    GRIPPEROPEN = auto()
    PLANNING = auto()
    PLANNING1 = auto()
    EXECUTING = auto()
    EXECUTING1 = auto()


class ILikeToMoveItMoveIt(Node):
    """
    Test Node. Commented code includes functions that work with interbotix.

    Tried with different positions and orientations.

    """

    def __init__(self):
        super().__init__("iliketomoveitmoveit")
        self.KingJulien = Wrapper(self, robot_type='panda_manipulator')
        self.Mort = FRANKA
        self.grasping = Gripper(self)
        self.joint_angle = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_angle_panda = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # self.pose = [0.47225, 0.17125, 0.055]
        self.orientation = Quaternion(x=0.96791, y=-0.24773, z=0.017813, w=0.038285)

        self.state = State.INITIALIZE
        self.KingJulien.add_box([0.0, 0.0, -0.6], 1.0)

        self.timer = self.create_timer(1/100, callback=self.timer_callback)
        # from IPython import embed; embed()
        coordinate_x, coordinate_y = np.loadtxt('/home/demiana/Documents/me495_ros/workspaces/final_project/src/final-project-Group5/mattagascar/mattagascar/circle_points.csv', unpack= True, delimiter=',')
        coordinate_list = []

        for x, y in zip(coordinate_x, coordinate_y):
            point = (x,y)
            coordinate_list.append(point)

        self.waypoints = []

        self.z = 0.21811

        # self.waypoints = [wp1, wp2, wp3, wp4, wp5, wp6, wp7, wp8]

        for cooridainte in coordinate_list:
            waypoint = Pose()
            # from IPython import embed; embed()
            waypoint.position.x = cooridainte[0]
            waypoint.position.y = cooridainte[1]
            waypoint.position.z = self.z
            waypoint.orientation = self.orientation
            self.waypoints.append(waypoint)
        # from IPython import embed; embed()
        # wp1.position.x = 0.40569
        # wp1.position.y = -0.01668
        # wp1.position.z = 0.21811
        # wp1.orientation = self.orientation # for the ee

        # wp2.position.x = 0.45354
        # wp2.position.y = -0.19164
        # wp2.position.z = 0.21811
        # wp2.orientation = self.orientation

        # wp3.position.x = 0.30669
        # wp3.position.y = 0.00000
        # wp3.position.z = 0.59108
        # wp3.orientation = self.orientation


        # self.waypoints.append(wp1)
        # self.waypoints.append(wp2)
        # self.waypoints.append(wp3)


    def timer_callback(self):
        self.get_logger().info(f"\n\tNOTE: State of King Julien: {self.KingJulien.state}")
        # every state wrapper has, need an if statement for each state
        if self.state == State.INITIALIZE:
            self.get_logger().info('IN INITIALIZE', once=True)
            # self.KingJulien.plan_path_to_position_orientation(
            #     self.pose, self.orientation)
            dt = [10.0, 20.0, 30.0]
            self.KingJulien.plan_path_cartesian(self.waypoints, dt)
            # self.state = State.PLANNING
            self.get_logger().info('State to waiting for compute', once=True)

        # elif self.state == State.PLANNING:
        #     self.get_logger().info('IN WAITING FOR COMPUTE', once=True)
        #     if self.KingJulien.state == self.Mort.EXECUTING:
        #         self.state = State.EXECUTING

        # elif self.state == State.EXECUTING:
        #     self.get_logger().info('\n\tNOTE: ILikeToMoveItMoveItEXECUTING', once=True)
        #     if self.KingJulien.state == self.Mort.DONE:
        #         self.get_logger().info('King Julien is done', once=True)
        #         self.state = State.GRIPPERCLOSE

        # elif self.state == State.GRIPPERCLOSE:
        #     self.get_logger().info('\n\tNOTE: ILikeToMoveItMoveItGRIPPERCLOSING', once=True)
        #     self.grasp_close_goal = self.grasping.create_close_grasp_msg()
        #     if self.grasping.state == self.Mort.CLOSE:
        #         self.state = State.WAYPOINT1

        # elif self.state == State.WAYPOINT1:
        #     self.get_logger().info(
        #         f"\n\tNOTE: State of King Julien: {self.KingJulien.state}", once=True
        #         )
        #     print("here")
        #     print(self.state)
        #     # self.pose = [0.2, 0.5, 0.6]
        #     # self.orientation = Quaternion(x=0.913161, y=-0.406408, z=-0.00135, w=0.0278)
        #     # self.KingJulien.plan_path_to_position_orientation(self.pose, self.orientation)
        #     self.KingJulien.plan_path_cartesian([self.waypoints[1]], dt)
        #     self.state = State.PLANNING1

        # elif self.state == State.PLANNING1:
        #     self.get_logger().info('IN WAITING FOR COMPUTE', once=True)
        #     if self.KingJulien.state == self.Mort.EXECUTING:
        #         self.state = State.EXECUTING1

        # elif self.state == State.EXECUTING1:
        #     self.get_logger().info('\n\tNOTE: ILikeToMoveItMoveItEXECUTING', once=True)
        #     if self.KingJulien.state == self.Mort.DONE:
        #         self.get_logger().info('King Julien is done', once=True)
        #         self.state = State.GRIPPEROPEN

        # elif self.state == State.GRIPPEROPEN:
        #     self.get_logger().info('\n\tNOTE: ILikeToMoveItMoveItGRIPPERCLOSING', once=True)
        #     self.grasp_open_goal = self.grasping.create_open_grasp_msg()
        #     if self.grasping.state == self.Mort.OPEN:
        #         self.state = State.DONE
        #         # to not cause it to open and close twice
        #         self.grasping.state = self.Mort.DONE


def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = ILikeToMoveItMoveIt()
    rclpy.spin(node)
    rclpy.shutdown()
