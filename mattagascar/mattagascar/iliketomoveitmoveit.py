"""This node is used to test the functionality of the frankastein package."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Pose
from .submodules.frankastein import Wrapper, Gripper, FRANKA
from enum import Enum, auto


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

        self.pose = [0.47225, 0.17125, 0.055]
        self.orientation = Quaternion(x=0.96791, y=-0.24773, z=0.017813, w=0.038285)

        self.state = State.INITIALIZE
        self.KingJulien.add_box([0.0, 0.0, -0.6], 1.0)

        self.timer = self.create_timer(1/100, callback=self.timer_callback)

        # the waypoints we want the robot to take
        self.waypoints = []
        wp1 = Pose()
        wp2 = Pose()
        wp3 = Pose()
        wp1.position.z = -0.2
        wp2.position.y = -0.2
        wp2.position.z = 0.2
        wp3.position.y = 0.2
        wp3.position.x = -0.2
        self.waypoints.append(wp1)
        self.waypoints.append(wp2)
        self.waypoints.append(wp3)
        
        # geometry_msgs::Pose target_pose3 = move_group.getCurrentPose().pose;

        # waypoints.push_back(target_pose3);

        # target_pose3.position.z -= 0.2;
        # waypoints.push_back(target_pose3);  // down

        # target_pose3.position.y -= 0.2;
        # waypoints.push_back(target_pose3);  // right

        # target_pose3.position.z += 0.2;
        # target_pose3.position.y += 0.2;
        # target_pose3.position.x -= 0.2;
        # waypoints.push_back(target_pose3);  // up and left

        
    def timer_callback(self):
        self.get_logger().info(f"\n\tNOTE: State of King Julien: {self.KingJulien.state}")
        # every state wrapper has, need an if statement for each state
        if self.state == State.INITIALIZE:
            self.get_logger().info('IN INITIALIZE', once=True)
            # self.KingJulien.plan_path_to_position_orientation(
            #     self.pose, self.orientation)
            dt = [1.0, 2.0, 3.0]
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
        #     self.pose = [0.2, 0.5, 0.6]
        #     self.orientation = Quaternion(x=0.913161, y=-0.406408, z=-0.00135, w=0.0278)
        #     self.KingJulien.plan_path_to_position_orientation(self.pose, self.orientation)
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
