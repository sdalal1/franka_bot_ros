"""
This class does not publish or subscribe to any topics.

It is used to populate the messages for the moveit_msgs.

"""

from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Vector3
from sensor_msgs.msg import JointState, MultiDOFJointState
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from builtin_interfaces.msg import Duration
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (JointConstraint, TrajectoryConstraints,
                             Constraints, PlanningScene,
                             PlanningOptions, RobotState,
                             AllowedCollisionMatrix, PlanningSceneWorld,
                             MotionPlanRequest, WorkspaceParameters,
                             PositionIKRequest, RobotTrajectory, CartesianPoint)
from octomap_msgs.msg import OctomapWithPose, Octomap
from enum import Enum, auto
from trajectory_msgs.msg import JointTrajectory


class State(Enum):
    """

    Current state of the system.

    Determines what the main timer function should be doing on each iteration.

    """

    NEUTRAL = auto()
    IS_DIFF = auto()


class PopulateMsgs():
    """
    This is the class that will populate the messages for the moveit_msgs.

    It gets called in the wrapper class to be used in the Node.

    """

    def __init__(self, node: Node, group_name):
        self.group_name = group_name
        self.state = State.NEUTRAL
        self.node = node

    def set_GetCartesianPositionRqt(self, group_name, frame_id, position, joint_names, waypoints):
        cart_pos_rqst = GetCartesianPath.Request()
        cart_pos_rqst.header.stamp = self.node.get_clock().now().to_msg()
        cart_pos_rqst.header.frame_id = frame_id

        cart_pos_rqst.group_name = group_name
        cart_pos_rqst.link_name = 'panda_link8'
        cart_pos_rqst.waypoints = waypoints  # this must be a list of Pose() msgs
        max_step = 0.05
        cart_pos_rqst.max_velocity_scaling_factor = 0.05
        cart_pos_rqst.max_acceleration_scaling_factor = 0.05
        cart_pos_rqst.max_cartesian_speed = 0.05
        cart_pos_rqst.cartesian_speed_limited_link = 'panda_link8'
        cart_pos_rqst.max_step = max_step
        return cart_pos_rqst

    def set_CartesianPoint(self, position):
        cp_msg = CartesianPoint()
        orientation = Quaternion(x=0.96791, y=-0.24773, z=0.017813, w=0.038285)
        cp_msg.pose = self.set_PoseMsgs(position, orientation)
        return cp_msg

    def set_PoseMsgs_position(self, position):
        pose_msgs = Pose()
        pose_msgs.position = position
        return pose_msgs

    def set_PoseMsgs(self, position, orientation):
        pose_msgs = Pose()
        pose_msgs.position = position
        pose_msgs.orientation = orientation
        return pose_msgs

    def set_RobotTrajectoryMsgs(self, path):
        robot_traj = RobotTrajectory()
        robot_traj.joint_trajectory = JointTrajectory()
        robot_traj.joint_trajectory.path = path
        return robot_traj

    def set_MotionPlanMsgs(self, frame_id, joint_names, positions):
        """
        Set the motion plan message for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.
            joint_names (list): The list of joint names.
            positions (list): The list of joint positions.

        Returns
        -------
            moveit_msgs: The motion plan message.

        """
        motion_plan_msgs = MoveGroup.Goal()
        motion_plan_msgs.request = self.set_MotionPlanRq(
            frame_id, joint_names, positions)
        self.state = State.IS_DIFF

        motion_plan_msgs.planning_options = self.set_PlanningOptions(
            frame_id, joint_names, positions)
        return motion_plan_msgs

    def set_PlanningOptions(self, frame_id, joint_names, position):
        """
        Set the planning options for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.
            joint_names (list): The list of joint names.
            positions (list): The list of joint positions.

        Returns
        -------
            moveit_msgs: The planning options message.

        """
        plan_opt = PlanningOptions()
        plan_opt.planning_scene_diff = self.set_PlanningDiff(
            frame_id, joint_names, position)

        plan_opt.plan_only = True

        plan_opt.look_around = False
        plan_opt.look_around_attempts = 0

        plan_opt.max_safe_execution_cost = 0.0
        plan_opt.replan = True
        plan_opt.replan_attempts = 0
        plan_opt.replan_delay = 0.0
        return plan_opt

    def set_PlanningDiff(self, frame_id, joint_names, position):
        """
        Set the planning scene diff for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.
            joint_names (list): The list of joint names.
            positions (list): The list of joint positions.

        Returns
        -------
            moveit_msgs: The planning scene diff message.

        """
        plan_diff = PlanningScene()
        plan_diff.robot_state = self.set_RobotState(
            frame_id, joint_names, position)
        plan_diff.allowed_collision_matrix = AllowedCollisionMatrix()

        plan_diff.world = PlanningSceneWorld()
        plan_diff.world.octomap = OctomapWithPose()
        plan_diff.world.octomap.header.stamp = self.node.get_clock(
        ).now().to_msg()
        plan_diff.world.octomap.header.frame_id = frame_id

        plan_diff.world.octomap.origin = Pose()
        plan_diff.world.octomap.origin.position = Point()
        plan_diff.world.octomap.origin.orientation = Quaternion()
        plan_diff.world.octomap.octomap = Octomap()
        plan_diff.world.octomap.octomap.header.stamp = self.node.get_clock(
        ).now().to_msg()
        plan_diff.world.octomap.octomap.header.frame_id = frame_id
        plan_diff.world.octomap.octomap.binary = True
        return plan_diff

    def set_MotionPlanRq(self, frame_id, joint_names, positions):
        """
        Set the motion plan request for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.
            joint_names (list): The list of joint names.
            positions (list): The list of joint positions.

        Returns
        -------
            moveit_msgs: The motion plan request message.

        """
        motion_plan_rq = MotionPlanRequest()
        motion_plan_rq.workspace_parameters = self.set_WorkspaceParameters(
            frame_id)
        motion_plan_rq.start_state = self.set_RobotState(
            frame_id, joint_names, positions)
        motion_plan_rq.goal_constraints = self.set_Constraints(
            frame_id, positions)

        motion_plan_rq.pipeline_id = "move_group"
        motion_plan_rq.group_name = self.group_name
        motion_plan_rq.num_planning_attempts = 10
        motion_plan_rq.allowed_planning_time = 7.0
        motion_plan_rq.max_velocity_scaling_factor = 0.1
        motion_plan_rq.max_acceleration_scaling_factor = 0.05
        motion_plan_rq.max_cartesian_speed = 0.0
        return motion_plan_rq

    def set_WorkspaceParameters(self, frame_id):
        """
        Set the workspace parameters for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.

        Returns
        -------
            moveit_msgs: The workspace parameters message.

        """
        ws_params = WorkspaceParameters()
        ws_params.header.stamp = self.node.get_clock().now().to_msg()
        ws_params.header.frame_id = frame_id
        ws_params.min_corner = Vector3(x=-1.0, y=-1.0, z=-1.0)
        ws_params.max_corner = Vector3(x=1.0, y=-1.0, z=-1.0)
        return ws_params

    def set_RobotState(self, frame_id, joint_names, position):
        """
        Set the robot state for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.
            joint_names (list): The list of joint names.
            positions (list): The list of joint positions.

        Returns
        -------
            moveit_msgs: The robot state message.

        """
        robot_state = RobotState()
        robot_state.joint_state = self.set_JointState(
            frame_id, joint_names, position)
        robot_state.multi_dof_joint_state = self.set_MultiDOFJointState(
            frame_id)

        if self.state == State.IS_DIFF:
            robot_state.is_diff = True
            self.state = State.NEUTRAL
        return robot_state

    def set_JointState(self, frame_id, joint_names, position):
        """
        Set the joint state for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.
            joint_names (list): The list of joint names.
            positions (list): The list of joint positions.

        Returns
        -------
            moveit_msgs: The joint state message.

        """
        joint_states = JointState()
        joint_states.header.stamp = self.node.get_clock().now().to_msg()
        joint_states.header.frame_id = frame_id
        joint_states.name = joint_names
        joint_states.position = position
        return joint_states

    def set_MultiDOFJointState(self, frame_id):
        """
        Set the multi degree of freedom joint state for the moveit_msgs.

        Args:
            frame_id (string): The frame id of the robot.

        Returns
        -------
            moveit_msgs: The multi degree of freedom joint state message.

        """
        MultiDOF = MultiDOFJointState()
        MultiDOF.header.stamp = self.node.get_clock().now().to_msg()
        MultiDOF.header.frame_id = frame_id
        # MultiDOF.joint_names = joint_names
        return MultiDOF

    def set_Constraints(self, joint_names, positions):
        """
        Set the constraints for the moveit_msgs.

        Args:
            joint_names (list): The list of joint names.
            positions (list): The list of joint positions.

        Returns
        -------
            moveit_msgs: The constraints message.

        """
        goal_contraints = [Constraints()]
        goal_contraints[0].joint_constraints = []

        for position, joint_name in zip(positions, joint_names):
            goal_contraints[0].joint_constraints.append((
                JointConstraint(joint_name=joint_name,
                                                        position=position,
                                                        tolerance_above=0.001,
                                                        tolerance_below=0.001,
                                                        weight=1.0)))
        return goal_contraints

    def set_TrajectoryConstraints(self):
        """Set the trajectory constraints for the moveit_msgs."""
        tc = TrajectoryConstraints()
        tc.pipeline

    def set_Duration(self, time):
        """
        Set the duration for the moveit_msgs.

        Args:
            time (float): The time in seconds.

        Returns
        -------
            moveit_msgs: The duration message.

        """
        duration = Duration()
        duration.sec = time
        return duration

    def set_PoseStamp(self, point, orientation):
        """
        Set the pose stamped for the moveit_msgs.

        Args:
            point (list): The list of x, y, z coordinates.
            orientation (Quaternion): The orientation of the pose.

        Returns
        -------
            moveit_msgs: The pose stamped message.

        """
        pose = Pose()  # pose stamped takes in position, orientation
        pose.position = point
        pose_stamped = PoseStamped()  # takes in pose, header
        pose_stamped.pose = pose
        pose_stamped.pose.orientation = orientation
        # pose now has an orientation
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        return pose_stamped

    def set_PoseStamp_position(self, point):
        """
        Set the pose stamped for the moveit_msgs.

        Args:
            point (list): The list of x, y, z coordinates.

        Returns
        -------
            moveit_msgs: The pose stamped message.

        """
        pose = Pose()  # takes in position, orientation
        pose.position = point
        pose_stamped = PoseStamped()  # takes in pose, header
        pose_stamped.pose = pose
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        return pose_stamped

    def set_PoseStamp_orientation(self, orientation):
        """
        Set the pose stamped for the moveit_msgs.

        Args:
            orientation (Quaternion): The orientation of the pose.

        Returns
        -------
            moveit_msgs: The pose stamped message.

        """
        pose_stamped = PoseStamped()
        pose_stamped.pose.orientation = orientation
        pose_stamped.header.stamp = self.node.get_clock().now().to_msg()
        return pose_stamped

    def set_IKRequest(self, robot_state, avoid_coll, pose_stamped, timeout):
        """
        Set the IK request for the moveit_msgs.

        Args:
            robot_state (RobotState): The robot state message.
            avoid_collisions (bool): Whether to avoid collisions.
            pose_stamped (PoseStamped): The pose stamped message.
            timeout (Duration): The duration message.

        Returns
        -------
            moveit_msgs: The IK request message.

        """
        IK_Request = GetPositionIK.Request()
        IK_Request.ik_request = self.set_IKRequestMsgs(
            robot_state, avoid_coll, pose_stamped, timeout)
        return IK_Request

    def set_IKRequestMsgs(self, robot_state, avoid, pose_stamped, timeout):
        """
        Set the IK request message for the moveit_msgs.

        Args:
            robot_state (RobotState): The robot state message.
            avoid_collisions (bool): Whether to avoid collisions.
            pose_stamped (PoseStamped): The pose stamped message.
            timeout (Duration): The duration message.

        Returns
        -------
            moveit_msgs: The IK request message.

        """
        IK_request_msgs = PositionIKRequest()
        IK_request_msgs.group_name = self.group_name
        IK_request_msgs.robot_state = robot_state  # includes joint states
        IK_request_msgs.avoid_collisions = avoid
        IK_request_msgs.pose_stamped = pose_stamped
        IK_request_msgs.timeout = timeout
        return IK_request_msgs
