"""

Create the wrapper for the MoveIt package.

# Subscribers:
+ joint_states (sensor_msgs/msg/JointState) -- subscribes to the topic
                                to get current joint states of the robots

# Clients:
+ GetPositionIK ( moveit_msgs/srv/) -- client to the service that calculates
                                        the IK for the robot

"""

import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from moveit_msgs.srv import (
    GetPositionIK,
    GetPlanningScene,
    ApplyPlanningScene,
    GetCartesianPath,
)
from shape_msgs.msg import SolidPrimitive
from .populate_msg import PopulateMsgs
from enum import Enum, auto
from franka_msgs.action import Grasp, Move


class FRANKA(Enum):
    """

    Current state of the system.

    Determines what the main timer function should be doing on each iteration.

    """

    DONE = auto()
    INITALIZE = auto()
    WAITING = auto()
    PLANNING = auto()
    EXECUTING = auto()
    GRIPPERDONE = auto()
    OPEN = auto()
    CLOSE = auto()


class Wrapper:
    """
    This class is a wrapper for the MoveIt! planning scene and motion planning.

    It is used to plan and execute trajectories
    for the Franka Panda manipulator.

    It is also used to plan and execute trajectories
    for the Interbotix arm (commented code).

    """

    def __init__(self, node: Node, robot_type):
        self.node = node

        self.scene_planner = self.node.create_client(
            GetPlanningScene, "get_planning_scene"
        )
        while not self.scene_planner.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info(
                "get planning scence service not available, waiting again..."
            )

        self.apply_planning_scene = self.node.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )
        while not self.apply_planning_scene.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info(
                "apply planning scence service not available, waiting again..."
            )

        self.cartesian_srv_client = self.node.create_client(
            GetCartesianPath, "compute_cartesian_path"
        )

        # Robot Type -- choice between franka and interbotix
        self.robot_type = robot_type
        if self.robot_type == "panda_manipulator":
            self.pop_msgs = PopulateMsgs(node=self.node, group_name="panda_manipulator")
            self.action_node = ActionClient(self.node, MoveGroup, "move_action")
            self.joint_names = [
                "panda_joint1",
                "panda_joint2",
                "panda_joint3",
                "panda_joint4",
                "panda_joint5",
                "panda_joint6",
                "panda_joint7",
            ]
            self.ee_joint_names = [
                "panda_hand_tcp",
                "panda_rightfinger",
                "panda_leftfinger",
            ]
            self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.joint_sub = self.node.create_subscription(
                JointState, "franka/joint_states", self.cb_joint_state, 10
            )

        elif self.robot_type == "interbotix":
            self.pop_msgs = PopulateMsgs(node=self.node, group_name="interbotix_arm")

            self.action_node = ActionClient(self.node, MoveGroup, "move_action")

            self.joint_names = ["shoulder", "elbow", "wrist_angle", "waist"]
            self.joint_states = [0.0, 0.0, 0.0, 0.0]
            self.joint_sub = self.node.create_subscription(
                JointState, "px100/joint_states", self.cb_joint_state, 10
            )

        self.ik_client = self.node.create_client(GetPositionIK, "compute_ik")

        self.frame_id = "world"

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "Timeout waiting for 'compute_ik' service to become available."
            )

        self.state = FRANKA.INITALIZE

        self.execute_action = ActionClient(
            self.node, ExecuteTrajectory, "execute_trajectory"
        )

    def cb_joint_state(self, msg: JointState):
        """
        Call back function for joint states subscriber.

        Args:
            msg (JointState): message type from joint states subscriber.

        Returns
        -------
            joint_states (list): list of joint states.

        """
        self.joint_states = np.array((msg.position))
        return self.joint_states

    def future_pos_orien_callback(self, future_po):
        """Show whether the goal is rejected or accepted."""
        new_robot_state = future_po.result().solution
        new_joint_states = new_robot_state.joint_state
        joint_angles = np.array(new_joint_states.position)

        goal_constraints = self.pop_msgs.set_Constraints(self.joint_names, joint_angles)

        motion_plan = self.pop_msgs.set_MotionPlanMsgs(
            self.frame_id, self.joint_names, new_joint_states.position
        )

        motion_plan.request.goal_constraints = goal_constraints
        future_po = self.action_node.send_goal_async(motion_plan)

        future_po.add_done_callback(self.goal_path_cb)

    def future_position_callback(self, future):
        """Show whether the goal is rejected or accepted."""
        new_robot_state = future.result().solution

        new_joint_states = new_robot_state.joint_state
        joint_angles = np.array(new_joint_states.position)

        goal_constraints = self.pop_msgs.set_Constraints(self.joint_names, joint_angles)

        motion_plan = self.pop_msgs.set_MotionPlanMsgs(
            self.frame_id, self.joint_names, new_joint_states.position
        )

        motion_plan.request.goal_constraints = goal_constraints
        future = self.action_node.send_goal_async(motion_plan)

        future.add_done_callback(self.goal_path_cb)

    def future_cartesian_cb(self, future):
        """Execute RobotState msg and Robot Trajectory msg."""
        robot_traj = future.result().solution
        frac = future.result().fraction
        print("fraction", frac)

        traj_motion = ExecuteTrajectory.Goal()
        traj_motion.trajectory = robot_traj

        if frac is not None or frac != 1.0:
            print(f"Fraction: {frac}")
        if traj_motion:
            self.node.get_logger().info("\n\tNOTE: Path found in Frankastein")
            future = self.execute_action.send_goal_async(traj_motion)
            future.add_done_callback(self.future_execute_callback)
            self.state = FRANKA.EXECUTING

        else:
            self.node.get_logger().info("Path not found in Frankastein")
            self.state = FRANKA.WAITING

    def goal_path_cb(self, future):
        """
        Call back function for the goal position path.

        Args:
            motion_plan (MotionPlan): message type from goal position path.

        Returns
        -------
            None

        """
        result = future.result()
        _position_result_future = result.get_result_async()
        _position_result_future.add_done_callback(self.get_result_cb)

    def get_result_cb(self, motion_plan_future):
        """
        Get path to the desired position and then we can execute it.

        And IK calculation has finishedbeing computed.

        """
        self.node.get_logger().info("In get result cb")
        self.path = motion_plan_future.result().result.planned_trajectory

        if self.path:
            self.robot_msg = ExecuteTrajectory.Goal()
            self.robot_msg.trajectory = self.path

            future = self.execute_action.send_goal_async(self.robot_msg)
            future.add_done_callback(self.future_execute_callback)
            self.state = FRANKA.EXECUTING

        else:
            self.node.get_logger().info("Path not found in Frankastein")
            self.state = FRANKA.WAITING

    def future_execute_callback(self, future):
        """Send the result for execution."""
        result = future.result()
        future2 = result.get_result_async()
        future2.add_done_callback(self.get_execute_result_cb)

    def get_execute_result_cb(self, future):
        """Change the state to done."""
        self.state = FRANKA.DONE

    def plan_path_cartesian(self, waypoints):
        """Cartesian path by specifying a list of waypoints for the end-effector to go through."""
        frame_id = "panda_link0"
        start_state = [0.30724, 0.00054, 0.59104]

        cartesian_msgs_request = self.pop_msgs.set_GetCartesianPositionRqt(
            self.robot_type, frame_id, start_state, self.ee_joint_names, waypoints
        )
        self.cartesian_future = self.cartesian_srv_client.call_async(
            cartesian_msgs_request
        )

        self.cartesian_future.add_done_callback(self.future_cartesian_cb)

    def plan_path_to_position(self, positions):
        """
        Plan the path to the desired position.

        Orientation of the ee is unspecified.

        Args:
            positions (list): list of x, y, z coordinates.

        Returns
        -------
            ik_calculations_future (future): future object of
                                            the IK calculations.

        """
        self.node.get_logger().info("In Plan Path to Position")
        x, y, z = positions
        point = Point(x=x, y=y, z=z)

        pose_stamped = self.pop_msgs.set_PoseStamp_position(point)

        robot_state = self.pop_msgs.set_RobotState(
            frame_id=self.frame_id,
            joint_names=self.joint_names,
            position=self.joint_states,
        )
        avoid_collisions = True
        timeout = self.pop_msgs.set_Duration(time=5)

        IK_request = self.pop_msgs.set_IKRequest(
            robot_state, avoid_collisions, pose_stamped, timeout
        )

        self.ik_calculations_position_future = self.ik_client.call_async(IK_request)

        self.ik_calculations_position_future.add_done_callback(
            self.future_position_callback
        )

    def plan_path_to_orientation(self, orientation):
        """
        Plan the path to the desired orientation.

        Position of the ee is unspecified.

        Args:
            orientation (list): list of x, y, z, w coordinates.

        Returns
        -------
            ik_calculations_future (future): future object of
                                            the IK calculations.

        """
        pose_stamped = self.pop_msgs.set_PoseStamp_orientation(orientation)

        robot_state = self.pop_msgs.set_RobotState(
            frame_id=self.frame_id,
            joint_names=self.joint_names,
            position=self.joint_states,
        )
        avoid_collisions = True
        timeout = self.pop_msgs.set_Duration(time=5)

        IK_request = self.pop_msgs.set_IKRequest(
            robot_state, avoid_collisions, pose_stamped, timeout
        )

        ik_calculations_future = self.ik_client.call_async(IK_request)

        return ik_calculations_future

    def plan_path_to_position_orientation(self, positions, orientation):
        """
        Plan the path to the desired position and orientation.

        Args:
            positions (list): list of x, y, z coordinates.
            orientation (list): list of x, y, z, w coordinates.

        Returns
        -------
            ik_calculations_future (future): future object of
                                            the IK calculations.

        """
        x, y, z = positions
        print("hello x, y, z in plan path", x, y, z)
        point = Point(x=x, y=y, z=z)
        pose_stamped = self.pop_msgs.set_PoseStamp(point, orientation)

        robot_state = self.pop_msgs.set_RobotState(
            frame_id=self.frame_id,
            joint_names=self.joint_names,
            position=self.joint_states,
        )
        avoid_collisions = True
        timeout = self.pop_msgs.set_Duration(time=5)

        IK_request = self.pop_msgs.set_IKRequest(
            robot_state, avoid_collisions, pose_stamped, timeout
        )

        ik_calculations_future = self.ik_client.call_async(IK_request)

        ik_calculations_future.add_done_callback(self.future_pos_orien_callback)

    def add_box(self, position, side_length):
        """
        Add a box to the planning scene.

        Args:
            position (list): list of x, y, z coordinates.
            side_length (float): length of the box.

        Returns
        -------
            None

        """
        self.box_collision = CollisionObject()
        self.box_collision.header.frame_id = "panda_link0"
        self.box_collision.header.stamp = self.node.get_clock().now().to_msg()
        self.box_collision.id = "box"

        self.box = SolidPrimitive()
        self.box.type = 1
        self.box.dimensions = [side_length, side_length, side_length]
        self.box_collision.primitives.append(self.box)

        self.box_pose = Pose()
        self.box_pose.position.x = position[0]
        self.box_pose.position.y = position[1]
        self.box_pose.position.z = position[2]

        self.box_pose.orientation.x = 0.0
        self.box_pose.orientation.y = 0.0
        self.box_pose.orientation.z = 0.0
        self.box_pose.orientation.w = 1.0

        self.box_collision.pose = self.box_pose
        self.box_collision.operation = CollisionObject.ADD

        future = self.scene_planner.call_async(GetPlanningScene.Request())
        future.add_done_callback(self.future_scene_cb)

    def future_scene_cb(self, future):
        """Send the result for execution."""
        self.scene = future.result().scene
        self.scene.world.collision_objects.append(self.box_collision)
        self.scene.is_diff = True
        future = self.apply_planning_scene.call_async(
            ApplyPlanningScene.Request(scene=self.scene)
        )
        future.add_done_callback(self.future_apply_scene_cb)

    def future_apply_scene_cb(self, future):
        """Send the result for execution."""
        self.state = FRANKA.DONE


class Gripper:
    """
    Wrapper class for the Franka Panda gripper.

    Need to add the two callbacks for sending the goal and getting the result.

    """

    def __init__(self, node: Node):
        self.node = node
        self.state = FRANKA.OPEN
        self.gripper_client = ActionClient(self.node, Grasp, "panda_gripper/grasp")
        self.open_client = ActionClient(self.node, Move, "panda_gripper/move")

    def create_close_grasp_msg(self):
        """Create a close grasp message."""
        self.node.get_logger().info("CREATED NEW CLOSE GRASP REQUEST")

        grasp_msg = Grasp.Goal()
        grasp_msg.width = -0.015  # without the block
        grasp_msg.speed = 0.1
        grasp_msg.force = 50.0
        grasp_msg.epsilon.inner = 0.01
        grasp_msg.epsilon.outer = 0.01
        future_gripper = self.gripper_client.send_goal_async(grasp_msg)
        future_gripper.add_done_callback(self.future_gripper_close_callback)

    def create_open_grasp_msg(self):
        """Create an open grasp message."""
        self.node.get_logger().info("CREATED NEW OPEN GRASP REQUEST")

        open_msg = Move.Goal()
        open_msg.width = 0.3
        open_msg.speed = 0.1
        future_gripper = self.open_client.send_goal_async(open_msg)
        future_gripper.add_done_callback(self.future_gripper_open_callback)

    def future_gripper_close_callback(self, future):
        """Send the result for execution."""
        self.node.get_logger().info("In future2_callback for close")
        result = future.result()
        future_gripper = result.get_result_async()
        future_gripper.add_done_callback(self.get_execute_gripper_close_result_cb)

    def future_gripper_open_callback(self, future):
        """Send the result for execution."""
        self.node.get_logger().info("In future2_callback for open")
        result = future.result()
        future_gripper = result.get_result_async()
        future_gripper.add_done_callback(self.get_execute_gripper_open_result_cb)

    def get_execute_gripper_close_result_cb(self, future):
        """Change the state to Close."""
        self.state = FRANKA.CLOSE

    def get_execute_gripper_open_result_cb(self, future):
        """Change the state to Open."""
        self.state = FRANKA.OPEN
