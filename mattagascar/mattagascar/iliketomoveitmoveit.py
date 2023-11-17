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

        self.orientation = Quaternion(x=0.96791, y=-0.24773, z=0.017813, w=0.038285)

        # self.state = State.PICKUP
        self.state = State.START
        self.KingJulien.add_box([0.0, 0.0, -0.6], 1.0)

        self.timer = self.create_timer(1/100, callback=self.timer_callback)
        self.apriltagsub = self.create_subscription(Loc,"paint_loc", self.apriltagloc_cb, 10)
        
        # from IPython import embed; embed()
        # coordinate_x, coordinate_y = np.loadtxt('circle_points_many.csv', unpack= True, delimiter=',')
        coordinate_x, coordinate_y = np.loadtxt('fiona.csv', unpack= True, delimiter=',')

        # coordinate_x, coordinate_y = np.loadtxt('/home/demiana/Documents/me495_ros/workspaces/final_project/src/final-project-Group5/mattagascar/mattagascar/circle_points_small.csv', unpack= True, delimiter=',')
        # coordinate_x, coordinate_y = np.loadtxt('/home/demiana/Documents/me495_ros/workspaces/final_project/src/final-project-Group5/mattagascar/mattagascar/picture_points.csv', unpack= True, delimiter=',')
        
        coordinate_list = []
        self.brushlocs = {}

        for x, y in zip(coordinate_x[:3], coordinate_y[:3]):
            point = (x,y)
            coordinate_list.append(point)

        self.waypoints = coordinate_list
        
        self.current_color = 'red'
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        
        #varibles for z
        self.z_brush_standoff = 0.25
        self.z_paint_standoff = 0.4
        self.z_brush_dot = 0.13
        self.z_paint_dip = 0.15
        self.z_brush_dip = 0.18
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


        #brush pickup location
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
        
        
    def apriltagloc_cb(self, msg:Loc):
        # message type for the paint brush locations
        # self.get_logger().info(f'IN APRILTAGLOC_CB {msg}')
        try:            
            self.brushlocs["red"] = msg.red
            self.brushlocs["blue"] = msg.blue
            self.brushlocs["green"] = msg.green
            self.brushlocs["orange"] = msg.orange
            self.brushlocs["yellow"] = msg.yellow
            self.brushlocs["palete"] = msg.palete
        except: 
            self.get_logger().info('Brush Locations Not Initlaized Yet')
            
    def set_PaintLocs(self):
        
        # paint standoff location
        self.paint_location_standoff = Pose()
        self.paint_location_standoff.position.x = self.brushlocs["palete"][0]
        self.paint_location_standoff.position.y = self.brushlocs["palete"][1]
        self.paint_location_standoff.position.z = self.z_paint_standoff
        self.paint_location_standoff.orientation = self.orientation
        
        # paint dip location
        self.paint_location_dip = Pose()
        self.paint_location_dip.position.x = self.brushlocs["palete"][0]
        self.paint_location_dip.position.y = self.brushlocs["palete"][1]
        # NOTE: we were subtracting 0.05 from the z value and keeping max_step = 0.1 and it was working but once we just decreased max_step, it also worked??
        self.paint_location_dip.position.z = self.z_paint_dip
        self.paint_location_dip.orientation = self.orientation
        
    def timer_callback(self):
        if self.state == State.START:
            self.get_logger().info("Making sure we start here all the time")
            start = input("Enter s to begin: " )
            if start == "s":
                self.state = State.PICKUP
            else:
                print("need to input a command to start")

        if self.state == State.TEST:
            try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
                trans = self.buffer.lookup_transform("panda_link0", "brush", rclpy.time.Time())
                self.paintx = trans.transform.translation.x
                self.painty = trans.transform.translation.y
                self.paintz = trans.transform.translation.z
                self.get_logger().info(f"Tag Location: {self.paintx,self.painty,self.paintz}")

            except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().info(f"Lookup exception: {e}")

        if self.state == State.PICKUP:
            # go to paint brush location and dip down

            # # this is the position of the paint pallete 
            try:
                # standoff pose
                print("in the 1st try")
                self.pickup_loc = Pose()
                self.pickup_loc.position.x = self.brushlocs[self.current_color][0]
                self.pickup_loc.position.y = self.brushlocs[self.current_color][1]
                self.pickup_loc.position.z = self.z_brush_standoff
                self.pickup_loc.orientation = self.orientation
                # self.get_logger().info(f'BRUSH LOCATION: {self.pickup_loc.position.x, self.pickup_loc.position.y}')
            
                self.pickup_dip = Pose()
                self.pickup_dip.position.x = self.pickup_loc.position.x
                self.pickup_dip.position.y = self.pickup_loc.position.y
                self.pickup_dip.position.z = self.z_brush_dip
                self.pickup_dip.orientation = self.orientation
                self.pick_msg_wpts = [self.pickup_loc, self.pickup_dip]
                self.KingJulien.plan_path_cartesian(self.pick_msg_wpts)
                
                self.state = State.PLANNING_GRIPPER
                # self.state = State.DONE

            except: 
                self.get_logger().info('Brush Locations in timer Not Initlaized Yet')


        elif self.state == State.PLANNING_GRIPPER:
            self.get_logger().info('IN PLANNING GRIPPER', once=True)
            if self.KingJulien.state == self.Mort.EXECUTING:
                self.get_logger().info('EXECUTING', once=True)
                self.state = State.EXECUTING1

        elif self.state == State.EXECUTING1:
            self.get_logger().info('Waiting for Execution to be done', once=True)
            if self.KingJulien.state == self.Mort.DONE:
                self.state = State.GRIPPERCLOSE

        elif self.state == State.GRIPPERCLOSE:
            self.get_logger().info('\n\tNOTE: ILikeToMoveItMoveItGRIPPERCLOSING', once=True)
            self.grasp_close_goal = self.grasping.create_close_grasp_msg()
            if self.grasping.state == self.Mort.CLOSE:
                # needs to go to standoff position of paintbursh to pick it up
                self.state = State.PICKBRUSH

        elif self.state == State.PICKBRUSH:
            # this is the position of the paint brush; must be taken in as a list
            # self.pickup = [0.44337, 0.244664, 0.25]
            try:
                # standoff pose
                self.pickup = [self.brushlocs[self.current_color][0], self.brushlocs[self.current_color][1], self.z_brush_standoff]
                self.KingJulien.plan_path_to_position_orientation(self.pickup, self.orientation)
                self.state = State.UP

            except: 
                self.get_logger().info('Brush Locations in timer 2 Not Initlaized Yet')

        elif self.state == State.UP:
            if self.KingJulien.state == self.Mort.EXECUTING:
                self.state = State.EXEC
        elif self.state == State.EXEC:
            if self.KingJulien.state == self.Mort.DONE:
                self.state = State.INITIALIZE

        elif self.state == State.INITIALIZE:
            self.get_logger().info('IN INITIALIZE', once=True)

            msg_waypoints = []
            
            if not self.waypoints:
                # return to paintbrush location to drop off at home
                print("self.waypoints is Empty")
                self.home = input("Return paintbrush back? (y/n): " )
                self.state = State.DONE

            else: 

                standoff = Pose()
                standoff.position.x = self.waypoints[0][0]
                standoff.position.y = self.waypoints[0][1]
                standoff.position.z = self.z_dot_standoff
                standoff.orientation = self.orientation

                dot_pos = Pose()
                dot_pos.position.x = standoff.position.x
                dot_pos.position.y = standoff.position.y
                dot_pos.position.z = self.z_brush_dot
                dot_pos.orientation = self.orientation

                msg_waypoints.append(standoff)
                msg_waypoints.append(dot_pos)
                msg_waypoints.append(standoff)
                self.state = State.EXECUTING

                self.visited.append(self.waypoints[0])
                self.waypoints.pop(0)

                # can do 15 points before needing to refill
                if self.count % 10 == 0:
                    # NOTE: needs paint
                    # NOTE: simulating for now until using cv (cv will tells us when to refill)
                    
                    # Updating pallete location from the dictionary to the message we are sending.
                    self.set_PaintLocs()
                    msg_waypoints.append(self.paint_location_standoff)
                    msg_waypoints.append(self.paint_location_dip)
                    msg_waypoints.append(self.paint_location_standoff)

                # from IPython import embed; embed()
                self.KingJulien.plan_path_cartesian(msg_waypoints)
                self.get_logger().info('Sent Waypoint msg, set state to EXECUTING', once=True)
                self.state = State.PLANNING

        elif self.state == State.PLANNING:
            if self.KingJulien.state == self.Mort.EXECUTING:
                self.state = State.EXECUTING

        elif self.state == State.EXECUTING:
            self.get_logger().info('Waiting for Execution to be done', once=True)
            if self.KingJulien.state == self.Mort.DONE:
                self.count += 1
                self.state = State.INITIALIZE

        elif self.state == State.DONE:
            if self.home == 'y':
                self.get_logger().info('DONE full painting', once=True)
                # need to go back to paintbrush location
                self.pickup_loc = Pose()
                self.pickup_loc.position.x = self.brushlocs[self.current_color][0]
                self.pickup_loc.position.y = self.brushlocs[self.current_color][1]
                self.pickup_loc.position.z = self.z_brush_standoff
                self.pickup_loc.orientation = self.orientation
            
                self.pickup_dip = Pose()
                self.pickup_dip.position.x = self.pickup_loc.position.x
                self.pickup_dip.position.y = self.pickup_loc.position.y
                self.pickup_dip.position.z = self.z_brush_dip
                self.pickup_dip.orientation = self.orientation
                self.pick_msg_wpts = [self.pickup_loc, self.pickup_dip]
                self.KingJulien.plan_path_cartesian(self.pick_msg_wpts)
                self.state = State.PLANHOME
            elif self.home == 'n':
                print("Not returning paintbrush and beginning at start")
                self.state = State.START

        elif self.state == State.PLANHOME:
            self.get_logger().info('in planning back to paintbrush location', once=True)
            if self.KingJulien.state == self.Mort.EXECUTING:
                self.get_logger().info('EXECUTING', once=True)
                self.state = State.EXECUTING2

        elif self.state == State.EXECUTING2:
            self.get_logger().info('Waiting for Execution to be done for execute 2', once=True)
            if self.KingJulien.state == self.Mort.DONE:
                self.state = State.GRIPPEROPEN

        elif self.state == State.GRIPPEROPEN:
            self.get_logger().info('\n\tNOTE: ILikeToMoveItMoveItGRIPPEROPENING', once=True)
            self.grasp_open_goal = self.grasping.create_open_grasp_msg()
            if self.grasping.state == self.Mort.OPEN:
                self.state = State.START


def main(args=None):
    """Run main function."""
    rclpy.init(args=args)
    node = ILikeToMoveItMoveIt()
    rclpy.spin(node)
    rclpy.shutdown()
