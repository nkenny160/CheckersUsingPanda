#!/usr/bin/env python3

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker, MarkerArray

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        marker_pub = rospy.Publisher(
            "visualization_marker_array", 
            MarkerArray, 
            queue_size=1)

        marker_array = MarkerArray()

        pieces = {} #pieceID is the name of the box/cylinder and positionNumber is the integer 0-63 which defines the current location of the piece on the board

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.marker_array_publisher = marker_pub
        self.marker_array = marker_array
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.pieces = pieces

    def get_piece_at_location(self, location):
        pieces = self.pieces
        piece_name = ""
        for name, loc in pieces.items():
            if loc == location:
                piece_name = name
        return piece_name


    def go_to_joint_state(self): #home position
        move_group = self.move_group
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()        
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0 
       

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group
        ## Cartesian Paths

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x += scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.1  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        return plan, fraction

        ## END_SUB_TUTORIAL
    def plan_cartesian_path_to_box(self, scale=1):
        move_group = self.move_group
        ## Cartesian Paths

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x = 0.25  # First move up (z)
        wpose.position.y = -0.18  # and sideways (y)
        wpose.position.z = 0.33  # First move up (z)
        wpose.orientation.x = 1.0
        wpose.orientation.y = 0
        wpose.orientation.z = 0
        wpose.orientation.w = 0.0
        
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
    """ 
    def plan_cartesian_path_to_next_box(self, scale=1):
        move_group = self.move_group
        ## Cartesian Paths

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x = 0.32  # First move up (z)
        wpose.position.y = 0.17  # and sideways (y)
        wpose.position.z = 0.33  # First move up (z)
        wpose.orientation.x = 1.0
        wpose.orientation.y = 0
        wpose.orientation.z = 0
        wpose.orientation.w = 0.0
        
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction
     """
    def plan_and_execute_play(self,start,end,casualty=-1, scale=1):
        move_group = self.move_group
        pieces_array = self.pieces
        #figure out what piece we are asked to move
        print(start,end)
        piece_id = self.get_piece_at_location(start)

        #if piece needs to be deleted, figure out piece name and delete it from the scene
        if(casualty!=-1):
            oof_piece_id = self.get_piece_at_location(casualty)
            self.remove_box(oof_piece_id)
        
        #start location
        i_start = start // 8
        j_start = start % 8
        
        #end location
        i_end = end // 8
        j_end = end % 8

        #pick trajectory
        pick_waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x = 0.5 - 0.56 / 2 + 0.07 / 2 + i_start * 0.07
        wpose.position.y = - 0.56 / 2 + 0.07 / 2 + j_start * 0.07	
        wpose.position.z = 0.34    
        wpose.orientation.x = 1
        wpose.orientation.y = 0 
        wpose.orientation.z = 0
        wpose.orientation.w = 0
        
        pick_waypoints.append(copy.deepcopy(wpose))
        (pick_plan, fraction) = move_group.compute_cartesian_path(
            pick_waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        
        #once planning is completed, start doing stuff

        self.execute_plan(pick_plan)
        self.attach_box(piece_id)
        self.go_to_joint_state() #return to home position before going to place location

        #plan and execute second move
        #place trajectory
        place_waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x = 0.5 - 0.56 / 2 + 0.07 / 2 + i_end * 0.07
        wpose.position.y = - 0.56 / 2 + 0.07 / 2 + j_end * 0.07	
        wpose.position.z = 0.34    
        wpose.orientation.x = 1
        wpose.orientation.y = 0 
        wpose.orientation.z = 0
        wpose.orientation.w = 0
        
        
        place_waypoints.append(copy.deepcopy(wpose))
        (place_plan, fraction) = move_group.compute_cartesian_path(
            place_waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        self.execute_plan(place_plan)
        self.detach_box(piece_id)
        pieces_array[piece_id] = end #update piece location in array after move
        self.go_to_joint_state()

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        box_name = self.box_name
        scene = self.scene
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        #table
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_link0"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.5  # above the panda_hand frame
        box_pose.pose.position.y = 0.0
        box_pose.pose.position.z = 0.1
        
        box_name = "table1"
        box_size = (0.56, 0.56, 0.2)
        scene.add_box(box_name, box_pose, size=box_size)


        # Add the checkerboard markers
        #state variables
        #marker_array = MarkerArray()
        marker_array = self.marker_array
        marker_id = 0
        num_squares_x = 8 #int(box_size[0] / marker.scale.x)
        num_squares_y = 8 #int(box_size[1] / marker.scale.y)
        pieces_array = self.pieces
        for i in range(num_squares_x):
            for j in range(num_squares_y):
                #universal creation code
                marker = Marker()
                marker.color.a = 1.0
                marker.header.frame_id = "panda_link0"
                marker.type = marker.CUBE
                marker.action = marker.ADD
                marker.scale.x = 0.07  # size of each square
                marker.scale.y = 0.07
                marker.scale.z = 0.001
                marker.pose.orientation.w = 1.0
                marker.pose.position.z = 0.2 
                marker.id = marker_id
                marker.pose.position.x = (
                    box_pose.pose.position.x - box_size[0] / 2.0 + marker.scale.x / 2.0 + i * marker.scale.x # initial x is 0.325
                )
                marker.pose.position.y = (
                    box_pose.pose.position.y - box_size[1] / 2.0 + marker.scale.y / 2.0 + j * marker.scale.y # initial y is 0.325
                )
                bok_name = f"box{i}_{j}"
                scene.remove_world_object(bok_name) #<--- to remove boxes
                #square specific
                if (i + j) % 2 == 0:
                    marker.color.r = 1.0
                    marker.color.g = 1.0
                    marker.color.b = 1.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                    #pieces only go on black squares to start
                    if marker_id not in range(24,40): 
                        bok_name = f"box{i}_{j}"
                        bok_pose = geometry_msgs.msg.PoseStamped()
                        bok_pose.header.frame_id = "panda_link0"
                        bok_pose.pose.orientation.w = 1.0
                        bok_pose.pose.position.x = marker.pose.position.x
                        bok_pose.pose.position.y = marker.pose.position.y
                        bok_pose.pose.position.z = marker.pose.position.z + 0.025  # Above the marker
                        bok_size = (0.04, 0.04, 0.04)
                        if i < 3:
                            scene.add_cylinder(bok_name, bok_pose, 0.05, 0.02)
                        else:
                            scene.add_box(bok_name, bok_pose, size=bok_size)
                        pieces_array[bok_name] = marker_id
                               
                marker_array.markers.append(marker)
                marker_id += 1

        """ marker_pub = rospy.Publisher("checkerboard_markers", MarkerArray, queue_size=1) """
        print(pieces_array)
        marker_pub = self.marker_array_publisher
        marker_pub.publish(marker_array)


        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
    
    """ def move_checker_board_square(self, timeout=4, start, end, remove):
       #start position (0-63), end position (0-63), remove (position of a piece to remove (0-63), -1 if null)


        marker_array = self.marker_array
        marker_to_move = marker_array.markers[3]
        marker_to_move.color.r = 1.0
        marker_to_move.color.g = 0.0
        marker_to_move.color.b = 0.0
        marker_to_move = marker_array.markers[5]
        marker_to_move.color.r = 0.0
        marker_to_move.color.g = 1.0
        marker_to_move.color.b = 0.0
        marker_to_move = marker_array.markers[7]
        marker_to_move.color.r = 0.0
        marker_to_move.color.g = 0.0
        marker_to_move.color.b = 1.0
        marker_to_move.pose.position.x = 2
        marker_to_move.pose.position.y = 2

        
        marker_pub = self.marker_array_publisher
        marker_pub.publish(marker_array)
       

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)
 """
    def attach_box(self, box, timeout=4):
        box_name = box #box name should be its id in "boxi_j" form
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names
        grasping_group = "panda_hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        hand_group = moveit_commander.MoveGroupCommander("panda_hand")
        hand_goal = hand_group.get_current_joint_values()
        hand_goal[0] = 0.02
        hand_goal[1] = 0.02
        hand_group.go(hand_goal, wait = True)


        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, box, timeout=4):
        box_name = box #box name should be its id in "boxi_j" form
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        scene.remove_attached_object(eef_link, name=box_name)

        hand_group = moveit_commander.MoveGroupCommander("panda_hand")
        hand_goal = hand_group.get_current_joint_values()
        hand_goal[0] = 0.04
        hand_goal[1] = 0.04
        hand_group.go(hand_goal, wait = True)

        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, box, timeout=4):
        # only need to use this if removing piece from scene entirely
        box_name = box #box name should be its id in "boxi_j" form
        scene = self.scene
        scene.remove_world_object(box_name)
        #scene.remove_world_object("table1")
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


def main():
    try:
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        tutorial = MoveGroupPythonInterfaceTutorial()

        input(
            "============ Press `Enter` to setup new game ..."
        )
        
        tutorial.go_to_joint_state() #go home
        tutorial.add_box() #add table, checker board, and boxes to scene

        input("============ Press `Enter` to make first move ...")
        tutorial.plan_and_execute_play(1,16)

        input("============ Press `Enter` to make second move ...")
        tutorial.plan_and_execute_play(16,26)

        input("============ Press `Enter` to make third move ...")
        tutorial.plan_and_execute_play(5, 13)
    

        """ input(
            "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        )
        tutorial.display_trajectory(cartesian_plan) """

        #input("============ Press `Enter` to execute a saved path ...")
        #tutorial.execute_plan(cartesian_plan)

        #input("============ Press `Enter` to plan and display a Cartesian path to next box ...")
        #tutorial.go_to_joint_state() #go home
        #cartesian_plan2, fraction2 = tutorial.plan_cartesian_path_to_next_box()
        #tutorial.execute_plan(cartesian_plan2)

        
        input("============ Press `Enter` to go back ...")
        tutorial.go_to_joint_state() #go home

        print("============ Game Complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()


