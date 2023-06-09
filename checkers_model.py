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


class playCheckers:
    def __init__(self):  
        #board array
        self.board=[0]*64

        #positions in the top row
        self.topRow=[0,1,2,3,4,5,6,7]

        #positions in the bottom row
        self.bottomRow=[56,57,58,59,60,61,62,63]

        #game is happening
        self.gameOn=True
        #0 is an empty space
        #1 is a black checker
        #2 is a red checker
        #3 is a black king
        #4 is red king

        """
        0  1  2  3  4  5  6  7 
        8  9  10 11 12 13 14 15 
        16 17 18 19 20 21 22 23 
        24 25 26 27 28 29 30 31 
        32 33 34 35 36 37 38 39 
        40 41 42 43 44 45 46 47 
        48 49 50 51 52 53 54 55 
        56 57 58 59 60 61 62 63 
        """
        self.wasTake = False
        self.playerTurn = 1
        self.setupBoard()

    def callArm(start, end, remove):
        #start is the checker starting index
        #end is checker ending index
        #remove will be position of checker to be removed if needed
        #otherwise -1
        move_group = MoveGroupPythonInterfaceTutorial()
        move_group.plan_and_execute_play(start,end,remove)
        pass



    #get a position input from user
    def getPos(self):
        try:
            return int(input(""))
        except:
            print("Invalid input. Please enter number between 0 and 63")
            return self.getPos()

    #forfeit game
    def forfeit(self, winner):
        gameOn = self.gameOn
        gameOn=False
        if winner == 1:
            print("Player 1 Wins!")
        elif winner == 2:
            print("Player 2 Wins!")

    #moving a piece from the top of the board to the bottom of the board
    def moveDown(self, start, end, enemy):
        wasTake = self.wasTake
        board = self.board
        callArm = self.callArm ##this needs to change to the robot method
        bottomRow = self.bottomRow
        #black pieces typically
        #check if move is diagonal and not going off end of board
        #makes sure you dont move one space after a take
        if ((end-start==9 and start%8 != 7) or (end-start==7 and start%8!=8)) and not wasTake:
            if board[end]==0:
                board[end]=board[start]
                board[start]=0
                wasTake=False
                #kings black piece
                if enemy==0 and end in bottomRow:
                    board[end]=3
                callArm(start, end, -1)
                return True
        #taking
        elif (end-start==18 and start%8 < 6):
            #makes sure enemy space is being jumped and not and empty or friendly space
            #makes sure the landing spot is unoccupied
            if board[start+9]%2==enemy and board[start+9]!=0 and board[end]==0:
                board[end]=board[start]
                board[start+9]=0
                board[start]=0
                wasTake = True
                #kings black piece
                if enemy==0 and end in bottomRow:
                    board[end]=3
                callArm(start, end, start+9)
                return True
        elif (end-start==14 and start%8 > 1):
            if board[start+7]%2==enemy and board[start+7]!=0 and board[end]==0:
                board[end]=board[start]
                board[start+7]=0
                board[start]=0
                wasTake = True
                if enemy==0 and end in bottomRow:
                    board[end]=3
                callArm(start, end, start+7)
                return True
        print("Invalid")
        return False

    def moveUp(self, start, end, enemy):
        wasTake = self.wasTake
        board = self.board
        callArm = self.callArm
        topRow = self.topRow
        #red pieces typically
        if ((start-end==9 and end%8 != 7) or (start-end==7 and end%8!=8)) and not wasTake:
            if board[end]==0:
                board[end]=board[start]
                board[start]=0
                wasTake = False
                #kings red piece
                if enemy==1 and end in topRow:
                    board[end]=4
                callArm(start, end, -1)
                return True
        elif (start-end==18 and end%8 < 6):
            if board[start-9]%2==enemy and board[start-9]!=0 and board[end]==0:
                board[end]=board[start]
                board[start-9]=0
                board[start]=0
                wasTake = True
                #kings red piece
                if enemy==1 and end in topRow:
                    board[end]=4
                callArm(start, end, start-9)
                return True
        elif (start-end==14 and end%8 > 1):
            if board[start-7]%2==enemy and board[start-7]!=0 and board[end]==0:
                board[end]=board[start]
                board[start-7]=0
                board[start]=0
                wasTake = True
                #kings red piece
                if enemy==1 and end in topRow:
                    board[end]=4
                callArm(start, end, start-7)
                return True
        print("Invalid")
        return False
    #determines which move is being made
    #0 means enemy piece are black, 1 means enemy pieces are red
    def makeMove(self, start, end, player):
        board = self.board
        moveDown = self.moveDown
        moveUp = self.moveUp

        if player==1:
            #black pieces
            if board[start]==1:
                return moveDown(board, start, end, 0)
            elif board[start]==3:
                if end-start>0:
                    return moveDown(board, start, end, 0)
                elif end-start<0:
                    return moveUp(board, start, end, 0)
            print("Invalid")
            return False
        elif player==2:
            #red pieces
            if board[start]==2:
                return moveUp(board, start, end, 1)
            elif board[start]==4:
                if end-start>0:
                    return moveDown(board, start, end, 1)
                elif end-start<0:
                    return moveUp(board, start, end, 1)
            print("Invalid")
            return False
        return False

    #display the ascii board
    def printBoard(self):
        board = self.board
        for i in range(len(board)):
            print(board[i], end =" ")
            if i%8==7:
                print("")
    #check if all enemy pieces are taken
    def isOver(self):
        board = self.board
        if 1 not in board and 3 not in board:
            return 2
        elif 2 not in board and 4 not in board:
            return 1
        else:
            return 0


    #creates initial setup
    def setupBoard(self):
        board = self.board
        for i in range(1,8,2):
            board[i]=1
        for i in range(8,16,2):
            board[i]=1
        for i in range(17,24,2):
            board[i]=1
        for i in range(40,48,2):
            board[i]=2
        for i in range(49,56,2):
            board[i]=2
        for i in range(56,64,2):
            board[i]=2

    #player 1 turn, did not take prev move
    def p1Turn(self):
        wasTake = self.wasTake
        board = self.board
        self.printBoard()
        print("Player 1")
        print("Type current piece position or -1 to Forfeit: ")
        start = self.getPos()

        if start==-1:
            self.forfeit(2)
            return True
        end = print("Type final piece position: ")
        end = self.getPos()

        moveValid = self.makeMove(start, end, 1)
        if not moveValid:
            self.p1Turn(board)
        if wasTake:
            self.p1Took(board, end)
        return True
    #p2 turn, did not take prev move
    def p2Turn(self):
        wasTake = self.wasTake
        board = self.board
        self.printBoard(board)
        print("Player 2")
        print("Type current piece position or -1 to Forfeit: ")
        start = self.getPos()
        if start==-1:
            self.forfeit(1)
            return True
        end = print("Type final piece position: ")
        end = self.getPos()
        moveValid=self.makeMove(start, end, 2)
        if not moveValid:
            self.p2Turn()
        return True
    #p1 turn, took on prev move
    def p1Took(self, startPiece):
        wasTake = self.wasTake
        board = self.board
        self.printBoard()
        print("Player 1")
        print("Start piece = "+str(startPiece))
        print("Type final piece position or -1 to end turn: ")
        end = self.getPos()

        if end == -1:
            wasTake=False
            return True
        moveValid=self.makeMove(startPiece, end, 1)
        if not moveValid:
            self.p1Took(startPiece)
        else:
            self.p1Took(end)
#p2 turn took on prev move
    def p2Took(self, startPiece):
        wasTake = self.wasTake
        self.printBoard()
        print("Player 2")
        print("Start piece = "+str(startPiece))
        print("Type final piece position or -1 to end turn: ")
        end = self.getPos()
        if end == -1:
            wasTake=False
            return True
        moveValid=self.makeMove(startPiece, end,2)
        if not moveValid:
            self.p2Took(startPiece)
        else:
            self.p2Took(end)
    
    def play(self):
        playerTurn = self.playerTurn
        wasTake = self.wasTake
        playerTurn = 1
        wasTake = False
        gameOn = self.gameOn
        gameOn = True
        self.setupBoard()
        # main loop
        while self.isOver() == 0 and self.gameOn:
            if playerTurn == 1:
                self.p1Turn()
                playerTurn = 2
            elif playerTurn == 2:
                self.p2Turn()
                playerTurn = 1
        gameOn = False
        if self.isOver() == 1:
            print("Player 1 Wins!")
        elif self.isOver() == 2:
            print("Player 2 Wins!")


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
    #test
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
        #setup
        tutorial = MoveGroupPythonInterfaceTutorial()
        gameController = playCheckers()
        
        tutorial.go_to_joint_state() #go home
        tutorial.add_box() #add table, checker board, and boxes to scene

        #start playing
        input("============ Press `Enter` to start the game ...")
        gameController.play()

        #game ends
        input("============ Press `Enter` to go back ...")
        tutorial.go_to_joint_state() #go home

        print("============ Game Complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()


