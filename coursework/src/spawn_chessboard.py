#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy

from pick_and_place_moveit import PickAndPlaceMoveIt
from gazebo_msgs.msg import LinkStates

hover_distance = 0.1  # meters

#To Spawn the Pieces on the Chessboard
auto_spawn_piece = True

# http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials

if __name__ == '__main__':
    rospy.init_node("spawn_chessboard")
    rospy.wait_for_service("gazebo/spawn_sdf_model")

    srv_call = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

    pnp = PickAndPlaceMoveIt('left', hover_distance)
    overhead_ori = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)
    
    
    # Table
    model_path = rospkg.RosPack().get_path('coursework')+"/models/"
    table_xml = ''
    with open(model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml = table_file.read().replace('\n', '')

    table_pose=Pose(position=Point(x=0.73, y=0.4, z=0.0))
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_sdf("cafe_table", table_xml, "/", table_pose, "world")
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    
    # ChessBoard
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    board_pose = Pose(Point(0.3,0.55,0.78), orient)
    frame_dist = 0.025
    model_path = rospkg.RosPack().get_path('coursework')+"/models/"
    
    with open(model_path + "chessboard/model.sdf", "r") as f:
        board_xml = f.read().replace('\n', '')

    # Add chessboard into the simulation
    print srv_call("chessboard", board_xml, "", board_pose, "world")

    # Define Home Pose
    home_pose = Pose(position=Point(x=0.55, y=0.3, z=0.0), orientation=overhead_ori)
    # Move to Home Pose
    pnp.move_to_start(home_pose)

    # Add chesspieces into the simulation
    origin_piece = 0.03125

    pieces_xml = dict()
    list_pieces = 'rnbqkpRNBQKP'
    for each in list_pieces:
        with open(model_path + 'chessboard/'+each+".sdf", "r") as f:
            pieces_xml[each] = f.read().replace('\n', '')

    
    #board_setup = ['rnbqkbnr', 'pppppppp', '********', '********', '********', '********', 'PPPPPPPP', 'RNBQKBNR'] # Full Game Setup
    board_setup = ['********', '*b*k**r*', '*****n**', '********', '********', '**N*****', 'P**K*Q**', '********']


    piece_positionmap = dict()
    piece_names = []

    # hard coded position for spawning chess pieces
    piece_spawn_loc = deepcopy(board_pose)
    piece_spawn_loc.position.x = 0.6
    piece_spawn_loc.position.y = 0.6
    piece_spawn_loc.position.z = 0.8

    rate = rospy.Rate(2000)

    for row, each in enumerate(board_setup):
        for col, piece in enumerate(each):
            pose = deepcopy(board_pose)
            pose.position.x = board_pose.position.x + frame_dist + origin_piece + row * (2 * origin_piece)
            pose.position.y = board_pose.position.y - 0.55 + frame_dist + origin_piece + col * (2 * origin_piece)
            pose.position.z += 0.018
            piece_positionmap[str(row)+str(col)] = [pose.position.x, pose.position.y, pose.position.z-0.93] #0.93 to compensate Gazebo RViz origin difference
            
            if piece in pieces_xml:
                try:
                    # spawn Piece
                    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
                    if auto_spawn_piece == True:
                        piece_spawn_loc.position.x = piece_positionmap[str(row)+str(col)][0]
                        piece_spawn_loc.position.y = piece_positionmap[str(row)+str(col)][1]
                        piece_spawn_loc.position.z = piece_positionmap[str(row)+str(col)][2] + 0.93
                    spawn_sdf("%s%d" % (piece, col), pieces_xml[piece], "/", piece_spawn_loc, "world")
                except rospy.ServiceException, e:
                    rospy.logerr("Spawn SDF service call failed: {0}".format(e))

            if piece in list_pieces:
                piece_names.append("%s%d" % (piece,col))

                place_pose = piece_positionmap[str(row)+str(col)]
                if auto_spawn_piece == False:
                    # Pick chess piece from hard coded position on table
                    pnp.pick(Pose(position=Point(piece_spawn_loc.position.x, piece_spawn_loc.position.y , place_pose[2] - 0.015), orientation=overhead_ori))
                    # Place the chess piece at the defined position
                    pnp.place(Pose(position=Point(place_pose[0], place_pose[1], place_pose[2] + 0.008), orientation=overhead_ori))
                    # Move to Home Pose
                    pnp.move_to_start(home_pose)


    rospy.set_param('board_setup', board_setup) # Board setup
    rospy.set_param('list_pieces', list_pieces) # List of unique pieces
    rospy.set_param('piece_target_position_map', piece_positionmap) # 3D positions for each square in the chessboard
    rospy.set_param('piece_names', piece_names) # Pieces that will be part of the game
    rospy.set_param('pieces_xml', pieces_xml) # File paths to Gazebo models, i.e. SDF files
