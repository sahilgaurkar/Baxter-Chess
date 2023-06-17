#!/usr/bin/env python
import sys
import copy

import rospy
import rospkg
import tf2_ros
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander

class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.2, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to Home pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        rospy.sleep(1.0)
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        rospy.sleep(1.0)
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)
        



    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z# + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        increment = delta = self._hover_distance / 10
        print(ik_pose)

        waypoints = []

        # start with the current pose
        waypoints.append(ik_pose)
        wpose = copy.deepcopy(ik_pose)

        while increment <= self._hover_distance+0.01:
            wpose = copy.deepcopy(wpose)
            wpose.position.z = waypoints[0].position.z + increment
            waypoints.append(wpose)
            increment += delta
            
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        self._group.execute(plan)


    def _servo_to_pose(self, pose):

        pose.position.z += self._hover_distance
        increment = delta = self._hover_distance / 10

        waypoints = []

        # start with the current pose
        waypoints.append(pose)
        wpose = copy.deepcopy(pose)

        while increment <= self._hover_distance+0.01:
            wpose = copy.deepcopy(wpose)
            wpose.position.z = waypoints[0].position.z - increment
            waypoints.append(wpose)
            increment += delta
            
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        self._group.execute(plan)
            
        

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        rospy.sleep(1.0)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        rospy.sleep(2.0)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def getPiecePose(pieceName):
    # This creates a transform listener object; once created, it starts receiving
    # transformations using the /tf topic and buffers them up for up to 10 seconds.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # This is where the magic happens, a query is passed to the listener for the
    # /base to /head transform by means of the lookupTransform fn. The arguments are
    # from "this frame" to "this frame" at "this specific time"
    # (if you pass "rospy.Time(0), the fn will give you the latest available transform 
    rate = rospy.Rate(1)
    pose = None
    while not rospy.is_shutdown():
        try:
            transformation = tfBuffer.lookup_transform('base', pieceName, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        eulerAngles = euler_from_quaternion([transformation.transform.rotation.x, transformation.transform.rotation.y, transformation.transform.rotation.z, transformation.transform.rotation.w])
        newQuaternion = quaternion_from_euler(eulerAngles[0], eulerAngles[1] + math.pi, eulerAngles[2])

        pose = {
            'position' : [transformation.transform.translation.x , transformation.transform.translation.y , transformation.transform.translation.z],
            'orientation': [newQuaternion[0], newQuaternion[1], newQuaternion[2], newQuaternion[3]]
        }

        if pose != None:
            break

    return pose


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15  # meters

   # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_ori = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

   # NOTE: Gazebo and Rviz has different origins, even though they are connected. For this
   # we need to compensagetPiecePosete for this offset which is 0.93 from the ground in gazebo to
   # the actual 0, 0, 0 in Rviz.

    pnp = PickAndPlaceMoveIt(limb, hover_distance)

    # Define Home PosegetPiecePose
    home_pose = Pose(position=Point(x=0.55, y=0.3, z=0.0), orientation=overhead_ori)

    # Move to Home Pose
    pnp.move_to_start(home_pose) 

    placePositions = rospy.get_param('piece_target_position_map')

    # piece name to pick
    #pick_list = ['p4', 'P3', 'n6', 'N6', 'b5'] # Moves for full game
    pick_list = ['r6', 'K3', 'n5', 'Q5', 'k3']

    # Place Position
    #place_list = ['34', '43', '25', '55', '32'] # Moves for full game
    place_list = ['66', '74', '46', '43', '14']
    

    for index, pick in enumerate(pick_list):
        p = getPiecePose(pick)
        pickOrientation = Quaternion(x = p['orientation'][0], y=p['orientation'][1], z=p['orientation'][2], w=p['orientation'][3])
        pick_block_poses = (Pose(position=Point(x=p['position'][0], y=p['position'][1], z=p['position'][2]- 0.015), orientation=pickOrientation))
        print('Picking : ', pick)
        pnp.pick(pick_block_poses)

        p = placePositions[place_list[index]]
        place_block_poses = (Pose(position=Point(x=p[0], y=p[1], z=p[2] - 0.003), orientation=overhead_ori))
        print('Placing : ', pick)
        pnp.place(place_block_poses)

    return 0


if __name__ == '__main__':
    sys.exit(main())
 
