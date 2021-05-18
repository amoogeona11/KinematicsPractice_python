#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sim as vrep
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import time
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)
if clientID!=-1:
    print("Connected to remote API server")
    print(clientID)
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")
print "================= Starting tutorial setup"
# joint_state_topic = ['joint_states:=/robot/joint_states']
# moveit_commander.roscpp_initialize(joint_state_topic)
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("wallEsub",anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
leftgroup = moveit_commander.MoveGroupCommander("leftarm")
rightgroup = moveit_commander.MoveGroupCommander("rightarm")
left_gripper = moveit_commander.MoveGroupCommander("lefthand")
right_gripper = moveit_commander.MoveGroupCommander("righthand")
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
print "=========== Waiting for RVIZ..."

print "============ Starting tutorial "
print "============ Reference frame: %s" % leftgroup.get_planning_frame()
print "============ Reference frame: %s" % leftgroup.get_end_effector_link()
print "============ Robot Groups: "
print robot.get_group_names()
print "============ Printing robot state"
# print leftgroup.get_current_state().joint_state.position
# print rightgroup.get_current_state().joint_state.position

# err_code, prox_sensor = vrep.simxGetObjectHandle(clientID,"Proximity_sensor",vrep.simx_opmode_blocking)
err_code, pillar = vrep.simxGetObjectHandle(clientID,"pillar",vrep.simx_opmode_blocking)

err_code, wheelbase = vrep.simxGetObjectHandle(clientID,"Cuboid",vrep.simx_opmode_blocking)

err_code, leftdummy0 = vrep.simxGetObjectHandle(clientID,"targetleft0",vrep.simx_opmode_blocking)
err_code, leftdummy1 = vrep.simxGetObjectHandle(clientID,"targetleft",vrep.simx_opmode_blocking)
err_code, leftdummy2 = vrep.simxGetObjectHandle(clientID,"targetleft1",vrep.simx_opmode_blocking)
err_code, leftdummy3 = vrep.simxGetObjectHandle(clientID,"targetleft2",vrep.simx_opmode_blocking)
err_code, leftarm_prismatic = vrep.simxGetObjectHandle(clientID,"leftarm_prismatic",vrep.simx_opmode_blocking)
err_code, leftarm_verticalpris = vrep.simxGetObjectHandle(clientID,"leftarm_verticalpris",vrep.simx_opmode_blocking)
err_code, leftarm_rotx = vrep.simxGetObjectHandle(clientID,"leftarm_rotx",vrep.simx_opmode_blocking)
err_code, leftarm_rotz = vrep.simxGetObjectHandle(clientID,"leftarm_rotz",vrep.simx_opmode_blocking)
err_code, leftforearm_prismatic = vrep.simxGetObjectHandle(clientID,"leftforearm_prismatic",vrep.simx_opmode_blocking)
err_code, leftupfinger = vrep.simxGetObjectHandle(clientID,"leftupfinger",vrep.simx_opmode_blocking)
err_code, leftlowfinger = vrep.simxGetObjectHandle(clientID,"leftlowfinger",vrep.simx_opmode_blocking)
err_code, leftwheel = vrep.simxGetObjectHandle(clientID,"leftmotor",vrep.simx_opmode_blocking)

err_code, rightdummy0 = vrep.simxGetObjectHandle(clientID,"targetright0",vrep.simx_opmode_blocking)
err_code, rightdummy1 = vrep.simxGetObjectHandle(clientID,"targetright",vrep.simx_opmode_blocking)
err_code, rightdummy2 = vrep.simxGetObjectHandle(clientID,"targetright1",vrep.simx_opmode_blocking)
err_code, rightdummy3 = vrep.simxGetObjectHandle(clientID,"targetright2",vrep.simx_opmode_blocking)
err_code, rightarm_prismatic = vrep.simxGetObjectHandle(clientID,"rightarm_prismatic",vrep.simx_opmode_blocking)
err_code, rightarm_verticalpris = vrep.simxGetObjectHandle(clientID,"rightarm_verticalpris",vrep.simx_opmode_blocking)
err_code, rightarm_rotx = vrep.simxGetObjectHandle(clientID,"rightarm_rotx",vrep.simx_opmode_blocking)
err_code, rightarm_rotz = vrep.simxGetObjectHandle(clientID,"rightarm_rotz",vrep.simx_opmode_blocking)
err_code, rightforearm_prismatic = vrep.simxGetObjectHandle(clientID,"rightforearm_prismatic",vrep.simx_opmode_blocking)
err_code, rightupfinger = vrep.simxGetObjectHandle(clientID,"rightupfinger",vrep.simx_opmode_blocking)
err_code, rightlowfinger = vrep.simxGetObjectHandle(clientID,"rightlowfinger",vrep.simx_opmode_blocking)
err_code, rightwheel = vrep.simxGetObjectHandle(clientID,"rightmotor",vrep.simx_opmode_blocking)

err_code, leftdummy0_pos = vrep.simxGetObjectPosition(clientID, leftdummy0, wheelbase, vrep.simx_opmode_blocking)
err_code, leftdummy0_ori = vrep.simxGetObjectQuaternion(clientID, leftdummy0, wheelbase, vrep.simx_opmode_blocking)
err_code, leftdummy1_pos = vrep.simxGetObjectPosition(clientID, leftdummy1, wheelbase, vrep.simx_opmode_blocking)
err_code, leftdummy1_ori = vrep.simxGetObjectQuaternion(clientID, leftdummy1, wheelbase, vrep.simx_opmode_blocking)
err_code, leftdummy2_pos = vrep.simxGetObjectPosition(clientID, leftdummy2, wheelbase, vrep.simx_opmode_blocking)
err_code, leftdummy2_ori = vrep.simxGetObjectQuaternion(clientID, leftdummy2, wheelbase, vrep.simx_opmode_blocking)
err_code, leftdummy3_pos = vrep.simxGetObjectPosition(clientID,leftdummy3,wheelbase,vrep.simx_opmode_blocking)
err_code, leftdummy3_ori = vrep.simxGetObjectQuaternion(clientID,leftdummy3,wheelbase,vrep.simx_opmode_blocking)
err_code, rightdummy0_pos = vrep.simxGetObjectPosition(clientID, rightdummy0, wheelbase, vrep.simx_opmode_blocking)
err_code, rightdummy0_ori = vrep.simxGetObjectQuaternion(clientID, rightdummy0, wheelbase, vrep.simx_opmode_blocking)
err_code, rightdummy1_pos = vrep.simxGetObjectPosition(clientID, rightdummy1, wheelbase, vrep.simx_opmode_blocking)
err_code, rightdummy1_ori = vrep.simxGetObjectQuaternion(clientID, rightdummy1, wheelbase, vrep.simx_opmode_blocking)
err_code, rightdummy2_pos = vrep.simxGetObjectPosition(clientID, rightdummy2, wheelbase, vrep.simx_opmode_blocking)
err_code, rightdummy2_ori = vrep.simxGetObjectQuaternion(clientID, rightdummy2, wheelbase, vrep.simx_opmode_blocking)
err_code, rightdummy3_pos = vrep.simxGetObjectPosition(clientID,rightdummy3,wheelbase,vrep.simx_opmode_blocking)
err_code, rightdummy3_ori = vrep.simxGetObjectQuaternion(clientID,rightdummy3,wheelbase,vrep.simx_opmode_blocking)
err_code, pillar_pos = vrep.simxGetObjectPosition(clientID,pillar,wheelbase,vrep.simx_opmode_blocking)

err_code, leftwrist = vrep.simxGetObjectHandle(clientID,"leftwrist", vrep.simx_opmode_blocking)
err_code, rightwrist = vrep.simxGetObjectHandle(clientID,"rightwrist", vrep.simx_opmode_blocking)
err_code, leftwrist_sph1 = vrep.simxGetObjectHandle(clientID,"leftwrist_sph1",vrep.simx_opmode_blocking)
err_code, leftwrist_sph2 = vrep.simxGetObjectHandle(clientID,"leftwrist_sph2",vrep.simx_opmode_blocking)
err_code, rightwrist_sph1 = vrep.simxGetObjectHandle(clientID,"rightwrist_sph1",vrep.simx_opmode_blocking)
err_code, rightwrist_sph2 = vrep.simxGetObjectHandle(clientID,"rightwrist_sph2",vrep.simx_opmode_blocking)

def go_to_default_pose():
    Ljoint_goal = leftgroup.get_current_joint_values()
    Ljoint_goal[0] = 0
    Ljoint_goal[1] = 0
    Ljoint_goal[2] = 0
    Ljoint_goal[3] = 0
    Ljoint_goal[4] = 0
    leftgroup.go(Ljoint_goal,wait=True)
    leftgroup.stop()
    Rjoint_goal = rightgroup.get_current_joint_values()
    Rjoint_goal[0] = 0
    Rjoint_goal[1] = 0
    Rjoint_goal[2] = 0
    Rjoint_goal[3] = 0
    Rjoint_goal[4] = 0
    rightgroup.go(Rjoint_goal,wait=True)
    rightgroup.stop()
    vrep.simxSetJointTargetPosition(clientID, leftarm_prismatic, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftarm_verticalpris, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftarm_rotx, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftarm_rotz, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftforearm_prismatic, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightarm_prismatic, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightarm_verticalpris, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightarm_rotx, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightarm_rotz, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightforearm_prismatic, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftwrist, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightwrist, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftwrist_sph1, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftwrist_sph2, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightwrist_sph1, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightwrist_sph2, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftupfinger, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, leftlowfinger, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightupfinger, 0, vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetPosition(clientID, rightlowfinger, 0, vrep.simx_opmode_blocking)


def get_dummy():
    global leftdummy0_pos, leftdummy0_ori, leftdummy1_pos, leftdummy1_ori, leftdummy2_pos, leftdummy2_ori, \
    rightdummy0_pos, rightdummy0_ori, rightdummy1_pos, rightdummy1_ori, rightdummy2_pos, rightdummy2_ori, rightdummy3_pos, rightdummy3_ori
    err_code, leftdummy0_pos = vrep.simxGetObjectPosition(clientID, leftdummy0, wheelbase, vrep.simx_opmode_blocking)
    err_code, leftdummy0_ori = vrep.simxGetObjectQuaternion(clientID, leftdummy0, wheelbase, vrep.simx_opmode_blocking)
    err_code, leftdummy1_pos = vrep.simxGetObjectPosition(clientID, leftdummy1, wheelbase, vrep.simx_opmode_blocking)
    err_code, leftdummy1_ori = vrep.simxGetObjectQuaternion(clientID, leftdummy1, wheelbase, vrep.simx_opmode_blocking)
    err_code, leftdummy2_pos = vrep.simxGetObjectPosition(clientID, leftdummy2, wheelbase, vrep.simx_opmode_blocking)
    err_code, leftdummy2_ori = vrep.simxGetObjectQuaternion(clientID, leftdummy2, wheelbase, vrep.simx_opmode_blocking)
    err_code, leftdummy3_pos = vrep.simxGetObjectPosition(clientID,leftdummy3,wheelbase,vrep.simx_opmode_blocking)
    err_code, leftdummy3_ori = vrep.simxGetObjectQuaternion(clientID,leftdummy3,wheelbase,vrep.simx_opmode_blocking)
    err_code, rightdummy0_pos = vrep.simxGetObjectPosition(clientID, rightdummy0, wheelbase, vrep.simx_opmode_blocking)
    err_code, rightdummy0_ori = vrep.simxGetObjectQuaternion(clientID, rightdummy0, wheelbase, vrep.simx_opmode_blocking)
    err_code, rightdummy1_pos = vrep.simxGetObjectPosition(clientID, rightdummy1, wheelbase, vrep.simx_opmode_blocking)
    err_code, rightdummy1_ori = vrep.simxGetObjectQuaternion(clientID, rightdummy1, wheelbase, vrep.simx_opmode_blocking)
    err_code, rightdummy2_pos = vrep.simxGetObjectPosition(clientID, rightdummy2, wheelbase, vrep.simx_opmode_blocking)
    err_code, rightdummy2_ori = vrep.simxGetObjectQuaternion(clientID, rightdummy2, wheelbase, vrep.simx_opmode_blocking)
    err_code, rightdummy3_pos = vrep.simxGetObjectPosition(clientID,rightdummy3,wheelbase,vrep.simx_opmode_blocking)
    err_code, rightdummy3_ori = vrep.simxGetObjectQuaternion(clientID,rightdummy3,wheelbase,vrep.simx_opmode_blocking)
def get_dummy_info(dummy_pos,dummy_ori):

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = dummy_ori[0]
    pose_goal.orientation.y = dummy_ori[1]
    pose_goal.orientation.z = dummy_ori[2]
    pose_goal.orientation.w = dummy_ori[3]
    pose_goal.position.x = dummy_pos[0]
    pose_goal.position.y = dummy_pos[1]
    pose_goal.position.z = dummy_pos[2]
    return pose_goal

def execute_motion(left_goal,right_goal):
    left_plan = leftgroup.plan(left_goal)
    right_plan = rightgroup.plan(right_goal)
    leftgroup.go()
    rightgroup.go()

    left_joint_state = []
    right_joint_state = []
    joint_trajectory_lftpris = []
    joint_trajectory_lftvpris = []
    joint_trajectory_lftrotx = []
    joint_trajectory_lftrotz = []
    joint_trajectory_lftfrm = []

    joint_trajectory_rgtpris = []
    joint_trajectory_rgtvpris = []
    joint_trajectory_rgtrotx = []
    joint_trajectory_rgtrotz = []
    joint_trajectory_rgtfrm = []
    print "=============leftplan=========="
    print left_plan
    print "=============rightplan========="
    print right_plan
    for i in range(len(left_plan.joint_trajectory.points)):
        left_joint_state.append(left_plan.joint_trajectory.points[i].positions)
    for j in range(len(right_plan.joint_trajectory.points)):
        right_joint_state.append(right_plan.joint_trajectory.points[j].positions)
    for i in range(len(left_plan.joint_trajectory.points)):
        joint_trajectory_lftpris.append(left_joint_state[i][0])
        joint_trajectory_lftvpris.append(left_joint_state[i][1])
        joint_trajectory_lftrotx.append(left_joint_state[i][2])
        joint_trajectory_lftrotz.append(left_joint_state[i][3])
        joint_trajectory_lftfrm.append(left_joint_state[i][4])
    for j in range(len(right_plan.joint_trajectory.points)):
        joint_trajectory_rgtpris.append(right_joint_state[j][0])
        joint_trajectory_rgtvpris.append(right_joint_state[j][1])
        joint_trajectory_rgtrotx.append(right_joint_state[j][2])
        joint_trajectory_rgtrotz.append(right_joint_state[j][3])
        joint_trajectory_rgtfrm.append(right_joint_state[j][4])
    for i in range(len(joint_trajectory_lftpris)):
        vrep.simxSetJointTargetPosition(clientID,leftarm_prismatic,joint_trajectory_lftpris[i],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,leftarm_verticalpris,joint_trajectory_lftvpris[i],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,leftarm_rotx,joint_trajectory_lftrotx[i],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,leftarm_rotz,joint_trajectory_lftrotz[i],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,leftforearm_prismatic,joint_trajectory_lftfrm[i],
                                        vrep.simx_opmode_streaming)
    for j in range(len(joint_trajectory_rgtpris)):
        vrep.simxSetJointTargetPosition(clientID, rightarm_prismatic, joint_trajectory_rgtpris[j],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID, rightarm_verticalpris, joint_trajectory_rgtvpris[j],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID, rightarm_rotx, joint_trajectory_rgtrotx[j],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID, rightarm_rotz, joint_trajectory_rgtrotz[j],
                                        vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID, rightforearm_prismatic, joint_trajectory_rgtfrm[j],
                                        vrep.simx_opmode_blocking)

    time.sleep(5)
######## left,right : bool#########
def close_hand(left,right,force):
    if left:
        vrep.simxSetJointForce(clientID,leftupfinger,force,vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(clientID,leftlowfinger,force,vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetPosition(clientID,leftupfinger,-0.1,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,leftlowfinger,0.1,vrep.simx_opmode_streaming)
    if right:
        vrep.simxSetJointForce(clientID,rightupfinger,force,vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(clientID,rightlowfinger,force,vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetPosition(clientID,rightupfinger,-0.1,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,rightlowfinger,0.1,vrep.simx_opmode_streaming)
def open_hand(left,right):
    if left:
        vrep.simxSetJointForce(clientID,leftupfinger,1000,vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(clientID,leftlowfinger,1000,vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetPosition(clientID,leftupfinger,0.1,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,leftlowfinger,-0.1,vrep.simx_opmode_streaming)
    if right:
        vrep.simxSetJointForce(clientID,rightupfinger,1000,vrep.simx_opmode_blocking)
        vrep.simxSetJointForce(clientID,rightlowfinger,1000,vrep.simx_opmode_blocking)
        vrep.simxSetJointTargetPosition(clientID,rightupfinger,0.1,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetPosition(clientID,rightlowfinger,-0.1,vrep.simx_opmode_streaming)
def lock_wrist():
    vrep.simxSetJointForce(clientID,leftwrist_sph1,10000,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,leftwrist_sph2,10000,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,rightwrist_sph1,10000,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,rightwrist_sph2,10000,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,leftwrist,10000,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,rightwrist,10000,vrep.simx_opmode_blocking)
def unlock_wrist():
    vrep.simxSetJointForce(clientID,leftwrist_sph1,0,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,leftwrist_sph2,0,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,rightwrist_sph1,0,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,rightwrist_sph2,0,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,leftwrist,0,vrep.simx_opmode_blocking)
    vrep.simxSetJointForce(clientID,rightwrist,0,vrep.simx_opmode_blocking)

###### direction ###### forward : 1, backward : 0 ########
def move(direction,speed):
    # vrep.simxSetJointForce(clientID,leftmotor,100,vrep.simx_opmode_blocking)
    # vrep.simxSetJointForce(clientID,rightmotor,100,vrep.simx_opmode_blocking)
    if direction == 1:
        vrep.simxSetJointTargetVelocity(clientID,leftwheel,speed,vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID,rightwheel,speed,vrep.simx_opmode_blocking)

    else:
        vrep.simxSetJointTargetVelocity(clientID, leftwheel, -speed, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightwheel, -speed, vrep.simx_opmode_blocking)
####### direction ####### right : 1, left : 0 #########
def turn(direction,speed):
    if direction == 1:
        vrep.simxSetJointTargetVelocity(clientID, leftwheel, -speed, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightwheel, speed, vrep.simx_opmode_blocking)
    else:
        vrep.simxSetJointTargetVelocity(clientID, leftwheel, speed, vrep.simx_opmode_streaming)
        vrep.simxSetJointTargetVelocity(clientID, rightwheel, -speed, vrep.simx_opmode_blocking)

def stop():
    vrep.simxSetJointTargetVelocity(clientID,leftwheel,0,vrep.simx_opmode_streaming)
    vrep.simxSetJointTargetVelocity(clientID, rightwheel, 0, vrep.simx_opmode_blocking)

(a, b, c, ultrasonic, d) = vrep.simxGetObjectGroupData(clientID, vrep.sim_object_proximitysensor_type, 13, vrep.simx_opmode_streaming)
dist=10
while(dist>1.1):
    move(1,0.5)
    (a,b,c,ultrasonic,d) = vrep.simxGetObjectGroupData(clientID, vrep.sim_object_proximitysensor_type, 13, vrep.simx_opmode_streaming)
    dist = ultrasonic[2]
    print(dist)

stop()
get_dummy()
left_pose_goal1 = get_dummy_info(leftdummy0_pos,leftdummy0_ori)
right_pose_goal1 = get_dummy_info(rightdummy0_pos,rightdummy0_ori)
left_pose_goal2 = get_dummy_info(leftdummy1_pos,leftdummy1_ori)
right_pose_goal2 = get_dummy_info(rightdummy1_pos,rightdummy1_ori)
left_pose_goal3 = get_dummy_info(leftdummy2_pos,leftdummy2_ori)
right_pose_goal3 = get_dummy_info(rightdummy2_pos,rightdummy2_ori)
left_pose_goal4 = get_dummy_info(leftdummy3_pos,leftdummy3_ori)
right_pose_goal4 = get_dummy_info(rightdummy3_pos,rightdummy3_ori)
print(left_pose_goal1)
lock_wrist()
execute_motion(left_pose_goal1,right_pose_goal1)
execute_motion(left_pose_goal2,right_pose_goal2)
close_hand(1,1,1000)
execute_motion(left_pose_goal3,right_pose_goal3)
move(0,1)
time.sleep(2)
stop()
unlock_wrist()
execute_motion(left_pose_goal4,right_pose_goal4)
close_hand(1,1,30)
time.sleep(2)
open_hand(1,1)
time.sleep(1)
move(0,1)
time.sleep(2)
stop()
lock_wrist()
go_to_default_pose()
# vrep.simxSetJointTargetPosition(clientID,leftarm_prismatic,)

#

# test = 1
# while test:
#     group.set_pose_target(pose_goal,"left_palm")
#     plan = group.plan()
# #     group.set_random_target()
# #     plan = group.plan()
#     if len(plan.joint_trajectory.points) != 0:
#         test = 0
# print group.get_current_joint_values()



# print plan.joint_trajectory.points()


# plan = group.plan().joint_trajectory.points
# print plan.joint_trajectory.points



# group.stop()
# group.clear_pose_targets()


# waypoints = []
# scale = 1
# wpose = group.get_current_pose().pose
# wpose.position.z -= scale*0.1
# wpose.position.y += scale*0.2
# waypoints.append(copy.deepcopy(wpose))
# wpose.position.x += scale * 0.1
# waypoints.append(copy.deepcopy(wpose))
# wpose.position.y -= scale * 0.1
# waypoints.append(copy.deepcopy(wpose))
# (plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
# group.execute(plan,wait=True)
# class wallEsub(object):
#     def __init__(self):
#         super(wallEsub,self).__init__()
#