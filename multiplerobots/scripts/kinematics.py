import sys
import config
sys.path.append(config.RobotIKfast)

import elfin5.ikfastpy as ikelfin
# import ur5e.ikfastpy as ikur5

# Eflin Robot 
# ---------------------------------------------------------------------------------------------------------------------------
import numpy as np

def choose_robot(robot):
    if robot == 'Elfin':
        return ikelfin
    elif robot == 'UR5e':
        return ikur5

    else:
        return None

def ForwardKinematics(joint_angles, robot):
    """Forward Kinematics for a given joint angles(6 dof industrial Robot)

    Args:
        joint_angles (list of floats):  Give the joint angles of the robot
        
    Returns:
        ee_pose(numpy array of dimension(3*4)) : Translation of the TCP of the robot(T06 for a 6 dof robot)

    """
    
    ikrobot = choose_robot(robot)
    if ikrobot ==  None:
        print('the robot {}entered is not existing'.format(robot))
        return
    

    # Initialize kinematics for UR5 robot arm
    robot_kin = ikrobot.PyKinematics()

    #to be tuned
    joint_angles = np.deg2rad(joint_angles)

    ee_pose = robot_kin.forward(joint_angles)
    ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
#     print("\nEnd effector pose:")
#     print(ee_pose)
    return ee_pose






def InverseKinematics(ee_pose, robot):
    """Ibnverse Kinematics of the given ee_pose.

    Args:
        ee_pose (numpy array of dimension(3*4)): Translation of the TCP of the robot(T06 for a 6 dof robot)

    Returns:
        (numpy array of floats):  array of joint angles of the robot solved by ik solver(8 solutions for a 6 dof robot)
    """

    ikrobot = choose_robot(robot)
    if ikrobot ==  None:
        print('the robot {}entered is not existing'.format(robot))
    
    robot_kin = ikrobot.PyKinematics()
    n_joints = robot_kin.getDOF()
    
    joint_configs = robot_kin.inverse(ee_pose.reshape(-1).tolist())
    n_solutions = int(len(joint_configs)/n_joints)
    
    print("%d solutions found:"%(n_solutions))
    joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)
    
    return joint_configs


# testing_purpose
# pose = ForwardKinematics(np.deg2rad([0,0,-90,90,-90,0]))   
# ik_sols = InverseKinematics(pose)
# print(pose, ik_sols)