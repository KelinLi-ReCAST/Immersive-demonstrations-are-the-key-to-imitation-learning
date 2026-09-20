import os
import pybullet as p
import pybullet_data
import numpy as np
import ctypes
import utils 
import matplotlib.pyplot as plt
#import multiprimport os
import pybullet as p
import pybullet_data
import numpy as np
import ctypes
import utils 
import triad_openvr
import matplotlib.pyplot as plt

np.set_printoptions(suppress=True)

joints = np.zeros(21)
hand_joints = np.zeros(20)
dll = ctypes.cdll.LoadLibrary
glove = dll('./libsenseglove.so') 
glove.get_hand_joints.restype = ctypes.POINTER(ctypes.c_int * 21)

def main(demons_num, demonstrator, force_flag, glove_calibration, panda_flag):
    kp = 0
    ki = 0
    # force_x_arr = []
    # force_y_arr = []
    # force_z_arr = []
    # time_arr = []
    p.connect(p.GUI)
    
    p.setGravity(0, 0, -10)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.7,
        cameraYaw=90.0,
        cameraPitch=-50.0,
        cameraTargetPosition=[0.0, 0.0, 0.0]
    )
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    handJointNameToID = {}
    handLinkNameToID = {}
    handRevoluteID = []
    joints_ids = []

    floorUid = p.loadURDF("plane.urdf")
    handUid = p.loadURDF('/home/kelin/workspace_kelin/LEAP_Hand_Sim/assets/leap_hand/robot.urdf',basePosition=[0,0,1],baseOrientation=[0,1,0,0])

    constraint_id = p.createConstraint(
        parentBodyUniqueId=handUid,
        parentLinkIndex=-1,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0.0, 0.0, 0.0],
        parentFramePosition=[0.0, 0.0, 0.0],
        childFramePosition=[0.0, 0.0, 1],
        parentFrameOrientation = [0,1,0,0])  

    for j in range(p.getNumJoints(handUid)):
        info = p.getJointInfo(handUid, j)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        handJointNameToID[jointName] = info[0]
        handLinkNameToID[info[12].decode('UTF-8')] = info[0]
        handRevoluteID.append(j)

    joints_ids.append(p.addUserDebugParameter(paramName='index_0',rangeMin=-0.314,rangeMax=2.23,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='index_1',rangeMin=-1.047,rangeMax=1.047,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='index_2',rangeMin=-0.506,rangeMax=1.885,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='index_3',rangeMin=-0.366,rangeMax=2.042,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='middle_0',rangeMin=-0.314,rangeMax=2.23,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='middle_1',rangeMin=-1.047,rangeMax=1.047,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='middle_2',rangeMin=-0.506,rangeMax=1.885,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='middle_3',rangeMin=-0.366,rangeMax=2.042,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='ring_0',rangeMin=-0.314,rangeMax=2.23,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='ring_1',rangeMin=-1.047,rangeMax=1.047,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='ring_2',rangeMin=-0.506,rangeMax=1.885,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='ring_3',rangeMin=-0.366,rangeMax=2.042,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='thumb_0',rangeMin=-0.349,rangeMax=2.094,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='thumb_1',rangeMin=-0.47,rangeMax=2.443,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='thumb_2',rangeMin=-1.20,rangeMax=1.90,startValue=0))
    joints_ids.append(p.addUserDebugParameter(paramName='thumb_3',rangeMin=-1.34,rangeMax=1.88,startValue=0))

    while 1:
        p.stepSimulation()
        joints_value = [p.readUserDebugParameter(param_id) for param_id in joints_ids]
        p.setJointMotorControlArray(handUid, handRevoluteID, p.POSITION_CONTROL, joints_value)
    glove_scale = np.load(os.getcwd()+"/data/glove_calibration/"+demonstrator+".npy")[:21]
    cali_min = np.load(os.getcwd()+"/data/glove_calibration/"+demonstrator+".npy")[21:]    
    flag = True

    while flag == True:
        # try:
            
            # finger_force = utils.hand_force_feedback(manoUid,manoJointNameToID, manoJoints,manoLinkNameToID,manoLinks)
            # finger_force_array.append(finger_force)
            # print(finger_force)
            # if force_flag == 'force':
            #     utils.hand_apply_force(glove,finger_force)
            # if panda_flag == True:
            #     utils.msg_to_robot(client, [force[0],force[1],force[2],0,0,0])

            # p.applyExternalForce(manoUid, -1, [0,0,0.9*10],[0,0,0], p.WORLD_FRAME) #The hand's weight is 0.9kg, 0.02kg for each link. 
            joints_temp = glove.get_hand_joints().contents
            for i in range(21):
                joints[i] = joints_temp[i]
            #print(joints)
            
            hand_joints = utils.hand_boundary(joints, glove_scale, cali_min)
            hand_joints1 = hand_joints[:4]
            hand_joints2 = hand_joints[4:16]
            hand_joints = np.hstack((hand_joints2,hand_joints1))
            print(hand_joints)

            p.stepSimulation()
            # values = [p.readUserDebugParameter(uid) for uid in slider_ids]
            utils.set_mano_position(handUid, handJointNameToID, handRevoluteID, hand_joints[:16])

            # p.changeConstraint(constraint_id,jointChildFrameOrientation = ori_tracker,maxForce = 1000)
            # p.applyExternalForce(manoUid, -1, force,[0,0,0], p.WORLD_FRAME)

            # pos = np.array(p.getLinkState(manoUid,0)[0])
            # ori = np.array(p.getLinkState(manoUid,0)[1])
            # ori = p.getEulerFromQuaternion(ori)
            # temp = [force[0], force[1], force[2], pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]
            # for i in range(20):
            #     temp.append(hand_joints[i])

            # obs.append(temp)
            # if p.getContactPoints(bodyA = objectUid, bodyB = trayUid):
            #     flag = False

            # try_time = try_time + 1
            # if try_time > 6000:
            #     fail_flag = True
            #     print("Time's up!")
            #     break
        # except:    
        #     print("Tracker Connection Problem!")
        #     input()
        #     fail_flag = True
        #     break

    else:
        p.disconnect()

    return flag
    

if __name__ == '__main__':
    glove.connect()



    #Settings for collecting demonstrations
    demonstrator = 'kelin'
    force = 'force'
    glove_calibration = False
    panda_flag = True
    start_num = 0

    # if panda_flag == True:
    #     client = utils.set_robot_arm()


    demons_num = start_num
    # while demons_num <5:
    #     flag = main(demons_num,demonstrator,force, glove_calibration,panda_flag)
    #     if not flag:
    #         demons_num = demons_num+1
    # glove.disconnect()    
    # os._exit(0)
    flag = main(demons_num,demonstrator,force, glove_calibration,panda_flag)