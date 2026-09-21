#Revolute joints: palm_index1_0_joint, index1_0_index1_joint, index1_index2_joint, index2_index3_joint
#                 palm_mid1_0_joint, mid1_0_mid1_joint, mid1_mid2_joint, mid2_mid3_joint
#                 palm_ring1_0_joint, ring1_0_ring1_joint, ring1_ring2_joint, ring2_ring3_joint
#                 palm_pinky1_0_joint, pinky1_0_pinky1_joint, pinky1_pinky2_joint, pinky2_pinky3_joint
#                 palm_thumb1_0_joint, thumb1_0_thumb1_joint, thumb1_thumb2_joint, thumb2_thumb3_joint
import os
REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
DATA_DIR = os.path.join(REPO_DIR, 'data', 'RUTH')
VIDEO_DIR = os.path.join(REPO_DIR, 'media', 'videos', 'RUTH')
import pybullet as p
import pybullet_data
import numpy as np
import ctypes
import utils 
import triad_openvr
import matplotlib.pyplot as plt
#import multiprimport os
import pybullet as p
import pybullet_data
import numpy as np
import ctypes
import utils 
import triad_openvr
import matplotlib.pyplot as plt
#import multiprocessingocessing

np.set_printoptions(suppress=True)

joints = np.zeros(21)
hand_joints = np.zeros(20)
dll = ctypes.cdll.LoadLibrary
glove = dll(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'libsenseglove.so')) 
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

    floorUid = p.loadURDF("plane.urdf")
    tableUid = p.loadURDF("table/table.urdf",basePosition=[0.5,0,0])
    trayUid = p.loadURDF("tray/traybox.urdf",basePosition=[0.65,0,0.65])
    #objectUid = p.loadURDF("/home/kelin/workspace_kelin/RAL-ICRA2023/iGibson/igibson/mano_pybullet/mano_pybullet/tools/YcbBanana/model.urdf",basePosition=[0,0,0.66])
    objectUid = p.loadURDF("duck_vhacd.urdf",basePosition=[0,0,0.66],globalScaling=1)
    manoJointNameToID = {}
    manoLinkNameToID = {}
    manoRevoluteID = []

    manoUid = p.loadURDF(os.path.join(REPO_DIR,'assets','RUTH','urdf','RUTH.urdf'),basePosition=[0,0,1],baseOrientation = [0,0,0,1])
    
    for j in range(p.getNumJoints(manoUid)):
        info = p.getJointInfo(manoUid, j)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        manoJointNameToID[jointName] = info[0]
        manoLinkNameToID[info[12].decode('UTF-8')] = info[0]
        manoRevoluteID.append(j)
    utils.initialize_RUTH(manoUid, manoLinkNameToID, manoJointNameToID)

    constraint_id = p.createConstraint(
        parentBodyUniqueId=manoUid,
        parentLinkIndex=0,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0.0, 0.0, 0.0],
        parentFramePosition=[0.0, 0.0, 0.0],
        childFramePosition=[0.0, 0.0, 1],
        parentFrameOrientation = [0,0,0,1])

    
    manoJoints = []
    manoLinks = []
    slider_ids = []


    for i, linkname in enumerate(manoLinkNameToID):
        p.changeDynamics(manoUid,manoLinkNameToID[linkname],mass=0.06)
    p.changeDynamics(manoUid,-1,mass=0.1)

    if glove_calibration:
        glove_scale, cali_min = utils.hand_calibration(glove,demonstrator)
    else:
        glove_scale = np.load(DATA_DIR+"/glove_calibration/"+demonstrator+".npy")[:21]
        cali_min = np.load(DATA_DIR+"/glove_calibration/"+demonstrator+".npy")[21:]

    v = triad_openvr.triad_openvr()
    x_c,y_c,z_c = utils.tracker_calibration(v)
    #time = 0
    #plt.ion()
    #plt.figure()
    #pool = multiprocessing.Pool(2)
    x_rot,y_rot,z_rot = 3.14,0,0
    obs = []
    finger_force_array = []
    flag = True
    fail_flag = False
    if panda_flag == True:
        log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, VIDEO_DIR+"/"+"panda"+"/"+demonstrator+"_"+str(demons_num)+".mp4")
    else:
        log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, VIDEO_DIR+"/"+force_flag+"/"+demonstrator+"_"+str(demons_num)+".mp4")
    try_time = 0

    while flag == True:
        # try:
            kp = kp + 0.5
            ki = ki + 0.1
            if kp >300:
                kp = 300
            if ki > 30:
                ki = 30
            print(kp,ki)
            temp = []
            pos_tracker,ori_tracker,force,torque,x_rot,y_rot,z_rot = utils.tracker(v,kp,ki,x_c,y_c,z_c,manoUid,x_rot,y_rot,z_rot)
            ori_tracker_euler = p.getEulerFromQuaternion(ori_tracker)
            finger_force = utils.hand_force_feedback(manoUid,manoJointNameToID, manoJoints,manoLinkNameToID,manoLinks)
            finger_force_array.append(finger_force)
            print(finger_force)
            if force_flag == 'force':
                utils.hand_apply_force(glove,finger_force)

            if panda_flag == True:
                utils.msg_to_robot(client, [force[0],force[1],force[2],0,0,0])

            # p.applyExternalForce(manoUid, -1, [0,0,1*10],[0,0,0], p.WORLD_FRAME) #The hand's weight is 0.9kg, 0.02kg for each link. 
            joints_temp = glove.get_hand_joints().contents
            for i in range(21):
                joints[i] = joints_temp[i]
            #print(joints)
            
            hand_joints = utils.hand_boundary(joints, glove_scale, cali_min)
            mano_joints = [hand_joints[3],hand_joints[11],hand_joints[7]]
            p.stepSimulation()
            
            utils.motor_control_ruth(manoUid, manoJointNameToID, mano_joints)

            p.changeConstraint(constraint_id,jointChildPivot = pos_tracker,jointChildFrameOrientation = ori_tracker,maxForce = 1000)
            # p.applyExternalForce(manoUid, -1, force,[0,0,0], p.WORLD_FRAME)

            pos = np.array(p.getLinkState(manoUid,0)[0])
            ori = np.array(p.getLinkState(manoUid,0)[1])
            ori = p.getEulerFromQuaternion(ori)
            temp = [force[0], force[1], force[2], pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]
            for i in range(3):
                temp.append(mano_joints[i])

            obs.append(temp)
            if p.getContactPoints(bodyA = objectUid, bodyB = trayUid):
                flag = False
            # time_arr.append(time*0.001)
            # force_x_arr.append(force[0])
            # force_y_arr.append(force[1])
            # force_z_arr.append(force[2])
            # if len(force_x_arr)>5000:
            #    del time_arr[0]
            #    del force_x_arr[0]
            #    del force_y_arr[0]
            #    del force_z_arr[0]
            #pool.apply_async(func=utils.plot, args=(time_arr,force_x_arr,force_y_arr,force_z_arr))disconnect
            # time = time+1
            p.stepSimulation()
            try_time = try_time + 1
            if try_time > 6000:
                fail_flag = True
                print("Time's up!")
                break

        # except:    
        #     print("Tracker Connection Problem!")
        #     input()
        #     fail_flag = True
        #     break
        
    # pool.close()
    # pool.join()
    # UDPSock.close()
    
    if fail_flag == False:
        
        p.stopStateLogging(log_id)
        p.disconnect()
        if panda_flag == False:
            np.save(DATA_DIR+"/"+force_flag+"/"+demonstrator+"_"+str(demons_num), np.array(obs))
            np.save(DATA_DIR+"/"+force_flag+"/"+"finger_force_"+demonstrator+"_"+str(demons_num), np.array(finger_force_array))
        else:
            np.save(DATA_DIR+"/"+"panda"+"/"+demonstrator+"_"+str(demons_num), np.array(obs))
            np.save(DATA_DIR+"/"+"panda"+"/"+"finger_force_"+demonstrator+"_"+str(demons_num), np.array(finger_force_array))

    else:
        p.disconnect()

    return fail_flag
    

if __name__ == '__main__':
    glove.connect()



    #Settings for collecting demonstrations
    demonstrator = 'kelin'
    force = 'force'
    glove_calibration = False
    panda_flag = True
    start_num = 0


    if panda_flag == True:
        client = utils.set_robot_arm()

    demons_num = start_num
    while demons_num <5:
        flag = main(demons_num,demonstrator,force, glove_calibration, panda_flag)
        if not flag:
            demons_num = demons_num+1
    glove.disconnect()    
    os._exit(0)
    