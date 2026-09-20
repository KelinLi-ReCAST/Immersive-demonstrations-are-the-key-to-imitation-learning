#Revolute joints: palm_index1_0_joint, index1_0_index1_joint, index1_index2_joint, index2_index3_joint
#                 palm_mid1_0_joint, mid1_0_mid1_joint, mid1_mid2_joint, mid2_mid3_joint
#                 palm_ring1_0_joint, ring1_0_ring1_joint, ring1_ring2_joint, ring2_ring3_joint
#                 palm_pinky1_0_joint, pinky1_0_pinky1_joint, pinky1_pinky2_joint, pinky2_pinky3_joint
#                 palm_thumb1_0_joint, thumb1_0_thumb1_joint, thumb1_thumb2_joint, thumb2_thumb3_joint
import os
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

    floorUid = p.loadURDF("plane.urdf")
    tableUid = p.loadURDF("table/table.urdf",basePosition=[0.5,0,0])
    trayUid = p.loadURDF("tray/traybox.urdf",basePosition=[0.65,0,0.65])
    #objectUid = p.loadURDF("/home/kelin/workspace_kelin/RAL-ICRA2023/iGibson/igibson/mano_pybullet/mano_pybullet/tools/YcbBanana/model.urdf",basePosition=[0,0,0.66])
    objectUid = p.loadURDF("duck_vhacd.urdf",basePosition=[0,0,0.66],globalScaling=1)
    manoJointNameToID = {}
    manoLinkNameToID = {}
    manoRevoluteID = []

    manoUid = p.loadURDF('franka_panda/panda.urdf',basePosition=[0,0,1],baseOrientation=[0,0,0,1])
    
    for j in range(3):
        info = p.getJointInfo(manoUid, j)
        jointID = info[0]
        jointName = info[1].decode('UTF-8')
        jointType = info[2]
        manoJointNameToID[jointName] = info[0]
        manoLinkNameToID[info[12].decode('UTF-8')] = info[0]
        manoRevoluteID.append(j)
    
    constraint_id = p.createConstraint(
        parentBodyUniqueId=manoUid,
        parentLinkIndex=-1,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_PRISMATIC,
        jointAxis=[0.0, 0.0, 0.0],
        parentFramePosition=[0.0, 0.0, 0.0],
        childFramePosition=[0.0, 0.0, 1],
        parentFrameOrientation = [0,0,0,1])

    manoJoints = ['panda_finger_joint1']

    manoLinks = ['panda_leftfinger','panda_rightfinger','panda_grasptarget']
    slider_ids = []

    # for i, joint in enumerate(manoJoints):
    #         name = joint
    #         uid = p.addUserDebugParameter(f'{name}', 0, 3, 0)
    #         slider_ids.append(uid)
   
    p.changeDynamics(manoUid,-1,mass=0.25)
    for i, linkname in enumerate(manoLinks):
        p.changeDynamics(manoUid,manoLinkNameToID[manoLinks[i]],mass=0.25)

    if glove_calibration:
        glove_scale, cali_min = utils.hand_calibration(glove,demonstrator)
    else:
        glove_scale = np.load(os.getcwd()+"/data/glove_calibration/"+demonstrator+".npy")[:21]
        cali_min = np.load(os.getcwd()+"/data/glove_calibration/"+demonstrator+".npy")[21:]

    v = triad_openvr.triad_openvr()
    x_c,y_c,z_c = utils.tracker_calibration(v)
    #time = 0
    #plt.ion()
    #plt.figure()
    #pool = multiprocessing.Pool(2)
    
    x_rot,y_rot,z_rot = 0,0,0
    obs = []
    finger_force_array = []
    flag = True
    fail_flag = False
    if panda_flag == False:
        log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, os.getcwd()+"/video/"+force_flag+"/"+demonstrator+"_"+str(demons_num)+".mp4")
    else:
        log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, os.getcwd()+"/video/"+"panda"+"/"+demonstrator+"_"+str(demons_num)+".mp4")
    try_time = 0
    while flag == True:
        try:
            kp = kp + 0.5
            ki = ki + 0.1
            if kp >500:
                kp = 500
            if ki > 50:
                ki = 50
            # print(kp,ki)
            temp = []
            pos_tracker,ori_tracker,force,torque,x_rot,y_rot,z_rot = utils.tracker(v,kp,ki,x_c,y_c,z_c,manoUid,x_rot,y_rot,z_rot)
            ori_tracker_euler = p.getEulerFromQuaternion(ori_tracker)
            
            finger_force = utils.hand_force_feedback(manoUid,manoJointNameToID, manoJoints,manoLinkNameToID,manoLinks)/4
            finger_force_array.append(finger_force)
            print(finger_force)
            if force_flag == 'force':
                utils.hand_apply_force(glove,finger_force)

            if panda_flag == True:
                utils.msg_to_robot(client, [force[0],force[1],force[2],torque[0],torque[1],torque[2]])

            p.applyExternalForce(manoUid, -1, [0,0,1*10],[0,0,0], p.WORLD_FRAME) #The hand's weight is 0.9kg, 0.02kg for each link. 
            joints_temp = glove.get_hand_joints().contents
            for i in range(21):
                joints[i] = joints_temp[i]
            #print(joints)
            
            hand_joints = utils.hand_boundary(joints, glove_scale, cali_min)
            
            #print(hand_joints)
            p.stepSimulation()
            #values = [p.readUserDebugParameter(uid) for uid in slider_ids]
            franka_joint = hand_joints[5]+0.04
            utils.set_mano_position(manoUid, manoJointNameToID, manoJoints, franka_joint)
            

            p.changeConstraint(constraint_id,jointChildFrameOrientation = ori_tracker,maxForce = 1000)
            p.applyExternalForce(manoUid, -1, force,[0,0,0], p.WORLD_FRAME)

            pos = np.array(p.getLinkState(manoUid,0)[0])
            ori = np.array(p.getLinkState(manoUid,0)[1])
            ori = p.getEulerFromQuaternion(ori)
            temp = [force[0], force[1], force[2], pos[0],pos[1],pos[2],ori[0],ori[1],ori[2]]
            temp.append(franka_joint)

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
            
            try_time = try_time + 1
            # print(try_time)
            if try_time > 6000:
                fail_flag = True
                print("Time's up!")
                break
        except:    
            print("Tracker Connection Problem!")
            input()
            fail_flag = True
            break
        
    # pool.close()
    # pool.join()
    # UDPSock.close()
    
    if fail_flag == False:
        
        p.stopStateLogging(log_id)
        p.disconnect()
        if panda_flag == False:
            np.save(os.getcwd()+"/data/"+force_flag+"/"+demonstrator+"_"+str(demons_num), np.array(obs))
            np.save(os.getcwd()+"/data/"+force_flag+"/"+"finger_force_"+demonstrator+"_"+str(demons_num), np.array(finger_force_array))
        else:
            np.save(os.getcwd()+"/data/"+"panda"+"/"+demonstrator+"_"+str(demons_num), np.array(obs))
            np.save(os.getcwd()+"/data/"+"panda"+"/"+"finger_force_"+demonstrator+"_"+str(demons_num), np.array(finger_force_array))
            

    else:
        p.disconnect()

    return fail_flag
    

if __name__ == '__main__':
    glove.connect()
    


    #Settings for collecting demonstrations
    demonstrator = 'kelin'
    force = 'force'
    glove_calibration = False
    panda_flag = False
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
    