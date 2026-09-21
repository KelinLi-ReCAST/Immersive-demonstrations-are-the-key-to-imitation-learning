import gym
from gym import spaces
from gym.utils import seeding
import numpy as np

import pybullet as p
import pybullet_data
import time
import math
manoJoints = ['palm_thumb1_0_joint', 'thumb1_0_thumb1_joint', 'thumb1_thumb2_joint', 'thumb2_thumb3_joint',
                'palm_index1_0_joint', 'index1_0_index1_joint', 'index1_index2_joint', 'index2_index3_joint',
                'palm_mid1_0_joint', 'mid1_0_mid1_joint', 'mid1_mid2_joint', 'mid2_mid3_joint',
                'palm_ring1_0_joint', 'ring1_0_ring1_joint', 'ring1_ring2_joint', 'ring2_ring3_joint',
                'palm_pinky1_0_joint', 'pinky1_0_pinky1_joint', 'pinky1_pinky2_joint', 'pinky2_pinky3_joint']

manoLinks = ['palm', 'index1_0_joint', 'index1_0', 'index1_joint', 'index1', 'index2_joint', 'index2', 
                 'index3_joint', 'index3', 'mid1_0_joint', 'mid1_0', 'mid1_joint', 'mid1', 'mid2_joint', 'mid2', 
                 'mid3_joint', 'mid3', 'ring1_0_joint', 'ring1_0', 'ring1_joint', 'ring1', 'ring2_joint', 'ring2', 
                 'ring3_joint', 'ring3', 'pinky1_0_joint', 'pinky1_0', 'pinky1_joint', 'pinky1', 'pinky2_joint', 
                 'pinky2', 'pinky3_joint', 'pinky3', 'thumb1_0_joint', 'thumb1_0', 'thumb1_joint', 'thumb1_dh', 
                 'thumb1', 'thumb2_joint', 'thumb2_dh', 'thumb2', 'thumb3_joint', 'thumb3_dh', 'thumb3']

def hand_force_feedback(manoUid,objectUid,manoJointNameToID, manoJoints,manoLinkNameToID,manoLinks):
    temp_data = []
    contact_flag = False
    torques = np.zeros(15)
    positions = np.zeros(15)
    finger_force = np.zeros(5)
    d = 0.0085
    links = [0.0794,0.0397,0.046,0.027,0.0573,0.029,0.0526,0.0283,0.0433,0.0266]
    for i in range(44):
        if p.getContactPoints(bodyA = manoUid, bodyB = objectUid,linkIndexA = manoLinkNameToID[manoLinks[i]]):
            contact_flag = True
            continue
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[1]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[2]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[3]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[5]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[6]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[7]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[9]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[10]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[11]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[13]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[14]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[15]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[17]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[18]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[19]]))
    for i in range(15):
        torques[i] = abs((temp_data[i])[3])
        positions[i] = abs((temp_data[i])[0])
        if contact_flag == False:
            torques[i] = 0
    finger_force[0] = (torques[0]+torques[1]+torques[2])/(3*d+2*links[1]*math.cos(positions[1])+links[0]*math.cos(positions[1]+positions[2]))
    finger_force[1] = (torques[3]+torques[4]+torques[5])/(3*d+2*links[3]*math.cos(positions[4])+links[2]*math.cos(positions[4]+positions[5]))
    finger_force[2] = (torques[6]+torques[7]+torques[8])/(3*d+2*links[5]*math.cos(positions[7])+links[4]*math.cos(positions[7]+positions[8]))
    finger_force[3] = (torques[9]+torques[10]+torques[11])/(3*d+2*links[7]*math.cos(positions[10])+links[6]*math.cos(positions[10]+positions[11]))
    finger_force[4] = (torques[12]+torques[13]+torques[14])/(3*d+2*links[9]*math.cos(positions[13])+links[8]*math.cos(positions[13]+positions[14]))
    return finger_force

def set_mano_joints(manoUid, manoJointNameToID, manoJoints, mano_joints):  # set joints of mano
    for i in range(20):
        p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[i]], p.POSITION_CONTROL, mano_joints[i])

def get_mano_joints(manoUid, manoJointNameToID):  # set joints of mano
    joints = np.zeros(20)
    for i in range(20):
        joints[i] = p.getJointState(manoUid, manoJointNameToID[manoJoints[i]])[0]
    return joints

def set_mano_position(constraint_id, position, orientation): # set position of mano
    p.changeConstraint(constraint_id,jointChildPivot = position,jointChildFrameOrientation = orientation, maxForce = 1000)

# def set_mano_position(constraint_id, position, orientation): # set position of mano
#     p.changeConstraint(constraint_id,jointChildFrameOrientation = orientation, maxForce = 1000)

class mano(gym.Env):

    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : 50
    }

    def __init__(self, version):
       
        #========= Initialize environment
        if version == 'GUI':
            physicsClient = p.connect(p.GUI)
        elif version == 'DIRECT':
            physicsClient = p.connect(p.DIRECT) #non-graphical version
        

        # 
        p.setGravity(0, 0, -10)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.7,
            cameraYaw=-40.0,
            cameraPitch=-40.0,
        cameraTargetPosition=[0.0, 0.0, 0.0]
        )
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setRealTimeSimulation(0) #0: disable

        #===== Initialize plane and import robot
        FixedBase = False #if fixed no plane is imported
        if (FixedBase == False):
            floor = p.loadURDF("plane.urdf")

        floorUid = p.loadURDF("plane.urdf")
        tableUid = p.loadURDF("table/table.urdf",basePosition=[0.5,0,0])
        trayUid = p.loadURDF("tray/traybox.urdf",basePosition=[0.65,0,0.65])
        objectUid = p.loadURDF("duck_vhacd.urdf",basePosition=[0,0,0.66],globalScaling=1)
        manoUid = p.loadURDF('/home/kelin/workspace_kelin/RAL-ICRA2023/third_party/GraspIt2URDF-master/urdf/HumanHand20DOF.urdf',basePosition=[0,0,1])

        #===== Save mano joint and link information
        manoJointNameToID = {}
        manoLinkNameToID = {}
        manoRevoluteID = []

        for j in range(44):
            info = p.getJointInfo(manoUid, j)
            jointID = info[0]
            jointName = info[1].decode('UTF-8')
            jointType = info[2]
            manoJointNameToID[jointName] = info[0]
            manoLinkNameToID[info[12].decode('UTF-8')] = info[0]
            manoRevoluteID.append(j)

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

        for i, linkname in enumerate(manoLinks):
            p.changeDynamics(manoUid,manoLinkNameToID[manoLinks[i]],mass=0.02)
        p.changeDynamics(manoUid,-1,mass=0.02)
        #=======Start simulation
        self.robot = manoUid
        self.object = objectUid
        self.target = trayUid
        self.linkNameToID = manoLinkNameToID
        self.jointNameToID = manoJointNameToID
        self.constraint_id = constraint_id

        self._mano_init = [0.75 , 0.1342 , 0.3 , 0.5106 , 0., 0.2597 , 0.225 , 0.27 , 0. , 0.3351 , 0., 0.1375 , 0.2045 , 0.2333 , 0.0 , 0.0667 , 0.0136 , 0.0 , 0.0694,0.0682]
        self._mano_pos_init = [-0.08137586,0.0002766,0.99972078]
        self._mano_ori_init = [0,0,0,1]
        
        
        self.version = version 


        self.action_space = spaces.Box(np.array([-1.,]*26), np.array([1.,]*26))
        self.observation_space = spaces.Box(np.array([-1.]*26), np.array([1.]*26))
        for link in manoLinks:
            p.setCollisionFilterGroupMask(manoUid, manoLinkNameToID[link], 1, 0)
        p.setCollisionFilterGroupMask(tableUid, -1, 1, 0)
        # self.reset_init()
        
        
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action, flag=True, count=0):

        #========================Initialization
        r = 0
        d = False
        contact = False
        #action = np.clip(action, -1., 1.)
        #assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))

        obs = self.get_obs() 
        mano_pos = action[:3]#+obs[:3]#- np.array([-0.08137444909035871,0.000313143494664086,0.001115205383354])
        mano_ori = action[3:6]#+ obs[3:6]
        mano_joints = action[6:]#+obs[6]
        pos_hand = np.array(p.getLinkState(self.robot, 0)[0])
        vel_hand = np.array(p.getLinkState(self.robot, 0, 1)[6])
        palm_force = 300*(mano_pos-pos_hand)+30*(-vel_hand)
        r = np.sqrt(palm_force[0]*palm_force[0]+palm_force[1]*palm_force[1]+palm_force[2]*palm_force[2])/3  
        #========================Move robot
        mano_ori = p.getQuaternionFromEuler(mano_ori)
        set_mano_position(self.constraint_id,mano_pos,mano_ori)
        set_mano_joints(self.robot, self.jointNameToID, manoJoints, mano_joints)
        for i in range(1):
            p.stepSimulation()
            
        obs = 0
        if p.getContactPoints(bodyA = self.object, bodyB = self.target) or (p.getLinkState(self.object,0)[0])[2]>0.7:
            obs = 100 
        # finger_force = hand_force_feedback(self.robot,self.object,self.jointNameToID, manoJoints,self.linkNameToID,manoLinks)
        # r = np.mean(finger_force)
        return obs, r, d, {}
        
    def reset(self, demo_obs_init,flag=True, count=0):
        set_mano_position(self.constraint_id,demo_obs_init[:3],demo_obs_init[3:6])
        set_mano_joints(self.robot, self.jointNameToID, manoJoints, demo_obs_init[6:])
        p.removeBody(self.object)
        self.object = p.loadURDF("duck_vhacd.urdf",basePosition=[0,0,0.66],globalScaling=1)  
        for i in range(1000):
            p.stepSimulation()
        
        


        state = self.get_obs()
        return state
    
    def my_norm_obs(self, obs):
        obs = obs / self._MAX_OBS_IN_DEMO
        return obs.copy()
    
    def reset_init(self, flag=True, count=0):
        set_mano_position(self.constraint_id,self._mano_pos_init,self._mano_ori_init)
        set_mano_joints(self.robot, self.jointNameToID, manoJoints, self._obs_demo[0,6:])
        for i in range(1000):
            p.stepSimulation()
        state = self.get_obs()
        self.STATE = state
        self.INIT_STATE = state.copy()
        self.t = 0

    def get_obs(self):
        pos = np.array(p.getLinkState(self.robot,0)[0]) #- np.array([-0.08137444909035871,0.000313143494664086,0.001115205383354])
        ori = np.array(p.getLinkState(self.robot,0)[1])
        joints = get_mano_joints(self.robot,self.jointNameToID)
        ori = p.getEulerFromQuaternion(ori)
        return np.concatenate((pos, ori, joints)).copy()

if __name__ == "__main__":
    import gym
    ro = gym.make("imitation-v2", version="DIRECT")
    sta = ro.reset() 
    ob = ro.get_obs()
    print(ob)
    for i in range(500):
        a = ro.action_space.sample() #* 0 + 0.1
        o, r, d, _ = ro.step(a)
        # print(o)
        if i % 20 == 0:
            ro.reset() 
        time.sleep(1./20.)
    sta = ro.reset() 
    ob = ro.get_obs()
    print(ob)
    pass 
