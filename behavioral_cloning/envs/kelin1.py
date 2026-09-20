import gym
from gym import spaces
from gym.utils import seeding
import numpy as np

import pybullet as p
import pybullet_data
import time

manoJoints = ['panda_finger_joint1']

manoLinks = ['panda_leftfinger','panda_rightfinger','panda_grasptarget']

def set_mano_joints(manoUid, manoJointNameToID, manoJoints, mano_joints):  # set joints of mano
    p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[0]], p.POSITION_CONTROL, mano_joints,force = 1000)
    p.setJointMotorControl2(manoUid, manoJointNameToID['panda_finger_joint2'], p.POSITION_CONTROL, mano_joints,force = 1000)

def get_mano_joints(manoUid, manoJointNameToID):  # set joints of mano
    joints = p.getJointState(manoUid, manoJointNameToID[manoJoints[0]])[0]
    return joints

def set_mano_position(constraint_id, position, orientation): # set position of mano
    p.changeConstraint(constraint_id,jointChildPivot = position,jointChildFrameOrientation = orientation, maxForce = 1000)

class franka(gym.Env):

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
            cameraDistance=0.5,
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
        manoUid = p.loadURDF('/franka_panda/panda.urdf',basePosition=[0,0,1],baseOrientation=[0,0,0,1])

        #===== Save mano joint and link information
        manoJointNameToID = {}
        manoLinkNameToID = {}
        manoRevoluteID = []

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
        parentLinkIndex=0,
        childBodyUniqueId=-1,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0.0, 0.0, 0.0],
        parentFramePosition=[0.0, 0.0, 0.0],
        childFramePosition=[0.0, 0.0, 1],
        parentFrameOrientation = [0,0,0,1])

        p.changeDynamics(manoUid,-1,mass=0.25)
        for i, linkname in enumerate(manoLinks):
            p.changeDynamics(manoUid,manoLinkNameToID[manoLinks[i]],mass=0.25)
        #=======Start simulation
        self.robot = manoUid
        self.object = objectUid
        self.target = trayUid
        self.linkNameToID = manoLinkNameToID
        self.jointNameToID = manoJointNameToID
        self.constraint_id = constraint_id

        self._mano_init = 0.04
        self._mano_pos_init = [0.0, 0.01, 0.9216]
        self._mano_ori_init = [0,1,0,0]
        self.version = version 


        self.action_space = spaces.Box(np.array([-0.05,]*7), np.array([0.05,]*7))
        self.observation_space = spaces.Box(np.array([0.0]*7), np.array([1.0]*7))
        
        self.max_delta_action = 1
        
        
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):

        #========================Initialization
        r = 0
        d = False
        finger_force = 0
        #action = np.clip(action, -1, 1) 
        #assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        #action = action * self.max_delta_action 


        # if p.getContactPoints(bodyA = self.robot,bodyB = self.object):
        #     r = p.getContactPoints(bodyA = self.robot)[0][9]



        target = np.array([1,0,1,1.57,0,0])
        obs = self.get_obs() 
        mano_pos = action[:3]#+obs[:3]#- np.array([-0.08137444909035871,0.000313143494664086,0.001115205383354])
        mano_ori = action[3:6]#+ obs[3:6]
        mano_joints = action[6]#+obs[6]
        pos_hand = np.array(p.getLinkState(self.robot, 0)[0])
        vel_hand = np.array(p.getLinkState(self.robot, 0, 1)[6])
        palm_force = 1500*(mano_pos-pos_hand)+150*(-vel_hand)
        r = np.sqrt(palm_force[0]*palm_force[0]+palm_force[1]*palm_force[1]+palm_force[2]*palm_force[2])/3        
        #========================Move robot
        mano_ori = p.getQuaternionFromEuler(mano_ori)
        set_mano_position(self.constraint_id,mano_pos,mano_ori)
        set_mano_joints(self.robot, self.jointNameToID, manoJoints, mano_joints)
        for i in range(1):
            p.stepSimulation()
        
        
        obs = self.get_obs() 
        #========================Error computation


        #========================Boundaries: 2m*2m*1.35m
        
        # for i in range (len(mano_ori)):
        #     if mano_ori[i] >3.14 or mano_ori[i] <-3.14:
        #         d = True
        #         r-=10
        # for i in range (len(mano_pos)-1):
        #     if mano_pos[i] >1.5 or mano_pos[i]<-0.5:
        #         d = True
        #         r-=10
        # if mano_pos[2] >2 or mano_pos[2]<0.65:
        #         d = True
        #         r-=10

        obs = 0

        if p.getContactPoints(bodyA = self.object, bodyB = self.target) or (p.getLinkState(self.object,0)[0])[2]>0.7:
            obs = 100    
        return obs, r, d, {} 
        
    def reset(self,demo_obs_init):
        set_mano_position(self.constraint_id,demo_obs_init[:3],demo_obs_init[3:6])
        set_mano_joints(self.robot, self.jointNameToID, manoJoints, demo_obs_init[6:])
        p.removeBody(self.object)
        self.object = p.loadURDF("duck_vhacd.urdf",basePosition=[0,0,0.66],globalScaling=1)
        for i in range(1000):
            p.stepSimulation()
        state = self.get_obs()

        return state

    def get_obs(self):
        kp = 100
        ki = 20
        pos = np.array(p.getLinkState(self.robot,0)[0]) #- np.array([-0.08137444909035871,0.000313143494664086,0.001115205383354])
        ori = np.array(p.getLinkState(self.robot,0)[1])
        joints = get_mano_joints(self.robot,self.jointNameToID)
        ori = p.getEulerFromQuaternion(ori)

        return np.array([pos[0],pos[1],pos[2],ori[0],ori[1],ori[2], joints])

if __name__ == "__main__":
    import gym
    ro = gym.make("imitation-v0", version="DIRECT")
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