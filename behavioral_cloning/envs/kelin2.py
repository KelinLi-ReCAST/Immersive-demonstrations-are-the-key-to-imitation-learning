import gym
from gym import spaces
from gym.utils import seeding
import numpy as np

import pybullet as p
import pybullet_data
import time

def motor_control_ruth(_robot, _RUTH_jointNameToID, _ruth_base_motors):
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Link_1'], p.POSITION_CONTROL, _ruth_base_motors[0])
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Link_3'], p.POSITION_CONTROL, _ruth_base_motors[1])  

    # Calculate relative individual finger joint movements
    finger_angles = [0, 0, 0]
    finger_pos_plus = 0.12745044
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_1A'], p.POSITION_CONTROL, -finger_angles[0])
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_2A'], p.POSITION_CONTROL, finger_angles[1])  
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_3A'], p.POSITION_CONTROL, finger_angles[2])
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_1B'], p.POSITION_CONTROL, -0.55-_ruth_base_motors[2]-finger_pos_plus)
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_2B'], p.POSITION_CONTROL, -0.55-_ruth_base_motors[2]-finger_pos_plus)
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_3B'], p.POSITION_CONTROL, 0.65+_ruth_base_motors[2]+finger_pos_plus) 
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_1C'], p.POSITION_CONTROL, 1-_ruth_base_motors[2]-finger_pos_plus)   
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_2C'], p.POSITION_CONTROL, 1-_ruth_base_motors[2]-finger_pos_plus)
    p.setJointMotorControl2(_robot, _RUTH_jointNameToID['Joint_Phal_3C'], p.POSITION_CONTROL, -1+_ruth_base_motors[2]+finger_pos_plus)

def get_mano_joints(manoUid):  # set joints of mano

    return 1

def set_mano_position(constraint_id, position, orientation): # set position of mano
    p.changeConstraint(constraint_id,jointChildPivot = position,jointChildFrameOrientation = orientation, maxForce = 1000)

# def set_mano_position(constraint_id, position, orientation): # set position of mano
#     p.changeConstraint(constraint_id,jointChildFrameOrientation = orientation, maxForce = 1000)

class ruth(gym.Env):

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
            cameraDistance=2,
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
        manoUid = p.loadURDF('/home/kelin/workspace_kelin/RAL-ICRA2023/demonstrations/RUTH/urdf/urdf/RUTH.urdf',basePosition=[0,0,1],baseOrientation = [0,1,0,0])
    
        


        #===== Save mano joint and link information
        manoJointNameToID = {}
        manoLinkNameToID = {}
        manoRevoluteID = []

        for j in range(p.getNumJoints(manoUid)):
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

        for i, linkname in enumerate(manoLinkNameToID):
            p.changeDynamics(manoUid,manoLinkNameToID[linkname],mass=0.06)
        p.changeDynamics(manoUid,-1,mass=0.1)
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


        self.action_space = spaces.Box(np.array([-100,]*9), np.array([100,]*9))
        self.observation_space = spaces.Box(np.array([0.0]*9), np.array([1.0]*9))
        
        self.max_delta_action = 1
        
        
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action, flag=True, count=0):

        #========================Initialization
        r = 0
        d = False
        # action = np.clip(action, -1, 1) 
        #assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        #action = action * self.max_delta_action 
        # target = np.array([1,0,1,1.57,0,0])
        obs = self.get_obs() 
        mano_pos = action[0:3]#obs[0:3]#- np.array([-0.08137444909035871,0.000313143494664086,0.001115205383354])
        mano_ori = action[3:6]#+ obs[3:6]
        mano_joints = action[6:]#+obs[6:]

        #========================Move robot
        mano_ori = p.getQuaternionFromEuler(mano_ori)
        set_mano_position(self.constraint_id,mano_pos,mano_ori)
        motor_control_ruth(self.robot, self.jointNameToID, mano_joints)

        for i in range(2):
            p.stepSimulation()

        if p.getContactPoints(bodyA = self.robot):
            r = p.getContactPoints(bodyA = self.robot)[0][9]
        if p.getContactPoints(bodyA = self.object, bodyB = self.target):
            obs = 100
        else:
            obs = 0 
        

                
        return obs, r, d, {} 
        
    def reset(self,demo_obs_init):
        set_mano_position(self.constraint_id,demo_obs_init[:3],demo_obs_init[3:6])
        motor_control_ruth(self.robot, self.jointNameToID, demo_obs_init[6:])
        for i in range(1000):
            p.stepSimulation()
        p.removeBody(self.object)
        self.object = p.loadURDF("duck_vhacd.urdf",basePosition=[0,0,0.66],globalScaling=1)
        state = (self.get_obs())
        return state

    def get_obs(self):
        pos = np.array(p.getLinkState(self.robot,0)[0]) #- np.array([-0.08137444909035871,0.000313143494664086,0.001115205383354])
        ori = np.array(p.getLinkState(self.robot,0)[1])
        joints = np.array([0,0,0])
        ori = p.getEulerFromQuaternion(ori)
        return np.concatenate((pos, ori, joints)).copy()

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
