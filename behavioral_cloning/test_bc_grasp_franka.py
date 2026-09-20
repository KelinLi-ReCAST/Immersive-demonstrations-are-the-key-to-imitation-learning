"""
Test behavioral cloning agent's performance
"""
import argparse
import logging
import os
import sys

sys.path.insert(0, "../utils")
import utils as BIU


import numpy as np
import torch
from simple_bc_agent import BCNet_rgbp, BCNet_taskObs1
import gym

log = logging.getLogger(__name__)

env = gym.make("mano-v0", version = "GUI")
def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate the trained behvaior_cloning agent in behavior")
    parser.add_argument("--model", type=str, help="Path and filename of trained model")
    return parser.parse_args()


device = "cuda" if torch.cuda.is_available() else "cpu"
args = parse_args()

args.spec = './model_force_mano/demo.npy'

success_count = 0
data = BIU.BHDataset(args.spec)
data.to_device()

force = []
success = []

for model_count in range(3500,4000,1):
        
        
        args.model = '/home/kelin/workspace_kelin/RAL-ICRA2023/behavior/behavior/baselines/behavioral_cloning/model_force_mano/model_'+str(model_count)+'.pth'
        bc_agent = torch.load(args.model)
        bc_agent.eval()
        task_obs = torch.tensor(data.task_obss[0,:], dtype=torch.float32).unsqueeze(0).to(device)
        demo_obs_init = task_obs.cpu().numpy().squeeze(0)
        for i in range(np.size(demo_obs_init)):
            demo_obs_init[i] = demo_obs_init[i]*(data.obs_max[i]-data.obs_min[i])+data.obs_min[i]
        i = 0
        state_init = demo_obs_init
        r_sum = 0
        count = 1e-6
        
        obs = env.reset(demo_obs_init=demo_obs_init)
        total_reward = 0
        done = False
        success_flag = False
        args = parse_args()
        with torch.no_grad():
            for j in range(len(data.task_obss)):

                #task_obs = torch.tensor(obs["task_obs"], dtype=torch.float32).unsqueeze(0).to(device)
                #proprioception = torch.tensor(obs["proprioception"], dtype=torch.float32).unsqueeze(0).to(device)
                task_obs = torch.tensor(data.task_obss[j,:], dtype=torch.float32).unsqueeze(0).to(device)
                task_acts = torch.tensor(data.actions[j,:], dtype=torch.float32).unsqueeze(0).to(device)
                demo_obs = task_obs.cpu().numpy().squeeze(0)
                demo_act = task_acts.cpu().numpy().squeeze(0)
                action = bc_agent(task_obs)
                a = action.cpu().numpy().squeeze(0)
                for i in range(np.size(demo_obs_init)):
                    a[i] = a[i]*(data.act_max[i]-data.act_min[i])+data.act_min[i]
                    demo_obs[i] = demo_obs[i]*(data.obs_max[i]-data.obs_min[i])+data.obs_min[i]
                demo_obs_init = demo_obs_init + a
                # a_no_reset = a  # we do not allow reset action for agents here
                obs, reward, done, info = env.step(action=demo_obs_init)
                total_reward += reward
                log.info("Reward {}, info {}".format(total_reward, info))
                i=i+1
                if reward != 0:
                    count += 1
                
                r_sum += reward
                if obs == 100: 
                    success_flag = True
            if success_flag:
                success.append(1)
            else:
                success.append(0)
            force.append(r_sum/count)
            print("load model: ",model_count,r_sum/count, success[-1])
                
np.save(os.getcwd()+"/evaluation_data/f_pforce_ruth.npy",np.array(force))
# np.save(os.getcwd()+"/evaluation_data/p_sr_ruth.npy",np.array(success))