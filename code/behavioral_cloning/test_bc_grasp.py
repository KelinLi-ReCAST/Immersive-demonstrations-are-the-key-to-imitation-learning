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

env = gym.make("ruth-v0", version = "GUI")
def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate the trained behvaior_cloning agent in behavior")
    parser.add_argument("--model", type=str, help="Path and filename of trained model")
    return parser.parse_args()


device = "cuda" if torch.cuda.is_available() else "cpu"
args = parse_args()
args.model = '/home/kelin/workspace_kelin/RAL-ICRA2023/behavior/behavior/baselines/behavioral_cloning/trained_models/model_0.pth'
# bc_agent = torch.load(args.model)
# bc_agent.eval()
success_count = 0


force = []
for participant in range(10):
    print(participant)
    for trail in range(5):
        participant = 7
        trail = 0
        args.spec = '/home/kelin/workspace_kelin/RAL-ICRA2023/data/RUTH/force/'+str(participant+1)+'_'+str(trail)+'.npy'

        data = BIU.BHDataset(args.spec)
        data.to_device()
        task_obs = torch.tensor(data.task_obss[0,:], dtype=torch.float32).unsqueeze(0).to(device)
        demo_obs_init = task_obs.cpu().numpy().squeeze(0)
        for i in range(9):
            demo_obs_init[i] = demo_obs_init[i]*(data.obs_max[i]-data.obs_min[i])+data.obs_min[i]
        i = 0
        state_x = 0.01
        r_sum = 0
        count = 1e-6
        
        obs = env.reset(demo_obs_init=demo_obs_init)
        total_reward = 0
        done = False

        args = parse_args()
        with torch.no_grad():
            for j in range(len(data.task_obss)):
                #task_obs = torch.tensor(obs["task_obs"], dtype=torch.float32).unsqueeze(0).to(device)
                #proprioception = torch.tensor(obs["proprioception"], dtype=torch.float32).unsqueeze(0).to(device)
                task_obs = torch.tensor(data.task_obss[i,:], dtype=torch.float32).unsqueeze(0).to(device)
                task_acts = torch.tensor(data.actions[i,:], dtype=torch.float32).unsqueeze(0).to(device)
                demo_obs = task_obs.cpu().numpy().squeeze(0)
                demo_act = task_acts.cpu().numpy().squeeze(0)
                # action = bc_agent(task_obs)
                # a = action.cpu().numpy().squeeze(0)
                # state_x = state_x+a[1]
                # a_no_reset = a  # we do not allow reset action for agents here
                for m in range(9):
                    demo_obs[m] = demo_obs[m]*(data.obs_max[m]-data.obs_min[m])+data.obs_min[m]
                obs, reward, done, info = env.step(demo_obs)
                total_reward += reward
                log.info("Reward {}, info {}".format(total_reward, info))
                i=i+1
                if reward != 0:
                    count += 1
                
                r_sum += reward

            if obs==100:
                success_count += 1
            force.append(r_sum/count)
                
np.save("test_force_ruth.npy",np.array(force))
print(success_count)