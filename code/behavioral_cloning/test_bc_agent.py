"""
Test behavioral cloning agent's performance
"""
import argparse
import logging
import os

import sys

sys.path.insert(0, "../utils")
import base_input_utils as BIU

import igibson
import numpy as np
import torch
from igibson.envs.igibson_env import iGibsonEnv
from simple_bc_agent import BCNet_rgbp, BCNet_taskObs

log = logging.getLogger(__name__)


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate the trained behvaior_cloning agent in behavior")
    parser.add_argument("--model", type=str, help="Path and filename of trained model")
    return parser.parse_args()


device = "cuda" if torch.cuda.is_available() else "cpu"
args = parse_args()
args.model = '/home/kelin/workspace_kelin/RAL-ICRA2023/behavior/behavior/baselines/behavioral_cloning/trained_models/model.pth'
bc_agent = torch.load(args.model)
bc_agent.eval()

config_file = "behavior_full_observability.yaml"
env = iGibsonEnv(
    config_file=os.path.join('/home/kelin/workspace_kelin/RAL-ICRA2023/behavior/behavior/configs', config_file),
    mode="headless",
    action_timestep=1 / 30.0,
    physics_timestep=1 / 300.0,
)

obs = env.reset()
total_reward = 0
done = False

args = parse_args()
args.spec = 'bottling_fruit_0_Wainscott_0_int_0_2021-05-24_19-46-46_episode.hdf5'
data = BIU.BHDataset(args.spec)
data.to_device()
i = 0
with torch.no_grad():
    while not (done):
        #task_obs = torch.tensor(obs["task_obs"], dtype=torch.float32).unsqueeze(0).to(device)
        #proprioception = torch.tensor(obs["proprioception"], dtype=torch.float32).unsqueeze(0).to(device)
        task_obs = torch.tensor(data.task_obss[i,:], dtype=torch.float32).unsqueeze(0).to(device)
        proprioception = torch.tensor(data.proprioceptions[i,:], dtype=torch.float32).unsqueeze(0).to(device)
        action = bc_agent(task_obs, proprioception)
        a = action.cpu().numpy().squeeze(0)
        a_no_reset = np.concatenate((a[:19], a[20:27]))  # we do not allow reset action for agents here
        obs, reward, done, info = env.step(a_no_reset)
        total_reward += reward
        log.info("Reward {}, info {}".format(total_reward, info))
        i=i+1