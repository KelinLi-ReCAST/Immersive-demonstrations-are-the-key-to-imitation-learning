"""
Base behavior dataset class.
"""
import argparse
import logging
import time

import h5py
import numpy as np

log = logging.getLogger(__name__)

# Constants
IMG_DIM = 128
ACT_DIM = 28
PROPRIOCEPTION_DIM = 22
TASK_OBS_DIM = 456


class BHDataset(object):
    def __init__(self, spec_file):
        #self.files = read_spec_file(spec_file)
        t1 = time.time()
        log.info("Reading all training data into memory...")
        self.task_obss = np.load(spec_file)
        self.obs_max = np.zeros(np.shape(self.task_obss)[1])
        self.obs_min = np.zeros(np.shape(self.task_obss)[1])
        self.act_max = np.zeros(np.shape(self.task_obss)[1])
        self.act_min = np.zeros(np.shape(self.task_obss)[1])
        self.task_obss = np.around(self.task_obss, 4)
        self.task_obss = np.delete(self.task_obss, [0,1,2], axis=1)
        self.actions = np.zeros((len(self.task_obss)-1,np.shape(self.task_obss)[1]))
        for i in range(len(self.task_obss)):
            if i>0:
                self.actions[i-1] = self.task_obss[i,:]-self.task_obss[i-1,:]
        self.task_obss = np.delete(self.task_obss, [-1], axis=0)
        #--------------------normalization
        for i in range(np.shape(self.task_obss)[1]):
            self.obs_max[i] = np.max(self.task_obss[:,i])
            self.obs_min[i] = np.min(self.task_obss[:,i])
            self.act_max[i] = np.max(self.actions[:,i])
            self.act_min[i] = np.min(self.actions[:,i])
            for j in range(len(self.task_obss)):
                self.task_obss[j,i]=(self.task_obss[j,i]-self.obs_min[i])/(self.obs_max[i]-self.obs_min[i]+1e-6)
                self.actions[j,i]=(self.actions[j,i]-self.act_min[i])/(self.act_max[i]-self.act_min[i]+1e-6)
        self.size = len(self.actions)
        log.debug("Time spent to read data: %.1fs" % (time.time() - t1))

    def to_device(self):
        log.info("Sending data to gpu...")
        import torch

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.actions = torch.tensor(self.actions, dtype=torch.float32).to(self.device)
        self.task_obss = torch.tensor(self.task_obss, dtype=torch.float32).to(self.device)
        log.debug("Done.")


def read_spec_file(fname):
    files = []
    f = open(fname, "r")
    for line in f:
        items = line.split()
        if len(items) == 0 or line.startswith("#"):
            continue
        elif items[0] == "DIR":
            dir_proc = items[1]
        else:
            files.append(dir_proc + items[0] + "_episode.hdf5")
    return files


def read_proc_parallel(files):
    # read action and state information from processed files
    actions, proprioceptions, rgbs, task_obss = (
        np.empty((0, ACT_DIM)),
        np.empty((0, PROPRIOCEPTION_DIM)),
        np.empty((0, IMG_DIM, IMG_DIM, 3)),
        np.empty((0, TASK_OBS_DIM)),
    )
    files=['bottling_fruit_0_Wainscott_0_int_0_2021-05-24_19-46-46_episode.hdf5']
    for f in files:
        log.info("Processing file %s..." % f)
        hf = h5py.File(f)
        actions = np.append(actions, np.asarray(hf["action"]), axis=0)
        proprioceptions = np.append(proprioceptions, np.asarray(hf["proprioception"]), axis=0)
        rgbs = np.append(rgbs, np.asarray(hf["rgb"]), axis=0)
        task_obss = np.append(task_obss, np.asarray(hf["task_obs"]), axis=0)
        hf.close()
    return actions, proprioceptions, rgbs, task_obss


if __name__ == "__main__":

    def parse_args():
        parser = argparse.ArgumentParser(description="Dataset loader")
        parser.add_argument("--spec", type=str, help="spec file name")
        return parser.parse_args()
    
    args = parse_args()
    data = BHDataset(args.spec)
    data.to_device()
