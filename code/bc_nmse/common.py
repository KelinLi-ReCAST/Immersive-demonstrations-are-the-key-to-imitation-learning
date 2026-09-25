"""Shared data loading, preprocessing and model code for the behaviour-cloning NMSE evaluation.

state   wrist position (3), wrist Euler orientation (3), end-effector joint targets; the three force columns are dropped
action  a_t = s_{t+1} - s_t on the unwrapped, smoothed state sub-sampled every STRIDE frames
input   current state (orientation relative to the first frame) + the previous HISTORY-1 actions
"""
import os
import numpy as np
import torch.nn as nn

REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
DATA_DIR = os.path.join(REPO_DIR, 'data')
RESULTS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')

GRIPPERS = [('franka', 'Franka Emika Hand'), ('RUTH', 'RUTH hand'), ('mano', 'MANO hand')]
CONDITIONS = [('no_force', 'NFF'), ('force', 'FFF'), ('panda', 'FPFF')]
PARTICIPANTS = range(1, 11)
DEMOS = range(5)

STRIDE = 10      # frames per policy step (240 Hz -> 24 Hz)
SMOOTH = 25      # moving-average window in frames
HISTORY = 5      # current state + previous HISTORY-1 actions


def demo_id(gripper, condition, participant, k):
    return f'{gripper}/{condition}/{participant}_{k}'


def load_state(gripper, condition, participant, k):
    """Return the recorded state (frames x [pos 3, euler 3, joints]) of one demonstration."""
    a = np.load(os.path.join(DATA_DIR, gripper, condition, f'{participant}_{k}.npy'))
    if gripper == 'franka' and a.shape[1] == 11:
        # franka/panda participant 7 stores the wrist orientation as a quaternion (x, y, z, w)
        x, y, z, w = a[:, 6], a[:, 7], a[:, 8], a[:, 9]
        euler = np.stack([np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
                          np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)),
                          np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))], axis=1)
        a = np.concatenate([a[:, :6], euler, a[:, 10:]], axis=1)
    # single-sample logging glitch (franka/panda/6_0, gripper joint = 6.8e5): hold the previous value
    for i, j in zip(*np.where(np.abs(a[:, 3:]) > 10)):
        a[i, j + 3] = a[i - 1, j + 3]
    return a[:, 3:]


def preprocess(state):
    """Unwrap the Euler angles (they jump by 2*pi at +-pi) and smooth with a moving average."""
    s = state.copy()
    s[:, 3:6] = np.unwrap(s[:, 3:6], axis=0)
    kernel = np.ones(SMOOTH) / SMOOTH
    cols = [np.convolve(np.pad(s[:, j], SMOOTH // 2, mode='edge'), kernel, 'valid') for j in range(s.shape[1])]
    return np.stack(cols, axis=1)


def make_xy(state, history=HISTORY):
    """Build (inputs, actions) of one demonstration, or None if it is too short."""
    s = preprocess(state)[::STRIDE]
    if len(s) < history + 2:
        return None
    actions = s[1:] - s[:-1]
    features = np.concatenate([s[:, :3], s[:, 3:6] - s[:1, 3:6], s[:, 6:]], axis=1)[:-1]
    if history == 1:
        return features, actions
    padded = np.concatenate([np.zeros((history - 1, actions.shape[1])), actions])   # zeros before the start
    previous = [padded[history - 1 - i:len(padded) - i] for i in range(1, history)]  # a_{t-1} .. a_{t-history+1}
    return np.concatenate([features] + previous, axis=1), actions


class MLP(nn.Module):
    def __init__(self, n_in, n_out, width=256):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(n_in, width), nn.ReLU(),
                                 nn.Linear(width, width), nn.ReLU(),
                                 nn.Linear(width, width), nn.ReLU(),
                                 nn.Linear(width, n_out))

    def forward(self, x):
        return self.net(x)


def nmse(pred, target):
    """Sum of squared errors over the total sum of squares about the per-component mean, pooled over components."""
    return float(((pred - target) ** 2).sum() / ((target - target.mean(0)) ** 2).sum())
