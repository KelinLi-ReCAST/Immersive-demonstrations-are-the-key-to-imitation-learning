"""Scan all demonstrations with one fixed physical criterion and list the anomalous files.

  jump     a single-frame wrist displacement > 0.05 m (normal frames move 0.001-0.005 m)
  outside  wrist leaves the workspace (|x| or |y| > 1.5 m, or z < 0.60 m, i.e. below the table top)
  short    fewer than 300 frames
The first 60 frames are exempt from 'jump' and 'outside': every RUTH demonstration starts with the same spawn transient.
"""
import json
import os
import numpy as np
import common as C

JUMP, Z_MIN, XY_MAX, MIN_FRAMES, EXEMPT = 0.05, 0.60, 1.5, 300, 60


def scan():
    flagged = {}
    for gripper, _ in C.GRIPPERS:
        for condition, _ in C.CONDITIONS:
            for p in C.PARTICIPANTS:
                for k in C.DEMOS:
                    pos = C.load_state(gripper, condition, p, k)[:, :3]
                    jump = np.linalg.norm(np.diff(pos, axis=0), axis=1)
                    jump[:EXEMPT] = 0
                    outside = (np.abs(pos[:, :2]) > XY_MAX).any(1) | (pos[:, 2] < Z_MIN)
                    outside[:EXEMPT] = False
                    why = []
                    if jump.max() > JUMP:
                        why.append(f'jump {jump.max():.2f} m ({int((jump > JUMP).sum())} frames)')
                    if outside.any():
                        why.append(f'outside workspace ({int(outside.sum())} frames, z min {pos[:, 2].min():.2f})')
                    if len(pos) < MIN_FRAMES:
                        why.append(f'short ({len(pos)} frames)')
                    if why:
                        flagged[C.demo_id(gripper, condition, p, k)] = why
    return flagged


if __name__ == '__main__':
    flagged = scan()
    for name, why in flagged.items():
        print(f'{name:28s} ' + '; '.join(why))
    print(f'\n{len(flagged)} of 450 demonstrations flagged')
    os.makedirs(C.RESULTS_DIR, exist_ok=True)
    with open(os.path.join(C.RESULTS_DIR, 'anomalous_demos.json'), 'w') as f:
        json.dump(flagged, f, indent=1)
