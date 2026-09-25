"""Train behaviour-cloning policies on the demonstrations and report the NMSE on held-out participants.

  python train_eval.py                       # 5 folds by participant (8 train / 2 test), all nine gripper x condition cells
  python train_eval.py --split lopo          # leave one participant out (9 train / 1 test)

norm    z-score per component, statistics from the training demonstrations only
output  the 6 wrist-pose components of the action (position 3 + orientation 3); the joint targets are part of the
        input state but are not predicted or scored
NMSE    sum (yhat - y)^2 / sum (y - mean y)^2 pooled over samples and components
        all          position + orientation, z-scored action space
        position / orientation   physical units
Demonstrations listed by scan_anomalies.py are excluded (use --keep_flagged to include them).
Results (mean and standard deviation over folds) are written to results/nmse_<split>.csv and .json.
"""
import argparse
import csv
import json
import os
import time
import numpy as np
import torch
import torch.nn as nn
import common as C
from scan_anomalies import scan

DEVICE = 'cuda' if torch.cuda.is_available() else 'cpu'
EPOCHS, BATCH, LR, WEIGHT_DECAY = 300, 4096, 1e-3, 1e-4


def fit(train_demos, test_demos, seed=0):
    """Train on train_demos (validation split inside) and return NMSE values on test_demos."""
    cat = lambda demos: [np.concatenate(v) for v in zip(*demos)]
    order = np.random.default_rng(seed).permutation(len(train_demos))
    n_val = max(1, len(order) // 10)
    Xv, Yv = cat([train_demos[i] for i in order[:n_val]])
    Xt, Yt = cat([train_demos[i] for i in order[n_val:]])
    Xe, Ye = cat(test_demos)
    Yv, Yt, Ye = Yv[:, :6], Yt[:, :6], Ye[:, :6]          # wrist pose only

    xm, xs = Xt.mean(0), Xt.std(0) + 1e-8
    ym, ys = Yt.mean(0), np.maximum(Yt.std(0), 1e-4)
    zx = lambda X: np.clip((X - xm) / xs, -10, 10)
    T = lambda v: torch.tensor(v, dtype=torch.float32, device=DEVICE)
    Xt_, Yt_, Xv_, Yv_ = T(zx(Xt)), T((Yt - ym) / ys), T(zx(Xv)), T((Yv - ym) / ys)

    torch.manual_seed(seed)
    net = C.MLP(Xt_.shape[1], Yt_.shape[1]).to(DEVICE)
    opt = torch.optim.AdamW(net.parameters(), LR, weight_decay=WEIGHT_DECAY)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, EPOCHS)
    best_val, best_state = float('inf'), None
    for _ in range(EPOCHS):
        net.train()
        perm = torch.randperm(len(Xt_), device=DEVICE)
        for i in range(0, len(Xt_), BATCH):
            j = perm[i:i + BATCH]
            opt.zero_grad()
            nn.functional.mse_loss(net(Xt_[j]), Yt_[j]).backward()
            opt.step()
        sched.step()
        net.eval()
        with torch.no_grad():
            val = nn.functional.mse_loss(net(Xv_), Yv_).item()
        if val < best_val:
            best_val, best_state = val, {k: v.clone() for k, v in net.state_dict().items()}

    net.load_state_dict(best_state)
    net.eval()
    with torch.no_grad():
        pred_z = net(T(zx(Xe))).cpu().numpy()
        train_z = net(Xt_).cpu().numpy()
    pred, target_z = pred_z * ys + ym, (Ye - ym) / ys
    return dict(train_all=C.nmse(train_z, Yt_.cpu().numpy()), all=C.nmse(pred_z, target_z),
                position=C.nmse(pred[:, :3], Ye[:, :3]), orientation=C.nmse(pred[:, 3:6], Ye[:, 3:6]))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--split', choices=['5fold', 'lopo'], default='5fold')
    parser.add_argument('--keep_flagged', action='store_true')
    args = parser.parse_args()

    flagged = {} if args.keep_flagged else scan()
    folds = [(q,) for q in C.PARTICIPANTS] if args.split == 'lopo' else [(2 * f + 1, 2 * f + 2) for f in range(5)]
    keys = ['train_all', 'all', 'position', 'orientation']
    results, rows, t0 = {}, [], time.time()
    for gripper, gripper_name in C.GRIPPERS:
        for condition, condition_name in C.CONDITIONS:
            demos = {(p, k): C.make_xy(C.load_state(gripper, condition, p, k))
                     for p in C.PARTICIPANTS for k in C.DEMOS if C.demo_id(gripper, condition, p, k) not in flagged}
            demos = {pk: v for pk, v in demos.items() if v is not None}
            per_fold = []
            for test_p in folds:
                train = [v for (p, k), v in demos.items() if p not in test_p]
                test = [v for (p, k), v in demos.items() if p in test_p]
                per_fold.append(fit(train, test))
            results[f'{gripper_name}|{condition_name}'] = per_fold
            mean = {k: float(np.mean([r[k] for r in per_fold])) for k in keys}
            std = {k: float(np.std([r[k] for r in per_fold])) for k in keys}
            rows.append([gripper_name, condition_name, len(demos)] + [round(v, 4) for k in keys for v in (mean[k], std[k])])
            print(f'{gripper_name:18s} {condition_name:5s} n={len(demos)} | train {mean["train_all"]:.3f} | '
                  f'test all {mean["all"]:.3f} +- {std["all"]:.3f}  position {mean["position"]:.3f}  '
                  f'orientation {mean["orientation"]:.3f} [{time.time() - t0:.0f}s]', flush=True)

    os.makedirs(C.RESULTS_DIR, exist_ok=True)
    with open(os.path.join(C.RESULTS_DIR, f'nmse_{args.split}.json'), 'w') as f:
        json.dump(results, f, indent=1)
    with open(os.path.join(C.RESULTS_DIR, f'nmse_{args.split}.csv'), 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['gripper', 'condition', 'n_demos'] + [f'{k}_{s}' for k in keys for s in ('mean', 'sd')])
        writer.writerows(rows)
