import os
REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
DATA_DIR = os.path.join(REPO_DIR, 'data', 'RUTH')
VIDEO_DIR = os.path.join(REPO_DIR, 'media', 'videos', 'RUTH')
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import make_interp_spline
f_force = []
f_no_force = []
f_panda = []

mean_eval = []
a = 0.1
name_list = ['1','2','3','4','5','6','7','8','9','10']
eval_force = np.sort(np.load(DATA_DIR+"/test_force_ruth.npy")*4)
force_mean = np.zeros(25)
eval_noforce = np.sort(np.load(DATA_DIR+"/test_noforce_ruth.npy")*4)
noforce_mean = np.zeros(25)

for i in range(25):
    force_mean[i] = (eval_force[i]+eval_force[49-i])/2
    noforce_mean[i] = (eval_noforce[i]+eval_noforce[49-i])/2

force_mean = np.concatenate((np.zeros(6),force_mean[:24]),axis=0)
noforce_mean = np.concatenate((np.zeros(5),noforce_mean),axis=0)

for demonstrator in name_list:
    for i in range(5):
        f_no_force.append(np.load(DATA_DIR+"/no_force/"+demonstrator+"_"+str(i)+".npy"))
        f_force.append(np.load(DATA_DIR+"/force/"+demonstrator+"_"+str(i)+".npy"))
        f_panda.append(np.load(DATA_DIR+"/panda/"+demonstrator+"_"+str(i)+".npy"))

for f in [f_force,f_no_force,f_panda]:
    f_avg = np.zeros(len(f))
    eval = np.zeros(len(f))
    x = range(1, len(f)+1)
    for i in range(len(f)):
        f[i] = np.delete(f[i], range(3,12), axis=1)
        fsum = 0
        count = 0
        for j in range(len(f[i])):
            if abs(sum((f[i])[j,:])) >2:
                fsum = fsum + np.sqrt(sum((f[i])[j,:]*(f[i])[j,:]))
                count = count + 1
        f_avg[i] = fsum/count
        eval[i] = f_avg[i]#1/(1+math.exp(-a*(40-f_avg[i])))
    # print(np.mean(f_avg))
    mean_eval.append(np.mean(eval)/4)
    # plt.plot(x,eval)
    # plt.show()
    #plt.ylim(0,1)

x = np.array(list(i for i in range(0,2000,int(2000/29))))
x_new = np.linspace(x.min(),x.max(),300)
force_mean_new = make_interp_spline(x, force_mean)(x_new)
noforce_mean_new = make_interp_spline(x, noforce_mean)(x_new)
for i in range(len(force_mean_new)):
    if force_mean_new[i] <0:
        force_mean_new[i] = 0
    if noforce_mean_new[i] <0:
        noforce_mean_new[i] = 0
# plt.plot(x_new,force_mean_new)
# plt.plot(x_new,noforce_mean_new)

plt.hlines(mean_eval[0]*3, 0, 2000,color="blue")
plt.hlines(mean_eval[1]*3, 0, 2000,color="orange")
plt.hlines(mean_eval[2], 0, 2000,color="green")
plt.title("RUTH") 
plt.show()
print(mean_eval)
print(np.mean(eval_force))
print(np.mean(eval_noforce))

