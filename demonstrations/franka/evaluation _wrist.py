import os
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import make_interp_spline
f_force = []
f_no_force = []
f_panda = []

mean_eval = []
arr_eval = []
a = 0.1
name_list = ['1','2','3','4','5','6','7','8','9','10']
eval_force = np.sort(np.load('test_force_franka.npy')*4)
force_mean = np.zeros(25)
eval_noforce = np.sort(np.load('test_noforce_franka.npy')*4)
noforce_mean = np.zeros(25)

for i in range(25):
    force_mean[i] = (eval_force[i]+eval_force[49-i])/2
    noforce_mean[i] = (eval_noforce[i]+eval_noforce[49-i])/2

force_mean = np.concatenate((np.zeros(6),force_mean[:24]),axis=0)
noforce_mean = np.concatenate((np.zeros(5),noforce_mean),axis=0)

for demonstrator in name_list:
    for i in range(5):
        f_no_force.append(np.load(os.getcwd()+"/data/no_force/"+demonstrator+"_"+str(i)+".npy"))
        f_force.append(np.load(os.getcwd()+"/data/force/"+demonstrator+"_"+str(i)+".npy"))
        f_panda.append(np.load(os.getcwd()+"/data/panda/"+demonstrator+"_"+str(i)+".npy"))

for f in [f_force,f_no_force,f_panda]:
    
    f_avg = np.zeros(len(f))
    eval = np.zeros(len(f))
    x = range(1, len(f)+1)
    for i in range(len(f)):
        f[i] = np.delete(f[i], range(3,7), axis=1)
        fsum = 0
        count = 0
        for j in range(len(f[i])):
            if abs(sum((f[i])[j,:])) >2:
                fsum = fsum + np.sqrt(sum((f[i])[j,:]*(f[i])[j,:]))
                count = count + 1
        f_avg[i] = fsum/count
        eval[i] = f_avg[i]#1/(1+math.exp(-a*(40-f_avg[i])))
    print(np.mean(f_avg))
    mean_eval.append(np.mean(eval))
    arr_eval.append(eval)
    # plt.plot(x,eval)
    # plt.show()
    #plt.ylim(0,1)
arr_eval[0] = arr_eval[0]*5
arr_eval[1] = arr_eval[1]*5

plt.boxplot(arr_eval, boxprops={'color': 'b', 'linewidth': 2, 'linestyle': '--'}, showfliers=False)
# plt.hlines(mean_eval[0]*5, 0, 2000,color="blue")
# plt.hlines(mean_eval[1]*5, 0, 2000,color="orange")
# plt.hlines(mean_eval[2], 0, 2000,color="green")

plt.title("FRANKA") 
plt.show()
print(mean_eval)
print(np.mean(eval_force))
print(np.mean(eval_noforce))

