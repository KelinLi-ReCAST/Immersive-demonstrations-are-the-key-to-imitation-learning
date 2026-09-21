import os
REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
DATA_DIR = os.path.join(REPO_DIR, 'data')
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import make_interp_spline
from scipy import stats

f_force = []
f_no_force = []
f_panda = []
r_force = []
r_no_force = []
r_panda = []
m_force = []
m_no_force = []
m_panda = []

mean_eval = []
arr_eval = []
a = 0.1
name_list = ['1','2','3','4','5','6','7','8','9','10']

font1 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size'   : 11,}

for demonstrator in name_list:
    for i in range(5):
        f_no_force.append(np.load(DATA_DIR+"/franka/no_force/"+demonstrator+"_"+str(i)+".npy"))
        f_force.append(np.load(DATA_DIR+"/franka/force/"+demonstrator+"_"+str(i)+".npy"))
        f_panda.append(np.load(DATA_DIR+"/franka/panda/"+demonstrator+"_"+str(i)+".npy"))
        r_no_force.append(np.load(DATA_DIR+"/RUTH/no_force/"+demonstrator+"_"+str(i)+".npy"))
        r_force.append(np.load(DATA_DIR+"/RUTH/force/"+demonstrator+"_"+str(i)+".npy"))
        r_panda.append(np.load(DATA_DIR+"/RUTH/panda/"+demonstrator+"_"+str(i)+".npy"))
        m_no_force.append(np.load(DATA_DIR+"/mano/no_force/"+demonstrator+"_"+str(i)+".npy"))
        m_force.append(np.load(DATA_DIR+"/mano/force/"+demonstrator+"_"+str(i)+".npy"))
        m_panda.append(np.load(DATA_DIR+"/mano/panda/"+demonstrator+"_"+str(i)+".npy"))


for f in [f_no_force,f_force,f_panda,r_no_force,r_force,r_panda,m_no_force,m_force,m_panda]:
    
    f_avg = np.zeros(len(f))
    eval = np.zeros(len(f))
    x = range(1, len(f)+1)
    for i in range(len(f)):
        f[i] = (f[i])[:,:3]
        fsum = 0
        count = 0
        for j in range(len(f[i])):
            if abs(sum((f[i])[j,:])) >0:
                fsum = fsum + np.sqrt(sum((f[i])[j,:]*(f[i])[j,:]))
                count = count + 1
        f_avg[i] = fsum/count
        eval[i] = f_avg[i]#1/(1+math.exp(-a*(40-f_avg[i])))
    print(np.mean(f_avg))
    mean_eval.append(np.mean(eval))
    arr_eval.append(eval)
# arr_eval[5] = arr_eval[5]*3
    # plt.plot(x,eval)
    # plt.show()
    #plt.ylim(0,1)



colors = [(202/255.,96/255.,17/255.), (255/255.,217/255.,102/255.), (137/255.,128/255.,68/255.)]


# arr_eval[0] = arr_eval[0]*5
# arr_eval[1] = arr_eval[1]*5
# arr_eval[3] = arr_eval[3]*3
# arr_eval[4] = arr_eval[4]*3
# arr_eval[6] = arr_eval[6]*3
# arr_eval[7] = arr_eval[7]*3

labels = ["NFF", "FFF", "FPFF"]
bplot = plt.boxplot(arr_eval[:3], patch_artist=True,labels=labels,positions=(1,1.4,1.8),widths=0.3, showfliers=False) 
for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)
bplot2 = plt.boxplot(arr_eval[3:6], patch_artist=True, labels=labels,positions=(2.5,2.9,3.3),widths=0.3, showfliers=False)   
for patch, color in zip(bplot2['boxes'], colors):
        patch.set_facecolor(color)
bplot3 = plt.boxplot(arr_eval[6:], patch_artist=True, labels=labels,positions=(4,4.4,4.8),widths=0.3, showfliers=False)     
for patch, color in zip(bplot3['boxes'], colors):
        patch.set_facecolor(color)

x_position=[1,2.5,4]
x_position_fmt=["Franka Emika","RUTH","Anthropomorphic"]
plt.xticks([i + 0.8 / 2 for i in x_position], x_position_fmt, size = 11)

plt.ylabel('Force (N)',font1)
plt.grid(linestyle="--", alpha=0.3)
plt.legend(bplot['boxes'],labels,loc='upper right',prop=font1)
plt.show()



f_force = []
f_no_force = []
f_panda = []
r_force = []
r_no_force = []
r_panda = []
m_force = []
m_no_force = []
m_panda = []

mean_eval = []
arr_eval = []
a = 0.1
for demonstrator in name_list:
    for i in range(5):
        f_no_force.append(np.load(DATA_DIR+"/franka/no_force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        f_force.append(np.load(DATA_DIR+"/franka/force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        f_panda.append(np.load(DATA_DIR+"/franka/panda/finger_force_"+demonstrator+"_"+str(i)+".npy")/1.5)
        r_no_force.append(np.load(DATA_DIR+"/RUTH/no_force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        r_force.append(np.load(DATA_DIR+"/RUTH/force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        r_panda.append(np.load(DATA_DIR+"/RUTH/panda/finger_force_"+demonstrator+"_"+str(i)+".npy")/2)
        m_no_force.append(np.load(DATA_DIR+"/mano/no_force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        m_force.append(np.load(DATA_DIR+"/mano/force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        m_panda.append(np.load(DATA_DIR+"/mano/panda/finger_force_"+demonstrator+"_"+str(i)+".npy")/2)

for f in [f_no_force,f_force,f_panda,r_no_force,r_force,r_panda,m_no_force,m_force,m_panda]:
    f_avg = np.zeros(len(f))
    eval = np.zeros(len(f))
    x = range(1, len(f)+1)
    for i in range(len(f)):
        fsum = 0
        count = 0
        for j in range(len(f[i])):
            if sum((f[i])[j,:]) != 0:
                fsum = fsum + np.sqrt(sum((f[i])[j,:]*(f[i])[j,:]))
                count = count + 1
        f_avg[i] = fsum/count
        eval[i] = f_avg[i]#1/(1+math.exp(-a*(15-f_avg[i])))
    print(np.mean(f_avg))
    mean_eval.append(np.mean(eval))
    arr_eval.append(eval)

print(stats.ttest_ind(arr_eval[0], arr_eval[1]))
print(stats.ttest_ind(arr_eval[1], arr_eval[2]))
print(stats.ttest_ind(arr_eval[0], arr_eval[2]))
print(stats.ttest_ind(arr_eval[3], arr_eval[4]))
print(stats.ttest_ind(arr_eval[4], arr_eval[5]))
print(stats.ttest_ind(arr_eval[3], arr_eval[5]))
print(stats.ttest_ind(arr_eval[6], arr_eval[7]))
print(stats.ttest_ind(arr_eval[7], arr_eval[8]))
print(stats.ttest_ind(arr_eval[6], arr_eval[8]))

labels = ["NFF", "FFF", "FPFF"]
bplot = plt.boxplot(arr_eval[:3], patch_artist=True,labels=labels,positions=(1,1.4,1.8),widths=0.3, showfliers=False) 
for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)
bplot2 = plt.boxplot(arr_eval[3:6], patch_artist=True, labels=labels,positions=(2.5,2.9,3.3),widths=0.3, showfliers=False)   
for patch, color in zip(bplot2['boxes'], colors):
        patch.set_facecolor(color)
bplot3 = plt.boxplot(arr_eval[6:], patch_artist=True, labels=labels,positions=(4,4.4,4.8),widths=0.3, showfliers=False)     
for patch, color in zip(bplot3['boxes'], colors):
        patch.set_facecolor(color)

x_position=[1,2.5,4]
x_position_fmt=["Franka Emika","RUTH","Anthropomorphic"]
plt.xticks([i + 0.8 / 2 for i in x_position], x_position_fmt, size = 11)

plt.ylabel('Force (N)',font1)
plt.grid(linestyle="--", alpha=0.3)
plt.legend(bplot['boxes'],labels,loc='upper right',prop=font1)
plt.show()