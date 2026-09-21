import os
REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
DATA_DIR = os.path.join(REPO_DIR, 'data', 'RUTH')
VIDEO_DIR = os.path.join(REPO_DIR, 'media', 'videos', 'RUTH')
import numpy as np
import matplotlib.pyplot as plt
import math
from scipy.interpolate import make_interp_spline
import pandas as pd
import matplotlib
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

f_force = []
f_no_force = []
f_panda = []

mean_eval = []
a = 0.1
count_force = 0
count_noforce = 0
success_force = []
success_noforce = []
name_list = ['1','2','3','4','5','6','7','8','9','10']

success_rate_force = np.load(DATA_DIR+"/training_results/f_pforce_ruth.npy")*3
success_rate_noforce = np.load(DATA_DIR+"/training_results/n_pforce_ruth.npy")
success_rate_panda = np.load(DATA_DIR+"/training_results/p_pforce_ruth.npy")/3
f_t_force = np.load(DATA_DIR+"/training_results/force_force.npy")
f_t_noforce = np.load(DATA_DIR+"/training_results/noforce_force.npy")
f_t_panda = np.load(DATA_DIR+"/training_results/panda_force.npy")

s_f_force = pd.Series(f_t_force)
s_f_noforce = pd.Series(f_t_noforce)
s_f_panda = pd.Series(f_t_panda)
s_sr_force = pd.Series(success_rate_force)
s_sr_noforce = pd.Series(success_rate_noforce)
s_sr_panda = pd.Series(success_rate_panda)

for demonstrator in name_list:
    for i in range(5):
        f_no_force.append(np.load(DATA_DIR+"/no_force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        f_force.append(np.load(DATA_DIR+"/force/finger_force_"+demonstrator+"_"+str(i)+".npy"))
        f_panda.append(np.load(DATA_DIR+"/panda/finger_force_"+demonstrator+"_"+str(i)+".npy")/2)


for f in [f_force,f_no_force,f_panda]:
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
    #plt.plot(x,eval)
    #plt.ylim(0,1)
colors = [(202/255.,96/255.,17/255.), (255/255.,217/255.,102/255.), (137/255.,128/255.,68/255.)]
x_new = range(4000)

mean_noforce = plt.hlines(mean_eval[1], 0, 4000,color=colors[0],linestyles = '--')
mean_force = plt.hlines(mean_eval[0], 0, 4000,color=colors[1],linestyles = '--')
mean_panda = plt.hlines(mean_eval[2], 0, 4000,color=colors[2],linestyles = '--')
# plt.hlines(0, 0, 150,color=colors[2],linestyles = '-')

y1=s_f_noforce.rolling(100).mean()-0.2*s_f_noforce.rolling(100).std()
y2=s_f_noforce.rolling(100).mean()+0.2*s_f_noforce.rolling(100).std()
for i in range(len(y1)):
    if y1[i] < 0:
        y1[i] = 0
plt.fill_between(x_new, y1, y2, alpha=0.3,color=colors[0])

y1=s_f_force.rolling(100).mean()-0.2*s_f_force.rolling(100).std()
y2=s_f_force.rolling(100).mean()+0.2*s_f_force.rolling(100).std()
for i in range(len(y1)):
    if y1[i] < 0:
        y1[i] = 0
plt.fill_between(x_new, y1, y2, alpha=0.3,color=colors[1])

y1=s_f_panda.rolling(100).mean()-0.3*s_f_panda.rolling(100).std()
y2=s_f_panda.rolling(100).mean()+0.3*s_f_panda.rolling(100).std()
for i in range(len(y1)):
    if y1[i] < 0:
        y1[i] = 0
plt.fill_between(x_new, y1, y2, alpha=0.3,color=colors[2])

train_noforce, = plt.plot(s_f_noforce.rolling(100).mean(),color=colors[0])
train_force, = plt.plot(s_f_force.rolling(100).mean(),color=colors[1])
train_panda, = plt.plot(s_f_panda.rolling(100).mean(),color=colors[2])
font1 = {'family' : 'Times New Roman',
'weight' : 'normal',
'size'   : 11,}
l1 = plt.legend([mean_noforce, train_noforce, mean_force, train_force, mean_panda, train_panda],
["Demo Mean NFF", "NFF", "Demo Mean FFF", "FFF","Demo Mean FPFF", "FPFF"],prop=font1,
ncol=3, bbox_to_anchor=[0.5, 1.07], loc='center')
plt.ylim(-1,210)
plt.yticks(fontproperties = 'Times New Roman', size = 11)
plt.xticks(fontproperties = 'Times New Roman', size = 11)
plt.ylabel('Force (N)',font1)
plt.xlabel('Training Epoch',font1)
plt.grid(linestyle="--", alpha=0.3)
plt.show()



f_force = []
f_no_force = []
f_panda = []
for demonstrator in name_list:
    for i in range(5):
        f_no_force.append(np.load(DATA_DIR+"/no_force/"+demonstrator+"_"+str(i)+".npy"))
        f_force.append(np.load(DATA_DIR+"/force/"+demonstrator+"_"+str(i)+".npy"))
        f_panda.append(np.load(DATA_DIR+"/panda/"+demonstrator+"_"+str(i)+".npy"))

for f in [f_no_force,f_force,f_panda]:
    
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
mean_noforce = plt.hlines(mean_eval[3], 0, 4000,color=colors[0],linestyles = '--')
mean_force = plt.hlines(mean_eval[4], 0, 4000,color=colors[1],linestyles = '--')
mean_panda = plt.hlines(mean_eval[5], 0, 4000,color=colors[2],linestyles = '--')

roll_num = 100
train_noforce, = plt.plot(s_sr_noforce.rolling(roll_num).mean(),color=colors[0])    
y1=s_sr_noforce.rolling(roll_num).mean()-1*s_sr_noforce.rolling(roll_num).std()
y2=s_sr_noforce.rolling(roll_num).mean()+1*s_sr_noforce.rolling(roll_num).std()
for i in range(len(y1)):
    if y1[i] < 0:
        y1[i] = 0
plt.fill_between(x_new, y1, y2, alpha=0.3,color=colors[0])

train_force, = plt.plot(s_sr_force.rolling(roll_num).mean(),color=colors[1])  
y1=s_sr_force.rolling(roll_num).mean()-1*s_sr_force.rolling(roll_num).std()
y2=s_sr_force.rolling(roll_num).mean()+1*s_sr_force.rolling(roll_num).std()
for i in range(len(y1)):
    if y1[i] < 0:
        y1[i] = 0
plt.fill_between(x_new, y1, y2, alpha=0.3,color=colors[1])

train_panda, = plt.plot(s_sr_panda.rolling(roll_num).mean(),color=colors[2])  
y1=s_sr_panda.rolling(roll_num).mean()-1*s_sr_panda.rolling(roll_num).std()
y2=s_sr_panda.rolling(roll_num).mean()+1*s_sr_panda.rolling(roll_num).std()
for i in range(len(y1)):
    if y1[i] < 0:
        y1[i] = 0
plt.fill_between(x_new, y1, y2, alpha=0.3,color=colors[2])

l2 = plt.legend([mean_noforce, train_noforce, mean_force, train_force, mean_panda, train_panda],
["Demo Mean NFF", "NFF", "Demo Mean FFF", "FFF","Demo Mean FPFF", "FPFF"],prop=font1,
ncol=3, bbox_to_anchor=[0.5, 1.07], loc='center')
plt.ylim(0,28)
# plt.xlim(0,1000)
plt.ylabel('Force (N)',font1)
plt.xlabel('Training Epoch',font1)
plt.yticks(fontproperties = 'Times New Roman', size = 11)
plt.xticks(fontproperties = 'Times New Roman', size = 11)
plt.grid(linestyle="--", alpha=0.3)
plt.show()