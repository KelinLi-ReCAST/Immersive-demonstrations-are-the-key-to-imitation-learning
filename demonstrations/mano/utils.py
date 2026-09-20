import os
import socket
from tkinter import Y
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
import math

def tracker_calibration(v):
    filter_num = 1000
    print("\nCalibrate the tracker, place it to the origin you want, press any button to start:")
    input()
    cali = np.zeros((filter_num,6))
    for i in range(filter_num):
        cali[i,:] = v.devices["tracker_1"].get_pose_euler()
    x = round(np.mean(cali[:,0]),4)
    y = round(np.mean(cali[:,1]),4)
    z = round(np.mean(cali[:,2]),4)
    return x,y,z

def hand_calibration(glove,demonstrator):
  print("\nCalibrate the glove, press any button to start:")
  input()
  print("****************************************************")
  print("Move your fingers")
  cali_steps = 500
  filter_num = 10
  cali_joints = np.zeros((cali_steps,21))
  cali_max = np.zeros(21)
  cali_min = np.zeros(21)
  sort_max = np.zeros(filter_num)
  sort_min = np.zeros(filter_num)
  for i in range(cali_steps):
    for j in range(21):
      cali_joints[i,j] = glove.get_hand_joints().contents[j]
    if i%5 == 0:
      print("\r", end="")
      print("Calibration progress: {}%: ".format(i/5), "▋" * (int(0.2*i) // 2), end="")
  for j in range(21):
    sort_max = np.sort(cali_joints[:,j])[cali_steps-filter_num:]
    sort_min = np.sort(cali_joints[:,j])[:filter_num]
    cali_max[j] = round(np.mean(sort_max))
    cali_min[j] = round(np.mean(sort_min))
  cali = cali_max - cali_min
  file = open('ax.txt','w')
  file.write(str([cali,cali_min]))
  file.close()
  npsave = np.concatenate((cali,cali_min)).copy()
  np.save("/home/kelin/workspace_kelin/RAL-ICRA2023/demonstrations/franka/data/glove_calibration/"+demonstrator,npsave)
  np.save("/home/kelin/workspace_kelin/RAL-ICRA2023/demonstrations/RUTH/data/glove_calibration/"+demonstrator,npsave)
  np.save("/home/kelin/workspace_kelin/RAL-ICRA2023/demonstrations/mano/data/glove_calibration/"+demonstrator,npsave)
  print("\n\nCalibration completed, press any button to continue")
  input()
  return cali, cali_min
    
def hand_boundary(joints, glove_scale, cali_min):
    finger_list = [5,9,13,17]
    bend_list = [7,8,11,12,15,16,19,20]
    thumb_list = [1,2]
    for i in range(21):
        if i in bend_list:
            joints[i] = round(1.5*(joints[i]-cali_min[i])/glove_scale[i],4)
            if joints[i] > 1.5:
                joints[i] = 1.5
        elif i in thumb_list:
            joints[i] = round(0.3*(joints[i]-cali_min[i])/glove_scale[i],4)
        else:
            joints[i] = round((joints[i]-cali_min[i])/glove_scale[i],4)
            if joints[i] > 2:
                joints[i] = 2
        if i in finger_list:
            joints[i] = joints[i] - 0.5
        if joints[i] < 0:
            joints[i] = 0
    hand_joints = np.delete(joints,1)
    hand_joints[0] = hand_joints[0] - 0.5
    if hand_joints[1] > 0.3:
        hand_joints[1] = 0.3
    if hand_joints[2] >0.3:
        hand_joints[2] = 0.3

    return hand_joints

def set_mano_position(manoUid, manoJointNameToID, manoJoints, mano_joints):  # set positions of ur5 joints
    for i in range(16):
        p.setJointMotorControl2(manoUid, manoJointNameToID[str(i)], p.POSITION_CONTROL, mano_joints[i])
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[0]], p.POSITION_CONTROL, mano_joints[0]) #1
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[1]], p.POSITION_CONTROL, mano_joints[1]) #3
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[2]], p.POSITION_CONTROL, mano_joints[2]) #5
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[3]], p.POSITION_CONTROL, mano_joints[3]) #7
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[4]], p.POSITION_CONTROL, mano_joints[4]) #9
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[5]], p.POSITION_CONTROL, mano_joints[5]) #11
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[6]], p.POSITION_CONTROL, mano_joints[6]) #13
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[7]], p.POSITION_CONTROL, mano_joints[7]) #15
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[8]], p.POSITION_CONTROL, mano_joints[8]) #17
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[9]], p.POSITION_CONTROL, mano_joints[9])  #19
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[10]], p.POSITION_CONTROL, mano_joints[10]) #21
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[11]], p.POSITION_CONTROL, mano_joints[11]) #23
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[12]], p.POSITION_CONTROL, mano_joints[12]) #25
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[13]], p.POSITION_CONTROL, mano_joints[13]) #27
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[14]], p.POSITION_CONTROL, mano_joints[14]) #29
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[15]], p.POSITION_CONTROL, mano_joints[15]) #31
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[16]], p.POSITION_CONTROL, mano_joints[16]) #33
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[17]], p.POSITION_CONTROL, mano_joints[17]) #35
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[18]], p.POSITION_CONTROL, mano_joints[18]) #38
    # p.setJointMotorControl2(manoUid, manoJointNameToID[manoJoints[19]], p.POSITION_CONTROL, mano_joints[19]) #41

def plot(x,fx,fy,fz):
   
    plt.clf()
    plt.subplot(221)
    plt.plot(x,fx,color='red')
    plt.xlabel('Time(s)') 
    plt.ylabel('Force(N)')
    plt.title('Force_X')
    plt.subplot(222)
    plt.plot(x,fy,color='green')
    plt.xlabel('Time(s)') 
    plt.ylabel('Force(N)')
    plt.title('Force_Y')
    plt.subplot(223)
    plt.plot(x,fz,color='blue')
    plt.xlabel('Time(s)') 
    plt.ylabel('Force(N)') 
    plt.title('Force_Z')
    plt.pause(0.1)
    plt.ioff()

def set_robot_arm():
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host = '192.168.0.104'
    port = 5008
    mySocket.bind((host, port))
    mySocket.listen(10)
    print("Waiting for connection...")
    client, address = mySocket.accept()
    return client

def msg_to_robot(client,data):
    data_msg = str(data)
    data_msg_b = data_msg.encode()
    # data = struct.pack("!I" + "d" * len(data), len(data), *data)
    client.send(data_msg_b)
    
# def hand_force_feedback(manoUid,manoJointNameToID, manoJoints,thumb,index,mid,ring,pinky):
    
#     finger_force = np.zeros(5)
#     thumb_data = p.getContactPoints(bodyA = manoUid, linkIndexA = thumb)
#     index_data = p.getContactPoints(bodyA = manoUid, linkIndexA = index)
#     mid_data = p.getContactPoints(bodyA = manoUid, linkIndexA = mid)
#     ring_data = p.getContactPoints(bodyA = manoUid, linkIndexA = ring)
#     pinky_data = p.getContactPoints(bodyA = manoUid, linkIndexA = pinky)
#     finger_list = [thumb_data, index_data, mid_data, ring_data, pinky_data]
#     for i in range(len(finger_list)):
#         if finger_list[i]:
#             #print(len((finger_list[i])[0]))
#             finger_force[i] = ((finger_list[i])[0])[9]   
#     return finger_force

def hand_apply_force(glove,force):
    a = 1.71706106e-03
    b = 2.56987447
    PWM = np.zeros(5)
    for i in range(len(force)):
        PWM[i] = math.sqrt(max((force[i] - b)*2 / a, 0))
        PWM[i] = round(clamp(PWM[i], 0, 100))
    glove.force_feedback(int(PWM[0]),int(PWM[1]),int(PWM[2]),int(PWM[3]),int(PWM[4]))

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def hand_force_feedback(manoUid,manoJointNameToID, manoJoints,manoLinkNameToID,manoLinks):
    temp_data = []
    contact_flag = False
    torques = np.zeros(15)
    positions = np.zeros(15)
    finger_force = np.zeros(5)
    d = 0.0085
    links = [0.0794,0.0397,0.046,0.027,0.0573,0.029,0.0526,0.0283,0.0433,0.0266]
    for i in range(44):
        if p.getContactPoints(bodyA = manoUid, linkIndexA = manoLinkNameToID[manoLinks[i]]):
            contact_flag = True
            continue
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[1]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[2]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[3]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[5]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[6]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[7]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[9]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[10]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[11]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[13]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[14]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[15]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[17]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[18]]))
    temp_data.append(p.getJointState(manoUid, manoJointNameToID[manoJoints[19]]))
    for i in range(15):
        torques[i] = abs((temp_data[i])[3])
        positions[i] = abs((temp_data[i])[0])
        if contact_flag == False:
            torques[i] = 0
    finger_force[0] = (torques[0]+torques[1]+torques[2])/(3*d+2*links[1]*math.cos(positions[1])+links[0]*math.cos(positions[1]+positions[2]))
    finger_force[1] = (torques[3]+torques[4]+torques[5])/(3*d+2*links[3]*math.cos(positions[4])+links[2]*math.cos(positions[4]+positions[5]))
    finger_force[2] = (torques[6]+torques[7]+torques[8])/(3*d+2*links[5]*math.cos(positions[7])+links[4]*math.cos(positions[7]+positions[8]))
    finger_force[3] = (torques[9]+torques[10]+torques[11])/(3*d+2*links[7]*math.cos(positions[10])+links[6]*math.cos(positions[10]+positions[11]))
    finger_force[4] = (torques[12]+torques[13]+torques[14])/(3*d+2*links[9]*math.cos(positions[13])+links[8]*math.cos(positions[13]+positions[14]))
    return finger_force

def tracker(v,kp,ki,x_c,y_c,z_c,manoUid,x_rot,y_rot,z_rot):
    tracker_pose = v.devices["tracker_1"].get_pose_euler()
    pos_hand = np.array(p.getLinkState(manoUid, 0)[0])
    vel_hand = np.array(p.getLinkState(manoUid, 0, 1)[6])
    x = round(tracker_pose[0],4) - x_c 
    y = round(tracker_pose[1],4) - y_c +1
    z = round(tracker_pose[2],4) - z_c 
    pos_tracker = np.array([z,x,y]) 
    force = kp*(pos_tracker  - pos_hand) + ki*( - vel_hand)

    ang_vel_tracker = v.devices["tracker_1"].get_angular_velocity()
    angular_hand = np.array(p.getLinkState(manoUid, 0, 1)[7])
    x_rot = x_rot + ang_vel_tracker[0]*3.14/180*2/5
    y_rot = y_rot + ang_vel_tracker[1]*3.14/180*2/5
    z_rot = z_rot + ang_vel_tracker[2]*3.14/180*2/5
    ori_tracker = p.getQuaternionFromEuler([z_rot, x_rot,y_rot])
    angular_tracker = np.array([ ang_vel_tracker[2]*3.14/180,ang_vel_tracker[0]*3.14/180,ang_vel_tracker[1]*3.14/180])
    torque = ki*(angular_tracker-angular_hand)

    return pos_tracker,ori_tracker,force,torque,x_rot,y_rot,z_rot
