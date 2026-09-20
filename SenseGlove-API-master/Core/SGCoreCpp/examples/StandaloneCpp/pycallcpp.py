import ctypes
import numpy as np
import os
import sys
import time
import math
from ctypes import *

sys.stdout.flush()

time.sleep(0.05)
dll = ctypes.cdll.LoadLibrary
lib1 = dll('./libsenseglove.so') 
lib1.get_hand_joints.restype = ctypes.POINTER(ctypes.c_int * 21)
def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def progress_bar():
  for i in range(1, 101):
    print("\r", end="")
    print("Calibration progress: {}%: ".format(i), "▋" * (i // 2), end="")

def calibration():
  print("Calibrate the glove, press any botton to start:")
  os.system('pause')
  print("****************************************************")
  print("please open and close your hand")
  cali_steps = 500
  filter_num = 10
  cali_joints = np.zeros((cali_steps,21))
  cali_max = np.zeros(21)
  cali_min = np.zeros(21)
  sort_max = np.zeros(filter_num)
  sort_min = np.zeros(filter_num)
  for i in range(cali_steps):
    for j in range(21):
      cali_joints[i,j] = glove.main(j)
    if i%5 == 0:
      print("\r", end="")
      print("Calibration progress: {}%: ".format(i/5), "▋" * (int(0.2*i) // 2), end="")

  for j in range(21):
    sort_max = np.sort(cali_joints[:,j])[cali_steps-filter_num:]
    sort_min = np.sort(cali_joints[:,j])[:filter_num]
    cali_max[j] = round(np.mean(sort_max))
    cali_min[j] = round(np.mean(sort_min))
   
  print("Calibration completed, press any botton to continue")
  os.system('pause')
  return cali_max, cali_min
  
def main():
  joints = np.zeros(21)
  j = 0
  a = 1.71706106e-03
  b = 2.56987447
  while True:
    joints = lib1.get_hand_joints().contents
    list = [joints[0],joints[1]]
    print(list)
    force = abs(15*math.sin(j/1000*math.pi))
    PWM = math.sqrt(max((force - b) / a, 0))
    PWM = round(clamp(PWM, 0, 100))
    #lib1.force_feedback(PWM,PWM,PWM,PWM,PWM)
    print(PWM)
    j = j+1
  lib3.main(a)

if __name__ == "__main__":
  a = lib1.connect()
  main()
  #calibration()