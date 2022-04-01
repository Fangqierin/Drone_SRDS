#here we use 15 as example. 
import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd
import math 
import random
import copy
import itertools
from Wp_sel import M_a,Wap, get_wp, Get_D, Get_WapCa, Get_TL, Calculate_inflos, GetFov
from Heuristic_drone import Heuristic_WpSh
win=pd.read_csv('~/Desktop/Drone_code/data/win_ro_f_5.csv',sep=' ') 
global task_dic
task_dic={'df1':[3,0.3],'dh1':[3,0.1],'dw1':[3,0.1],'dw2':[2,0.01],'df2':[3,0.01],'dh2':[2,0.1],
          'mf':[5,0.5],'mh1':[4,0.5],'mh2':[4,0.3]}
tasks=task_dic.keys()
# ins_m_dic={'30':{'df':0.8,'dh':0.6,'dw':0.5,'mf':0.3,'mh':0.3},'15':{'df':1,'dh':1,'dw':1,'mf':0.7,'mh':0.5},
#            '10':{'df':1,'dh':1,'dw':1,'mf':1,'mh':1},'0':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0}}  
ins_m_dic={'30':{'df':0.8,'dh':0.6,'dw':0.5,'mf':0.3,'mh':0.3},'15':{'df':1,'dh':1,'dw':1,'mf':0.7,'mh':0.5},
           '10.5':{'df':1,'dh':1,'dw':1,'mf':1,'mh':1},'10':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0},'0':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0}} 
global R_v # drone rate  
R_v=3
global To# loiter time 
To=3 
###########################################################
drone=pd.read_csv('~/Desktop/Drone_code/drone_char.csv',sep=' ')
task_com=[['df1','dh1','dw1'],['df2','dh1','dw2'],['df2','dh2'],['mf','mh1'],['mh2'],['df2'],['mf','dh1','dw1']]
random.seed(0)
task=[random.choice(task_com) for i in range(len(win))] 
id_list=list(win['id'])
m_set={}
for i in range(len(win)):  
    value=[]
    for j in task[i]:
        value.append(task_dic.get(j)[0])
    last_v=-100*np.ones(len(task[i]))
    a=M_a(id_list[i],task[i],value,last_v) # here assume the inital time is 0, and all initial value is the intial value of each task. 
    m_set[id_list[i]]=a # Here we get all monitoring areas with tasks ###################################################
###################################
Wap_set=[]    # waypoint set 
wp,tp,co=Get_WapCa([15,30],win,drone,3)
for i in range(len(wp)):
    wapp=Wap(i,wp[i],tp[i],co[i])
    Wap_set.append(wapp) 
#####################################################
# Q=[53,7,9,4,10,2,30,50]#visiting sequence list, Here we rule the first one is the initial location, will not loiter. 
# T=Get_TL(Wap_set,Q,win,R_v,To)
# res,m1=Calculate_inflos(m_set,T,0,360,task_dic,ins_m_dic,To)
# print(res)













