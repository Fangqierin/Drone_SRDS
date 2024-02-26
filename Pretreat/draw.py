# #here we use 15 as example. 
# import numpy as np
import matplotlib.pyplot as plt 
# import pandas as pd
# import math 
# import random
# import copy
# import itertools
# import MCTS_drone as MCTS
# import Drone_fly as game
# import Node as nd
# from Wp_sel import M_a,Wap, get_wp, Get_D, Get_WapCa, Get_TL, Calculate_inflos, GetFov
# win=pd.read_csv('~/Desktop/Drone_code/data/win_ro_f_5.csv',sep=' ') 
# global task_dic
# task_dic={'df1':[3,0.3],'dh1':[3,0.1],'dw1':[3,0.1],'dw2':[2,0.01],'df2':[3,0.01],'dh2':[2,0.1],
#           'mf':[5,0.5],'mh1':[4,0.5],'mh2':[4,0.3]}
# tasks=task_dic.keys()
# # ins_m_dic={'30':{'df':0.8,'dh':0.6,'dw':0.5,'mf':0.3,'mh':0.3},'15':{'df':1,'dh':1,'dw':1,'mf':0.7,'mh':0.5},
# #            '10':{'df':1,'dh':1,'dw':1,'mf':1,'mh':1},'0':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0}}  
# ins_m_dic={'30':{'df':0.8,'dh':0.6,'dw':0.5,'mf':0.3,'mh':0.3},'15':{'df':1,'dh':1,'dw':1,'mf':0.7,'mh':0.5},
#            '10.5':{'df':1,'dh':1,'dw':1,'mf':1,'mh':1},'10':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0},'0':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0}} 
# global R_v # drone rate  
# R_v=3
# global To# loiter time 
# To=3 
# ###########################################################
# drone=pd.read_csv('~/Desktop/Drone_code/drone_char.csv',sep=' ')
# task_com=[['df1','dh1','dw1'],['df2','dh1','dw2'],['df2','dh2'],['mf','mh1'],['mh2'],['df2'],['mf','dh1','dw1']]
# random.seed(0)
# task=[random.choice(task_com) for i in range(len(win))] 
# id_list=list(win['id'])
# m_set={}
# for i in range(len(win)):  
#     value=[]
#     for j in task[i]:
#         value.append(task_dic.get(j)[0])
#     last_v=-100*np.ones(len(task[i]))
#     a=M_a(id_list[i],task[i],value,last_v) # here assume the inital time is 0, and all initial value is the intial value of each task. 
#     m_set[id_list[i]]=a # Here we get all monitoring areas with tasks ###################################################
# ###################################
# Wap_set=[]    # waypoint set 
# wp,tp,co=Get_WapCa([15,30],win,drone,3)
# for i in range(len(wp)):
#     wapp=Wap(i,wp[i],tp[i],co[i])
#     Wap_set.append(wapp) 
# #####################################################
# # Q=[53,7,9,4,10,2,30,50]#visiting sequence list, Here we rule the first one is the initial location, will not loiter. 
# # T=Get_TL(Wap_set,Q,win,R_v,To)
# # res,m1=Calculate_inflos(m_set,T,0,360,task_dic,ins_m_dic,To)
# # print(res)
# RootState = game.State(0, [], 0)
# Root = nd.Node(RootState)
# D=Get_D(wp,R_v)=
# x = MCTS.MCTS(Root,True)
# 
# d={'d':1,'s':2}
# for k,v in sorted(list(d.keys()), key = lambda t : t[0],t[1]):
#     print(k,'->',v)
#     
# d={'d':3,'s':2}
# for k,v in sorted(d.items(),key=lambda t : t[1]):
#     print(k,'->',v)
# def p (a):
#     return a+1
# def f(d : dict, p : callable):
#          if len(d) == 0:    
#              return 0    
#          else:    
#              count = 0    
#              for k in d:    
#                  if p(d[k]):    
#                      count += d[k]    
#              return count
# print(f(d,p))
# d={}
# for k in d:
#     print(k)
# print(sum(v for v in d.values() if p(v)))
# 
# def alternative(lob):
#     if len(lob)<=1:
#         return True
#    
#     return lob[0]!=lob[1] and alternative(lob[1:])
#        
#     
# # def alternative2(lob):
# #     print(lob)
# #     if len(lob)<=1:
# #         return True
# #     if len(lob)==2:
# #         return lob[0]!=lob[1]
# #     else: 
# #         if alternative2(lob[1:]):
# #             print(lob,"now")
# #             return lob[0]!=lob[1]
# #         else:
# #             return False
#     
#     
#     
# print(alternative([True,False,True,False,True]))
# print(alternative([True]))
#print("s",False==0)

#for i in range(-2):
   # print(i)
'''
import numpy as np
v={20:0.6,50:1,100:0.6,120:0.6}
dy=0.02
y=[]
iv=0
u=iv
la_t=0
la_v=iv
for i in np.arange(0,130,0.5):
    if v.get(i)!=None:
        la_v=v.get(i)
        la_t=i
#     print(la_v*np.exp((1-dy),i-la_t))
    print(pow((1-dy),i-la_t))
    y.append(la_v*pow((1-dy),i-la_t))
    
x=list(np.arange(0,130,0.5))

# plt.grid( linestyle='--', linewidth=1)
# plt.xlabel('Time')
# plt.ylabel('Reliability')
# plt.plot(x,y)
# plt.show()

y=[];z=[];s=[];c=[]
dy=0.2
x=list(np.arange(0,10))
f=list(np.arange(2,10,0.5))
for i in np.arange(0,10):
    y.append(1*pow(1-dy,i-0))
plt.plot(x,y)
for i in np.arange(0,10):
    z.append(0.6*pow(1-dy,i-2))
    c.append(0.4*pow(1-dy,i-0))
    s.append(y[i]-z[i])
# plt.plot(x,z)
# plt.plot(x,s)
# plt.plot(x,c)
# plt.show()


from scipy.integrate import quad

def integrand(x):
    return pow(1-dy,x)

ans, err = quad(integrand, 0, 3)
def integral(cur_acc,acc,dy,t,T):
    if cur_acc< acc:
        a=acc-cur_acc
        if dy==0:
            return a*(T-t)
        d=1-dy
        return round(a*(pow(d,T)-pow(d,t))/np.log(d),2)
    else:
        return 0
print(integral(0,1,0,0,3))
#print(np.log(1))
'''
a=[[] for i in range(2)]
a[0]=[2,3,4]
print(a)
# s=[(0,0) for i in range(3)]
# print([s[i][0]for i in range(3)])