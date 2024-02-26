
# arguments: 

import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd
import math 
import matplotlib
import itertools
import math
import random
import time
import copy
# from numpy import cross, eye, dot
from Wp_sel import M_a, Wap, get_wp, Get_D, Get_WapCa,Get_ty_WapCa, GetFov, M,Get_VG_D,rot,rot_back,Figure,Write_WP, Read_D
from Heuristic_drone_update import Dynamic_Sample, Dynamic_Acc, Greedy_WPS,Tabu,Get_TL,Greedy_Min,Tabu_min,Calculate_Acc, Greedy_TSP, Update_M_TL
from lib_partition import Write_M_WP,Get_VG_D_M
from Auction2020 import Auction,Partitioner,Groups,Partitioner
from collections import defaultdict
from fire_sim import Sim_fire
from Get_SMT import Greedy_SMT
import sys
import time
#import pickle 
import copy 
import csv
# import paho.mqtt.client as mqtt
# impprove_flag=True
# try:
#   #  T_f=int(sys.argv[1]);  # flight time 
#     Drone_Num=int(sys.argv[1]);
#     file_name=str(sys.argv[2]);
#     va=int(sys.argv[3]) # random seed 
# #     ii=int(sys.argv[4])
# #     ii_end=int(sys.argv[5])
#     policy=int((sys.argv[4]))
#     par=int(sys.argv[5])
#     improve_time=int(sys.argv[6])
# except: 
#   #  T_f=600
#     Drone_Num=3
#     file_name='see'
#     va=1
#     policy=2
#     par=0
#     improve_time=0
# policy= 0: detection threshold ==0.6
# policy =1: detection threshold =1 
# policy =2: give the df2 at the beginning, and detection threshold== 0.6
# time_slot=1
# plan_duration=300 
# simulation_time=1800
# T_t=np.arange(60,simulation_time,120)
# replan_period=300
# if policy==0 or policy==2:
#     threshold=0.5
# if policy==1 or policy==3:
#     threshold=0.1

# global R_v # drone rate  
# global To# loiter time 

# Outputfile=f"result/result_2021/{file_name}__{improve_time}_{Drone_Num}_{va}_{policy}_{par}.txt"
# Drone_N=3
# D=[5,15]    
# Record_flag=True
# faces=7
# R_v=3
# To=5
# st_nor=np.array([0,-1,0])
# drone=pd.read_csv('../data/drone_char.csv',sep=' ')
# building=pd.read_csv('../data/dbh_stu.csv',delimiter=' ')
# Cov_file='../data2/wp_cover_'
# d_file='../data/Travel_Distance.csv'
# m_d_file='../data/M_Distance.csv'
# win_fname='../data/win_ro_f_'
#print("get wp")
# Wap_set,Wap_ty=Write_WP(Drone_N,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname)
# wps=[Wap_set[i].loc for i in range(len(Wap_set))]
#print([Wap_set[i].cover for i in range(len(Wap_set))])
#Figure(building, wps)
# ty_set=[Wap_set[i].ty for i in range(len(Wap_set))]
# Wap_ty_set=[[]for i in range(len(D))]
# n=0
# for i in np.sort(list(set(ty_set))):
 #         if j.ty==i:
#             Wap_ty_set[n].append(j)
#     n=n+1     ############## Got a set of different type of waps. 
## for partition ####################################3
# W_M_set,Id_M_set=Write_M_WP(Drone_N,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname)
# dic_M_W=dict(zip(Id_M_set,W_M_set))
# #D=Get_VG_D_M(dic_M_W,building,R_v,1,m_d_file)  
# M_dis=Read_D(m_d_file,R_v=1,To=0,T_iter=0)
#print(M_dis,max(max(M_dis[i][:]) for i in range(len(M_dis)) ))
# high_task=['mf','mh','dw2','df2','dh2','mw']


# co=[Wap_set[i].cover for i in range(len(Wap_set))]
# #print([Wap_ty[1][i].ty for i in range(len(Wap_ty[1]))])
# in_loc=[0,-20,0]
# #D=Get_VG_D(wps,building,R_v,1,d_file,in_loc)
# T_iter=3 # the interval time of to sequential shots
# Dis=Read_D(d_file,R_v,To,T_iter)
# #print(Dis)
# ########################################. Event Generation 
# global task_dic
# ins_m_dic={'15':{'df2':0.7,'df3':0.7,'df':0.7,'ds':0.9,'dh2':0.69,'dh':0.69,'dw':0,'dw2':0,'dw3':0,'mf':0,'mh':0,'mh1':0,'ms':0,'mw':0},'10':{'df':0.7,'df3':0.7,'ds':0.8,'dh':0.9,'dw':0.8,'mf':0.7,'mh':0.5,'ms':0.7,'mh1':0},
#            '5':{'df':0.99,'df2':0.99,'df3':0.99,'ds':1,'dh':0.98,'dh2':0.98,'dw':0.8,'dw2':0.8,'dw3':0.8,'mf':0.89,'mh':0.9,'mh1':0.9,'ms':1,'mw':0.80},'0':{'df2':0,'df':0,'df3':0,'dh2':0,'dw3':0,'dh':0,'ds':0,'dw2':0,'dw':0,'mf':0,'mh':0,'ms':0,'mw':0,'mh1':0}} 

# #####get the correlations among windows in one layer. 
# win_layer=pd.read_csv('../data/layer_win.csv',sep=' ')
# all_wins=pd.read_csv('../data/all_win.csv',sep=' ')
# # co=np.zeros((len(win_layer),len(win_layer)))  # get the distance among the same layer. 
# lay_num=len(win_layer)
# all_num=len(all_wins)
############################## for simulation. 
# # max_v=len(win_layer)-1 
# # for i in range(len(win_layer)):    # get the 
# #     for j in range(i, len(win_layer)):
# #         d=min(abs(j-i),max_v-j+i+1)
# #         co[i][j]=d
# #         co[j][i]=d 

# global f_num
# global f_up
# global p_floor
# global w_num
# f_num=2 
# f_up=1
# p_floor=3
# w_num=32
# task_dic={'df1':[3,0.3],'dh1':[3,0.1],'dw1':[3,0.1],'dw2':[2,0.01],'df2':[3,0.01],'dh2':[2,0.1],
#           'mf':[5,0.5],'mh1':[4,0.5],'mh2':[4,0.3]}
#task_dic={'df':[1,0.002],'ds':[1,0.002],'dh':[1,0.002],'dw':[1,0.0005],'mf':[1,0.006],'ms':[1,0.006],'mh':[1,0.006]}
#task_dic={'df':[3,0],'ds':[1,0],'dh':[2,0],'dw':[1,0],'mf':[5,0],'ms':[5,0],'mh':[4,0]}




class Controller: 
    def __init__(self,all_num,lay_num,Wap_ty_set=[],report=[],task_dic={},ins_m_dic={},Dis=[],f_num=2,f_up=1,p_floor=3,w_num=32,To=5,T=0,T_total=0,Drone_num=2,Dic_M_W=[],fire_floors=[],hum_floors=[],win_floors=[]):

        self.State=-1*(np.ones([all_num,4]))
        Table={}
        self.win_floors=win_floors
        self.all_num=all_num
        self.fire_floors=fire_floors
        self.hum_floors=hum_floors
        self.begin_monitor=0
        Table['fire']=set()
        Table['smoke']=set()
        Table['human']=set()
        Table['window']=set()
        self.to_monitor=False
#         Table['f_s']={}
#         Table['h_s']={}
        Table['ext']=set()
        self.Table=Table 
        self.report=report 
        self.lay_num=lay_num # not sure if we keep this 
        self.Task=defaultdict(set)# for different tasks 
        self.task_dic=task_dic
        self.cu_wp=0
        self.cu_time=0
        self.initial_time=0
        #self.Record_table={}
        self.Record_state={}  # [t][f] or [t][h] for record for human state and fire state.
        for i in task_dic.keys(): 
            self.Task[i]=set()    
        # for rules: 
        self.f_num=f_num
        self.f_up=f_up
        self.p_floor=p_floor
        self.w_num=w_num
        self.To=To
        self.IM_set=[]
        self.IM_cu_set=[]
        self.Ma_set={}
        self.Wap_ty_set=Wap_ty_set
        self.Cu_Wap_ty=[]
        self.ini_time=int(time.time())
        self.last_run_time=0
        self.T=T
        self.Dis=Dis
        self.ins_m_dic=ins_m_dic
        # for Tabu search. 
        self.MaxTabuSize = 100000
        self.NeigSize = 500#500
        self.stopTurn = 20 #500
        self.Drone_num=Drone_num
        self.M_dis=M_dis
        self.Dic_M_W=Dic_M_W
        self.M_set_split=[]
        self.T_max=1800
        self.T_total=T_total
        self.partition=None
        self.perception=defaultdict(dict)
        self.disappear=defaultdict(dict)
        self.Record_Ma={}
        self.Record_Tl=defaultdict(list)
    def get_neigh(self, ii,la_num,n):
        ne=list(range(-n,n+1))  # left and right 
        i=ii%la_num  # at which layer? 
        ni=ii//la_num   # the offset of layer 
        nn=[]
        for j in ne:
            k=i-j
            if k<0:
                k=k+la_num
            if k>(la_num-1):
                k=-(la_num-k)
            k=k+la_num*ni
            nn.append(k)
        nn=list(set(nn))
        return (nn)
#     def increase_fire(self):
#         #random.seed(time.time())
#         times=random.randint(1,2)
#         layers=self.all_num//self.lay_num
#         neigh=[]
#         for i in self.report:
#             if random.random()>0.6: # left to right
#                 neigh=neigh+self.get_neigh(i, self.lay_num, times)
#             if random.random()>0.6:
#                 for t in range(times):
#                     if i//self.lay_num<(layers-1):
#                         neigh=neigh+[i+t*self.lay_num]
#                     if i//self.lay_num>0:
#                         neigh=neigh+[i-t*self.lay_num]
#         return neigh
    def Gen_Task(self,first): # Define all tasks according to 'plicy', report and collected data. 
        if first:
            self.Task['df'].update(set(self.report))
            if policy==2 or policy==3:
                self.Task['df3'].update(set(self.report))
        else:
            self.Task['df3']=set()
        tmp=copy.deepcopy(self.Table['fire'])
        tmp.update(self.report)
        layers=self.all_num//self.lay_num
        if first:
            n=[]
            for i in self.win_floors:
                #print(f"???" ,self.all_floors,i, all_num//lay_num)
                if i>=0 and i<all_num//lay_num:
                    #print(f"sorry",all_num//lay_num)
                    n=n+self.get_neigh(i*(lay_num),self.lay_num,self.lay_num)
            self.Task['dw'].update(n)
        else:
            self.Task['dw']=set()
        if first:
            n=[]
            for i in self.fire_floors:
                if i>=0 and i<all_num//lay_num:
                    n=n+self.get_neigh(i*lay_num,self.lay_num,self.lay_num)
            self.Task['df'].update(n)
            n=[]
            for i in self.hum_floors:
                if i>=0 and i<all_num//lay_num:
                    n=n+self.get_neigh(i*lay_num,self.lay_num,self.lay_num)
            self.Task['dh'].update(n)
            #self.Task['df'].update(n)
        else:
            self.Task['dh']=set()
                ################################################# risky area detection.
        if not first: 
            n2=[]
            for i in list(self.Table['fire']):
                n2=self.get_neigh(i,self.lay_num,1)
                for  j in range(2):
                    if i//self.lay_num<(layers-j):
                        n2=n2+self.get_neigh(i+j*self.lay_num,lay_num,1)
                self.Task['df2'].update(n2) 
        ############################################################
        if not first:
            self.Task['mf'].update(list(self.Table['fire']))
            #self.Task['ms'].update(list(self.Table['smoke']))
            tp=list(self.Table['fire'])
            #fire_floors=set([i//self.lay_num for i in tp]+[i//self.lay_num+1 for i in tp])
           # print(f"see fire_floors {fire_floors}")
            for i in list(self.Table['human']):
                if i//self.lay_num in self.fire_floors:
                    self.Task['mh'].update([i])
                else:
                    self.Task['mh1'].update([i])
            #self.Task['mh'].update(list(self.Table['human']))
            self.Task['dh2'].update(list(self.Table['human']))
            #print(self.Task)
            self.Task['dw2'].update(list(self.Table['fire']-self.Table['window'])) #close to broken
            #self.Task['df']=set()

    def First_task(self): # this one need to implemented! according to report, task and table. 
        self.Gen_Task(True)
        #print(f"check task {self.Task}")
        self.Gen_Ma()
        self.M_partition()
        
    def Coordiniation(self,T,T_total,la_w=0): # generate drones with assigned monitoring targets. 
        #print(f"check coordination {la_w} {self.cu_time}")
        Drones=[]
        Wap_set=list(itertools.chain(*self.Wap_ty_set))
        Wap_set.append(Wap(0,[],0,[]))
        Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
        if la_w==0:
            la_w=[self.cu_wp]*self.Drone_num
        for i in range(self.Drone_num):
            M_set=dict(zip(self.M_set_split[i],[self.Ma_set.get(i) for i in self.M_set_split[i]]))
            drone=Drone(i,self.Wap_ty_set,M_set,self.Dis,self.task_dic,self.ins_m_dic,la_w[i],self.cu_time,T,T_total) 
            Drones.append(drone)
        return Drones,Wap_dic
    
    def Gen_Ma(self):
        tmp=[list(list(self.Task.values())[k])for k in range(len(self.Task.values()))]
        self.IM_set=list(set(itertools.chain.from_iterable(tmp)))
        for i in self.IM_set:  # how to judge if it is already be generated? 
            if self.Ma_set.get(i)==None: 
                self.Ma_set[i]=M_a(i,[],[],[]) # initial last visiting time is -1 
        for i in list(self.Task.keys()):
            for j in list(self.Task[i]):
                if not(i in self.Ma_set[j].task): 
                    self.Ma_set[j].task=self.Ma_set[j].task+[i]
                    self.Ma_set[j].va=self.Ma_set[j].va+[0]
                    self.Ma_set[j].la_t=self.Ma_set[j].la_t+[-1]
                    self.Ma_set[j].ac_re=self.Ma_set[j].ac_re+[0]
                    self.Ma_set[j].start=self.Ma_set[j].start+[self.initial_time]
                    self.Ma_set[j].end=self.Ma_set[j].end+[self.T_total]
        self.Record_Ma=copy.deepcopy(self.Ma_set)
    def M_partition(self): 
        dic_M_W=self.Dic_M_W
        M_dis=self.M_dis
        M_set_split=[[] for i in range(self.Drone_num)]
        m_w=dict(zip(list(range(self.Drone_num)),[0]*self.Drone_num))
        T_M=set(self.IM_set)
        if self.Drone_num==1:
            self.M_set_split=[list(self.IM_set)]
        else:
            M_loc=[dic_M_W.get(i) for i in T_M]
            S_M=[]
            M_fre={}  
            for i, j in self.Ma_set.items():
                M_fre[i]=max([(self.T_max*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))])    
            for i, j in self.Ma_set.items():
                #M_fre[i]=max([(600*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))])    
            #    M_fre[i]=max([(self.T*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))])    
            #initialization.
                if par==0:
                    #M_fre[i]=max([(self.T_max*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))])    
                    max_si=max([self.task_dic.get(j.task[i])[0] for i in range(len(j.task))]) 
                    si_tasks=[ j.task[i] for i in range(len(j.task)) if self.task_dic.get(j.task[i])[0]==max_si ]
                    max_fre=max([(self.T_max*(self.task_dic.get(si_t)[1]) if self.task_dic.get(si_t)[1]!=0 else 1 ) for si_t in si_tasks])  
                    M_fre[i]=max_si*max_fre
                elif par==1:
                    M_fre[i]=1 #max([(self.T_max*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))]) 
                elif par==2:
                    M_fre[i]=1
            #print(f"check items",self.Ma_set.items)
            M_s=sorted(M_fre.items(), key=lambda k:-k[1])
            Av_F=(sum(list(M_fre.values()))/self.Drone_num)*1.1
            ini=M_s[0][0]
            Center=[]
            Center.append(ini)
            S_M.append(ini)
            for i in range(1,self.Drone_num):
                tm=T_M-set(S_M)         
                Dur=sorted([(M_fre[j]*sum((M_dis[j][i] for i in S_M)),j) for j in tm], reverse=True)
                Center.append(Dur[0][1])
                S_M.append(Dur[0][1])
            #print(f" here is center {Center}")
            for i in range(len(M_set_split)):
                M_set_split[i].append(S_M[i])
                m_w[i]=m_w[i]+M_fre.get(S_M[i])
            #print(f"frequency {M_fre.items()}")
            Av_center=copy.deepcopy(Center)
            groups=Groups(T_M,Center,M_dis,M_fre,dic_M_W,To,0)
            groups.Get_initial()
            #self.draw(groups.M_set_split, groups.Center,[self.report,list(self.Task['mh'])],f"../drone_matlab/first_output_{self.Drone_num}_{self.T}.txt")
            self.partition=Partitioner(groups,M_dis,M_fre,len(Center),dic_M_W,To,improve_time,80,0)
            if par!=2:
                groups=self.partition.do_improve()
            # self.draw(best_group.M_set_split, best_group.Center,[self.report,list(self.Task['mh'])],f"../drone_matlab/partition_output_{self.Drone_num}_{self.T}.txt")
            self.M_set_split=groups.M_set_split
            self.group=groups

    def Update_Ma(self,at_t,la_w,enter_monitor):
        tmp=[list(list(self.Task.values())[k])for k in range(len(self.Task.values()))]
        old_IM=self.IM_set
        self.IM_set=list(set(itertools.chain.from_iterable(tmp)))
        Ma_new={}; add_M=[];update_M=[]
        for i in self.IM_set:  # how to judge if it is already be generated? 
            Ma_new[i]=M_a(i,[],[],[])
            if self.Ma_set.get(i)==None: 
                add_M.append(i)
        remove_M=[i for i in old_IM if i not in self.IM_set]
        for i in list(self.Task.keys()):
            for j in list(self.Task[i]):
                if self.Ma_set.get(j)==None:  # new monitoring area 
                    Ma_new[j].task=Ma_new[j].task+[i]
                    Ma_new[j].va=Ma_new[j].va+[0]
                    Ma_new[j].la_t=Ma_new[j].la_t+[-1]
                    Ma_new[j].ac_re=Ma_new[j].ac_re+[0]
                    Ma_new[j].start=Ma_new[j].start+[at_t*60]
                    Ma_new[j].end=Ma_new[j].end+[self.T_total]
                    #print(f"not in task ---> {Ma_new[j].__dict__}")
                    #if self.Record_Ma.get(j)==None:
                    if Record_flag==True:
                        self.Record_Ma[j]=M_a(j,[],[],[])
                        self.Record_Ma[j].task=self.Record_Ma[j].task+[i]
                        self.Record_Ma[j].va=self.Record_Ma[j].va+[0]
                        self.Record_Ma[j].la_t=self.Record_Ma[j].la_t+[-1]
                        self.Record_Ma[j].ac_re=self.Record_Ma[j].ac_re+[0]
                        self.Record_Ma[j].start=self.Record_Ma[j].start+[at_t*60]
                        self.Record_Ma[j].end=self.Record_Ma[j].end+[self.T_total]    
                elif not(i in self.Ma_set[j].task): # exist monitoring area but not tasks 
                    Ma_new[j].task=Ma_new[j].task+[i]
                    Ma_new[j].va=Ma_new[j].va+[0]
                    Ma_new[j].la_t=Ma_new[j].la_t+[-1]
                    Ma_new[j].ac_re=Ma_new[j].ac_re+[0]
                    Ma_new[j].start=Ma_new[j].start+[at_t*60]
                    Ma_new[j].end=Ma_new[j].end+[self.T_total]
                    if par==0:
                        max_si=max([self.task_dic.get(self.Ma_set[j].task[iii])[0] for iii in range(len(self.Ma_set[j].task))]) 
                        si=self.task_dic.get(i)[0]
                        #print(f"compare ",max_si,si)
                        if si>max_si:
                            update_M.append(j)
                    #if not(i in self.Record_Ma[j].task):
                    if Record_flag==True:
                        self.Record_Ma[j].task=self.Record_Ma[j].task+[i]
                        self.Record_Ma[j].va=self.Record_Ma[j].va+[0]
                        self.Record_Ma[j].la_t=self.Record_Ma[j].la_t+[-1]
                        self.Record_Ma[j].ac_re=self.Record_Ma[j].ac_re+[0]
                        self.Record_Ma[j].start=self.Record_Ma[j].start+[at_t*60]
                        self.Record_Ma[j].end=self.Record_Ma[j].end+[self.T_total]
                    #print(f"not in task ---> {Ma_new[j].__dict__}")
                else: # already exist (no change) 
                    Ma_new[j].task=Ma_new[j].task+[i]
                    seq=self.Ma_set[j].task.index(i)
                    Ma_new[j].va=Ma_new[j].va+[self.Ma_set[j].va[seq]]
                    Ma_new[j].la_t=Ma_new[j].la_t+[self.Ma_set[j].la_t[seq]]
                    Ma_new[j].ac_re=Ma_new[j].ac_re+[self.Ma_set[j].ac_re[seq]]
                    #print(f"why here is {self.Ma_set[j].ac_re}")            
                    Ma_new[j].start=Ma_new[j].start+[self.Ma_set[j].start[seq]]
                    Ma_new[j].end=Ma_new[j].end+[self.T_total]
        if Record_flag==True:
            for i, j in self.Ma_set.items():
                if Ma_new.get(i)==None:
                    #print(f"why {self.Record_Ma.get(i)}")
                    old_end=self.Record_Ma.get(i).end
                    self.Record_Ma.get(i).end=[at_t*60]*len(old_end)
                else:
                    remove_task=list(set(self.Ma_set[i].task)-set(Ma_new[i].task))
                    if len(remove_task)!=0:
                        for m in remove_task:
                            index=[k for k in range(len(self.Record_Ma[i].task)) if self.Record_Ma[i].task[k]==m][-1]
                            self.Record_Ma[i].end[index]=at_t*60
        self.Ma_set=Ma_new      
        if old_IM!=self.IM_set:
            self.Update_partition(add_M, remove_M, list(set(update_M)),la_w,enter_monitor)
        
    def Update_partition(self,add_M,remove_M,update_M,la_w,enter_monitor):
        #print(f"see check update",add_M,remove_M,update_M)
        if self.Drone_num==1:
                self.M_set_split=[list(self.IM_set)]
        else:
            M_fre={}  
            for i, j in self.Ma_set.items():
                if par==0:
                    max_si=max([self.task_dic.get(j.task[i])[0] for i in range(len(j.task))]) 
                    si_tasks=[ j.task[i] for i in range(len(j.task)) if self.task_dic.get(j.task[i])[0]==max_si ]
                    max_fre=max([(self.T_max*(self.task_dic.get(si_t)[1]) if self.task_dic.get(si_t)[1]!=0 else 1 ) for si_t in si_tasks])  
                    M_fre[i]=max_si*max_fre
                elif par==1:
                    M_fre[i]=1 
                elif par==2:
                    M_fre[i]=1 
            if len(remove_M)>0:
                Group=self.partition.Remove_M(remove_M,M_fre)
                self.M_set_split=Group.M_set_split
                self.group=Group
            if len(add_M)>0:
                #print(f" see why fail ", add_M, M_fre)
                Group=self.partition.Add_M(add_M,M_fre)
                self.M_set_split=Group.M_set_split
                self.group=Group
            if len(update_M)>0:
                Group=self.partition.Remove_M(update_M,M_fre)
                self.M_set_split=Group.M_set_split
                self.group=Group
                Group=self.partition.Add_M(update_M,M_fre)
                self.M_set_split=Group.M_set_split
                self.group=Group
            if par!=2 and enter_monitor:
                print(f"enter_monitor and update")
                groups=self.partition.do_improve()
                self.M_set_split=groups.M_set_split
                self.group=groups
            elif par!=2:
                groups=self.partition.do_improve(max_time=0)
                self.M_set_split=groups.M_set_split
                self.group=groups

    def Update_Task(self,update,Replan,P_list,ww,at_t,Table,Wap_dic,enter_monitor):   
        M_p=copy.deepcopy(self.Ma_set)
        M_split=copy.deepcopy(self.M_set_split)
        cu_t=self.cu_time
        Tl_com=defaultdict(list)
        for i in range(len(P_list)):
            P=P_list[i][0:ww[i][0]]
            TL=Get_TL([Wap_dic.get(i) for i in P],self.Dis,cu_t)
            #print(f"check TL {TL}")
            if len(TL)>0:
                for i,j in TL.items():
                    Tl_com[i]=sorted(Tl_com[i]+TL[i])
                if Record_flag==True:
                    for m, see in TL.items():
                        self.Record_Tl[m]=self.Record_Tl[m]+TL[m]
        M_new=Update_M_TL(M_p,Tl_com,cu_t,self.Dis,self.task_dic,self.ins_m_dic,self.T_total)
        self.Ma_set=M_new      # update the initial one. 
        #self.Table=Table
        if update==True:
            self.Table=Table
            print(f" see Updated Table {self.Table}") 
            if Replan==True:
                self.Gen_Task(False)
            self.Update_Ma(at_t,[i[1] for i in ww],enter_monitor)
        return self.Task
#         drone=Drone(i,self.Wap_ty_set,M_set,self.Dis,self.task_dic,self.ins_m_dic,la_w[i],self.cu_time,T_total,T_total) 
#         re,min_re,event_size=Calculate_Acc(self.Ma_set,TL,self.cu_time,self.T_total,self.task_dic,self.ins_m_dic)
 ###############################################################################     
    def check_update(self,conclusion):
        enter_monitor=False
        Update=False
        Replan=False
        Table2=copy.deepcopy(self.Table)
        #print(f"want to see table {t} {self.Table}")
        #print(f"what to see disappear {self.disappear}")
        dis_copy=copy.deepcopy(self.disappear)
        for tar,w in dis_copy.items():
            if len(w)>0:
                for m, visit in w.items():
                    if visit>0:
                        self.Table[tar].remove(m)   # here update self Table 
                        self.disappear[tar].pop(m)
                        try: 
                            conclusion[tar].pop(m)
                        except:
                            pass
                        if tar=='fire':
                            self.Table['ext'].update([m])
                        Update=True
                        Replan=True
        Table2=copy.deepcopy(self.Table)
        for ii,j in conclusion.items():
                tmp=list(j.keys())
                #print(f"see the accuracy {j}")
                if len(set(tmp)-Table2.get(str(ii)))!=0:
                    if str(ii) in ['fire','human']:
                        Replan=True
                    Update=True
                    Table2[str(ii)].update(tmp)
        #if Update==True:
            #c.Record_table[tt]=Table2
    #                 print(f"update state at {tt} {Table2}")
    #                 print(f"self   state  at {tt} {self.Table}")
        sum_len=0
        if self.to_monitor==False:
            check=True
            for i,j in self.perception.items():
                if len(j)>0:
                    Replan=False
                    check=False
            if self.begin_monitor==0:
                self.to_monitor=check
            if check==True and self.begin_monitor==0:
                enter_monitor=True
                Replan=True
                #print(f" enter monitoring phase {tt}")
        return Update, Replan, Table2, enter_monitor 
 ############################################     
    def track(self,Sim,P_list,sim_logging_file,check_slot,Wap_dic,start_time):

        # with open(sim_logging_file, mode="a", newline="") as rewardfile:
        #     fieldnames = [
        #         "seed",
        #         "drone_id",
        #         "time",
        #         "waypoint_seq",
        #         "waypoint_id",
        #         "waypoint_location"
        #         "monitor_area",
        #         "task",
        #     ]
        #     rewardwriter = csv.DictWriter(rewardfile, fieldnames=fieldnames)
        dis_fire=[]
        dis_hum=[]
        def check_update():
            enter_monitor=False
            Update=False
            Replan=False
            Table2=copy.deepcopy(self.Table)
            dis_copy=copy.deepcopy(self.disappear)
            for tar,w in dis_copy.items():
                if len(w)>0:
                    for m, visit in w.items():
                        if visit>0:
                            self.Table[tar].remove(m)   # here update self Table 
                            self.disappear[tar].pop(m)
                            try: 
                                conclusion[tar].pop(m)
                            except:
                                pass
                            if tar=='fire':       ####2024 If detected fire disappear. 
                                self.Table['ext'].update([m])
                            Update=True
                            Replan=True 
            Table2=copy.deepcopy(self.Table)
            for ii,j in conclusion.items():
                    tmp=list(j.keys())
                    if len(set(tmp)-Table2.get(str(ii)))!=0: ### Detected new events
                        if str(ii) in ['fire','human']:
                            Replan=True  #2024 comment it for stopping event-driven
                        Update=True
                        Table2[str(ii)].update(tmp)
            sum_len=0
            if self.to_monitor==False:
                check=True
                for i,j in self.perception.items(): ### 2024 why? What is perception? 
                    if len(j)>0:
                        Replan=False
                        check=False
                if self.begin_monitor==0:
                    self.to_monitor=check
                if check==True and self.begin_monitor==0:   # 2024 Enter to the monitoring phase! 
                    enter_monitor=True
                    Replan=True
                    print(f"Enter monitoring phase {tt}")
            print(f"Current Table at {tt-1} {Table2}")
            return Update, Replan, Table2, enter_monitor
        def Where(tt): #Return the drone's waypoint at time tt. 
            where=[]
            for d in range(len(P_list)):
                if P_track.get(d)!=None:
                    arr=list(P_track.get(d).keys())
                    times=[i for i in arr if i>(tt-start_time)*60-20]
                    if len(times)>0:
                        t=min(times)
                        where.append(P_track.get(d).get(t))
                    else:
                        where.append((0,P_list[d][0]))
                else:
                    where.append((0,P_list[d][0]))
            return where    
        ##################################### Combine the arrival times at each waypoint 
        D=self.Dis
        Table2=copy.deepcopy(self.Table)
        #log=open(logfile,'a')
        conclusion=defaultdict(dict)
        P_com=defaultdict(list)# Combine waypoint sequence of multiple drones. 
        P_track=defaultdict(dict)
        num=0
        for P in P_list:
            Time=c.cu_time
            tt=(Time//(60))+start_time
            w_id=[Wap_dic.get(i).id for i in P]
            for i in range(1,len(P)):
                Time=Time+D[w_id[i-1]][w_id[i]]
                P_track[num][Time]=(i,P[i])  # 2024 logging time and id 
                tt=(Time//(60))+start_time
                #tt=Time+start_time*60
                P_com[tt].append(P[i])
            num=num+1
        ####################
        with open(sim_logging_file, mode="a", newline="") as rewardfile:
            fieldnames = [
                "seed",
                "drone_id",
                "time",
                "waypoint_seq",
                "waypoint_id",
                "waypoint_location",
                "monitor_area",
                "task",
            ]
            rewardwriter = csv.DictWriter(rewardfile, fieldnames=fieldnames)
            for drone_id, values in P_track.items():
                for time, waypoints in values.items():
                    waypoint_seq=waypoints[0]
                    id=waypoints[1]
                    cov=Wap_dic.get(id).cover
                    loc=Wap_dic.get(id).loc
                    task_set=set()
                    for m in cov:
                        task_set.update(self.Ma_set.get(m).task)
                    write_data = {
                        "seed":0,
                        "drone_id": drone_id,
                        "time": time,
                        "waypoint_seq": waypoint_seq,
                        "waypoint_id": id, 
                        "waypoint_location": loc, 
                        "monitor_area": cov,
                        "task": task_set,
                    }
                    rewardwriter.writerow(write_data)
            ################################
            rewardfile.close()
        P_c=sorted(list(P_com.items()))
        timeline=[int(i[0]) for i in P_c]# The arrival time of waypoints for all drones 
        if len(timeline)>0:   
            check_t=timeline[0]+check_slot
        else:
            ww=Where(tt)
            return False,False,Table2, tt-start_time, ww, False, True
        ############################################################
        tasks=['df','dh','dw']
        if c.cu_time==0: # if current time is 0, 
            for tk in tasks:
                check_fire=c.Task.get(tk)
                self.perception[tk]=dict(zip(list(check_fire),[1]*len(check_fire)))
        for i in range(len(timeline)):
            tt=timeline[i]
            if c.begin_monitor!=0 and c.begin_monitor==tt-start_time and self.to_monitor==False: # If start monitor? 
                ww=Where(tt)
                self.to_monitor=True
                self.disappear=defaultdict(dict)
                return True,True,Table2, tt-start_time, ww, True,False
            ####################### Check update to decide if to re-plan the motion. 
            if int(tt)==int(check_t):
                Update,Replan,Table2,enter_monitor=check_update()
                if (Update==True and Replan==True) or enter_monitor==True:
                    ww=Where(tt)
                    return Update,Replan,Table2, tt-start_time, ww, enter_monitor,False
                else:
                    check_t=check_t+check_slot   
            ############################################Get the simulated status
            try:
                fire=Sim.get(tt)['f']
                fire_state=Sim.get(tt)['f_s']
            except:
                fire=[]
                fire_state=[]
            try:
                human=Sim.get(tt)['h']
                human_state=Sim.get(tt)['h_s']
            except:
                human=[]
                human_state=[]
            try:
                window=Sim.get(tt)['w']
            except:
                window=[]
            self.Record_state[tt]={}
            self.Record_state[tt]['f_s']=[]
            self.Record_state[tt]['h_s']=[]
            targets=['fire','human','window']
            s_p=['f','h','w']
            ################################# Check what events disappeared. 
            for i in range(len(targets)):
                try:
                    remove=list(self.Table.get(targets[i])-set(Sim.get(tt).get(s_p[i])))
                    #remove=list(Table2.get(targets[i])-set(Sim.get(tt).get(s_p[i])))
                    for m in remove:
                        if self.disappear[targets[i]].get(m)==None:
                            self.disappear[targets[i]][m]=0
                except:
                    pass
            if self.disappear.get('fire')!=None:
                    dis_fire=self.disappear.get('fire').keys()
            if self.disappear.get('human')!=None:
                    dis_hum=self.disappear.get('human').keys()
            ###################################################### 
            for w in P_com.get(tt):
                cov=Wap_dic.get(w).cover ## 2024 logging coverage 
                ty=Wap_dic.get(w).ty
                see=[];dis_see=[];check_area=[]
                dis_see.append([n for n in cov if n in dis_fire])
                dis_see.append([n for n in cov if n in dis_hum])
                see.append([n for n in cov if n in fire])
                see.append([n for n in cov if n in human])
                see.append([n for n in cov if n in window])
                for tk in tasks:
                    if len(self.perception[tk])!=0:
                        check_area=check_area+[n for n in cov if n in list(self.perception[tk].keys())]
                tasks=['df','dh','dw']
                for i in range(len(targets)):
                    if len(see[i])!=0:
                        acc=self.ins_m_dic.get(str(int(ty))).get(tasks[i]) 
                        if acc!=0:
                            for s in see[i]:
                                a=random.random()
                                a=a/2+0.5   # it must greater than 0.5
                                if a<acc:
                                    if conclusion[targets[i]].get(s)==None:
                                        conclusion[targets[i]][s]=acc
                                    elif conclusion[targets[i]].get(s)<acc:
                                        conclusion[targets[i]][s]=acc  
                                    print(f"Event {targets[i]} {s} at {tt}")
                ############ Record fire_state and human_state:
#                 task_m=['mf','mh']; #see_m=[0,1] it just match 0 and 1
#                 target_s=['f_s', 'h_s']
#                 for i in range(len(target_s)):
#                     if len(see[i])!=0:
#                         acc=self.ins_m_dic.get(str(int(ty))).get(task_m[i]) 
#                         if acc!=0:
#                             for s in see[i]:
#                                 a=random.random()
#                                 a=a/2+0.5   # it must greater than 0.5
#                                 if a<acc:
#                                     if self.Record_state[tt].get(target_s[i])==None:
#                                         self.Record_state[tt][target_s[i]]=[s]
#                                     else: 
#                                         self.Record_state[tt][target_s[i]].append(s)
                ###############################################
                for i in range(len(targets)-1):
                    if len(dis_see[i])!=0:
                        acc=self.ins_m_dic.get(str(int(ty))).get(tasks[i]) 
                        if acc!=0:
                            for s in dis_see[i]:
                                a=random.random()
                                a=a/2+0.5 
                                if a<acc:
                                    print(f" Disappear {targets[i]} {s} at {tt}")
                                    if self.disappear[targets[i]].get(s)==None:
                                        self.disappear[targets[i]][s]=acc
                                    else:
                                        self.disappear[targets[i]][s]=self.disappear[targets[i]][s]+acc
                #if c.cu_time==0:
                for s in check_area: # Perception is for checking if to go monitoring phase. 
                    for tk in tasks:
                        acc=self.ins_m_dic.get(str(int(ty))).get(tk) 
                        if self.perception[tk].get(s)!=None:
                            self.perception[tk][s]=self.perception[tk][s]*(1-acc)
                            if self.perception[tk][s]<=threshold:
                                self.perception[tk].pop(s)
            #print(f"see what is your perception", self.perception)
                #print(f" what we see at waypoint at time {tt} {conclusion} ")
        Update,Replan,Table2,enter_monitor=check_update()
        ww=Where(tt)
        if (tt-start_time+1)*60>=c.T_total:
            return Update,Replan,Table2, tt-start_time, ww, enter_monitor,True
        else:
            return Update,Replan,Table2, tt-start_time, ww, enter_monitor,False
   
######################################################################################## 
class Drone:     # WPC   # All Monitoring areas with tasks

    def __init__(self,id,Wap_ty_set,Ma_set,Dis,task_dic,ins_m_dic,cu_wp,cu_time,T,T_total):
        self.id=id
        self.Wap_ty_set=Wap_ty_set
        self.Ma_set=Ma_set
        self.Dis=Dis
        self.task_dic=task_dic
        self.ins_m_dic=ins_m_dic
        self.cu_wp=cu_wp
        self.cu_time=cu_time
        self.T=T
        self.T_total=T_total
        self.IM_set=[]
        self.last_run_time=0
        self.Cu_Wap_ty=[]
    def Get_WPC(self):       
        IM_set=[i.id for i in self.Ma_set.values()]
        if self.IM_set!=IM_set:  # if no change, no need to change Wapc 
            self.IM_set=IM_set
            self.Cu_Wap_ty=[]
            #print(f" see if this drone has no area: {len(self.Wap_ty_set)}")
            for i in range(len(self.Wap_ty_set)):
                ty=self.Wap_ty_set[i][0].ty 
                al_cov=[self.Wap_ty_set[i][k].cover for k in range(len(self.Wap_ty_set[i])) ]
                cov=[list(set(al_cov[k])&set(self.IM_set)) for k in  range((len(al_cov)))]
                #print("here",[set(al_cov[k]) for k in  range(len(al_cov))],cov,self.IM_set)
                #covv=[list(al_cov[k]) for k in  range((len(al_cov)))]
                w_id=[self.Wap_ty_set[i][k].id for k in range(len(self.Wap_ty_set[i])) ]
                cov_com=list([cov[i],w_id[i]]for i in range(len(cov)))
                cov_com.sort(key = lambda s:len(s[0]),reverse=True)
                cov_dic=dict(zip(w_id,cov)) # cover current monitoring area not
                co=copy.deepcopy(cov_com)
                M=copy.deepcopy(self.IM_set)
                W=[];C=[]
                while len(M)>0: 
                    co.sort(key = lambda s:len(s[0]),reverse=True)  # sort first 
                    tmp=co[0][0]
                    w=Wap(co[0][1],[],ty,cov_dic.get(co[0][1]))
                    W.append(w) # do not use location of that. 
                    M=list(set(M)-set(tmp))
                    for ii in range (len(co)): 
                        co[ii][0]=list(set(co[ii][0])-set(tmp))
                #print(f"check {self.id} {i} {[W[i].id for i in range(len(W))] }")
                self.Cu_Wap_ty.append(W) 
                #print(f"not sure if can get here {self.Cu_Wap_ty}")
    def Do_Heuristic(self,kind,case):  
        self.Get_WPC()    
        if kind==-1:
            #print(f" why", self.Cu_Wap_ty)
            Wap_set=list(itertools.chain(*self.Cu_Wap_ty))
        else: 
            Wap_set=list(self.Cu_Wap_ty[kind])
        if len(Wap_set)!=0:
            Wap_set.append(Wap(0,[],0,[]))
            Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
            if case==3:
                #P,cu_t=Greedy_TSP(kind,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
                P,cu_t=Greedy_TSP(kind,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time, min(self.cu_time+self.T,self.T_total))
                #print(f"why case 3 is same {kind} {case} {P}")
            else:
                if case==11 or case==12:
                    P,cu_t=Greedy_SMT(case,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,min(self.cu_time+self.T,self.T_total))
                #P,cu_t=Greedy_WPS(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.T_total,self.T_total)
                #P,cu_t=Greedy_WPS(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.T_total)
                #print(f"make the plan from {self.cu_time} to {min(self.cu_time+self.T,self.T_total)}")
                    #print(f"make plan from {(self.cu_time)/60} {(min(self.cu_time+self.T,self.T_total))/60} ")
                else:
                    P,cu_t=Greedy_Min(case,1,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time, min(self.cu_time+self.T,self.T_total),self.T_total,self.Cu_Wap_ty)
                #print(f"should not be same {kind} {case} {P}")
        #         if case==5: 
        #             P,cu_t=Tabu(P,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.NeigSize,self.MaxTabuSize,self.stopTurn,
        #stop_condition=120,time_condition=True)
        #         if case==3:
        #             P,cu_t=Greedy_Min(1,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
        #             print("start Tabu")
        #             P,cu_t=Tabu_min(P,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.NeigSize,self.MaxTabuSize,self.stopTurn,
        #                        stop_condition=120,time_condition=True )
                N_WP=[Wap_dic.get(i) for i in P]
        else:
            P=[self.cu_wp]
        #self.last_run_time=self.last_run_time+(self.T)
        #TL=Get_TL([Wap_dic.get(i) for i in P],self.Dis,self.cu_time)
        #re,min_re,event_size=Calculate_Acc(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic)
        #re,min_re,event_size=Calculate_Acc(self.Ma_set,TL,self.cu_time,self.T_total,self.task_dic,self.ins_m_dic)
        #print(f"{kind},{case},End with reliability value",re,"min_re:",min_re,"at",cu_t)
        #print(P)
        return P
        #return re,min_re,event_size,P
        
def Get_sim(rseed,a,output,time_slot,lay_num=12):  # here a is a window!!!!! 
    Sim_fire(rseed,a,output,time_slot)  # write the simulation in a file.
    Timeline=pd.read_csv(output,sep='/ ',engine='python')
    Sim=defaultdict(dict)
    Sim_real=defaultdict(dict)
    win=[]
    firs=list(Timeline['f'])
    tmp=[i[1:-1].split(', ') for i in firs]
    all_fire=set()
    for i in tmp:
        try:
            all_fire.update([int(k) for k in i])
        except:
            pass
    hum_floors=set([i//lay_num for i in all_fire]+[i//lay_num +1 for i in all_fire])
    fire_floors=set([i//lay_num for i in all_fire])  
    win_floors=set([i//lay_num for i in all_fire]+[i//lay_num -1 for i in all_fire])  # this layer and the lower layer. 
    #print(f"ss {all_floors}")
    for index,row in Timeline.iterrows():
        #print(row)
        tmp=row['f'][1:-1].split(', ')
        tmps=row['f_s'][1:-1].split(', ')
        #print(f"check  {tmp}")
        if len(tmp)==1 and tmp[0]=='':
            Sim[row['t']]['f']=[]
            Sim_real[row['t']]['f']=[]
            Sim[row['t']]['f_s']=[]
            Sim_real[row['t']]['f_s']=[]
        else:
            fire=[int(tmp[i]) for i in range(len(tmp))]
            fire_state=[int(tmps[i]) for i in range(len(tmp))]
            #fire_floors=[fire[i]//lay_num+1 for i in range(len(fire))]
            #only_fire=[fire[i]//lay_num for i in range(len(fire))]
            #fire_floors=list(set(fire_floors+[fire[i]//lay_num+1 for i in range(len(fire))]+[fire[i]//lay_num+1 for i in range(len(fire))]+[fire[i]//lay_num-1 for i in range(len(fire))])) #here should add 2 upper layer, and one lower layer.
#             if fire[i]//lay_num>1:
#                 tmp=[fire[i]//lay_num-1 for i in range(len(fire))]+[fire[i]//lay_num-2 for i in range(len(fire))]
#             elif fire[i]//lay_num>0:
#                 tmp=[fire[i]//lay_num-1 for i in range(len(fire))]
#             else:  
#                 tmp=[]
#             human_floors=list(set(tmp+[fire[i]//lay_num+2 for i in range(len(fire))]))
            Sim[row['t']]['f']=fire
            Sim[row['t']]['f_s']=fire_state
            Sim_real[row['t']]['f']=fire
            Sim_real[row['t']]['f_s']=fire_state
        tmp=row['w'][1:-1].split(', ')
        old_win=copy.deepcopy(win)
        #if len(tmp)>0:
        winn=[int(tmp[i]) for i in range(len(tmp))]
            #Sim_real[row['t']]['w']=winn
        #else:
        #   Sim_real[row['t']]['w']=[]
        win=[]
        for i in winn:
            if i//lay_num in win_floors: # 
                win.append(i)
#             winn=[int(tmp[i]) for i in range(len(tmp))]
#             win=[]
#             for i in winn:
#                 if i//lay_num in only_fire:
#                     win.append(i)
            #print(f"check {fire} {win}")
            #win=list(set(win+old_win))
        Sim[row['t']]['w']=win
        Sim_real[row['t']]['w']=win
        #except:
            #Sim[row['t']]['w']=[]
        tmp=row['h'][1:-1].split(', ')
        tmps=row['h_s'][1:-1].split(', ')
        try:
            humm=[int(tmp[i]) for i in range(len(tmp))]
            hum_s=[int(tmps[i]) for i in range(len(tmp))]
            h_s=dict(zip(humm,hum_s))
            hum=[];state=[]
            for i in humm:
                if i//lay_num in hum_floors:
                    hum.append(i)
                    state.append(h_s.get(i))
            Sim[row['t']]['h']=hum 
            Sim[row['t']]['h_s']=state
            Sim_real[row['t']]['h']=hum
            Sim_real[row['t']]['h_s']=state
        except:
            Sim[row['t']]['h']=[]
            Sim[row['t']]['h_s']=[]
            Sim_real[row['t']]['h_s']=[]
            Sim_real[row['t']]['h']=[]     
    return Sim,Sim_real,fire_floors, hum_floors, win_floors

def check_perception(Sim,R_Table,start_sim,Total,T_time,outlog,i,method,ii,R_state,fire_floors,begin_monitor):
    track_miss=defaultdict(list)
    track_acm=defaultdict(list)
    track_sim=defaultdict(list)
    tt=start_sim
    times=sorted(list(R_Table.keys()))
    targets=['fire','human','window']
    tag=['f','h','w']
    T_t=[ int(i)+start_sim for i in list(np.array(T_time)/60)]
    #print(f" see it is not equal", T_t)
    miss=0
    miss_list=[]
    miss_tar=[0]*len(targets)
    for i in range(len(times)-1):
        tt=times[i]
        while tt >=times[i] and tt<times[i+1]:
            for ta in range(len(targets)):
                sim= Sim[tt].get(tag[ta])
                see=R_Table[times[i]].get(targets[ta])
                track_sim[targets[ta]].append(len(sim))
                if ta==2:  # it is window 
                    miss=miss+len(set(sim)-set(see))
                    miss_tar[ta]=miss_tar[ta]+len(set(sim)-set(see))  # because broken window never close again!
                    track_miss[targets[ta]].append(len(set(sim)-set(see)))
                    track_acm[targets[ta]].append(miss_tar[ta])
                    #print(f" miss {tag[ta]} {tt}: {len(set(sim)-set(see))}")
                else:
                    miss=miss+len(set(sim)-set(see))+len(set(see)-set(sim))
                    miss_tar[ta]=miss_tar[ta]+len(set(sim)-set(see))+len(set(see)-set(sim))
                    track_miss[targets[ta]].append(len(set(sim)-set(see))+len(set(see)-set(sim)))
                    track_acm[targets[ta]].append(miss_tar[ta])
            if tt in T_t:
                miss_list.append(miss)
                #print (tt,T_t,miss_list)
            tt=tt+1
    while tt >=times[-1] and tt<=start_sim+Total/60:
        for ta in range(len(targets)):
            sim= Sim[tt].get(tag[ta])
            see=R_Table[times[-1]].get(targets[ta])
            try:
                track_sim[targets[ta]].append(len(sim))
            except:
                track_sim[targets[ta]].append(0)
            if ta==2:
                miss=miss+len(set(sim)-set(see))
                miss_tar[ta]=miss_tar[ta]+len(set(sim)-set(see))
                track_miss[targets[ta]].append(len(set(sim)-set(see)))
                track_acm[targets[ta]].append(miss_tar[ta])
            else:
                miss=miss+len(set(sim)-set(see))+len(set(see)-set(sim))
                miss_tar[ta]=miss_tar[ta]+len(set(sim)-set(see))+len(set(see)-set(sim))
                track_miss[targets[ta]].append(len(set(sim)-set(see))+len(set(see)-set(sim)))
                track_acm[targets[ta]].append(miss_tar[ta])
                #print(f" miss {tag[ta]} {tt}: {len(set(sim)-set(see))+len(set(see)-set(sim))}")
            #print(f" miss what {set(sim)-set(see)} {set(see)-set(sim)}")
        if tt in T_t:
            miss_list.append(miss)
            #print(tt,T_t,miss_list )
        tt=tt+1
    ################################## for tracking fire and human state. 
    Perception_state={}  # track the perception
    Com_state={}
    last_s={}
    last_s['f_s']={}
    last_s['h_s']={}
    last_s['h_s2']={}
    #print(f"why R_state {R_state}")
    for t,state in R_state.items():
        Perception_state[t]={}
        Com_state[t]={}
        for s, j in state.items(): # for f_s and h_s
            tmp=[];tmp2=[]
            com_tmp=[];com_tmp2=[]
            state=dict(zip(Sim[t].get(s[0]),Sim[t].get(s)))
            for i in Sim[t].get(s[0]):
                if s[0]=='h' and i//lay_num not in fire_floors: #### only monitoring status in fire floor. 
                    #print(t,f"not in fire_floors", i, i//lay_num, fire_floors)
                    if last_s[s].get(i)==None:
                        last_s[s][i]=-1
                    if i in j:
                        tmp2.append(state.get(i))
                        last_s[s][i]=state.get(i)
                    else:
                        tmp2.append(last_s[s].get(i))
                    com_tmp2.append(state.get(i))
                else:
                    #print(t,f" it is in",s[0],i,i//lay_num,fire_floors)
                    if last_s[s].get(i)==None:
                        last_s[s][i]=-1
                    if i in j:
                        tmp.append(state.get(i))
                        last_s[s][i]=state.get(i)
                    else:
                        tmp.append(last_s[s].get(i))
                    com_tmp.append(state.get(i))
#             if s[0]=='h':
#                 print( Sim[t].get(s[0]))
#                 print(f"see not in risky area human",s, tmp2, com_tmp2,tmp,com_tmp)
            if s[0]=='h':
                Perception_state[t]['h_s2']=tmp2
                Com_state[t]['h_s2']=com_tmp2
            Perception_state[t][s]=tmp
            Com_state[t][s]=com_tmp
    #print(f"see to see \n { Perception_state} \n {Com_state}")
    miss_state=[]; miss_h2=[]
    miss=0;miss2=0
    for t, state in Com_state.items():
        for s, j in state.items():
            sim_s=Perception_state.get(t).get(s)
            try:
                ms=len([k for k in range(len(j)) if j[k]!= sim_s[k]])
            except:
                ms=0
            track_miss[s].append(ms)
            if s!='h_s2':
                miss=miss+ms
            else:
                miss2=miss2+ms
        if t in T_t:
            miss_state.append(miss)
            miss_h2.append(miss2)
    j=method
    for i in range(len(T_time)):
        try:
            outlog.write(f"sum {i} {j} {ii} {T_time[i]} {miss_list[i]} \n")
        except:
            outlog.write(f"sum {i} {j} {ii} {T_time[i]} 100000000 \n")
    for i in range(len(T_time)):
#         print(f"{T_time[i]} {miss_list}")
#         print(f"{T_time[i]} {miss_state}")
        if T_time[i] >=begin_monitor:
            try:
                outlog.write(f"sum_state {i} {j} {ii} {T_time[i]} {miss_list[i]+miss_state[i]+miss_h2[i]/2} \n")
            except:
                outlog.write(f"sum_state {i} {j} {ii} {T_time[i]} 100000000 \n")
        else:
            try:
                outlog.write(f"sum_state {i} {j} {ii} {T_time[i]} {miss_list[i]} \n")
            except:
                outlog.write(f"sum_state {i} {j} {ii} {T_time[i]} 100000000 \n")
    for i in range(len(T_time)):
        try:
            outlog.write(f"sum_h2 {i} {j} {ii} {T_time[i]} {miss_h2[i]} \n")
        except:
            outlog.write(f"sum_h2 {i} {j} {ii} {T_time[i]} 100000000 \n")
    for i in list(track_miss.keys()):
        outlog.write(f"miss {i} {j} {ii}")
        for val in track_miss[i]:
            outlog.write(f" {val}")
        outlog.write(f" \n")
    for i in list(track_acm.keys()):
        outlog.write(f"acm {i} {j} {ii}")
        for val in track_acm[i]:
            outlog.write(f" {val}")
        outlog.write(f" \n")
    for i in list(track_sim.keys()):
        outlog.write(f"tsim {i} {j} {ii}")
        for val in track_sim[i]:
            outlog.write(f" {val}")
        outlog.write(f" \n")
    return miss_list, miss_state,miss_h2

def on_connect(client, userdata, flags, rc):
    print("Connected with task "+str(rc))
    client.subscribe([("Task_add", 0),("Task_del", 0)])


def run_sim(Drone_Num, plan_duration, simulation_time, R_v, To): 
        
        D=[5] # 2024 change it only 5 meters    
        policy=2
        # policy= 0: detection threshold ==0.6
        # policy =1: detection threshold =1 
        # policy =2: give the df2 at the beginning, and detection threshold== 0.6
        time_slot=1
        T_t=np.arange(60,simulation_time,120)
        replan_period=300
        if policy==0 or policy==2:
            threshold=0.5
        if policy==1 or policy==3:
            threshold=0.1
        # Outputfile=f"result/result_2021/{file_name}__{improve_time}_{Drone_Num}_{va}_{policy}_{par}.txt"
        ######## Drone Number, monitoring distance 
        # D=[5,15]
        Record_flag=True
        faces=7 
        # global R_v # drone speed   
        # global To # loiter time 
        st_nor=np.array([0,-1,0])
        drone=pd.read_csv('../data/drone_char.csv',sep=' ')  # Drone sensor configuration
        ######## Loading waypoint information 
        building=pd.read_csv('../data/dbh_stu.csv',delimiter=' ')
        Cov_file='../data2/wp_cover_'
        d_file='../data/Travel_Distance.csv'
        m_d_file='../data/M_Distance.csv'
        win_fname='../data/win_ro_f_'
        #### 2024 Generating waypoints for drones 
        Wap_set,Wap_ty=Write_WP(Drone_Num,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname)
        wps=[Wap_set[i].loc for i in range(len(Wap_set))]
        #Figure(building, wps)
        ty_set=[Wap_set[i].ty for i in range(len(Wap_set))]
        ## for partition ###############
        W_M_set,Id_M_set=Write_M_WP(Drone_Num,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname)
        dic_M_W=dict(zip(Id_M_set,W_M_set))
        M_dis=Read_D(m_d_file,R_v=1,To=0,T_iter=0)
        high_task=['mf','mh','dw2','df2','dh2','mw']
        co=[Wap_set[i].cover for i in range(len(Wap_set))]
        in_loc=[0,-20,0]  ###Initial loaction of drones 
        #D=Get_VG_D(wps,building,R_v,1,d_file,in_loc)
        T_iter=3 # the interval time of to sequential shots
        Dis=Read_D(d_file,R_v,To,T_iter)
        ########################################. Event Generation 
        ins_m_dic={'15':{'df2':0.7,'df3':0.7,'df':0.7,'ds':0.9,'dh2':0.69,'dh':0.69,'dw':0,'dw2':0,'dw3':0,'mf':0,'mh':0,'mh1':0,'ms':0,'mw':0},'10':{'df':0.7,'df3':0.7,'ds':0.8,'dh':0.9,'dw':0.8,'mf':0.7,'mh':0.5,'ms':0.7,'mh1':0},
                '5':{'df':0.99,'df2':0.99,'df3':0.99,'ds':1,'dh':0.98,'dh2':0.98,'dw':0.8,'dw2':0.8,'dw3':0.8,'mf':0.89,'mh':0.9,'mh1':0.9,'ms':1,'mw':0.80},'0':{'df2':0,'df':0,'df3':0,'dh2':0,'dw3':0,'dh':0,'ds':0,'dw2':0,'dw':0,'mf':0,'mh':0,'ms':0,'mw':0,'mh1':0}} 
        #####get the correlations among windows in one layer. 
        win_layer=pd.read_csv('../data/layer_win.csv',sep=' ')
        all_wins=pd.read_csv('../data/all_win.csv',sep=' ')
        # co=np.zeros((len(win_layer),len(win_layer)))  # get the distance among the same layer. 
        lay_num=len(win_layer)
        all_num=len(all_wins)
        ############################## for fire simulation. 
        floor_num=12
        rooms_d=pd.read_csv('../data/rooms.csv',sep=',')
        room_AF=dict(zip(list(rooms_d['r']),(zip(list(rooms_d['w']),list(rooms_d['d'])))))
        all_room=len(room_AF)*floor_num
        ############################################# 
        ii=0 # Seed value for fire simulation 
        start_sim=5
        Enter_time=0
        rseed=ii
        random.seed(ii)
        va=3   ### Number of fire sources 
        a=random.sample(range(all_room),va)
        output=f"./result/sim/sim_{a}_{file_name}_{Drone_Num}_{va}_{policy}.csv"
        Sim,Sim_real,fire_floors,hum_floors,win_floors=Get_sim(rseed,a,output,time_slot,lay_num)
        fire_source=Sim.get(start_sim).get('f')
        ##############################################
        sim_logging_file=f"./sim_log_2024/log_seed_{ii}_droneNum_{Drone_Num}_fires_{va}_simtime_{int(simulation_time//60)}min_plan_{int(plan_duration/60)}min.csv"
        with open(sim_logging_file, mode="w", newline="") as file:  ## Logging Actions
            fieldnames = [
                    "seed",
                    "drone_id",
                    "time",
                    "waypoint_seq",
                    "waypoint_id",
                    "waypoint_location",
                    "monitor_area",
                    "task",
                ]
            writer = csv.DictWriter(file, fieldnames=fieldnames)
            writer.writeheader()
            file.close()
        for T_total in [simulation_time]:
            for t_f in [plan_duration]: 
                #c_orgin=Controller(all_num,lay_num,Wap_ty,fire_source,task_dic,ins_m_dic,Dis,Drone_num=Drone_Num,T_total=T_total,fire_floors=fire_floors,hum_floors=hum_floors,win_floors=win_floors)   
                c_orgin=Controller(all_num,lay_num,Wap_ty,fire_source,task_dic,ins_m_dic,Dis,To=To,Drone_num=Drone_Num,Dic_M_W=dic_M_W,T_total=T_total,fire_floors=fire_floors,hum_floors=hum_floors,win_floors=win_floors)   
                c_orgin.First_task()
                # outlog.write(f"random:{rseed}T:{T_total}\n")
                miss=0
                t=0 
                for i in range(-1,0):
                    for j in [10]:#range(14):  
                        done=False
                        c=copy.deepcopy(c_orgin)
                        c.begin_monitor=0
                        Drones,Wap_dic=c.Coordiniation(t_f,T_total)
                        time_start=time.time()
                        running_time=[]
                        while done==False:
                            P_list=[]
                            t=time.time() 
                            run_time=[]
                            for drone in Drones:
                                P=drone.Do_Heuristic(i,j)
                                P_list.append(P)
                            print(f"-------Drone Flight Planning------{P_list}--")
                            time_slot=int(plan_duration/60)  ## 2024 Set checking slot to plan duration, to stop event-driven
                            update,Replan,Table,cu_time,ww,enter_monitor,done=c.track(Sim_real,P_list,sim_logging_file,time_slot,Wap_dic,start_sim) # here check_slot=1 s
                            Task=c.Update_Task(update,Replan,P_list,ww,cu_time,Table,Wap_dic,enter_monitor)  # update task.update Ma, cu_time. 
                            print(f"------Update Task at {cu_time} min---- {Task}-----")
                            c.cu_time=cu_time*60 
                            Drones,Wap_dic=c.Coordiniation(t_f,T_total,[i[1] for i in ww])
                        #miss,miss_state,miss_h2=check_perception(Sim,c.Record_table,start_sim,c.T_total,T_t,outlog,i,j,ii,c.Record_state,c.fire_floors) # fire floors for tracking human state. 
                        #############################################################

if __name__ == "__main__":
    #task_dic={'df':[2,0],'ds':[1,0],'dh':[1,0],'dw':[1,0],'mf':[3,0],'mh':[3,0],'mw':[3,0]}
    try:
    #  T_f=int(sys.argv[1]);  # flight time 
        Drone_Num=int(sys.argv[1])
        file_name=str(sys.argv[2])
        va=int(sys.argv[3]) # Number of fire sources 
        # ii=int(sys.argv[4])
        # policy=int((sys.argv[4]))
        # par=int(sys.argv[5])
        # improve_time=int(sys.argv[6])
    except: 
    #  T_f=600
        Drone_Num=10
        file_name='see'
        va=1
        # par=0
        # improve_time=0
    task_dic={'df':[1,0.002],'df3':[2,0.002],'ds':[1,0.001],'dh':[1,0.002],'dw':[1.5,0.002],'mf':[3,0.007],'mh1':[1.5,0.007],'mh':[3,0.007],'dw2':[3,0.007],'dw3':[1,0.008],'df2':[4,0.007],'dh2':[3,0.006],'mw':[3,0.005]}
    Drone_Num=3
    plan_duration=300 
    simulation_time=3600
    R_v=3 # Drone flying speed 
    To=5 # Drone loitor time 

    run_sim(Drone_Num, plan_duration, simulation_time, R_v, To)
    