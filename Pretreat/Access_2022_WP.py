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
from numpy import cross, eye, dot
from Wp_sel import M_a, Wap, get_wp, Get_D, Get_WapCa,Get_ty_WapCa, GetFov, M,Get_VG_D,rot,rot_back,Figure,Write_WP, Read_D
from Heuristic_drone_update import Dynamic_Sample, Dynamic_Acc, Greedy_WPS,Tabu,Get_TL,Greedy_Min,Tabu_min,Calculate_Acc, Greedy_TSP, Update_M_TL
from lib_partition import Write_M_WP,Get_VG_D_M
from Auction2020 import Auction,Partitioner,Groups,Partitioner
from collections import defaultdict
from fire_sim import Sim_fire
from Get_SMT import Greedy_SMT
import sys
from pymongo import MongoClient
#import pickle 
import copy 

impprove_flag=True
try:
  #  T_f=int(sys.argv[1]);  # flight time 
    Drone_Num=int(sys.argv[1]);
    file_name=str(sys.argv[2]);
    va=int(sys.argv[3]) # random seed 
    policy=int((sys.argv[4]))
    par=int(sys.argv[5])
    improve_time=int(sys.argv[6])
except: 
  #  T_f=600
    Drone_Num=3
    file_name='see'
    va=1
    policy=2
    par=0
    improve_time=0
# policy= 0: detection threshold ==0.6
# policy =1: detection threshold =1 
# policy =2: give the df2 at the beginning, and detection threshold== 0.6
time_slot=1
plan_duration=300 
simulation_time=1800
T_t=np.arange(60,simulation_time,120)
replan_period=300
if policy==0 or policy==2:
    threshold=0.5
if policy==1 or policy==3:
    threshold=0.1

Outputfile=f"result/result_2021/{file_name}__{improve_time}_{Drone_Num}_{va}_{policy}_{par}.txt"
Drone_N=3
D=[5,15]    
Record_flag=True
faces=7
global R_v # drone rate  
R_v=3
global To# loiter time 
To=5
st_nor=np.array([0,-1,0])
drone=pd.read_csv('../data/drone_char.csv',sep=' ')
building=pd.read_csv('../data/dbh_stu.csv',delimiter=' ')
Cov_file='../data2/wp_cover_'
d_file='../data/Travel_Distance.csv'
m_d_file='../data/M_Distance.csv'
win_fname='../data/win_ro_f_'
#print("get wp")
Wap_set,Wap_ty=Write_WP(Drone_N,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname)
wps=[Wap_set[i].loc for i in range(len(Wap_set))]
print(f"see wps {Wap_set[0].loc}")
#print([Wap_set[i].cover for i in range(len(Wap_set))])
#Figure(building, wps)
ty_set=[Wap_set[i].ty for i in range(len(Wap_set))]
# Wap_ty_set=[[]for i in range(len(D))]
# n=0
# for i in np.sort(list(set(ty_set))):
 #         if j.ty==i:
#             Wap_ty_set[n].append(j)
#     n=n+1     ############## Got a set of different type of waps. 
## for partition ####################################3
W_M_set,Id_M_set=Write_M_WP(Drone_N,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname)
dic_M_W=dict(zip(Id_M_set,W_M_set))
#D=Get_VG_D_M(dic_M_W,building,R_v,1,m_d_file)  
M_dis=Read_D(m_d_file,R_v=1,To=0,T_iter=0)
#print(M_dis,max(max(M_dis[i][:]) for i in range(len(M_dis)) ))
high_task=['mf','mh','dw2','df2','dh2','mw']

co=[Wap_set[i].cover for i in range(len(Wap_set))]
#print([Wap_ty[1][i].ty for i in range(len(Wap_ty[1]))])
in_loc=[0,-20,0]
#D=Get_VG_D(wps,building,R_v,1,d_file,in_loc)
T_iter=3 # the interval time of to sequential shots
Dis=Read_D(d_file,R_v,To,T_iter)
#print(Dis)
########################################. Event Generation 
global task_dic
# task_dic={'df1':[3,0.3],'dh1':[3,0.1],'dw1':[3,0.1],'dw2':[2,0.01],'df2':[3,0.01],'dh2':[2,0.1],
#           'mf':[5,0.5],'mh1':[4,0.5],'mh2':[4,0.3]}
# task_dic={'df':[3,0.005],'ds':[3,0.3],'dh':[3,0.1],'dw':[3,0.1],'dw2':[2,0.01],'df2':[3,0.01],'dh2':[2,0.1],
#           'mf':[5,0.5],'ms':[5,0.5],'mh':[4,0.5],'mh2':[4,0.3]}
# ins_m_dic={'30':{'df':0.8,'dh':0.6,'dw':0.5,'mf':0.3,'mh':0.3},'15':{'df':1,'dh':1,'dw':1,'mf':0.7,'mh':0.5},
#            '10':{'df':1,'dh':1,'dw':1,'mf':1,'mh':1},'0':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0}}  
# ins_m_dic={'30':{'df':0.8,'dh':0.6,'dw':0.5,'mf':0.3,'mh':0.3},'15':{'df':1,'dh':1,'dw':1,'mf':0.7,'mh':0.5},
          # '10.5':{'df':1,'dh':1,'dw':1,'mf':1,'mh':1},'0':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0},'0':{'df':0,'dh':0,'dw':0,'mf':0,'mh':0}} 
# ins_m_dic={'15':{'df':0.8,'ds':0.7,'dh':0.6,'dw':0.5,'mf':0.3,'mh':0.3,'ms':0.3},'10':{'df':0.9,'ds':1,'dh':1,'dw':1,'mf':0.7,'mh':0.5,'ms':0.7},
#            '5':{'df':1,'ds':1,'dh':1,'dw':1,'mf':1,'mh':1,'ms':1},'0':{'df':0,'dh':0,'ds':0,'dw':0,'mf':0,'mh':0,'ms':1}} 

ins_m_dic={'15':{'df2':0.7,'df3':0.7,'df':0.7,'ds':0.9,'dh2':0.69,'dh':0.69,'dw':0,'dw2':0,'dw3':0,'mf':0,'mh':0,'mh1':0,'ms':0,'mw':0},'10':{'df':0.7,'df3':0.7,'ds':0.8,'dh':0.9,'dw':0.8,'mf':0.7,'mh':0.5,'ms':0.7,'mh1':0},
           '5':{'df':0.99,'df2':0.99,'df3':0.99,'ds':1,'dh':0.98,'dh2':0.98,'dw':0.8,'dw2':0.8,'dw3':0.8,'mf':0.89,'mh':0.9,'mh1':0.9,'ms':1,'mw':0.80},'0':{'df2':0,'df':0,'df3':0,'dh2':0,'dw3':0,'dh':0,'ds':0,'dw2':0,'dw':0,'mf':0,'mh':0,'ms':0,'mw':0,'mh1':0}} 

# ins_m_dic={'15':{'df2':0.7,'df3':0.7,'df':0.7,'ds':0.9,'dh2':0.7,'dh':0.7,'dw':0,'dw2':0,'dw3':0,'mf':0.6,'mh':0,'ms':0.9,'mw':0},'10':{'df':0.7,'df3':0.7,'ds':0.8,'dh':0.9,'dw':0.8,'mf':0.7,'mh':0.5,'ms':0.7},
#            '5':{'df':0.94,'df2':0.94,'df3':0.94,'ds':1,'dh':0.98,'dh2':0.98,'dw':1,'dw2':1,'dw3':1,'mf':0.89,'mh':0.8,'ms':1,'mw':0.73},'0':{'df2':0,'df':0,'df3':0,'dh2':0,'dw3':0,'dh':0,'ds':0,'dw2':0,'dw':0,'mf':0,'mh':0,'ms':1,'mw':0.78}} 



#####get the correlations among windows in one layer. 
win_layer=pd.read_csv('../data/layer_win.csv',sep=' ')
all_wins=pd.read_csv('../data/all_win.csv',sep=' ')
# co=np.zeros((len(win_layer),len(win_layer)))  # get the distance among the same layer. 
lay_num=len(win_layer)
all_num=len(all_wins)
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
task_dic={'df':[1,0.002],'df3':[2,0.002],'ds':[1,0.001],'dh':[1,0.002],'dw':[1.5,0.002],'mf':[3,0.007],'mh1':[1.5,0.007],'mh':[3,0.007],'dw2':[3,0.007],'dw3':[1,0.008],'df2':[4,0.007],'dh2':[3,0.006],'mw':[3,0.005]}

#task_dic={'df':[2,0],'ds':[1,0],'dh':[1,0],'dw':[1,0],'mf':[3,0],'mh':[3,0],'mw':[3,0]}

State=-1*(np.ones(4)) # 1) fire 2) smoke 3) human 4) open window 
report=[]   # is locations of risky area. 
class Controller: 
    def __init__(self,all_num,lay_num,Wap_ty_set=[],report=[],task_dic={},ins_m_dic={},Dis=[],f_num=2,f_up=1,p_floor=3,w_num=32,To=To,T=0,T_total=0,Drone_num=2,Dic_M_W=dic_M_W,fire_floors=[],hum_floors=[],win_floors=[]):

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
        Table['ext']=set()
        self.Table=Table 
        self.report=report 
        self.lay_num=lay_num # not sure if we keep this 
        self.Task=defaultdict(set)# for different tasks 
        self.task_dic=task_dic
        self.cu_wp=0
        self.cu_time=0
        self.initial_time=0
        self.Record_table={}
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
                if i>=0 and i<all_num//lay_num:
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
            tp=list(self.Table['fire'])
            for i in list(self.Table['human']):
                if i//self.lay_num in self.fire_floors:
                    self.Task['mh'].update([i])
                else:
                    self.Task['mh1'].update([i])
            self.Task['dh2'].update(list(self.Table['human']))
            self.Task['dw2'].update(list(self.Table['fire']-self.Table['window'])) #close to broken

    
    def WriteTask(self, writefile):
        writeto=open(writefile,"w")
        for tak, ids in self.Task.items():
            ids=list(ids)
            for i in ids:
                writeto.write(f"{tak} {i} {self.task_dic[tak][0]} {self.task_dic[tak][1]}\n")
        writeto.close()
    
    
    def ReadTask(self,readfile):
        readfrom=open(readfile,"r")
        filedata = readfrom.readlines()
        for line in filedata:
            tmp=line.split(' ')
            tak=tmp[0]
            id=int(tmp[1])
            #print(f"check {tmp}")
            self.Task[tak].add(id)
        #print(f"task {self.Task}")
        readfrom.close()
    
    def Sim_task(self,writefile): # this one need to implemented! according to report, task and table. 
        self.Gen_Task(True)
        self.WriteTask(writefile)
        #self.ReadTask(writefile)
        self.Gen_Ma()
        self.M_partition()
        
    def Mongo_task(self,writefile): # this one need to implemented! according to report, task and table. 
#         self.Gen_Task(True)
#         self.WriteTask(writefile)
        self.ReadTask(writefile)
        self.Gen_Ma()
        self.M_partition()
    
        
    def Coordiniation(self,T,T_total,la_w=0): # generate drones with assigned monitoring targets. 
        #print(f"check coordination {la_w} {self.cu_time}")
        Drones=[]
        Wap_set=list(itertools.chain(*self.Wap_ty_set))
        Wap_set.append(Wap(0,in_loc,0,[]))
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
            #self.draw(best_group.M_set_split, best_group.Center,[self.report,list(self.Task['mh'])],f"../drone_matlab/partition_output_{self.Drone_num}_{self.T}.txt")
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
            if len(TL)>0:
                for i,j in TL.items():
                    Tl_com[i]=sorted(Tl_com[i]+TL[i])
                if Record_flag==True:
                    for m, see in TL.items():
                        self.Record_Tl[m]=self.Record_Tl[m]+TL[m]
        M_new=Update_M_TL(M_p,Tl_com,cu_t,self.Dis,self.task_dic,self.ins_m_dic,self.T_total)
        self.Ma_set=M_new      # update the initial one. 
        if update==True:
            self.Table=Table
            if Replan==True:
                self.Gen_Task(False)
            self.Update_Ma(at_t,[i[1] for i in ww],enter_monitor)
        
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
            for i in range(len(self.Wap_ty_set)):
                ty=self.Wap_ty_set[i][0].ty 
                al_cov=[self.Wap_ty_set[i][k].cover for k in range(len(self.Wap_ty_set[i])) ]
                cov=[list(set(al_cov[k])&set(self.IM_set)) for k in  range((len(al_cov)))]
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
                self.Cu_Wap_ty.append(W) 
    def Do_Heuristic(self,kind,case):  
        self.Get_WPC()    
        if kind==-1:            
            Wap_set=list(itertools.chain(*self.Cu_Wap_ty))
        else: 
            Wap_set=list(self.Cu_Wap_ty[kind])
        if len(Wap_set)!=0:
            Wap_set.append(Wap(0,in_loc,0,[]))
            Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
            if case==3:
                #P,cu_t=Greedy_TSP(kind,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
                P,cu_t=Greedy_TSP(kind,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time, min(self.cu_time+self.T,self.T_total))
            else:
                if case==11 or case==12:
                    P,cu_t=Greedy_SMT(case,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,min(self.cu_time+self.T,self.T_total))
                else:
                    P,cu_t=Greedy_Min(case,1,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time, min(self.cu_time+self.T,self.T_total),self.T_total,self.Cu_Wap_ty)
                N_WP=[Wap_dic.get(i) for i in P]
        else:
            P=[self.cu_wp]
        return P
        #return re,min_re,event_size,P
        
def Get_sim(rseed,a,output,time_slot):  # here a is a window!!!!! 
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
        tmp=row['f'][1:-1].split(', ')
        tmps=row['f_s'][1:-1].split(', ')
        if len(tmp)==1 and tmp[0]=='':
            Sim[row['t']]['f']=[]
            Sim_real[row['t']]['f']=[]
            Sim[row['t']]['f_s']=[]
            Sim_real[row['t']]['f_s']=[]
        else:
            fire=[int(tmp[i]) for i in range(len(tmp))]
            fire_state=[int(tmps[i]) for i in range(len(tmp))]
            Sim[row['t']]['f']=fire
            Sim[row['t']]['f_s']=fire_state
            Sim_real[row['t']]['f']=fire
            Sim_real[row['t']]['f_s']=fire_state
        tmp=row['w'][1:-1].split(', ')
        old_win=copy.deepcopy(win)
        winn=[int(tmp[i]) for i in range(len(tmp))]
        win=[]
        for i in winn:
            if i//lay_num in win_floors: # 
                win.append(i)
        Sim[row['t']]['w']=win
        Sim_real[row['t']]['w']=win
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

def WriteWPS(WP_list,WPfile):
    writeto=open(WPfile,"w")          
    for i in range(len(WP_list)):
        wps=WP_list[i]
        for w in wps:
            writeto.write(f"{i} {w[0]} {w[1]} {w[2]}\n")
    writeto.close()

def Write_WPS_Mongo(WP_list):
    client = MongoClient("mongodb://140.114.89.210:27017/")
    mydb = client["Command"]
    collection = mydb.WPS  
    #collection.delete_many({})
    for i in range(len(WP_list)):
        wps=WP_list[i]
        for w in wps:
            record={"Drone": i, "x":w[0], "y":w[1], "z":w[2]}
            collection.insert_one(record)
    


floor_num=12
rooms_d=pd.read_csv('../data/rooms.csv',sep=',')
room_AF=dict(zip(list(rooms_d['r']),(zip(list(rooms_d['w']),list(rooms_d['d'])))))
all_room=len(room_AF)*floor_num
ii=0
start_sim=5
Enter_time=0
rseed=ii
outlog=open(Outputfile,'a+')
random.seed(ii)
a=random.sample(range(all_room),va)
output=f"./result/sim/sim_{a}_{file_name}_{Drone_Num}_{va}_{policy}_{par}.csv"
Sim,Sim_real,fire_floors,hum_floors,win_floors=Get_sim(rseed,a,output,time_slot)
fire_source=Sim.get(start_sim).get('f')
for T_total in [simulation_time]:
    for t_f in [plan_duration]: 
        c_orgin=Controller(all_num,lay_num,Wap_ty,fire_source,task_dic,ins_m_dic,Dis,Drone_num=Drone_Num,T_total=T_total,fire_floors=fire_floors,hum_floors=hum_floors,win_floors=win_floors)   
        c_orgin.Mongo_task(f"Access_tasks.csv")
        miss=0
        t=0 
        for i in range(-1,0):
            for j in [10]:#range(14):  
                done=False
                c=copy.deepcopy(c_orgin)
                c.begin_monitor=0
                Drones,Wap_dic=c.Coordiniation(t_f,T_total)
                time_start=time.time()
                c.Record_table[start_sim]=c.Table
                running_time=[]
                all_time=[]
                P_list=[]
                WP_list=[]
                t=time.time() 
                run_time=[]
                for drone in Drones:
                    IM_set=[i.id for i in drone.Ma_set.values()]
                    P=drone.Do_Heuristic(i,j)
                    P_list.append(P)
                    WP_list.append([Wap_dic[seq].loc for seq in P ])
                    tt=time.time()-t
                    run_time.append(tt)
                    t=time.time()
                    logfile=f"./result/sim/log_{j}_sim_{a}.csv"
                running=max(run_time)
                running_time.append(running)
                t_l=time.time()
                print(WP_list)

WriteWPS(WP_list,"Access_WPS.csv")

Write_WPS_Mongo(WP_list)
        
        
        
