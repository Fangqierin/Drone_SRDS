
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
from numpy import cross, eye, dot
from Wp_sel import M_a, Wap, get_wp, Get_D, Get_WapCa,Get_ty_WapCa, GetFov, M,Get_VG_D,rot,rot_back,Figure,Write_WP, Read_D, Update_Ma
from Heuristic_drone_weighted import Greedy_WPS,Tabu,Get_TL,Greedy_Min,Tabu_min,Calculate_Acc, Calculate_Acc2,Greedy_TSP,Sample_Acc,Sample_Acc2
from lib_partition import Write_M_WP,Get_VG_D_M
from Auction import Auction,Partitioner,Groups,Partitioner
from collections import defaultdict
from Get_SMT_new import Greedy_SMT
import sys
#import pickle 
import copy 
flag_improve=True
try:
  #  T_f=int(sys.argv[1]);  # flight time 
    Drone_Num=int(sys.argv[1]);
    file_name=str(sys.argv[2]);
   # rseed=int(sys.argv[2]) # random seed 
    par_case=int(sys.argv[3])
    va=int(sys.argv[4])
    threshold=int(sys.argv[5])
    #method=int(sys.argv[6])
except: 
    #T_f=600
    Drone_Num=5
    file_name=''
    par_case=0
    va=3
    threshold=10
    
    #######
method=0  # for use average length.
    
    ################
c_t=3
sample_rate=20
detect_threshold=0.5
Outputfile=f"result/2021_new_result/{file_name}_{Drone_Num}_{par_case}_{va}_{threshold}.txt"
Drone_N=3
D=[5,15]    
#D=[5,16,24]
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
wps_ty=[]
for i in range(len(Wap_ty)):
    wps_ty.append([Wap_ty[i][j].loc for j in range(len(Wap_ty[i]))])
#print([Wap_set[i].cover for i in range(len(Wap_set))])
Figure(building, wps,wps_ty)

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

# ins_m_dic={'15':{'df':0.6,'ds':0.9,'dh':0.6,'dw':0,'mf':0.6,'mh':0.6,'ms':0,'mw':0},'10':{'df':0.7,'ds':0.8,'dh':0.9,'dw':0.8,'mf':0.7,'mh':0.5,'ms':0.7,'mw':0.5},
#            '5':{'df':1,'ds':1,'dh':1,'dw':1,'mf':1,'mh':1,'ms':1,'mw':1},'0':{'df':0,'dh':0,'ds':0,'dw':0,'mf':0,'mh':0,'ms':0,'mw':0}} 

ins_m_dic={'15':{'df':0.7,'ds':0.7,'dh':0.7,'dw':0,'mf':0,'mh':0,'ms':0.9,'mw':0},'10':{'df':0.7,'ds':0.8,'dh':0.9,'dw':0.8,'mf':0.7,'mh':0.5,'ms':0.7},
           '5':{'df':1,'ds':1,'dh':1,'dw':1,'mf':1,'mh':1,'ms':1,'mw':1},'0':{'df':0,'dh':0,'ds':0,'dw':0,'mf':0,'mh':0,'ms':1,'mw':0}} 


#####
#####get the correlations among windows in one layer. 

#win_layer=pd.read_csv('~/Desktop/Drone_code/data/layer_win.csv',sep=' ')
win_layer=pd.read_csv('../data/layer_win.csv',sep=' ')

all_wins=pd.read_csv('../data/all_win.csv',sep=' ')
#all_wins=pd.read_csv('~/Desktop/Drone_code/data/all_win.csv',sep=' ')
# co=np.zeros((len(win_layer),len(win_layer)))  # get the distance among the same layer. 
# # max_v=len(win_layer)-1 
# # for i in range(len(win_layer)):    # get the 
# #     for j in range(i, len(win_layer)):
# #         d=min(abs(j-i),max_v-j+i+1)
# #         co[i][j]=d
# #         co[j][i]=d 
lay_num=len(win_layer)
all_num=len(all_wins)
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
# task_dic={'df':[1,0.002],'ds':[1,0.0015],'dh':[1,0.002],'dw':[1,0.001],'mf':[3,0.005],'mh':[3,0.005],'mw':[3,0.005]}
# task_dic={'df':[1,0.002],'ds':[1,0.0015],'dh':[1,0.002],'dw':[1,0.001],'mf':[3,0.005],'mh':[3,0.005],'dw2':[3,0.008],'df2':[3,0.008],'dh2':[3,0.005],'mw':[3,0.005]}
if c_t==3:
    task_dic={'df':[1.2,0.003],'ds':[1,0.003],'dh':[1,0.003],'dw':[1,0.001],'mf':[3,0.01],'mh':[3,0.01],'dw2':[3,0.008],'df2':[3,0.01],'dh2':[3,0.01],'mw':[3,0.005]}
if c_t==2:
    task_dic={'df':[1.2,0.003],'ds':[1,0.003],'dh':[1,0.003],'dw':[1,0.001],'mf':[2,0.01],'mh':[2,0.01],'dw2':[2,0.008],'df2':[2,0.01],'dh2':[2,0.01],'mw':[2,0.005]}
if c_t==1:
    task_dic={'df':[1.2,0.003],'ds':[1,0.003],'dh':[1,0.003],'dw':[1,0.001],'mf':[1,0.01],'mh':[1,0.01],'dw2':[1,0.008],'df2':[1,0.01],'dh2':[1,0.01],'mw':[1,0.005]}

#task_dic1={'df':[1,0.001],'ds':[1,0.001],'dh':[1,0.001],'dw':[1,0.001],'mf':[3,0.005],'mh':[3,0.005],'dw2':[3,0.008],'df2':[3,0.008],'dh2':[3,0.005],'mw':[3,0.005]}
high_task=['mf','mh','dw2','df2','dh2','mw']

State=-1*(np.ones(4)) # 1) fire 2) smoke 3) human 4) open window 
report=[]   # is locations of risky area. 
co_s=1
class Controller: 
    def __init__(self,all_num,lay_num,Wap_ty_set=[],report=[],hum=[],task_dic={},ins_m_dic={},Dis=[],f_num=1,f_up=1,p_floor=2,w_num=32,To=To,T=0,Drone_num=0,Dic_M_W=dic_M_W):
        self.State=-1*(np.ones([all_num,4]))
        Table={}
        self.all_num=all_num
        Table['fire']=set()
        Table['smoke']=set()
        Table['human']=set()
        Table['open']=set()
        self.hum=hum
        self.Table=Table 
        self.report=report 
        self.Ma_set=[]   # monitoring area with tasks 
        self.lay_num=lay_num # not sure if we keep this 
        self.Task={} # for different tasks 
        self.task_dic=task_dic
        self.cu_wp=0
        self.cu_time=0
        self.start_time=0
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
        self.MaxTabuSize = 100000
        self.NeigSize = 500#500
        self.stopTurn = 20 #500
        self.Drone_num=Drone_num
        self.M_dis=M_dis
        self.Dic_M_W=Dic_M_W
        self.M_set_split=[]
        self.T_max=1500
    # def updat_State(self, m_id, state,time):  
    #  # according to the received data. 
    # def updat_Table (self,type,add_list,time):
 
    def get_neigh(self, ii,la_num,n):
        ne=list(range(-n,n+1))  # left and right 
        i=ii%la_num  # at which layer? 
        ni=ii//la_num   # the offset of layer 
        #print(ii,la_num,n)
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
    def increase_fire(self):
        #random.seed(time.time())
        times=random.randint(1,2)
        layers=self.all_num/self.lay_num
        neigh=[]
        for i in self.report:
            if random.random()>0.6: # left to right
                neigh=neigh+self.get_neigh(i, self.lay_num, times)
            if random.random()>0.6:
                for t in range(times):
                    if i//self.lay_num<(layers-1):
                        neigh=neigh+[i+t*self.lay_num]
                    if i//self.lay_num>0:
                        neigh=neigh+[i-t*self.lay_num]
        print(neigh)
        return neigh
    def Gen_Task(self): # Define all tasks according to 'plicy', report and collected data. 
#         if random.random()>0.6:
#             a=self.increase_fire() # if increase
#             print(f"we increase fire size from to {a}")
#             self.report=self.report+a
        self. Task['df'].update(set(self.report))
        tmp=copy.deepcopy(self.Table['fire'])
        tmp.update(self.report)
        layers=self.all_num/self.lay_num
        for i in list(tmp):
            n=self.get_neigh(i,self.lay_num,self.f_num) #for fire left2, right2, upper3 
            if i//self.lay_num<(layers-1):
                n=n+self.get_neigh(i+self.lay_num,lay_num,lay_num)
            self. Task['df'].update(n)
            ###########################. for somke the whole upper 1 layer and this layer 
            n=self.get_neigh(i,self.lay_num,self.lay_num)
            if i//self.lay_num<(layers-1):
                #print("see here",i//self.lay_num)
                n=n+self.get_neigh(i+self.lay_num,self.lay_num,self.lay_num)
            if i//self.lay_num>0:
                n=n+self.get_neigh(i-self.lay_num,self.lay_num,self.lay_num)
            #self. Task['ds'].update(n)
            ############################ for human, The whole layer, and 2 upper layer 
            n=[]
            for j in range(self.p_floor):  
                if i//self.lay_num<(layers-1-j):
                    n=n+self.get_neigh(i+j*self.lay_num,self.lay_num,self.lay_num)
            if i//self.lay_num>0:
                n=n+self.get_neigh(i-self.lay_num,self.lay_num,self.lay_num)
            #print(f"human detect {n}")
            self. Task['dh'].update(n)
            hum_set=[i for i in n if i in self.hum]
            self.Task['mh'].update(hum_set)
            ##############################for open window. the whole layer.
            n=self.get_neigh(i,self.lay_num,self.lay_num)
            self. Task['dw'].update(n)
            ############################ for tracking fire left, right and upper 1, lower 1
            n=[]
            n=n+self.get_neigh(i,self.lay_num,2)
            if i//self.lay_num<(layers-1-j): n=n+[i+self.lay_num]
            if i//self.lay_num>0: n=n+[i-self.lay_num]
           # self.Task['mf'].update(n)  
            self.Task['mf'].update(n)  
            #self.Task['mw'].update(n)
        self.Task['mf'].update(list(self.Table['fire']))
       # self.Task['ms'].update(list(self.Table['smoke']))
        self.Task['mh'].update(list(self.Table['human']))
#         self.Task['mf'].update(list(self.Table['fire']))
#         self.Task['ms'].update(list(self.Table['smoke']))
#         self.Task['mh'].update(list(self.Table['human']))
    def First_task(self,par_case): # this one need to implemented! according to report, task and table. 
        self.Gen_Task()
        print(self.Task)
        self.Gen_Ma()
        t=time.time() 
        max_t,sum_t,std_t,to_set=self.M_partition(par_case)
        tt=time.time()-t
        
        
        return tt,max_t,sum_t,std_t,to_set
    def Coordiniation(self,T,T_total):
        Drones=[]
        Wap_set=list(itertools.chain(*self.Wap_ty_set))
        Wap_set.append(Wap(0,[],0,[]))
        Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
        for i in range(self.Drone_num):
            M_set=dict(zip(self.M_set_split[i],[self.Ma_set.get(i) for i in self.M_set_split[i]]))
            #print(f" to check M_set {M_set}")
            drone=Drone(i,self.Wap_ty_set,M_set,self.Dis,self.task_dic,self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+T,T_total)
            Drones.append(drone)
           # drone.Get_WPC()
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
                    self.Ma_set[j].start=self.Ma_set[j].start+[self.start_time]
                    self.Ma_set[j].end=self.Ma_set[j].end+[self.T]
    def draw (self,M_set_split,Center=[],num=[],mat_f=None):
        output=open(mat_f,'w+')
        co=['b','g','orange','black','y']
        fig=plt.figure()
     #   ax=fig.add_subplot(111,projection='3d')
#             ax.scatter([M_loc[i][0] for i in range(len(M_loc))],[M_loc[i][1] for i in range(len(M_loc))],
#                        [M_loc[i][2] for i in range(len(M_loc))])
        output.write(str(len(M_set_split))+'\n')
        for i in range(len(M_set_split)):
            output.write(str(len(M_set_split[i]))+' ')
            print(str(len(M_set_split[i])))
        output.write('\n')
        for i in range(len(M_set_split)):
            for j in M_set_split[i]:
                output.write(f"{dic_M_W.get(j)[0]} {dic_M_W.get(j)[1]} {dic_M_W.get(j)[2]}\n")
          #  ax.scatter([dic_M_W.get(i)[0] for i in M_set_split[i]],[dic_M_W.get(i)[1] for i in M_set_split[i]],[dic_M_W.get(i)[2] for i in M_set_split[i]],color=co[i])
       # if Center!=[]:
           # ax.scatter([dic_M_W.get(i)[0] for i in Center],[dic_M_W.get(i)[1] for i in Center],[dic_M_W.get(i)[2] for i in Center],color='black',s=50)
        if num!=[]:
            seq=0; color=['red','black']
            for i in range(len(num)):
                output.write(str(len(num[i]))+' ')
            output.write('\n') 
            for o in num:
                for j in o:
                    output.write(f"{dic_M_W.get(j)[0]} {dic_M_W.get(j)[1]} {dic_M_W.get(j)[2]}\n")
              #  ax.scatter([dic_M_W.get(i)[0] for i in o],[dic_M_W.get(i)[1] for i in o],[dic_M_W.get(i)[2] for i in o],color=color[seq],marker='^',s=50)
                seq=seq+1
        output.close()
#plt.savefig(f"./result/partition_{self.Drone_num}_{self.T}.eps",bbox_inches = 'tight')
       # plt.show()               
    def M_partition(self,par_case): 
        dic_M_W=self.Dic_M_W
        M_dis=self.M_dis
        M_set_split=[[] for i in range(self.Drone_num)]
        m_w=dict(zip(list(range(self.Drone_num)),[0]*self.Drone_num))
        T_M=set(self.IM_set)
        if self.Drone_num==1:
            self.M_set_split=[list(self.IM_set)]
        else:
            #M_loc=[dic_M_W.get(i) for i in T_M]
            S_M=[]
            #local dictionary for M partition. 
            M_fre={}; M_fre2={}
            for i, j in self.Ma_set.items():
                #M_fre[i]=max([(600*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))])    
            #    M_fre[i]=max([(self.T*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))])    
            #initialization.
                max_si=max([self.task_dic.get(j.task[i])[0] for i in range(len(j.task))]) 
                si_tasks=[ j.task[i] for i in range(len(j.task)) if self.task_dic.get(j.task[i])[0]==max_si ]
                max_fre=max([(self.T_max*(self.task_dic.get(si_t)[1]) if self.task_dic.get(si_t)[1]!=0 else 1 ) for si_t in si_tasks])  
                mfre=max_si*max_fre # there for calculate workload
                if par_case==0:
                    #M_fre[i]=max([(self.T_max*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))])    
#                     max_si=max([self.task_dic.get(j.task[i])[0] for i in range(len(j.task))]) 
#                     si_tasks=[ j.task[i] for i in range(len(j.task)) if self.task_dic.get(j.task[i])[0]==max_si ]
#                     max_fre=max([(self.T_max*(self.task_dic.get(si_t)[1]) if self.task_dic.get(si_t)[1]!=0 else 1 ) for si_t in si_tasks])  
                    M_fre[i]=mfre# there for  
                elif par_case==1:
                    M_fre[i]=1 #max([(self.T_max*(self.task_dic.get(j.task[i])[1]) if self.task_dic.get(j.task[i])[1]!=0 else 1 ) for i in range(len(j.task))]) 
                    M_fre2[i]=mfre
                elif par_case==2:
                    M_fre[i]=1
                    M_fre2[i]=mfre
            M_s=sorted(M_fre.items(), key=lambda k:-k[1])
            #Av_F=(sum(list(M_fre.values()))/self.Drone_num)*1.1
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
            #sort the residual M:
            #print(f"frequency {M_fre.items()}")
            #Av_center=copy.deepcopy(Center)
            groups=Groups(T_M,Center,M_dis,M_fre,dic_M_W,To,method)
            groups.Get_initial()
            #self.draw(groups.M_set_split, groups.Center,[self.report,list(self.Task['mh'])],f"../drone_matlab/first_output_{self.Drone_num}_{self.T}.txt")
            partition=Partitioner(groups,M_dis,M_fre,len(Center),dic_M_W,To,threshold,800,method)
            #print(f"groups ",groups.M_set_split) 
            if par_case!=2:
                groups,max_t,sum_t,std_t,to_set=partition.do_improve()
            else:
                max_t,sum_t,std_t,to_set=self.get_t_to(groups,M_dis,M_fre2,len(Center),self.To)
            #self.draw(best_group.M_set_split, best_group.Center,[self.report,list(self.Task['mh'])],f"../drone_matlab/partition_output_{self.Drone_num}_{self.T}.txt")
            #print(f"finish partition {best_group.M_set_split}")
            self.M_set_split=groups.M_set_split
            return max_t,sum_t,std_t,to_set
            
    def get_t_to(self,groups,M_dis,M_fre,Drone_num,To):
        #print(f"get into get_t_to")
        M_set_split=groups.M_set_split
        to_t_set=[]
        for i in range(Drone_num):
            id=M_set_split[i]
            if len(id)==0:
                to_t_set.append(0)
            else:
                D=np.zeros((len(M_set_split[i]),len(M_set_split[i])))
                for a in range(len(id)):
                    for b in range(len(id)):
                        if a!=b:
                            D[a][b]=M_fre[id[b]]*M_dis[id[a]][id[b]]
                    dis=sorted([(sum(D[i][:]),i) for i in range(len(D))])
                to_t_set.append(dis[0][0]+sum([M_fre[id[k]]*To for k in range(len(id))]))
                    #ind=dis[0][1]
        #print(max(to_t_set),sum(to_t_set),to_t_set,np.std(to_t_set))
        return max(to_t_set),sum(to_t_set),np.std(to_t_set),to_t_set
        
    def Get_Interval(self,Tl_list,ins,jns,log_file):
        Track_visit=defaultdict(dict);Track_interval=defaultdict(dict)
        
        #TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
        ma_set=copy.deepcopy(self.Ma_set)
        sort_fre=defaultdict(list)
        times=defaultdict(dict)
        for i in ma_set:
            tasks=ma_set.get(i).task
            m=max([self.task_dic.get(i)[1] for i in tasks])
            #tmp=tasks[[self.task_dic.get(i)[1] for i in tasks].index(m)]
            tmp=[i for i in tasks if self.task_dic.get(i)[1]==m]
            #print(f" seeee {tmp}")
            for nn in tmp:
                sort_fre[nn].append(i)
            times[i][0]=0
            times[i][1]=0
        #print(f"times {times}")
        timeline_list=defaultdict(list)
        for i,s in sort_fre.items():
            for j in s:
                for Tl in Tl_list:
                    #print(f"tl",Tl)
                    if Tl.get(j)!=None:
                        timeline_list[j]=timeline_list[j]+[t[0] for t in Tl.get(j)]
                        times[j][0]=times[j][0]+len([k for k in Tl.get(j) if k[1]==15])
                        times[j][1]=times[j][1]+len([k for k in Tl.get(j) if k[1]==5])
                t=ma_set.get(j).start; interval=[]
                timeline=sorted(list(set(timeline_list[j])))
                #print(f"timeline",timeline)
                for k in range(len(timeline)):
                    interval.append(timeline[k]-t)
                    t=timeline[k]
                Track_visit[i][j]=times[j]
                print(f"interval",i,j,interval)
                Track_interval[i][j]=interval
        track_0=defaultdict(list);track_1=defaultdict(list)
        for i, j in Track_visit.items():
            for k, m in j.items():
                #print(k,m.get(0))
                track_0[i].append(m.get(0))
                track_1[i].append(m.get(1))
            print((track_0[i]))
            print(track_1[i])
            log_file.write(f"{ins} {jns} {i} {0} {np.mean(track_0[i])} {np.min(track_0[i])}\n")
            log_file.write(f"{ins} {jns} {i} {1} {np.mean(track_1[i])} {np.min(track_1[i])}\n")
            #print(i, Track_visit.get(i))



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
    def Get_WPC(self):        #print(self.Task,self.Table)
        IM_set=[i.id for i in self.Ma_set.values()]
        #print(f" to see {self.id},{IM_set}")
        ################################################### repeat the method above!
        #tmp=[list(list(self.Task.values())[k])for k in range(len(self.Task.values()))]
        if self.IM_set!=IM_set:  # if no change, no need to change Wapc 
            self.IM_set=IM_set
            self.Cu_Wap_ty=[]
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
    def Do_Heuristic(self,kind,case,co_s):  
        self.Get_WPC()     
        if kind==-1:
            Wap_set=list(itertools.chain(*self.Cu_Wap_ty))
        else: 
            Wap_set=list(self.Cu_Wap_ty[kind])
        #print([w.id for w in Wap_set])
        Wap_set.append(Wap(0,[],0,[]))
        Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
        if case==3:
            t=time.time()
            P,cu_t=Greedy_TSP(kind,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,min(self.cu_time+self.T,self.T_total))
            tt=time.time()-t
        else:
            if case==11 or case==12:
                t=time.time()
                P,cu_t=Greedy_SMT(case,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,min(self.cu_time+self.T,self.T_total))
                tt=time.time()-t
            else:
            #P,cu_t=Greedy_WPS(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
                t=time.time()
                P,cu_t=Greedy_Min(case,co_s,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,min(self.cu_time+self.T,self.T_total),self.T_total,self.Cu_Wap_ty)
                tt=time.time()-t
#         if case==5: 
#             P,cu_t=Tabu(P,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.NeigSize,self.MaxTabuSize,self.stopTurn,
#stop_condition=120,time_condition=True)
        #N_WP=[Wap_dic.get(i) for i in P]
       # print(f"{kind},{case},Generate Waypoints for next duration",cu_t,"s. : ",P, "with types: ",[N_WP[i].ty for i in range(len(N_WP))])
        #print("with coverage:", [N_WP[i].cover for i in range(len(N_WP))])
#         self.last_run_time=self.last_run_time+(self.T)
        TL=Get_TL([Wap_dic.get(i) for i in P],self.Dis,self.cu_time)
#         re,min_re,event_size,high_sum,high_size,min_high,sum_w_v,w_min_v,min_dict=Calculate_Acc(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic,high_task)
#         Sample_Acc(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic,sample_rate=60)
#         print(f"{kind},{case},End with reliability value",re,"min_re:",min_re,"at",cu_t)
        #return re,min_re,event_size,TL,high_sum,high_size,min_high,sum_w_v,w_min_v,min_dict,tt
        return P,tt,TL
        
#T=[300,600,900,1200,1500]    ################################################# for test
T=[120,300,480,600,900,1080,1200,1500]    ################################################# for test
ii=0
# log_file_name=f"result/log_interval_{Drone_Num}.txt"
# log_interval=open(log_file_name,'a+')
while ii<20:   ####################### for test
    #print(f"here is {i} \n")
    rseed=ii
    output=open(Outputfile,"a+")
    random.seed(ii)
    a=random.sample(range(all_num),va)
    print(f"fire source {a}")
    p_hum=0.05
    p_e=[random.random() for i in range(all_num)]
    h_e=[(p_e[i],i)[1] for i in range(all_num) if p_e[i]<p_hum] 
    c=Controller(all_num,lay_num,Wap_ty,a,h_e,task_dic,ins_m_dic,Dis,Drone_num=Drone_Num)   
    pa_t,max_t,sum_t,std_t,to_set=c.First_task(par_case)
    for t_f in T: 
        t=time.time()
        Drones,Wap_dic=c.Coordiniation(t_f,1500)
        #print(f"how long of the coordination? {pa_t}")
        output.write(f"12random:{rseed}T:{t_f} {pa_t} {max_t} {sum_t} {std_t} {to_set} \n")
        for i in range(-1,0):
            if i ==-1:
                for j in range(13):
                    TL_list=[]; P_list=[]
                    min_dict=defaultdict(list)
                    Tl_com=defaultdict(list)
                    run_t=[]
                    for drone in Drones:
                        #print(f"why {i} {drone.id} {j} {t_f} ")
                        P,tt,TL=drone.Do_Heuristic(i,j,co_s)
                        run_t.append(tt)
                        #print(f" drone {drone.id} {sorted (TL)} ")
                        #re,min_re,miss,TL,high_sum,high_size,min_high,sum_w_v,w_min_v,min_set,tt=drone.Do_Heuristic(i,j,co_s)
                        #print(f" to see runtime  {tt}")
                        P_list.append(P)
                        for n,k in TL.items():
                            Tl_com[n]=sorted(Tl_com[n]+TL[n])
#                     for i, j in Tl_com.items():
#                         print(i,j)
                    run_time=max(run_t)
                    print(f"to see" ,run_time,t_f, i,j,run_t)
                    #re,min_re,event_size,high_sum,high_size,min_high,sum_w_v,w_min_v,min_dict=Calculate_Acc(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic,high_task)
                    re,min_re,miss,high_sum,high_size,min_high,sum_w_v,w_min_v,w_task_min,weighted_min=Calculate_Acc2(c.Ma_set,Tl_com,c.cu_time,c.cu_time+t_f,c.task_dic,c.ins_m_dic,high_task)
                    s_av,s_h_av,s_m_h,s_m_ac,s_lm_d,s_hm_d,min_l_d,min_h_d,w_sample_min,w_detect_min,High_va,Va_set,sum_w_dect,sum_w_sample,min_detect,min_sample=Sample_Acc2(c.Ma_set,Tl_com,c.cu_time,c.cu_time+t_f,c.task_dic,c.ins_m_dic,high_task,sample_rate,detect_threshold)
                    output.write(f"{drone.id} {ii} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {w_min_v} {run_time} {s_av} {s_h_av} {s_m_h} {s_m_ac} {s_lm_d} {s_hm_d} {w_task_min} {min_l_d} {min_h_d} {w_sample_min} {w_detect_min} {weighted_min} {sum_w_dect} {sum_w_sample} {min_detect} {min_sample} \n")
                    #print(f"{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {w_min_v} {run_time} {s_av} {s_h_av} {s_m_h} {s_m_ac} {s_lm_d} {s_hm_d} {w_task_min} {min_l_d} {min_h_d} {w_sample_min} {w_detect_min} \n")
                    #print(f"{drone.id} {ii} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {w_min_v} {run_time} {s_av} {s_h_av} {s_m_h} {s_m_ac} {s_lm_d} {s_hm_d} {w_task_min} {min_l_d} {min_h_d} {w_sample_min} {w_detect_min} {weighted_min} {sum_w_dect} {sum_w_sample} {min_detect} {min_sample} \n")
                    #output.write(f"high {High_va}\n")
                    #output.write(f"low {Va_set}\n")
                    #print(f"check {re}")
                    
#                     print(f"see combined TL", (Tl_com))
#                     print(re,min_re,miss,high_sum,high_size,min_high,sum_w_v,w_min_v,min_set)
                    
                    
                    '''
                    for q in min_set:
                        min_dict[q[0]].append(q[1])
                    print("finish first one with",i,drone.id,tt)
                    if drone.id==Drone_Num-1:
                        min_w_new=0
                        print(f"why",min_dict)
                        for key, l in min_dict.items():
                            min_w_new=min_w_new+int(key)*min(l)
                        output.write(f"{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {min_w_new} {tt} {ii}\n")
                    else:
                        output.write(f"{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {w_min_v} {tt} {ii}\n")
                    '''      
                            
                            
                            
                        #TL_list.append(TL)
                        #output.write(f"{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {miss} {tt} {ii}\n")
                        #print(f" test tl",TL)
                    #print(TL_list)
                    #c.Get_Interval(TL_list,i,j,log_interval)
#                 else:
#                     for j in [3]:
#                         t=time.time()
#                         re,min_re,miss=drone.Do_Heuristic(i,j) # do no update task list. 
#                         tt=time.time()-t
#                         print("finish first one with",i,drone.id,time.time()-t)                   # output.write(f"{i} {j} {re} {min_re[0]} {min_re[1]} {miss} {tt}\n")
                        #output.write(f"{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {miss} {tt} {ii}\n")
    ii=ii+1   
    

    
#         print("finish first one with",i,"0",)
#         c.Do_Heuristic(i,1) # do no update task list. 
         
#         c.Do_Heuristic(i,2)
#         print("finish Tabu",i,"2",time.time()-t)
#     print("finish one time")
#     ii=ii+1
#print(random.sample(range(10),random.choice(range(10))))
# c=Controller(all_num,lay_num,Wap_ty,list(random.sample(range(10),random.choice(range(10)))),task_dic,ins_m_dic,Dis)
# #c=Controller(all_num,lay_num,Wap_ty,range(10),task_dic,ins_m_dic,Dis)
 
# c.Get_WPC()
# c.Gen_Ma()
# for i in range(-1,len(D)):
#     t=time.time()
#     c.Do_Heuristic(i) # do no update task list. 
#     print(time.time()-t)
# print("finish one time")
# ii=ii+1
# print([c.Ma_set.get(i).va for i in (list(c.Ma_set.keys()))])
# while c.cu_time<360:
#     c.Do_Heuristic() # do no update task list. 
#     print("finish one time")
#     print([c.Ma_set[i].va for i in range(len(c.Ma_set))])
#     #c.Col_Data()  # update event tables 
#     # c.Get_WPC()
#     # c.Gen_Ma()
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
# #################################################################################
# def Get_Neigh_Person(i,lay_num): # The whole layer, and 2 upper layer 
#     n=[]
#     for j in range(p_floor):
#         if i//la_num<(lay_num-1):
#             n=n+get_neigh(i+j*lay_num,lay_num,lay_num)
#     return (n)
# def Get_Neigh_Window(i,lay_num): # the whole layer. 
#     n=get_neigh(i,lay_num,lay_num)
#     return (n)
# def Get_Neigh_Smoke(i,lay_num): # the whole layer 
#     n=get_neigh(i,lay_num,lay_num)
#     n=n+get_neigh(i+lay_num,lay_num,lay_num)
#     return (n)
 
 
#############################
#     def Get_WPC(self):
#         #Wap_ty=[]; ;Cov_ty=[];Cov=[]
#         self.Gen_Task()                  # here  not touch 
#         #print(self.Task,self.Table)
#         print(self.Task)
#         tmp=[list(list(self.Task.values())[k])for k in range(len(self.Task.values()))]
#         if self.IM_set!=list(set(itertools.chain.from_iterable(tmp))):  # if no change, no need to change Wapc 
#             self.IM_set=list(set(itertools.chain.from_iterable(tmp)))
#             self.Cu_Wap_ty=[]
#             for i in range(len(self.Wap_ty_set)):
#                 ty=self.Wap_ty_set[i][0].ty 
#                 al_cov=[self.Wap_ty_set[i][k].cover for k in range(len(self.Wap_ty_set[i])) ]
#                 cov=[list(set(al_cov[k])&set(self.IM_set)) for k in  range((len(al_cov)))]
#                 #print("here",[set(al_cov[k]) for k in  range(len(al_cov))],cov,self.IM_set)
#                 #covv=[list(al_cov[k]) for k in  range((len(al_cov)))]
#                 w_id=[self.Wap_ty_set[i][k].id for k in range(len(self.Wap_ty_set[i])) ]
#                 cov_com=list([cov[i],w_id[i]]for i in range(len(cov)))
#                 cov_com.sort(key = lambda s:len(s[0]),reverse=True)
#                 cov_dic=dict(zip(w_id,cov)) # cover current monitoring area not
#                 co=copy.deepcopy(cov_com)
#                 #print(co)
#                 M=copy.deepcopy(self.IM_set)
#                 W=[];C=[]
#                 while len(M)>0: 
#                     co.sort(key = lambda s:len(s[0]),reverse=True)  # sort first 
#                     tmp=co[0][0]
#                     w=Wap(co[0][1],[],ty,cov_dic.get(co[0][1]))
#                     W.append(w) # do not use location of that. 
#                     M=list(set(M)-set(tmp))
#                     for ii in range (len(co)): 
#                         co[ii][0]=list(set(co[ii][0])-set(tmp))
#                 self.Cu_Wap_ty.append(W) 
                #print("using ",len(W)," waypoints to cover all Monitor area",len(self.IM_set))
 
 
 #########
 #     def Do_Heuristic(self,kind,case):
#         if kind==-1:
#             Wap_set=list(itertools.chain(*self.Cu_Wap_ty))
#         else: 
#             Wap_set=list(self.Cu_Wap_ty[kind])
#         Wap_set.append(Wap(0,[],0,[]))
#         Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
#         if case==3:
#             P,cu_t=Greedy_TSP(kind,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
#         else:
#             #P,cu_t=Greedy_WPS(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
#             P,cu_t=Greedy_Min(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
# #         if case==5: 
# #             P,cu_t=Tabu(P,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.NeigSize,self.MaxTabuSize,self.stopTurn,
# #stop_condition=120,time_condition=True)
# #         if case==3:
# #             P,cu_t=Greedy_Min(1,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
# #             print("start Tabu")
# #             P,cu_t=Tabu_min(P,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.NeigSize,self.MaxTabuSize,self.stopTurn,
# #                        stop_condition=120,time_condition=True )
#         N_WP=[Wap_dic.get(i) for i in P]
#        # print(f"{kind},{case},Generate Waypoints for next duration",cu_t,"s. : ",P, "with types: ",[N_WP[i].ty for i in range(len(N_WP))])
#         #print("with coverage:", [N_WP[i].cover for i in range(len(N_WP))])
#         self.last_run_time=self.last_run_time+(self.T)
#         TL=Get_TL([Wap_dic.get(i) for i in P],self.Dis,self.cu_time)
#         re,min_re,miss=Calculate_Acc(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic)
#         print(f"{kind},{case},End with reliability value",re,"min_re:",min_re,"at",cu_t)
#         return re,min_re,miss
 


