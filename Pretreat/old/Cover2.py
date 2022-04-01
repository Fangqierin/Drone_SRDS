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
from Heuristic_drone import Greedy_WPS,Tabu,Get_TL,Greedy_Min,Tabu_min,Calculate_Acc, Greedy_TSP
import sys
#import pickle 
import copy 
try:
    T_f=int(sys.argv[1]);  # flight time 
   # rseed=int(sys.argv[2]) # random seed 
except: 
    T_f=1500
    #rseed=0   
Outputfile=f"result/max_{T_f}.txt"
Drone_N=3
D=[5,15]    
#D=[5,16,24]
faces=7
global R_v # drone rate  
R_v=2
global To# loiter time 
To=5
st_nor=np.array([0,-1,0])
drone=pd.read_csv('../data/drone_char.csv',sep=' ')
building=pd.read_csv('../data/dbh_stu.csv',delimiter=' ')
Cov_file='../data2/wp_cover_'
d_file='../data/Travel_Distance.csv'
win_fname='../data/win_ro_f_'
#print("get wp")
Wap_set,Wap_ty=Write_WP(Drone_N,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname)
wps=[Wap_set[i].loc for i in range(len(Wap_set))]
#print([Wap_set[i].cover for i in range(len(Wap_set))])
#Figure(building, wps)
ty_set=[Wap_set[i].ty for i in range(len(Wap_set))]
# Wap_ty_set=[[]for i in range(len(D))]
# n=0
# for i in np.sort(list(set(ty_set))):
 #         if j.ty==i:
#             Wap_ty_set[n].append(j)
#     n=n+1     ############## Got a set of different type of waps. 
co=[Wap_set[i].cover for i in range(len(Wap_set))]
#print([Wap_ty[1][i].ty for i in range(len(Wap_ty[1]))])
in_loc=[0,-20,0]
#D=Get_VG_D(wps,building,R_v,1,d_file,in_loc)
T_iter=3 # the interval time of to sequential shots
Dis=Read_D(d_file,R_v,To,T_iter)
print(Dis)
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

ins_m_dic={'15':{'df':0.6,'ds':0.9,'dh':0.7,'dw':0.5,'mf':0.3,'mh':0.3,'ms':0.9},'10':{'df':0.7,'ds':0.8,'dh':0.9,'dw':0.8,'mf':0.7,'mh':0.5,'ms':0.7},
           '5':{'df':1,'ds':1,'dh':1,'dw':1,'mf':1,'mh':1,'ms':1},'0':{'df':0,'dh':0,'ds':0,'dw':0,'mf':0,'mh':0,'ms':1}} 

#####get the correlations among windows in one layer. 
win_layer=pd.read_csv('~/Desktop/Drone_code/data/layer_win.csv',sep=' ')
all_wins=pd.read_csv('~/Desktop/Drone_code/data/all_win.csv',sep=' ')
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
task_dic={'df':[1,0.005],'ds':[1,0.003],'dh':[1,0.005],'dw':[1,0.0005],'mf':[1,0.02],'ms':[1,0.005],'mh':[1,0.02]}
#task_dic={'df':[3,0],'ds':[1,0],'dh':[2,0],'dw':[1,0],'mf':[5,0],'ms':[5,0],'mh':[4,0]}

State=-1*(np.ones(4)) # 1) fire 2) smoke 3) human 4) open window 
report=[]   # is locations of risky area. 

class Controller: 
    def __init__(self,all_num,lay_num,Wap_ty_set=[],report=[],hum=[],task_dic={},ins_m_dic={},Dis=[],f_num=2,f_up=1,p_floor=3,w_num=32,To=To,T=T_f):
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
    def Gen_Task(self): # Define all tasks according to 'plicy', report and collected data. 
        self. Task['df'].update(set(self.report))
        tmp=copy.deepcopy(self.Table['fire'])
        tmp.update(self.report)
        layers=self.all_num/self.lay_num
        for i in list(tmp):
            n=self.get_neigh(i,self.lay_num,self.f_num) #for fire left2, right2, upper3 
            if i//self.lay_num<(layers-1):
                n=n+self.get_neigh(i+self.lay_num,lay_num,self.f_up)
            self. Task['df'].update(n)
            ###########################. for somke the whole upper 1 layer and this layer 
            n=self.get_neigh(i,self.lay_num,self.lay_num)
            if i//self.lay_num<(layers-1):
                #print("see here",i//self.lay_num)
                n=n+self.get_neigh(i+self.lay_num,self.lay_num,self.lay_num)
            if i//self.lay_num>0:
                n=n+self.get_neigh(i-self.lay_num,self.lay_num,self.lay_num)
            self. Task['ds'].update(n)
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
           # print(f"get track fire")
            n=[]
            n=n+self.get_neigh(i,self.lay_num,1)
            if i//self.lay_num<(layers-1-j): n=n+[i+self.lay_num]
            if i//self.lay_num>0: n=n+[i-self.lay_num]
            self.Task['mf'].update(n)  
        self.Task['mf'].update(list(self.Table['fire']))
        self.Task['ms'].update(list(self.Table['smoke']))
        self.Task['mh'].update(list(self.Table['human']))
    def Get_WPC(self):
        #Wap_ty=[]; ;Cov_ty=[];Cov=[]
        self.Gen_Task()
        #print(self.Task,self.Table)
        print(self.Task)
        tmp=[list(list(self.Task.values())[k])for k in range(len(self.Task.values()))]
        if self.IM_set!=list(set(itertools.chain.from_iterable(tmp))):  # if no change, no need to change Wapc 
            self.IM_set=list(set(itertools.chain.from_iterable(tmp)))
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
                #print(co)
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
                #print("using ",len(W)," waypoints to cover all Monitor area",len(self.IM_set))
    def Gen_Ma(self):
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
    def Do_Heuristic(self,kind,case):
        if kind==-1:
            Wap_set=list(itertools.chain(*self.Cu_Wap_ty))
        else: 
            Wap_set=list(self.Cu_Wap_ty[kind])
        #print("there are ",len(Wap_set),"waypoints")
        #here I want to add the inital value 
        Wap_set.append(Wap(0,[],0,[]))
        Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
        #print(Wap_set[0].loc,len(Wap_set),Wap_set[0].id,[Wap_set[i].cover for i in range (len(Wap_set))])
        #ini=int(time.time())
        #P,cu_t=Greedy_WPS(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
        #print("what P here  ",P)
        #if case!=3:
        # let us add a tsp methods. 
        if case==4:
            P,cu_t=Greedy_TSP(kind,self.Cu_Wap_ty,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
        else:
            P,cu_t=Greedy_WPS(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)

            #P,cu_t=Greedy_Min(case,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
#         if case==5: 
#             P,cu_t=Tabu(P,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.NeigSize,self.MaxTabuSize,self.stopTurn,
#stop_condition=120,time_condition=True)
        
#         if case==3:
#             P,cu_t=Greedy_Min(1,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T)
#             print("start Tabu")
#             P,cu_t=Tabu_min(P,Wap_set,self.Ma_set,self.Dis,self.task_dic, self.ins_m_dic,self.cu_wp,self.cu_time,self.cu_time+self.T,self.NeigSize,self.MaxTabuSize,self.stopTurn,
#                        stop_condition=120,time_condition=True )
        N_WP=[Wap_dic.get(i) for i in P]
       # print(f"{kind},{case},Generate Waypoints for next duration",cu_t,"s. : ",P, "with types: ",[N_WP[i].ty for i in range(len(N_WP))])
        #print("with coverage:", [N_WP[i].cover for i in range(len(N_WP))])
        self.last_run_time=self.last_run_time+(self.T)
        TL=Get_TL([Wap_dic.get(i) for i in P],self.Dis,self.cu_time)
        #print(TL)
        #print("Travel time: ", [self.Dis[P[i-1]][P[i]]for i in range(1,len(P))],sum([self.Dis[P[i-1]][P[i]]for i in range(1,len(P))]))
        #re,min_re,miss=Calculate_Re(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic)
        re,min_re,miss=Calculate_Acc(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic)
       
        #re2,miss2=Calculate_Re2(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic,self.To)
        print(f"{kind},{case},End with reliability value",re,"min_re:",min_re,"at",cu_t)
        return re,min_re,miss
        
        #self.Ma_set=Update_Ma(self.Ma_set,TL,self.cu_time,self.cu_time+self.T,self.task_dic,self.ins_m_dic,self.To)
        #self.cu_wp=P[-1]
        #self.cu_time=self.cu_time+self.T
         
         
         
# def Fire_source(all_num,k):
#     return list(np.random.randint(0, all_num, size=(1, k)))
# #print(Dis)
ii=0

while ii<10:
    rseed=ii
    output=open(Outputfile,"a+")
    random.seed(ii)
    va=1
    output.write(f"random:{rseed}T:{T_f}\n")
    #print(all_num)
    a=random.sample(range(all_num),va)
   # a=[318, 130, 379]
    print(f"fire source {a}")
    p_hum=0.03
    p_e=[random.random() for i in range(all_num)]
    h_e=[(p_e[i],i)[1] for i in range(all_num) if p_e[i]<p_hum] 
    c=Controller(all_num,lay_num,Wap_ty,a,h_e,task_dic,ins_m_dic,Dis)
    c.Get_WPC()
    c.Gen_Ma()
    ii=ii+1
    for i in range(-1,2):
        if i ==-1:
            for j in range(5):
                t=time.time()
                re,min_re,miss=c.Do_Heuristic(i,j) # do no update task list. 
                tt=time.time()-t
                print("finish first one with",i,"1",time.time()-t)
                output.write(f"{i} {j} {re} {min_re[0]} {min_re[1]} {miss} {tt}\n")
        else:
            for j in [3]:
                t=time.time()
                re,min_re,miss=c.Do_Heuristic(i,j) # do no update task list. 
                tt=time.time()-t
                print("finish first one with",i,"1",time.time()-t)
                output.write(f"{i} {j} {re} {min_re[0]} {min_re[1]} {miss} {tt}\n")

    
    
    
    
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
 
 
 
 
 
 
 


