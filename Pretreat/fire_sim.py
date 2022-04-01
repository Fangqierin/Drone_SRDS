'''
Created on Jul 31, 2020

@author: fangqiliu
'''
import numpy as np
import random as rm
import pandas as pd
import random
import copy
from collections import defaultdict
from scipy.integrate import quad
import time
LL=10000000
class room:
    def __init__(self,floor=1,id=0,j_fd=0,j_gr=0,w_open=False,d_open=False):
        self.w_open=w_open
        self.d_open=d_open
        self.floor=floor
        self.id=id
        self.u_gr=30
        self.v_gr=4
        self.u_fd=80
        self.v_fd=30 
        self.u_fo=9
        self.v_fo=1.27
        self.j_fd=j_fd
        self.j_gr=j_gr
        self.P_ig=0
        self.t_ig=LL
        self.P_fd=0 
        self.t_fo=LL
        self.P_h_bf={}
        self.P_v_bf=0
        self.P_leave=0
        self.trap=False
        self.leave=False
        self.t_exist=0
        self.open_window=False
        self.human_exist=False
        self.decay=False
        #self.if_flash=if_flash
        self.h_wall_neigh=[]
        self.h_cor_neigh=[]
        self.h_win_neigh=[]
        ####### vertical
        self.v_win_neigh=[]
        self.v_cel_neigh=[]
    def add_fire(self,if_flash):
        if if_flash==True:
            self.P_ig=1
            self.t_ig=0
            self.P_fd=1
            self.t_fo=0
        else:
            self.P_ig=1
            self.t_ig=0
            self.P_fd=0
            self.t_fo=LL
        
    def Get_adjacent(self,room_set,win_to_rooms,win_c_dict,lay_num):
        seq=room_set.index(self.id)
        if seq==0 or seq==len(room_set)-1:
            if seq==0:
                right=room_set[-1]
                left=room_set[seq+1]
            else:
                left=room_set[0]
                right=room_set[seq-1]
        else:
            left=room_set[seq+1]
            right=room_set[seq-1]
        wall_n=[(self.floor,left),(self.floor,right)]
        win_n=[]
        if win_c_dict.get(self.id)!=None:
            win_n=[(self.floor,i) for i in win_c_dict.get(self.id)]
            self.h_win_neigh=win_n
        self.h_wall_neigh=list(set(wall_n)-set(win_n)) 
        if win_c_dict.get(self.id)!=None:
            
            if self.floor!=floor_num-1:
                self.v_win_neigh=[(self.floor+1,self.id)]
        else:
            if self.floor!=floor_num-1:
                self.v_cel_neigh=[(self.floor+1,self.id)]
       # print(self.v_win_neigh,self.v_cel_neigh,self.h_wall_neigh,self.h_win_neigh)
class Simulator:
    def __init__(self,rooms,fire_source,human_exist,open_window,win_num,floor_num,slot):
        self.slot=slot
        self.rooms=rooms
        self.open_window=open_window
        for i in fire_source:
            print(i[0],i[1])
        self.fire_source=fire_source
        self.human_exist=human_exist
        self.fire_rooms={}
        self.has_fire=[]
        for i in fire_source:
            self.fire_rooms[i]=rooms[i[0]][i[1]]
            self.has_fire.append(i)
        self.af_area={}
        for i in fire_source:
            self.af_area[i]=rooms[i[0]][i[1]]  # already added fire
        self.human_rooms={}
        self.humans=[]
        for i in self.human_exist:
            self.human_rooms[i]=rooms[i[0]][i[1]]
            self.humans.append(i)
        self.break_windows=set(open_window)
        self.win_number=win_num
        self.floor_number=floor_num
#         print(self.break_windows)
    def normal(self,t,u,v,t_fo):
        return 1/(v * np.sqrt(2 * np.pi)) *np.exp( - ((t-t_fo) - u)**2 / (2 * v**2))
    def integrate(self,x_min,x_max,u,v,t_fo):
        result= quad(self.normal,x_min,x_max,args=(u,v,t_fo))
        return result[0]
    def simulate(self,t,T,output,sed):
        random.seed(sed)
        f=open(output,'w')
        f.write(f"t/ f/ h/ w/ f_s/ h_s\n")
        stop_flag=False
        win_w=[]
        fire_time={}
        human_time={}
        fire_state={}
        human_state={}
        #fire_state=[]
        #human_state=[]
        fire_u=5
        fire_v=2
        human_u=5
        human_v=2
        while t<T:
         #   print(t)
            
           # rooms2=copy.deepcopy(self.rooms)
            for kk,room in self.fire_rooms.items():  # check room which has flash over
                if room!=None:
                    if room.t_fo>t or t-room.t_fo>room.j_fd:
                        if t-room.t_fo>room.j_fd:
                            room.P_ig=0
                            room.t_ig=LL
                            self.decay=True
                           # print((room.floor, room.id) ,f"decay")
                        break
                    elif t-room.t_fo>0 and  t-room.t_fo<room.j_fd:
                        for i in room.h_win_neigh:
                            P=self.integrate(room.t_fo,t,u_wfr,v_wfr,room.t_fo)
                            if self.rooms[i[0]][i[1]].P_h_bf.get((room.floor,room.id))==None:
                                self.rooms[i[0]][i[1]].P_h_bf[(room.floor,room.id)]=P
                            elif self.rooms[i[0]][i[1]].P_h_bf.get((room.floor,room.id))<P:
                                self.rooms[i[0]][i[1]].P_h_bf[(room.floor,room.id)]=P
                            if self.af_area.get(i)==None:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            elif self.af_area.get(i).P_h_bf.get((room.floor,room.id))==None:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            else:
                                if self.af_area.get(i).P_h_bf.get((room.floor,room.id))<P:
                                    self.af_area[i]=self.rooms[i[0]][i[1]]
                        for i in room.h_wall_neigh:
#                             if t==3:
#                                print(f"h_wall",i, (room.floor,room.id))
                            P=self.integrate(room.t_fo,t,u_walfr,v_walfr,room.t_fo)
                            if self.rooms[i[0]][i[1]].P_h_bf.get((room.floor,room.id))==None:
                                self.rooms[i[0]][i[1]].P_h_bf[(room.floor,room.id)]=P
                            elif self.rooms[i[0]][i[1]].P_h_bf.get((room.floor,room.id))<P:
                                self.rooms[i[0]][i[1]].P_h_bf[(room.floor,room.id)]=P
                            if self.af_area.get(i)==None:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            elif self.af_area.get(i).P_h_bf.get((room.floor,room.id))==None:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            else:
                                if self.af_area.get(i).P_h_bf.get((room.floor,room.id))<P:
                                    self.af_area[i]=self.rooms[i[0]][i[1]]
                           # print(f"wall",P,i)
                        ########## vertical 
                        for i in room.v_cel_neigh:
                         #   if t==3:
                         #       print(f"v_cel",i, (room.floor,room.id))
                            P=self.integrate(room.t_fo,t,u_walfr,v_walfr,room.t_fo)
                            if self.rooms[i[0]][i[1]].P_v_bf<P:
                                self.rooms[i[0]][i[1]].P_v_bf=P
                            if self.af_area.get(i)==None:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            elif self.af_area.get(i).P_v_bf<P:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            #print(f"vertical cel",P)
                        for i in room.v_win_neigh:
                    #        if t==3:
                    #            print(f"v_win",i, (room.floor,room.id))
                            if (self.rooms[i[0]][i[1]].open_window==True and room.open_window==False) or (self.rooms[i[0]][i[1]].open_window==False and room.open_window==True):
                                P=self.integrate(room.t_fo,t,u_wfr,v_wfr,room.t_fo)
                            elif self.rooms[i[0]][i[1]].open_window==False and room.open_window==False:
                             #   print(f"it is normal")
                                P=self.integrate(room.t_fo,t,u_wfr*2,v_wfr*2,room.t_fo)
                            else:
                                P=self.integrate(room.t_fo,t,u_open,v_open,room.t_fo)
                            if self.rooms[i[0]][i[1]].P_v_bf<P:
                                self.rooms[i[0]][i[1]].P_v_bf=P
                            if self.af_area.get(i)==None:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            elif self.af_area.get(i).P_v_bf<P:
                                self.af_area[i]=self.rooms[i[0]][i[1]]
                            #print(f"vertical win",P,i,(room.floor,room.id)) 
            af_a=copy.deepcopy(self.af_area)  
            #######################################################################
            for i,room in self.af_area.items():
                if room.t_fo>t or t-room.t_fo>room.j_fd:  
                    if t-room.t_fo>room.j_fd:    # decayed! 
                        room.P_ig=0
                        room.t_ig=LL
                        room.decay=True
                        room.P_fd=0
#                         if 52<=t<=55:
#                             print(f"decay",(room.floor, room.id),room.t_fo)
                        try:
                            self.has_fire.remove((room.floor,room.id))
                        except:
                            pass
                        self.fire_rooms[i]=None
            for i,room in self.af_area.items():
                if t>room.t_fo  and t-room.t_fo<room.j_fd:
                    room.P_ig=1
#                     if 50<=t<=55:
#                         print(f"already ignited {t} {i}")
                   # af_a[i]=rooms
                elif room.P_ig!=1 and room.decay==False:# ################# vertical 
#                     if 50<=t<=55:
#                         print(f"not be ignited {t} {i}")
                    room.P_ig=0
                    if self.af_area.get((i[0]-1,i[1]))!=None:
                        if (t>self.af_area.get((i[0]-1,i[1])).t_fo and t-self.af_area.get((i[0]-1,i[1])).t_fo<self.af_area.get((i[0]-1,i[1])).j_fd):
                            room.P_ig=room.P_v_bf*(self.af_area.get((i[0]-1,i[1])).P_fd)
#                             if 50<=t<=55:
#                                 print(f"below can spread to {t} {i} {room.P_ig} {self.af_area.get((i[0]-1,i[1])).t_fo} {self.af_area.get((i[0]-1,i[1])).j_fd}  ")
#                     ################################ horizontal 
                    for j,r in room.P_h_bf.items(): #
                        if self.af_area.get((j[0],j[1]))!=None: 
    #                         tmp=self.af_area.get((j[0],j[1])).P_fd
    #                         room.P_ig=room.P_ig+r*tmp-room.P_ig*r*tmp
                            if ( t>self.af_area.get((j[0],j[1])).t_fo and t-self.af_area.get((j[0],j[1])).t_fo<self.af_area.get((j[0],j[1])).j_fd):
                                tmp=self.af_area.get((j[0],j[1])).P_fd
                                room.P_ig=room.P_ig+r*tmp-room.P_ig*r*tmp
#                                 if 50<=t<=55:
#                                     print(f"spread to adjacent {t} {i} {j} {room.P_ig} {self.af_area.get((j[0],j[1])).t_fo} {self.af_area.get((j[0],j[1])).j_fd} ")
#                        # else:
#                             if 50<=t<=55:
#                                 print(f"see something skip {j} {t} {self.af_area.get((j[0],j[1])).t_fo} {self.af_area.get((j[0],j[1])).j_fd} ")
#                                 print(f"so show Pig {room.P_ig}")
            ########################################################### Vertical + horizontal. 
#                 if 52<=t<=55: 
#                     print(f"rrr",i, room.P_ig)
                if room.t_ig>t and room.decay==False:
                    aa=random.random()
                    if aa <room.P_ig:
#                         if 52<=t<=55: 
                        #print(f"got ignited, {t} {aa} {room.P_ig} {i} ")
                        room.t_ig=t
                        room.P_ig=1
                      #  print((room.floor,room.id),f" ignite")
                        self.has_fire.append((room.floor,room.id))
#                     else:
#                         if 52<=t<=55:
#                             print(f" not ignite",(room.floor,room.id,room.P_ig))
                if room.t_ig<t:
                    if room.t_fo<t:
                        if t-room.t_ig>j_fo+room.j_fd:
                            P=0
                        else:
                            P=self.integrate(room.t_ig, t, room.u_fo, room.v_fo, room.t_ig)
                    else:
                        if t-room.t_ig>j_gr:
                            P=0
                        else:
                            P=self.integrate(room.t_ig, t, room.u_fo, room.v_fo, room.t_ig)
                    room.P_fd=P
                    #random.seed(time.time())
                    rr=random.random()
                    if rr<P:
                        if room.t_fo>t:
                            room.t_fo=t
                        self.fire_rooms[i]=room
                        if room_to_win.get(room.id)!=None:
                            for kk in room_to_win.get(room.id):
                                self.break_windows.update([(room.floor,kk)])
                af_a[i]=room
                self.rooms[i[0]][i[1]]=room
            self.af_area=af_a
            for i,room in self.human_rooms.items():
                if room!=None:
                    if room.trap==False:
                        if i[0] in [j[0] for j in self.has_fire]:
                            room.trap=True
                            self.human_rooms[i]=room
                         #   print(f"trap",i)
                        else:
                            P=self.integrate(room.t_exist, t, u_leave, v_leave, room.t_exist)
                            aa=random.random()
                            if aa<P:
                                room.P_leave=1
                                room.leave=True
                                self.human_rooms[i]=None
                                self.humans.remove(i)
            fire=set()
            for i in self.has_fire:
                #print(f"i",i)
                if room_to_win.get(i[1])!=None:
                    for kk in room_to_win.get(i[1]):
                       # print(f"kk",kk)
                        fire.update([(i[0],kk)])
#                         if 0<=t<=200:
#                             print(f"fire {t}",i)
            human=set()
            for i in self.humans:
                if room_to_win.get(i[1])!=None:
                    for kk in room_to_win.get(i[1]):
                        human.update([(i[0],kk)])
            fire_w=[list(fire)[i][0]*self.win_number+list(fire)[i][1] for i in range(len(fire))]
          #  print(fire, fire_w)
            hum_w=[list(human)[i][0]*self.win_number+list(human)[i][1] for i in range(len(human))]
            win_w_c=copy.deepcopy(win_w)
            win_w=[list(self.break_windows)[i][0]*self.win_number+list(self.break_windows)[i][1] for i in range(len(self.break_windows))]
            #print(f" {t} fire {fire_w[:]}\n")
            #print(f" {t} window {set(win_w[:])-set(win_w_c)}\n")
            
#             if 0<=t<=100:
#                  print(f"{t}/ {fire_w[:]}/\n human {hum_w[:]}/\n window {win_w[:]} \n")
            ######### 2021_4_12 add fire state 
            for i in fire_w:
                if fire_time.get(i)==None:
                    fire_time[i]=t
                    fire_state[i]=0
                p=self.integrate(fire_time.get(i),t,fire_u,fire_v,fire_time.get(i))
                #print(i, p)
                rr=random.random()
                if p>rr:
                    fire_time[i]=t
                    fire_state[i]= fire_state[i]+1   # fire current state
                    #print(f" {i} room change {t}")
            for i in hum_w:
                if human_time.get(i)==None:
                    human_time[i]=t
                    human_state[i]=0
                p=self.integrate(human_time.get(i),t,human_u,human_v,human_time.get(i))
                #print(i, p)
                rr=random.random()
                if p>rr:
                    human_time[i]=t
                    human_state[i]= human_state[i]+1   # current state
                  # print(f" {i} human change {t}")
            h_state=[]
            f_state=[]
            for i in fire_w:
                f_state.append(fire_state.get(i))
            for i in hum_w:
                h_state.append(human_state.get(i))
            f.write(f"{t}/ {fire_w[:]}/ {hum_w[:]}/ {win_w[:]}/ {f_state[:]}/ {h_state[:]}\n")
#             print(f"{t}/ \n fire {list(fire)}/ \n human {list(human)}/ \n window {list(self.break_windows)} \n")
#             print(f"{f_state}\n {h_state}")
            if stop_flag==False and len(fire_w)==0:
                T=min(t+20,T)
                stop_flag=True
            t=t+self.slot
            #print(f"see {t}")
        #print(fire_time,fire_state,human_state)
        f.close()

floor_num=12
win_layer=pd.read_csv('../data/layer_win.csv',sep=' ')   # 32 
all_wins=pd.read_csv('../data/all_win.csv',sep=' ')
room_win=pd.read_csv('../data/window_room.csv',sep=' ')
windo=[list(room_win['r'])[i].split(',') for i in range(len(list(room_win['r'])))]
win_to_rooms=dict(zip(list(room_win['w']),windo))
rooms_d=pd.read_csv('../data/rooms.csv',sep=',')
room_set=list(rooms_d['r'])
room_AF=dict(zip(list(rooms_d['r']),(zip(list(rooms_d['w']),list(rooms_d['d'])))))
room_to_win=defaultdict(list)
for i,j in win_to_rooms.items():
    for k in j:
        room_to_win[k].append(i)
#print(room_to_win)

all_num=len(room_AF)*floor_num
alph=0.0029 # 0.012 medium  0.047 high 
T=180
#list(win_to_rooms.keys())
#random.seed(time.time())
P_open=0.05
P_human=0.08
windows=[random.random()<P_open for i in range(len(list(win_to_rooms.keys()))*floor_num)]
tmp=[i for i in range(len(windows)) if windows[i]==True]
opens=[]
if len(tmp)!=0:
    for i in tmp:
        floor=i//len(win_layer)
        id=i%len(win_layer)
        opens.append((floor,id))
# print(f"opens",opens)
open_rooms=[]
for i,j in opens:
    for k in win_to_rooms.get(j):
        open_rooms.append((i,k))
w_f=24.8 #mean 
A_0H=(pow(508,2)*alph)/750
H_ch=12.4*1000
A_F=3*4
j_fd=(10.6*A_F*w_f/A_0H)/60
j_fo=pow(750*A_0H/alph,1/2)/60
j_gr=(pow(3*w_f*A_F*H_ch/alph,1/3))/60
#print(j_fo,j_fd,j_gr)
u_wfr=20# #https://www.glassonweb.com/article/introduction-fire-rated-glass 
v_wfr=3
u_open=2  # open window 
v_open=0.02
u_dfr=10  # mean value of door 
v_dfr=1.5
u_walfr=30 # mean value of gypsum wall  https://evstudio.com/calculating-fire-resistance-ratings-of-wood-assemblies-using-the-ibc/ 
v_walfr=4
u_fofr=60  # mean value of wood floor  Requirements, B. C., & Assemblies, F. T. (2018). Fire-Resistance-Rated Wood-Frame Wall and Floor / Ceiling Assemblies. (February).
v_fofr=10
u_leave=35 # mean value of human leave 
v_leave=8
output='./result/sim.csv'
def normal(t,u,v):
    return 1/(v * np.sqrt(2 * np.pi)) *np.exp( - (t - u)**2 / (2 * v**2))
def integrate(x_min,x_max,u,v):
    return quad(normal,x_min,x_max,args=(u,v))
win_c_dict=defaultdict(list)
for i, j in win_to_rooms.items():
    if len(j)>1:
        for k in j:
            win_c_dict[k]=win_c_dict[k]+([n for n in j if n!=k])
#print(win_c_dict)
#if __name__ == '__main__':

def Sim_fire(seed,a,outputfile,slot):
    #print(f"source {a} {seed} {slot}")
    random.seed(seed)
    windows=[random.random()<P_open for i in range(len(list(win_to_rooms.keys()))*floor_num)]
    #print(f"seessss", range(len(list(win_to_rooms.keys()))*floor_num))
    tmp=[i for i in range(len(windows)) if windows[i]==True]
    opens=[]
    if len(tmp)!=0:
        for i in tmp:
            floor=i//len(win_layer)
            id=i%len(win_layer)
            opens.append((floor,id))
            
            
            ############################ Change the floor number. 
    # print(f"opens",opens)
    open_rooms=[]
    for i,j in opens:
        for k in win_to_rooms.get(j):
            open_rooms.append((i,k))
    ##################################### open room
    rooms=[{}for i in range(floor_num)]
    room_num=len(room_AF)
    human_exist=[]
    for i in range(floor_num):
        for j in room_set:
            tmp=room(floor=i,id=j,j_fd=j_fd,j_gr=j_gr)
            tmp.Get_adjacent(room_set, win_to_rooms, win_c_dict,room_num)
            #random.seed(time.time())
            if (i,j) in open_rooms:
                tmp.open_window=True
            aa=random.random()
            if aa<P_human:
                human_exist.append((i,j))
                tmp.human_exist=True
            rooms[i][j]=tmp
    #print(f"human exist",human_exist
    fire_source=[]
    for i in a:
        floor=i//room_num
        id=i%room_num
        #print(f"fire source in Sim",i,(floor,room_set[id]))
        fire_source.append((floor,room_set[id]))
        rooms[floor][room_set[id]].add_fire(if_flash=True)
    #print(f" see roome ",room_set,room_num,fire_source)
    sim=Simulator(rooms,fire_source,human_exist,opens,len(win_layer),floor_num,slot)
    sim.simulate(0,40,outputfile,seed)

    
#Sim_fire(seed,a,outputfile,slot)
#Sim_fire(6,[293]," ",1)
    
#     a=[[ for i in range(2)]for j in range(2)]
#     print(a[0][1])
    # # max_v=len(win_layer)-1 
    # # for i in range(len(win_layer)):    # get the 
    # #     for j in range(i, len(win_layer)):
    # #         d=min(abs(j-i),max_v-j+i+1)
    # #         co[i][j]=d
    # #         co[j][i]=d 
   
    


# Sim=defaultdict(dict)
# Sim[0]['5']=[2]
# Sim[0]['r']=[3]
#print(Sim)

 #   print(pow(508,2)*alph/750)








