
import numpy as np
import matplotlib.pyplot as plt 
from matplotlib import cm
import pandas as pd
import math 
import random
import copy
import itertools
from scipy.linalg import expm, norm
import math
import random
from numpy import cross, eye, dot
import pyvisgraph as vg
from mpl_toolkits.mplot3d import Axes3D
import pickle
class M_a:
    def __init__(self,m_id=-1,task=[],ini_va=[],la_t=[],ac_re=[]):
        self.id = m_id 
        self.task=task
        self.va=ini_va
        self.la_t=la_t
        self.ac_re=ac_re
        self.start=[]
        self.end=[]
    def print_M(self):
        print(self.id)
        print(self.task)  
        print(self.va)     # here value is current value 
    def change(self,old=None ,new=None):
        print(self.task)
class Wap:
    def __init__(self,m_id=-1,loc=[],ty=-1,cover=[]):
        self.id=m_id
        self.loc=loc
        self.ty=ty
        self.cover=cover

def get_wp(dis,drone, win, n):  # get wp for one face with one distance. 
    Wap=[]
    h,v,hr,vr=GetFov(dis,drone,n)
    win_y=win.sort_values(by='v_2_y', ascending=False)
    h_c_dic=win.groupby(['v_2_y']).groups
    h_b_dic=win.groupby(['v_1_y']).groups 
    w_1_dic=win.groupby(['v_1_x']).groups
    w_2_dic=win.groupby(['v_3_x']).groups
    if np.sort(list(w_1_dic.keys()))[0]<np.sort(list(w_2_dic.keys()))[0]:
        w_l_dic=w_1_dic; w_r_dic=w_2_dic
    else:
        w_l_dic=w_2_dic; w_r_dic=w_1_dic
#travse from top to bottom 
    order=np.sort(list(h_c_dic.keys()))[::-1]
    covers=[]
    win_num=[[]for i in range(len(win))]    
    for i in range(len(order)):
        for j in list(h_c_dic.get(order[i])): 
            k=order[i]-v
            tmp=order[i]
            cover=[i for i in list(h_b_dic.keys())[::-1] if k<=i<tmp]
            num=1
            while len(cover)==0: 
                num=num+1
                k=k-v
                cover=[i for i in list(h_b_dic.keys())[::-1] if k<=i<tmp]
            win_num[j].append(num)
            cover_set=[]
            for z in range(len(cover)):
                cover_set=cover_set+list(h_b_dic.get(cover[z]))
            covers.append(cover_set)
    h_covers = list(covers for covers,_ in itertools.groupby(covers)) 
###travse from left to right 
    order2=np.sort(list(w_l_dic.keys()))
    covers=[]
    for i in range(len(order2)):
        for j in list(w_l_dic.get(order2[i])): 
            k=order2[i]+h
            tmp=order2[i]
            cover=[a for a in list(w_r_dic.keys()) if tmp<a<=k]
            num=1
            while len(cover)==0: 
                num=num+1
                k=k+h
                cover=[a for a in list(w_r_dic.keys()) if tmp<a<=k]
            win_num[j].append(num)
            cover_set=[]
            for z in range(len(cover)):
                cover_set=cover_set+list(w_r_dic.get(cover[z]))
            covers.append(cover_set)
    w_covers = list(covers for covers,_ in itertools.groupby(covers))
    com_i=[]
    for i in (h_covers):
        for j in (w_covers):
            com_i.append((list(set(i)&set(j))))
    com_i = list(com_i for com_i,_ in itertools.groupby(com_i))
    com_id=list(range(len(com_i)))
    com_dic=dict(zip(com_id,com_i))
    com_list=[[com_i[i],com_id[i]]for i in range(len(com_i))]
    com=copy.deepcopy(com_list)
    #heuristic method for waypoint selection for total coverage 
    #com.sort(key=len)
    com.sort(key = lambda com:len(com[0]),reverse=True)
    S=[];Co=[];C=[]
    while len(Co)<len(win): 
        tmp=com[0][0]
        Co=set(itertools.chain(Co,tmp))
        cc=com_dic.get(com[0][1])
        S.append(list(win.loc[cc]['id']))
        #print(tmp,cc,list(win.loc[tmp]['id']),list(win.loc[cc]['id']))
        com.remove([tmp,com[0][1]])
        for i in range (len(com)): 
            com[i][0]=list(set(com[i][0])-set(tmp))
        com.sort(key = lambda com:len(com[0]),reverse=True)
    #print("generate", len(S),"waypoints to cover all windows")
    win_count=[]
    for i in range(len(win_num)):
        win_count.append(win_num[i][0]*win_num[i][1])
    waypoint=[]
    axis=win.loc[win['id']==S[0][0]]['fix_aix'].values[0]
    dirt=win.loc[win['id']==S[0][0]]['dir'].values[0]
    fix=win.loc[win['id']==S[0][0]]['fix_v'].values[0]  # all windows in one facades are same 
    waypoints=[]
    for i in S: 
        y_max=max(win.loc[win['id'].isin(i)]['v_2_y'].values)
        y_min=min(win.loc[win['id'].isin(i)]['v_1_y'].values)   
        x_max=max(list(win.loc[win['id'].isin(i)]['v_3_x'].values)+list(win.loc[win['id'].isin(i)]['v_1_x'].values))
        x_min=min(list(win.loc[win['id'].isin(i)]['v_3_x'].values)+list(win.loc[win['id'].isin(i)]['v_1_x'].values))    
        if str(axis)=='y':
            v_y=fix+(dirt*dis)
            position=[round((x_max+x_min)/2,2),v_y,round((y_max+y_min)/2,2)]
        else:
            v_x=fix+(dirt*dis)
            position=[v_x, round((x_max+x_min)/2,2),round((y_max+y_min)/2,2)]
        waypoints.append(position)  
    return waypoints,win_count,S

def Get_D(wps,R_v):
    D=np.zeros((len(wps),len(wps)))
    for i in range(len(wps)):
        for j in range(i+1,len(wps)):
            squared_dist = np.sum((np.array(wps[i])-np.array(wps[j]))**2, axis=0)
            D[i][j]=math.ceil(np.sqrt(squared_dist)/R_v)
            #D[i][j]=round(np.sqrt(squared_dist)/R_v,2)
            D[j][i]=D[i][j]
    return D
def Get_WapCa(D,win,drone,d_id,nor,st_nor): # Get all waypoints for all distances
    waypoint_sets=[]
    type_set=[]
    cover_set=[]
    for i in D:
        wp,a,c=get_wp(i,drone,win,d_id)
        if str(np.cross(nor,st_nor))!='[0 0 0]':
            for p in range(len(wp)):
                wp[p]=list(rot_back(nor,st_nor,np.array(wp[p]))) 
        waypoint_sets=waypoint_sets+wp
        type_set=type_set+list(i*np.ones(len(wp)))
        cover_set=cover_set+c
    return waypoint_sets,type_set,cover_set
def Get_ty_WapCa(D,win,drone, d_id,nor,st_nor): # Get all waypoints for all distances
    waypoint_sets=[]
    type_set=[]
    cover_set=[]
    for i in D:
        wp,a,c=get_wp(i,drone,win,d_id)
        if str(np.cross(nor,st_nor))!='[0 0 0]':
            for p in range(len(wp)):
                wp[p]=list(rot_back(nor,st_nor,np.array(wp[p]))) 
        waypoint_sets.append(wp)
        type_set.append(list(i*np.ones(len(wp))))
        cover_set.append(c)
    return waypoint_sets,type_set,cover_set
def Get_TL(N_WP,D,To,cu_time):
    TL={}
    w_id=[N_WP[i].id for i in range(len(N_WP))]
    typ=[N_WP[i].ty for i in range(len(N_WP))]
    cov=[N_WP[i].cover for i in range(len(N_WP))]
    Time=cu_time
    for i in range(1,len(N_WP)):
        if i==1:       
                Time=Time+D[w_id[i-1]][w_id[i]]
        else:
                Time=Time+D[w_id[i-1]][w_id[i]]+To   # Here is okay, the first one no wait. 
        #print("time is",Time)
        for j in cov[i]:   
            if TL.get(j)==None: 
                TL[j]=[]
            TL[j].append([Time,typ[i]])
    return TL

# moved to Heuristice_drone.
def Calculate_Re(M_set,Tl,Ti,Time,task_dic,ins_m_dic,To):
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0
    Sum_miss=0
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                ma_set[k].la_t=TL[k][-1][0]*np.ones(len(ma_set[k].la_t))
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
            else:
                TL[k]=[[Time,0]]
        else:
            TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        la_t=M_set[k].la_t  # last visit time of all tasks 
        sum_re=0
        sum_miss=0
        for j in range(len(task_set)):  # for all tasks of a m 
            [si,dy]=task_dic.get(task_set[j])    
            b=ini_va[j]  
            for i in range(len(TL[k])):
                if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                    dur=TL[k][i][0]-Ti    
                else:
                    dur=(TL[k][i][0]-TL[k][i-1][0])
                if b-(dur*dy)<0:
                    sum_re=sum_re+((b/dy)*b*0.5)
                    b=0
                    if i!=0:
                        sum_miss=sum_miss+(b/dy-dur)
                else:
                    sum_re=sum_re+(b-dur*dy*0.5)*dur
                    b=b-dur*dy
                acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                b=max(b,acc)
            ma_set[k].va[j]=b
            #print(sum_re/Time)               
        Sum=Sum+(sum_re/len(task_set))
        Sum_miss=Sum_miss+(sum_miss/len(task_set))
    Sum=round(Sum/(len(M_set.keys())),2)
    Sum_miss=round(Sum_miss/len(M_set.keys()),2)
    # print([ma_set[i].la_t for i in range(len(ma_set))])
    # print([ma_set[i].va for i in range(len(ma_set))])
    return Sum,Sum_miss
def Calculate_inflos(M_set,Tl,Ti,Time,task_dic,ins_m_dic,To):
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                ma_set[k].la_t=TL[k][-1][0]*np.ones(len(ma_set[k].la_t))
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
            else:
                TL[k]=[[Time,0]]
        else:
            TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        la_t=M_set[k].la_t  # last visit time of all tasks 
        #print("here I want to track value", [ma_set[k].va[n] for n in range(len(task_set))])
        for j in range(len(task_set)):  # for all tasks of a m 
            [si,dy]=task_dic.get(task_set[j])
            sum_voi=0
            b=ini_va[j]  
            for i in range(len(TL[k])):
                if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                    dur=TL[k][i][0]-Ti
                else:
                    #off=min(TL[k][i][0]-TL[k][i-1][0],To)
                    dur=(TL[k][i][0]-TL[k][i-1][0])
                sum_voi=sum_voi+(b+dur*si*dy*0.5)*dur # calcaute loitor duration at the next time. 
                acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                b=(b+dur*si*dy)*(1-acc)
                #print("here I want to track value", k,b,acc,dur,sum_voi)
            ma_set[k].va[j]=b
            
        Sum=Sum+sum_voi
    Sum=round(Sum,2)
    # print([ma_set[i].la_t for i in range(len(ma_set))])
    # print([ma_set[i].va for i in range(len(ma_set))])
    return Sum 



def Calculate_inflos_old(M_set,Tl,Ti,Time,task_dic,ins_m_dic,To):
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                ma_set[k].la_t=TL[k][-1][0]*np.ones(len(ma_set[k].la_t))
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
        else:
            TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        la_t=M_set[k].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):  # for all tasks of a m 
            [si,dy]=task_dic.get(task_set[j])
            sum_voi=0
            b=ini_va[j]  
            for i in range(len(TL[k])):
                if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                    if Ti-la_t[i]>To:
                        off=0  
                    else:
                        off=min(To-Ti+la_t[i],TL[k][i][0]-Ti)
                    dur=TL[k][i][0]-Ti-off 
                else:
                    off=min(TL[k][i][0]-TL[k][i-1][0],To)
                    dur=(TL[k][i][0]-TL[k][i-1][0])-off
                sum_voi=sum_voi+(b+dur*si*dy*0.5)*dur+off*b  # calcaute loitor duration at the next time. 
                acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                b=(b+dur*si*dy)*(1-acc)
            ma_set[k].va[j]=round(b,2)
        Sum=Sum+sum_voi
    Sum=round(Sum,2)
    # print([ma_set[i].la_t for i in range(len(ma_set))])
    # print([ma_set[i].va for i in range(len(ma_set))])
    return Sum 
def Update_Ma(M_set,Tl,Ti,Time,task_dic,ins_m_dic,To):
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    # print([ma_set[i].la_t for i in range(len(ma_set))])
    # print([ma_set[i].va for i in range(len(ma_set))])
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                ma_set[k].la_t=TL[k][-1][0]*np.ones(len(ma_set[k].la_t))
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
        else:
            TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        la_t=M_set[k].la_t  # last visit time of all tasks 
        
        for j in range(len(task_set)):  # for all tasks of a m 
            [si,dy]=task_dic.get(task_set[j])
            #sum_voi=0
            b=ini_va[j]  
            for i in range(len(TL[k])):
                if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                    if Ti-la_t[i]>To:
                        off=0  
                    else:
                        off=min(To-Ti+la_t[i],TL[k][i][0]-Ti)
                    dur=TL[k][i][0]-Ti-off 
                else:
                    off=min(TL[k][i][0]-TL[k][i-1][0],To)
                    dur=(TL[k][i][0]-TL[k][i-1][0])-off
                #sum_voi=sum_voi+(b+dur*si*dy*0.5)*dur+off*b  # calcaute loitor duration at the next time. 
                acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                b=(b+dur*si*dy)*(1-acc)
            ma_set[k].va[j]=round(b,2)
    return (ma_set)

def GetFov(d,drone,No):
    if (No>=len(drone.values)):
        print("There is no ",No)
    else:
        data=drone.loc[No,['ss_w','ss_h','fl','pixel_w','pixel_h']].values
        h=data[0];v=data[1];fl=data[2];ph=data[3];pv=data[4]
        #print(data)
        hfov=round(d*h/fl,2)
        hr=round(ph/hfov,2)
        haov=round(2*math.degrees(math.atan(h/(2*fl))),2)
        #print(haov)
        vfov=round(d*v/fl,2)
        vr=round(pv/vfov,2)
        vaov=round(2*math.degrees(math.atan(v/(2*fl))),2)
        #print(vaov)
        #print("fov with width=" ,hfov, "; height=",vfov,"; HRS=",hr,"; VRS=",vr )
        return hfov,vfov,hr,vr
def M(axis, theta):
    return expm(cross(eye(3), axis/norm(axis)*theta))
def rot(a,b,v):  # a to b 
    Lx=np.sqrt(a.dot(a))
    Ly=np.sqrt(b.dot(b))
    theta = math.acos(np.dot(a,b)/(Lx*Ly))  # here theta always < 180, from a to b, a is the facade, b is our standarded axis
    axis=np.cross(a,b)  # here should be a x b. so a to b < 180, up, the same. 
    return np.round(np.dot(M(np.cross(a,b), theta), v),2) 
def rot_back(a,b,v):
    Lx=np.sqrt(a.dot(a))
    Ly=np.sqrt(b.dot(b))
    theta = math.acos(np.dot(a,b)/(Lx*Ly))  # here theta always < 180, from a to b, a is the facade, b is our standarded axis
    axis=np.cross(a,b)  # here should be a x b. so a to b < 180, up, the same. 
    return np.round(np.dot(M(np.cross(a,b), 2*np.pi-theta), v),2)
def Get_VG_D(wps,building,R_v,off,file,in_loc):
    f=open(file,'w')
    polygens=[]
    polygen=[]
    w=[];h=[];node=[]
    for index,row in building.iterrows():
        polygen=[]
        w.append(row['width'])
        h.append(row['hight'])
        node.append(row.loc[['x','y']].values)
    polygen=[]
    polygen.append(vg.Point(node[0][0]-off,node[0][1]+off))
    polygen.append(vg.Point(node[0][0]-off,node[0][1]+off))
    polygen.append(vg.Point(node[0][0]-off,node[0][1]-h[0]-off))
    polygen.append(vg.Point(node[1][0]-off,node[1][1]-off))
    polygen.append(vg.Point(node[1][0]-off,node[1][1]-h[1]-off))
    polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]-h[1]-off))
    polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]+off))
    polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]-h[0]+off))
    polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]+off))
    graph=vg.VisGraph()
    graph.build([polygen])
    # shortest = graph.shortest_path(vg.Point(100,6), vg.Point(-1,60))
    wps=[in_loc]+wps
    D=np.zeros((len(wps),len(wps)))
    for i in range(len(wps)):
        for j in range(i+1,len(wps)):
            #squared_dist = np.sum((np.array(wps[i])-np.array(wps[j]))**2, axis=0)
            shortest = graph.shortest_path(vg.Point(wps[i][0],wps[i][1]), vg.Point(wps[j][0],wps[j][1]))
            length=0
            for p in range(len(shortest)-1):
                tmp=np.sum((np.array([shortest[p].x,shortest[p].y])-np.array([shortest[p+1].x,shortest[p+1].y]))**2, axis=0)
                length=length+round(np.sqrt(tmp),2)
            d_z=abs(wps[i][2]-wps[j][2])
            d=np.sqrt((d_z**2)+(length**2))
            #D[i][j]=round(d/R_v,2)
            D[i][j]=round(d,2)
            D[j][i]=D[i][j]
            f.write(str(i)+' '+str(j)+' '+ str(D[i][j])+'\n')
            f.write(str(j)+' '+str(i)+' '+str(D[i][j])+'\n')
    f.close()
    return D








def Get_Win_D(off_win,building,R_v,off,file,in_loc):
    f=open(file,'w')
    off_win=3
    ang_dic={'90':np.array([0,1,0]),'180':np.array([-1,0,0]),'0':np.array([1,0,0]),'270':np.array([0,-1,0])}
    wins=pd.read_csv("/Users/fangqiliu/Drone.git/trunk/data/all_win.csv",sep=' ')
    win_dict={}
    for index,row in wins.iterrows():
        x_c=round(sum(list(row[['v_1_x','v_2_x','v_3_x','v_4_x']]))/4,2)
        y_c=round(sum(list(row[['v_1_y','v_2_y','v_3_y','v_4_y']]))/4,2)
        z_c=round(sum(list(row[['v_1_z','v_2_z','v_3_z','v_4_z']]))/4,2)
        vector=np.array([x_c,y_c,z_c])+ang_dic.get(str(row['ang']))*off_win
        win_dict[row['id']]=list(vector)   # here I want to change to dict !!!!! 
    #####create polygens
    polygens=[]
    polygen=[]
    w=[];h=[];node=[]
    for index,row in building.iterrows():
        polygen=[]
        w.append(row['width'])
        h.append(row['hight'])
        node.append(row.loc[['x','y']].values)
    polygen=[]
    polygen.append(vg.Point(node[0][0]-off,node[0][1]+off))
    polygen.append(vg.Point(node[0][0]-off,node[0][1]+off))
    polygen.append(vg.Point(node[0][0]-off,node[0][1]-h[0]-off))
    polygen.append(vg.Point(node[1][0]-off,node[1][1]-off))
    polygen.append(vg.Point(node[1][0]-off,node[1][1]-h[1]-off))
    polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]-h[1]-off))
    polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]+off))
    polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]-h[0]+off))
    polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]+off))
    graph=vg.VisGraph()
    graph.build([polygen])
    D=np.zeros((len(win_dict),len(win_dict)))
    for i in range(len(win_dict)):
        for j in range(i+1,len(win_dict)):
            seq=list(win_dict.keys())
            #squared_dist = np.sum((np.array(wps[i])-np.array(wps[j]))**2, axis=0)
            shortest = graph.shortest_path(vg.Point(win_dict.get(seq[i])[0],win_dict.get(seq[i])[1]), vg.Point(win_dict.get(seq[j])[0],win_dict.get(seq[j])[1]))
            length=0
            for p in range(len(shortest)-1):
                tmp=np.sum((np.array([shortest[p].x,shortest[p].y])-np.array([shortest[p+1].x,shortest[p+1].y]))**2, axis=0)
                length=length+round(np.sqrt(tmp),2)
            d_z=abs(win_dict.get(seq[i])[2]-win_dict.get(seq[j])[2])
            d=np.sqrt((d_z**2)+(length**2))
            #D[i][j]=round(d/R_v,2)
            D[i][j]=round(d,2)
            D[j][i]=D[i][j]
            f.write(str(i)+' '+str(j)+' '+ str(D[i][j])+'\n')
            f.write(str(j)+' '+str(i)+' '+str(D[i][j])+'\n')
    f.close()
    return D
    
    
    
    
    
    
    
    
    
    
    
    
    
def Figure(building, W_set,W_ty_set):
    mat_f=f"../drone_matlab/wpc"
    output=open(mat_f,'w+')
    for i in range(len(W_ty_set)):
        output.write(f"{i} {len(W_ty_set[i])}\n")
        for j in W_ty_set[i]:
            output.write(f"{j[0]} {j[1]} {j[2]}\n")
    '''
    polygen=[]
    
    w=[];h=[];node=[]
    for index,row in building.iterrows():
        polygen=[]
        w.append(row['width'])
        h.append(row['hight'])
        node.append(row.loc[['x','y']].values)
    print(f"see here {node},{w},{h}")
    polygen=[]
    polygen.append(vg.Point(node[0][0],node[0][1]))
    polygen.append(vg.Point(node[0][0],node[0][1]-h[0]))
    polygen.append(vg.Point(node[1][0],node[1][1]))
    polygen.append(vg.Point(node[1][0],node[1][1]-h[1]))
    polygen.append(vg.Point(node[1][0]+w[1],node[1][1]-h[1]))
    polygen.append(vg.Point(node[1][0]+w[1],node[1][1]))
    polygen.append(vg.Point(node[0][0]+w[0],node[0][1]-h[0]))
    polygen.append(vg.Point(node[0][0]+w[0],node[0][1]))
    graph=vg.VisGraph()
    graph.build([polygen])
    plt.plot([polygen[i].x for i in list(range(len(polygen)))+[0]],[polygen[i].y for i in list(range(len(polygen)))+[0]],color='black')
    seq=0
    col=['g','b']
    for ii in (W_ty_set): 
        plt.scatter([ii[i][0]for i in range(len(ii))],[ii[i][1] for i in range(len(ii))],color=col[seq])
        seq=seq+1
    co='blue'
    plt3d = plt.figure().gca(projection='3d')
    z=range(37)
    #1 face3 
    yy, zz = np.meshgrid(range(int(node[0][1]-h[0]),int(node[0][1]+1)), z); 
    xx=np.zeros([len(z),len(range(int(node[0][1]-h[0]),int(node[0][1]+1)))])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #2 face2 
    xx,zz=np.meshgrid(range(int(node[0][0]),int(node[0][0]+w[0]+1)), z);
    yy=np.ones([len(z),len(range(int(node[0][0]),int(node[0][0]+w[0]+1)))])*(node[0][1])
    # xx, zz = np.meshgrid(range(20), range(10));
    # yy=np.zeros([10,20])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #3 face1 
    yy, zz = np.meshgrid(range(int(node[0][1]-h[0]),int(node[0][1]+1)), z); 
    xx=np.ones([len(z),len(range(int(node[0][1]-h[0]),int(node[0][1]+1)))])*(node[0][0]+w[0])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #4 face7
    xx,zz=np.meshgrid(range(int(node[0][0]+w[0]),int(node[1][0]+w[1]+1)), z);
    yy=np.ones([len(z),len(range(int(node[0][0]+w[0]),int(node[1][0]+w[1]+1)))])*(node[0][1]-h[0])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    
    #5 face5
    xx,zz=np.meshgrid(range(int(node[1][0]),int(node[1][0]+w[1]+1)), z);
    yy=np.zeros([len(z),len(range(int(node[1][0]),int(node[1][0]+w[1]+1)))])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #6 face6
    yy, zz = np.meshgrid(range(0,int(node[1][1]+1)), z); 
    xx=np.ones([len(z),len(range(0,int(node[1][1]+1)))])*(node[1][0]+w[1])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #7 face4
    yy, zz = np.meshgrid(range(0,int(node[1][1]+1)), z); 
    xx=np.ones([len(z),len(range(0,int(node[1][1]+1)))])*(node[1][0])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #faceinter 
    xx,zz=np.meshgrid(range(int(node[0][0]),int(node[1][0]+1)), z);
    yy=np.ones([len(z),len(range(int(node[0][0]),int(node[1][0]+1)))])*(node[1][1])
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #upper face 1
    xx,yy=np.meshgrid(range(int(node[0][0]),int(node[0][0]+w[0]+1)),range(int(node[0][1]-h[0]),int(node[0][1]+1)));
    zz=np.ones([len(range(int(node[0][1]-h[0]),int(node[0][1]+1))),len(range(int(node[0][0]),int(node[0][0]+w[0]+1)))])*(36)
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #upper face 2
    xx,yy=np.meshgrid(range(int(node[1][0]),int(node[1][0]+w[1]+1)),range(int(node[1][1]-h[1]),int(node[1][1]+1)));
    zz=np.ones([len(range(int(node[1][1]-h[1]),int(node[1][1]+1))),len(range(int(node[1][0]),int(node[1][0]+w[1]+1)))])*(36)
    plt3d.plot_surface(xx, yy, zz, alpha=0.2,color=co)
    #plt3d.scatter([W_set[i][0]for i in range(len(W_set))],[W_set[i][1] for i in range(len(W_set))],[W_set[i][2]for i in range(len(W_set))])
    
    plt3d.scatter([W_ty_set[1][i][0]for i in range(len(W_ty_set[1]))],[W_ty_set[1][i][1] for i in range(len(W_ty_set[1]))],
        [W_ty_set[1][i][2]for i in range(len(W_ty_set[1]))],color='black')
   
    # plt.show()
    #plt.show()
    #pickle.dump(plt3d, open('FigureObject.fig.pickle', 'wb')) 
    # figx = pickle.load(open('FigureObject.fig.pickle', 'rb'))
    plt.show()
    '''
def Write_WP(Drone_N,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname):
    nor=[];num=[]
    for i in range(len(building)):
        tmp=building.loc[[i],['fac']].values
        num=num+tmp[0][0].split(',')
        tmp=building.loc[[i],['nom']].values
        tm=tmp[0][0].split(',')
        for j in range(len(tm)):
            nor.append([int(tm[j].split('/')[k]) for k in range(3)])
    nor_map=dict(zip(num,nor))
    Wap_set=[];Wap_ty_set=[[]for i in range(len(D))];W_set=[];W_ty=[[]for i in range(len(D))];Cov_ty=[[]for i in range(len(D))]
    for j in range(faces):   
        a=np.array(nor_map.get(str(j+1)))
        fname=win_fname+str(j+1)+'.csv'
        #print("for face: ",j+1)
        win=pd.read_csv(fname,sep=' ') 
        wp_ty,tp_ty,co_ty=Get_ty_WapCa(D,win,drone,Drone_N,a,st_nor)
        for i in range(len(D)):
            W_ty[i]=W_ty[i]+wp_ty[i]
            Cov_ty[i]=Cov_ty[i]+co_ty[i]
            W_set=W_set+wp_ty[i]
    seq=1
    for i in range(len(D)):   # for each distance, give the id. 
        for j in range(len(W_ty[i])):
            wapp=Wap(seq,W_ty[i][j],D[i],Cov_ty[i][j])
            Wap_ty_set[i].append(wapp)
            Wap_set.append(wapp) 
            seq=seq+1
    no=1
    for i in range(len(D)):
        f=open(Cov_file+str(D[i])+'.csv','w')
        for j in range(len(Cov_ty[i])):
            f.write(str(no)+' ')
            for k in range(len(Cov_ty[i][j])):
                f.write(str(Cov_ty[i][j][k])+',')
            no=no+1
            f.write('\n')    #here I modified it to from 1 !!!!!!!!!!!!
    return Wap_set,Wap_ty_set

def Read_D(d_file, R_v,To,T_iter):
    Dis=pd.read_csv(d_file,sep=' ')
    i=list(Dis['i'])
    j=list(Dis['j'])
    d=list(Dis['d'])
    print(max(i),max(j),max(d))
    D=np.zeros([max(i)+1,max(i)+1])
    for k in range(len(Dis)): 
        D[i[k]][j[k]]=round((d[k]/R_v)+To,2)   # flying time plus loiter time.
    for i in range(1,len(D)):
        D[i][i]=T_iter    # the interval time of two sequential shots
    return (D)
# here ! D [0][0]=0, D[i][i]=T_iterval. D[i][j]=travel_time+T_o



####### old 
def Calculate_inflos_oldd(M_set,Tl,Ti,Time,task_dic,ins_m_dic,To):# here the every time, the inital value is 0. We already give inital value
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0
    for k in range(len(ma_set)):  # for all monitoring areas 
        TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
        if len(TL[k])>0:
            ma_set[k].la_t=TL[k][-1][0]*np.ones(len(ma_set[k].la_t))
            if TL[k][-1][0]<Time:
                TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
        else:
            TL[k]=TL[k]+[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        la_t=ma_set[k].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):  # for all tasks of a m 
            [si,dy]=task_dic.get(task_set[j])
            sum_voi=0
            b=ini_va[j]  
            for i in range(len(TL[k])):
                if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                    if Ti-la_t[i]>To:
                        off=0  
                    else:
                        off=min(To-Ti+la_t[i],TL[k][i][0]-Ti)
                    dur=TL[k][i][0]-Ti-off 
                else:
                    off=min(TL[k][i][0]-TL[k][i-1][0],To)
                    dur=(TL[k][i][0]-TL[k][i-1][0])-off
                sum_voi=sum_voi+(b+dur*si*dy*0.5)*dur+off*b  # calcaute loitor duration at the next time. 
                acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                b=(b+dur*si*dy)*(1-acc)
            #ma_set[k].va[j]=round(b,2)
        Sum=Sum+sum_voi
    Sum=round(Sum,2)
    return Sum
def Get_TL_old(Wap,Q,win,R_v,To):
    TL=[[]for i in range(len(win))]   # for all monitoring area 
    wap=[Wap[i].loc for i in range(len(Wap))]
    typ=[Wap[i].ty for i in range(len(Wap))]
    cov=[Wap[i].cover for i in range(len(Wap))]
    D=Get_D(wap,R_v)
    #print(D)
    Time=0
    for i in range(1,len(Q)):
        #print(cov[i])
        if i==1:       # Here the first one is not included into account. 
                Time=Time+D[Q[i-1]][Q[i]]
        else:
                Time=Time+D[Q[i-1]][Q[i]]+To
        for j in cov[Q[i]]:
            TL[j].append([round(Time,2),typ[Q[i]]])
    return TL
def get_wp_old(dis, drone, win,n):   # get wp for one face with one distance. 
    Wap=[]
    h,v,hr,vr=GetFov(dis,drone,n)
    win_y=win.sort_values(by='v_2_y', ascending=False)
    h_c_dic=win.groupby(['v_2_y']).groups
    h_b_dic=win.groupby(['v_1_y']).groups 
    w_1_dic=win.groupby(['v_1_x']).groups
    w_2_dic=win.groupby(['v_3_x']).groups
    if np.sort(list(w_1_dic.keys()))[0]<np.sort(list(w_2_dic.keys()))[0]:
        w_l_dic=w_1_dic; w_r_dic=w_2_dic
    else:
        w_l_dic=w_2_dic; w_r_dic=w_1_dic
#travse from top to bottom 
    order=np.sort(list(h_c_dic.keys()))[::-1]
    covers=[]
    win_num=[[]for i in range(len(win))]    
    for i in range(len(order)):
        for j in list(h_c_dic.get(order[i])): 
            k=order[i]-v
            tmp=order[i]
            cover=[i for i in list(h_b_dic.keys())[::-1] if k<=i<tmp]
            num=1
            while len(cover)==0: 
                num=num+1
                k=k-v
                cover=[i for i in list(h_b_dic.keys())[::-1] if k<=i<tmp]
            win_num[j].append(num)
            cover_set=[]
            for z in range(len(cover)):
                cover_set=cover_set+list(h_b_dic.get(cover[z]))
            covers.append(cover_set)
    h_covers = list(covers for covers,_ in itertools.groupby(covers)) 
###travse from left to right 
    order2=np.sort(list(w_l_dic.keys()))
    covers=[]
    for i in range(len(order2)):
        for j in list(w_l_dic.get(order2[i])): 
            k=order2[i]+h
            tmp=order2[i]
            cover=[a for a in list(w_r_dic.keys()) if tmp<a<=k]
            num=1
            while len(cover)==0: 
                num=num+1
                k=k+h
                cover=[a for a in list(w_r_dic.keys()) if tmp<a<=k]
            win_num[j].append(num)
            cover_set=[]
            for z in range(len(cover)):
                cover_set=cover_set+list(w_r_dic.get(cover[z]))
            covers.append(cover_set)
    w_covers = list(covers for covers,_ in itertools.groupby(covers))
    com=[]
    # get the combination 
    for i in (h_covers):
        for j in (w_covers):
            com.append((list(set(i)&set(j))))
    com = list(com for com,_ in itertools.groupby(com))
    #heuristic method for waypoint selection for total coverage 
    com.sort(key=len)
    com=com[::-1]
    S=[];Co=[]
    i=0
    while len(Co)<len(win): 
        tmp=com[0]
        Co=set(itertools.chain(Co,tmp))
        S.append(list(win.loc[tmp]['id']))
        com.remove(tmp)
        for i in range (len(com)): 
            com[i]=list(set(com[i])-set(tmp))
        com.sort(key=len)
        com=com[::-1]
    #print("generate", len(S),"waypoints to cover all windows")
    win_count=[]
    for i in range(len(win_num)):
        win_count.append(win_num[i][0]*win_num[i][1])
    waypoint=[]
    axis=win.loc[win['id']==S[0][0]]['fix_aix'].values[0]
    dirt=win.loc[win['id']==S[0][0]]['dir'].values[0]
    fix=win.loc[win['id']==S[0][0]]['fix_v'].values[0]  # all windows in one facades are same 
    waypoints=[]
    for i in S: 
        y_max=max(win.loc[win['id'].isin(i)]['v_2_y'].values)
        y_min=min(win.loc[win['id'].isin(i)]['v_1_y'].values)   
        x_max=max(list(win.loc[win['id'].isin(i)]['v_3_x'].values)+list(win.loc[win['id'].isin(i)]['v_1_x'].values))
        x_min=min(list(win.loc[win['id'].isin(i)]['v_3_x'].values)+list(win.loc[win['id'].isin(i)]['v_1_x'].values))    
        if str(axis)=='y':
            v_y=fix+(dirt*dis)
            position=[round((x_max+x_min)/2,2),v_y,round((y_max+y_min)/2,2)]
        else:
            v_x=fix+(dirt*dis)
            position=[v_x, round((x_max+x_min)/2,2),round((y_max+y_min)/2,2)]
        waypoints.append(position)  
    return waypoints,win_count,S
def Get_VG_D_old(wps,building,R_v,off,file):
    f=open(file,'w')
    polygens=[]
    polygen=[]
    w=[];h=[];node=[]
    for index,row in building.iterrows():
        polygen=[]
        w.append(row['width'])
        h.append(row['hight'])
        node.append(row.loc[['x','y']].values)
    polygen=[]
    polygen.append(vg.Point(node[0][0]-off,node[0][1]+off))
    polygen.append(vg.Point(node[0][0]-off,node[0][1]+off))
    polygen.append(vg.Point(node[0][0]-off,node[0][1]-h[0]-off))
    polygen.append(vg.Point(node[1][0]-off,node[1][1]-off))
    polygen.append(vg.Point(node[1][0]-off,node[1][1]-h[1]-off))
    polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]-h[1]-off))
    polygen.append(vg.Point(node[1][0]+w[1]+off,node[1][1]+off))
    polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]-h[0]+off))
    polygen.append(vg.Point(node[0][0]+w[0]+off,node[0][1]+off))
    graph=vg.VisGraph()
    graph.build([polygen])
    # shortest = graph.shortest_path(vg.Point(100,6), vg.Point(-1,60))
    D=np.zeros((len(wps),len(wps)))
    f.write('i'+' '+'j'+' '+'d'+'\n')
    for i in range(len(wps)):
        for j in range(i+1,len(wps)):
            #squared_dist = np.sum((np.array(wps[i])-np.array(wps[j]))**2, axis=0)
            shortest = graph.shortest_path(vg.Point(wps[i][0],wps[i][1]), vg.Point(wps[j][0],wps[j][1]))
            length=0
            for p in range(len(shortest)-1):
                tmp=np.sum((np.array([shortest[p].x,shortest[p].y])-np.array([shortest[p+1].x,shortest[p+1].y]))**2, axis=0)
                length=length+round(np.sqrt(tmp),2)
            d_z=abs(wps[i][2]-wps[j][2])
            d=np.sqrt((d_z**2)+(length**2))
            #D[i][j]=round(d/R_v,2)
            D[i][j]=round(d,2)
            D[j][i]=D[i][j]
            f.write(str(i)+' '+str(j)+' '+ str(D[i][j])+'\n')
            f.write(str(j)+' '+str(i)+' '+str(D[i][j])+'\n')
    f.close()
    return D
def Get_TL_old2(N_WP,D,To):
    TL={}
    w_id=[N_WP[i].id for i in range(len(N_WP))]
    typ=[N_WP[i].ty for i in range(len(N_WP))]
    cov=[N_WP[i].cover for i in range(len(N_WP))]
    Time=0
    for i in range(1,len(N_WP)):
        if i==1:       
                Time=Time+D[w_id[i-1]][w_id[i]]
        else:
                Time=Time+D[w_id[i-1]][w_id[i]]+To   # Here is okay, the first one no wait. 
        #print("time is",Time)
        for j in cov[i]:   
            if TL.get(j)==None: 
                TL[j]=[]
            TL[j].append([Time,typ[i]])
    return TL
def Get_TL_old1(Wap,Q,D,R_v,To):
    TL={}
    wap=[Wap[i].loc for i in range(len(Wap))]
    typ=[Wap[i].ty for i in range(len(Wap))]
    cov=[Wap[i].cover for i in range(len(Wap))]
    #D=Get_D(wap,R_v)
    Time=0
    for i in range(1,len(Q)):
        if i==1:       # Here the first one is not included into account. 
                Time=Time+D[Q[i-1]][Q[i]]
        else:
                Time=Time+D[Q[i-1]][Q[i]]+To
        print(cov[Q[i]])
        for j in cov[Q[i]]:
            TL[j]=[]
            TL[j].append([round(Time,2),typ[Q[i]]])
    return TL
import itertools
# a={0:[1,4],23:[3,4],2:[4]}
# print(sorted(a.items(), key=lambda item: item[1],reverse=True))
# c=list(a.values())
# ab=itertools.chain(c)
# c=set([1,2,3])
# for i in c: 
#     print(i)
# c.update([1,4])
# print(len(dict([])))
