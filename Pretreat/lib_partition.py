import pandas as pd
import numpy as np
import itertools
import copy
import math
from Wp_sel import rot_back, M,Wap,GetFov
import pyvisgraph as vg

# class Wap:
#     def __init__(self,m_id=-1,loc=[],ty=-1,cover=[]):
#         self.id=m_id
#         self.loc=loc
#         self.ty=ty
#         self.cover=cover

# def GetFov(d,drone,No):
#     if (No>=len(drone.values)):
#         print("There is no ",No)
#     else:
#         data=drone.loc[No,['ss_w','ss_h','fl','pixel_w','pixel_h']].values
#         h=data[0];v=data[1];fl=data[2];ph=data[3];pv=data[4]
#         #print(data)
#         hfov=round(d*h/fl,2)
#         hr=round(ph/hfov,2)
#         haov=round(2*math.degrees(math.atan(h/(2*fl))),2)
#         #print(haov)
#         vfov=round(d*v/fl,2)
#         vr=round(pv/vfov,2)
#         vaov=round(2*math.degrees(math.atan(v/(2*fl))),2)
#         #print(vaov)
#         #print("fov with width=" ,hfov, "; height=",vfov,"; HRS=",hr,"; VRS=",vr )
#         return hfov,vfov,hr,vr


def get_m_wp(dis,drone, win, n):  # get wp for one face with one distance. 
    Wap=[]
#     com_i = list(com_i for com_i,_ in itertools.groupby(com_i))
#     com_id=list(range(len(com_i)))
#     com_dic=dict(zip(com_id,com_i))
#     com_list=[[com_i[i],com_id[i]]for i in range(len(com_i))]
#     com=copy.deepcopy(com_list)
#     #heuristic method for waypoint selection for total coverage 
#     #com.sort(key=len)
#     com.sort(key = lambda com:len(com[0]),reverse=True)
    #print("here to see", list(win['id']))
    S=list(win['id']);Co=[];C=[]
#     while len(Co)<len(win): 
#         tmp=com[0][0]
#         Co=set(itertools.chain(Co,tmp))
#         cc=com_dic.get(com[0][1])
#         S.append(list(win.loc[cc]['id']))
#         #print(tmp,cc,list(win.loc[tmp]['id']),list(win.loc[cc]['id']))
#         com.remove([tmp,com[0][1]])
#         
#         for i in range (len(com)): 
#             com[i][0]=list(set(com[i][0])-set(tmp))
#         com.sort(key = lambda com:len(com[0]),reverse=True)
#     #print("generate", len(S),"waypoints to cover all windows")
#     win_count=[]
#     for i in range(len(win_num)):
#         win_count.append(win_num[i][0]*win_num[i][1])
    waypoint=[]
    axis=win.loc[win['id']==S[0]]['fix_aix'].values[0]
    dirt=win.loc[win['id']==S[0]]['dir'].values[0]
    fix=win.loc[win['id']==S[0]]['fix_v'].values[0]  # all windows in one facades are same 
    waypoints=[];id_set=[]
    for i in S: 
        y_max=max(win.loc[win['id']==i]['v_2_y'].values)
        y_min=min(win.loc[win['id']==i]['v_1_y'].values)   
        x_max=max(list(win.loc[win['id']==i]['v_3_x'].values)+list(win.loc[win['id']==i]['v_1_x'].values))
        x_min=min(list(win.loc[win['id']==i]['v_3_x'].values)+list(win.loc[win['id']==i]['v_1_x'].values))    
        if str(axis)=='y':
            v_y=fix+(dirt*dis)
            position=[round((x_max+x_min)/2,2),v_y,round((y_max+y_min)/2,2)]
        else:
            v_x=fix+(dirt*dis)
            position=[v_x, round((x_max+x_min)/2,2),round((y_max+y_min)/2,2)]
        waypoints.append(position)  
        id_set.append(i)
    return waypoints,id_set
def Write_M_WP(Drone_N,D,faces,R_v,To,st_nor,drone,building,Cov_file,d_file,win_fname):
    nor=[];num=[]
    for i in range(len(building)):
        tmp=building.loc[[i],['fac']].values
        num=num+tmp[0][0].split(',')
        tmp=building.loc[[i],['nom']].values
        tm=tmp[0][0].split(',')
        for j in range(len(tm)):
            nor.append([int(tm[j].split('/')[k]) for k in range(3)])
    nor_map=dict(zip(num,nor))
    W_M_set=[]; Id_M_set=[]
    for j in range(faces):   
        a=np.array(nor_map.get(str(j+1)))
        fname=win_fname+str(j+1)+'.csv'
        #print("for face: ",j+1)
        win=pd.read_csv(fname,sep=' ') 
        wp_ty,id_set=Get_m_WapCa(D,win,drone,Drone_N,a,st_nor)
        W_M_set=W_M_set+wp_ty
        Id_M_set=Id_M_set+id_set
    return W_M_set,Id_M_set

#     for i in range(len(D)):   # for each distance, give the id. 
#         for j in range(len(W_ty[i])):
#             wapp=Wap(seq,W_ty[i][j],D[i],Cov_ty[i][j])
#             Wap_ty_set[i].append(wapp)
#             Wap_set.append(wapp) 
#             seq=seq+1
#     no=1
#     for i in range(len(D)):
#         f=open(Cov_file+str(D[i])+'.csv','w')
#         for j in range(len(Cov_ty[i])):
#             f.write(str(no)+' ')
#             for k in range(len(Cov_ty[i][j])):
#                 f.write(str(Cov_ty[i][j][k])+',')
#             no=no+1
#             f.write('\n')    #here I modified it to from 1 !!!!!!!!!!!!
def Get_m_WapCa(D,win,drone, d_id,nor,st_nor): # Get all waypoints for all distances
    waypoint_sets=[]
    for i in [min(D)]:
        wp,id=get_m_wp(i,drone,win,d_id)
        if str(np.cross(nor,st_nor))!='[0 0 0]':
            for p in range(len(wp)):
                wp[p]=list(rot_back(nor,st_nor,np.array(wp[p]))) 
        
    return wp,id

def Get_VG_D_M(dic_M_W,building,R_v,off,file):
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
    
    D=np.zeros((len(dic_M_W),len(dic_M_W)))
    m_id=sorted(list(dic_M_W.keys()))
    print("see",len(m_id))
    f.write('i j d')
    for i in range(len(m_id)):
        for j in m_id[i+1:]:
            #print(m_id[i],j)
            #squared_dist = np.sum((np.array(wps[i])-np.array(wps[j]))**2, axis=0)
            shortest = graph.shortest_path(vg.Point(dic_M_W.get(m_id[i])[0],dic_M_W.get(m_id[i])[1]), vg.Point(dic_M_W.get(j)[0],dic_M_W.get(j)[1]))
            length=0
            for p in range(len(shortest)-1):
                tmp=np.sum((np.array([shortest[p].x,shortest[p].y])-np.array([shortest[p+1].x,shortest[p+1].y]))**2, axis=0)
                length=length+round(np.sqrt(tmp),2)
            d_z=abs(dic_M_W.get(m_id[i])[2]-dic_M_W.get(j)[2])
            d=np.sqrt((d_z**2)+(length**2))
            #D[i][j]=round(d/R_v,2)
            D[m_id[i]][j]=round(d,2)
            D[j][m_id[i]]=D[m_id[i]][j]
            print(f" {i} {j} {round(d,2)}, {d_z}, {length} ")
            f.write(str(m_id[i])+' '+str(j)+' '+ str(D[m_id[i]][j])+'\n')
            f.write(str(j)+' '+str(m_id[i])+' '+str(D[m_id[i]][j])+'\n')
    f.close()
    return D

