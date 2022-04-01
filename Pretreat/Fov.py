import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd
import math 
import matplotlib
import itertools

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
        print("fov with width=" ,hfov, "; height=",vfov,"; HRS=",hr,"; VRS=",vr )
        return hfov,vfov,hr,vr
def get_wp(D, win,n):
    Wap=[]
    for d in D:
        h,v,hr,vr=GetFov(d,drone,n)
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
        print("generate", len(S),"waypoints to cover all windows")
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
            x_max=max(win.loc[win['id'].isin(i)]['v_3_x'].values)
            x_min=max(win.loc[win['id'].isin(i)]['v_1_x'].values)    
            if str(axis)=='y':
                v_y=fix+(dirt*dis)
                position=[round((x_max+x_min)/2,2),v_y,round((y_max+y_min)/2,2)]
            else:
                v_x=fix+(dirt*dis)
                position=[v_x, round((x_max+x_min)/2,2),round((y_max+y_min)/2,2)]
            waypoints.append(position)     
        Wap=Wap+wapoints
    return waypoints,win_count,S















