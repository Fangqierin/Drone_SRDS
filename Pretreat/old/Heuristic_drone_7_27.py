import copy
import numpy as np 
import itertools
from collections import defaultdict # Use defaultdict for prefix and query
import random 
import time
######################################################
def Greedy_WPS(case,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):
    seq_set=set(range(len(Wap_set)-1))
   # GetTsp(Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm)
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    # I want to get a dict for mapping cover-- id of Wap_set     
    cu_t=T_in
    w_id=w_in
    ma_set=copy.deepcopy(M_set)
    P=[w_id]
    M_W_dict=defaultdict(list)  # Mapping from M to W
    for i in range(len(Wap_set)):
        for j in list(Wap_set[i].cover):
            M_W_dict[j].append(i)
    if case==0 or case==2:
        need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
        #if len(need)>0: 
        #print("first round to cover all NEED",need)
        la_t=cu_t
        P2,cu_t,w_id=Greedy_Cov(need,rest,to_w_seq,Wap_set,ma_set,D,task_dic,ins_m_dic,w_id,cu_t,T_tm) 
        P=P+P2
        #print("at time", cu_t,"HERE IS",P2)
        TL=Get_TL([Wap_dic.get(i) for i in P],D,la_t)
        #print(sorted(TL.items()))
        ma_set=Update_M_TL(ma_set, TL, la_t, cu_t, D, task_dic, ins_m_dic,T_tm)
#print(f" at case {case}, with {P}")
    de=[]
    while cu_t<T_tm:
        
        for i in list(seq_set):
            d=D[w_id][Wap_set[i].id]
            if d+cu_t>T_tm:
                de.append(i)
        seq_set=seq_set-set(de)
        if len(seq_set)==0:
            break
        if case==2 or case==3:
            # This one is good.
            V=[Get_Re_Dy(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_tm) for i in list(seq_set)]
        else:
            if case==6:  # this result is not good.
                #(P,Wap_set,wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm):
                V=[Get_Re_TL(P,Wap_set,Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm) for i in list(seq_set)]
            else:
                V=[Get_Re_One(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic) for i in list(seq_set)]
        #V=[Get_voi(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic) for i in range(len(Wap_set))]
        opt=max(V, key=lambda item: item[0])[1]
        if max(V, key=lambda item: item[0])[0]==0:
            break
        ma_set,cu_t=M_update_Min(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm) # current waypoint w_id 
        #print(f"ma_set {[ma_set[i].ac_re for i in ma_set.keys()]}")
        P.append(opt.id)
        w_id=opt.id
    return P,cu_t
#################### 5/24 for max min 
def Greedy_TSP(kind,Cu_Wap_ty,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):
    Wap_set=[]
    if kind==-1:
        for i in Cu_Wap_ty:
            Wap_set.append(list(i))
    else: 
        Wap_set=list(Cu_Wap_ty[kind])
    TSP=[];w_id=w_in; cu_t=T_in; P=[w_in]
    if kind==-1:
        num=1   
        id_set=[Wap_set[num][i].id for i in range(len(Wap_set[num]))]
        while True:
            if len(id_set)>0:
                Dis=sorted([(D[w_id][w],w) for w in id_set])
                w_c=Dis[0][1]
                id_set.remove(w_c)
                TSP.append(w_c)
                w_id=w_c
            else:
                if num==1: num=0 
                else: break
                id_set=[Wap_set[num][i].id for i in range(len(Wap_set[num]))]
    else:
        id_set=[Wap_set[i].id for i in range(len(Wap_set))]
        while len(id_set)>0:
            Dis=sorted([(D[w_id][w],w) for w in id_set])
            w_c=Dis[0][1]
            id_set.remove(w_c)
            TSP.append(w_c)
            w_id=w_c
    seq=0; w_id=w_in
   # print(f"TSP {TSP}")
    while cu_t<T_tm:
        if seq<len(TSP):
            #  sP.append(TSP[seq])
            cu_t=cu_t+D[w_id][TSP[seq]]
            if cu_t>=T_tm:
                cu_t=cu_t-D[w_id][TSP[seq]]
                break
            P.append(TSP[seq])
            w_id=TSP[seq]
            seq=seq+1; 
        else: 
            seq=0     
    return(P,cu_t)
def Greedy_Min(case,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):

    seq_set=set(range(len(Wap_set)-1))
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    # I want to get a dict for mapping cover-- id of Wap_set     
    cu_t=T_in
    w_id=w_in
    ma_set=copy.deepcopy(M_set)
    P=[w_id]
    M_W_dict=defaultdict(list)  # Mapping from M to W
    for i in range(len(Wap_set)):
        for j in list(Wap_set[i].cover):
            M_W_dict[j].append(i)
    if case==1:
        need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
        la_t=cu_t
        P2,cu_t,w_id=Greedy_Cov(need,rest,to_w_seq,Wap_set,ma_set,D,task_dic,ins_m_dic,w_id,cu_t,T_tm) 
        P=P+P2
        print(f"here is Cov case {case}")
        TL=Get_TL([Wap_dic.get(i) for i in P],D,la_t)
        ma_set=Update_M_TL(ma_set, TL, la_t, cu_t, D, task_dic, ins_m_dic,T_tm)
        print("already cover all",min([min(list(ma_set.values())[i].ac_re) for i in range(len(ma_set))]))
    de=[]
    print(f"here is the case {case}")
    while cu_t<T_tm:
        for i in list(seq_set):
            d=D[w_id][Wap_set[i].id]
            if d+cu_t>T_tm:
                de.append(i)
        seq_set=seq_set-set(de)
        if len(seq_set)==0:
            break
            # This one is good. 
        if case==0:
            V=[Get_Re_Dy(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_tm) for i in list(seq_set)]
            opt=max(V, key=lambda item: item[0])[1]
            if max(V, key=lambda item: item[0])[0]==0:
                break
        else: 
           
            V=[Get_Re_Min(case,Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_tm) for i in list(seq_set)]
            if max(V, key=lambda item: item[0])[0]==0:
                break
            max_min=max([V[i][1] for i in range(len(V))])
            if max_min==0:
                max_count=max([V[i][2] for i in range(len(V))])
                if max_count==0:
                   # print("min and count stay 0")
                    opt=max(V, key=lambda item: item[0])[3]
                else:
                   # print(f"min 0 count changed to {max_count}")
                    opt_set=[V[i] for i in range(len(V)) if V[i][2]==max_count]
                    opt=max(opt_set, key=lambda item: item[0])[3]
            else:
              #  print(f"min changed {max_min}")
                opt_set=[V[i] for i in range(len(V)) if V[i][1]==max_min]
                opt=max(opt_set, key=lambda item: item[0])[3]
                #print(max(V, key=lambda item: item[1])[2])
        #print(f"here {opt}")
        ma_set,cu_t=M_update_Min(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm) # current waypoint w_id 
        #print(f"change from {min_re} to ",min([min(list(ma_set.values())[i].ac_re) for i in range(len(ma_set))]))
#print(f"ma_set {[(ma_set[i].ac_re,ma_set[i].id) for i in ma_set.keys()]}")
        P.append(opt.id)
        w_id=opt.id
    return P,cu_t
    
####################  7_4
def Get_Re_Min(case,wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_tm):
   # print(f"to check this one ooooo! ")
    min_re=min([min(list(m_set.values())[i].ac_re) for i in range(len(m_set))])
    count=sum([list(m_set.values())[i].ac_re.count(min_re) for i in range(len(m_set))])
    m_new_set=copy.deepcopy(m_set)
    if cu_t+D[w_id][wap.id]>T_tm:
        return 0,wap
    cover=wap.cover
    all_id=list(m_set.keys())
    rest=list(set(all_id)-set(cover))
    #####################################################
    Grain=0
    T_rest=T_tm-(cu_t+D[w_id][wap.id])
    if T_rest<0:
        return 0,0,0,wap
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        mac_re=m_set[i].ac_re
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            get_t=cu_t+D[w_id][wap.id]
            la_t=max(0,lt_set[j])
            #dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])            
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            #tmp=max(0,ini_va[j]-dur*dy) 
            tmp= Reliable(ini_va[j],la_t,get_t, 0, dy)    
#             if len(cover)>1:
#                 print(f" why {ini_va[j]}, {acc}, {dy}, {tmp}")     
            if T_rest==0:
                Grain=Grain+(max(tmp,acc)-tmp)
                m_new_set[i].ac_re[j]=mac_re[j]+(max(tmp,acc)-tmp)
                
                #print(f"check {m_new_set[i].ac_re[j]},{m_set[i].ac_re[j]}")
            else:
#                 d_t=(T_rest if dy==0 else min(T_rest,tmp/dy))
#                 pre_re=(tmp+max(0,tmp-T_rest*dy))*d_t*0.5
                increase=Integral(tmp, acc, dy, get_t, T_tm)
                
#                 new=max(tmp,acc)
#                 d_t=(T_rest if dy==0 else min(T_rest,new/dy))
#                 new_re=(new+max(0,new-T_rest*dy))*d_t*0.5
                Grain=Grain+increase
                m_new_set[i].ac_re[j]=mac_re[j]+increase
#                 if len(cover)>1:
#                     print(f"{tmp},{acc}, {get_t}, {la_t}, {T_tm}, {increase}")
#     if len(cover)>1:
#         print(f"Grain {Grain}")
    min_new=min([min(list(m_new_set.values())[i].ac_re) for i in range(len(m_set))])
    min_count=sum([list(m_new_set.values())[i].ac_re.count(min_new) for i in range(len(m_set))])
    if case==2:
        return Grain/(D[w_id][wap.id]),(min_new-min_re)/(D[w_id][wap.id]),(count-min_count)/(D[w_id][wap.id]),wap
    if case==1:
        return Grain/(D[w_id][wap.id]),(min_new-min_re),(count-min_count),wap
################################## D####################

################################## D####################
def Greedy_Cov(need,rest,to_w_seq,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):
    P=[]
    seq_set=to_w_seq
    cu_t=T_in
    to_cov=set(need.keys())
    tmp=list(to_w_seq)
    w_co=dict(zip(tmp,[Wap_set[i].cover for i in tmp]))
    ty=dict(zip(tmp,[Wap_set[i].ty for i in tmp]))
    task_cov={}
    w_id=w_in
    for i in (to_w_seq): 
        w_co[i]=list(set(w_co.get(i))& to_cov) # what waypoint can cover
        co_task=[need.get(k) for k in (w_co.get(i))] 
        m_task=defaultdict(list)
        for k in range(len(co_task)):
            m_task[w_co.get(i)[k]]=[ins_m_dic.get(str(ty.get(i))).get(a) for a in co_task[k]]
            #print("m_task",m_task)
        task_cov[i]=m_task
    while len(to_cov)>0 or cu_t<T_tm:
        seq_set_co=copy.deepcopy(seq_set)
        for i in seq_set_co:
            if cu_t+D[w_id][Wap_set[i].id]>T_tm:
                seq_set.remove(i)    # remove waypoints cannot be reached within T_tm
        if len(seq_set)==0:
            #print("time is out")
            break
        Prof={}
        for i in seq_set:
            prof=0 
            for j in w_co[i]:
                tmp=np.matmul(np.array(task_cov.get(i).get(j)),rest.get(j))
                prof=prof+tmp 
            Prof[i]=(prof/((D[w_id][Wap_set[i].id])),prof,D[w_id][Wap_set[i].id])
        choice=sorted(Prof.items(),key=lambda item: item[1],reverse=True)[0][0]  # get seq_number
        if (sorted(Prof.items(),key=lambda item: item[1],reverse=True)[0][1][0])==0:
            break
    ########## next update id_set and rest
        for j in w_co[choice]:
            if j in to_cov:
                rest[j]=[min(1-np.ceil(i*j),j) for i, j in zip(np.array(task_cov.get(choice).get(j)),rest.get(j))]
                if max(list(rest.get(j))).any()==0:
                    to_cov.remove(j)
        seq_set.remove(choice) 
        P.append(Wap_set[choice].id)
        cu_t=cu_t+D[w_id][Wap_set[choice].id]
        w_id=Wap_set[choice].id
    return P,cu_t,w_id

def Check_EM_M(ma_set,M_W_dict):  # Check the area which are being monitoried 
    in_va=dict(zip(list(ma_set.keys()),[(ma_set[i].va,ma_set[i].task) for i in list(ma_set.keys())]))
    need=[]
    rest=[]
    for key,item in in_va.items():
        if 0 in item[0]:
            k=[i for i in range(len(item[0])) if item[0][i]==0]
            need.append((key,[item[1][i] for i in k]))
            rest.append((key,np.ones(len(k))))
    need=dict(need)
    rest=dict(rest)
    to_w_seq=set()
    for i in list(need.keys()):
        to_w_seq.update(M_W_dict.get(i))
    return need,rest,to_w_seq

############### finish 7 4 
def Update_M_TL(M_set,Tl,Ti,Time,D,task_dic,ins_m_dic,T_tm):
    #print(f" see new haha to see")
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0   # do not need to change TL.
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                if TL[k][-1][0]<Time: # here is cu_t, must greater than the last one.
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
            else:
                TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        ma_set[k].la_t=np.ones(len(ini_va))*Time  ###### all get the value at Time 
        if TL.get(k)==None:   # if areas did not be covered 
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])
                b=ini_va[j]  
                if b>0:
#                     dur=T_tm-Ti 
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=(b+max(0,b-dur*dy))*d_t*0.5
                    #print(f" old b {b}")
                    sum_one=Integral(0,b,dy,Ti,T_tm)
                    ma_set[k].ac_re[j]=sum_one
                    ma_set[k].va[j]=Reliable(b, Ti, Time, 0, dy)
                    #print(f" {ma_set[k].va[j]}, {Ti}, {Time},{dy},{sum_one}")
        else:  
            for j in range(len(task_set)):  # for all tasks of a m 
                sum_one=0
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]        
                for i in range(len(TL[k])):
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                       # dur=TL[k][i][0]-Ti   
                        sum_one=sum_one+Integral(0, b, dy, Ti, TL[k][i][0]) 
                        b=Reliable(b, Ti, TL[k][i][0], acc, dy)
                      #  print(f" {b}, {Ti}, {Time},{dy},{sum_one}")

                    else:
                        #dur=(TL[k][i][0]-TL[k][i-1][0])
                        sum_one=sum_one+Integral(0, b, dy, TL[k][i-1][0], TL[k][i][0])
                        b=Reliable(b, TL[k][i-1][0], TL[k][i][0], acc, dy)
                        #print(f" {b}, {TL[k][i-1][0]}, {TL[k][i][0]},{dy},{sum_one}")

#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
#                     b=max(0,b-dur*dy)
#                     b=max(b,acc)
#                 dur=T_tm-Time 
#                 d_t=(dur if dy==0 else min(dur,b/dy))
#                 sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
                sum_one=sum_one+Integral(0, b, dy, Time, T_tm)   # plus the re to T_tm
                ma_set[k].va[j]=b
                ma_set[k].ac_re[j]=sum_one
               # print(f" {ma_set[k].va[j]}, {Time}, {T_tm},{dy},{sum_one}")
                
    return ma_set 
# ###### finished 7 4  oh, it is not used. 
# def M_update_Re(wap,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in):
#     m_co=copy.deepcopy(ma_set)
#     arr_t=cu_t+D[w_id][wap.id]
#     for i in wap.cover:
#         
#         task_set=ma_set[i].task
#         ini_va=ma_set[i].va  # inital value of all tasks 
#         lt_set=ma_set[i].la_t  # last visit time of all tasks 
#         #print("start",ma_set[i].va)
#         m_co[i].la_t=(arr_t)*np.ones(len(lt_set))      
#         for j in range(len(task_set)):
#             [si,dy]=task_dic.get(task_set[j])
#             dur=(arr_t-max(T_in,lt_set[j]))  # increase duration 
#             acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
#             tmp=max(0,ini_va[j]-dur*dy)
#             m_co[i].va[j]=max(tmp,acc) # update the end value 
# 
#     return m_co,(arr_t) # here is leave time
###### I want to see 7 4 
def M_update_Min(wap,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm):
    m_co=copy.deepcopy(ma_set)
    arr_t=cu_t+D[w_id][wap.id]
    #print(f"{wap.cover} at{wap.id} at time{cu_t}")
    T_rest=T_tm-(cu_t+D[w_id][wap.id])
   # print(f" see {wap.cover}")
    for i in wap.cover:
        task_set=ma_set[i].task
        ini_va=ma_set[i].va  # inital value of all tasks 
        lt_set=ma_set[i].la_t  # last visit time of all tasks 
        mac_re=ma_set[i].ac_re
        m_co[i].la_t=(arr_t)*np.ones(len(lt_set))  
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            #dur=(arr_t-max(T_in,lt_set[j]))  # increase duration 
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            #tmp=max(0,ini_va[j]-dur*dy)
            la_t=max(0,lt_set[j])
            tmp=Reliable(ini_va[j],la_t,arr_t, 0, dy)
            m_co[i].va[j]=max(tmp,acc) # update the end value 
            if T_rest==0:
                m_co[i].ac_re[j]=mac_re[j]+(max(acc,tmp)-tmp)
            else:
                increase=Integral(tmp, acc, dy, arr_t, T_tm)        
#                 d_t=(T_rest if dy==0 else min(T_rest,tmp/dy))
#                 pre_re=(tmp+max(0,tmp-T_rest*dy))*d_t*0.5
#                 new=max(tmp,acc)
#                 d_t=(T_rest if dy==0 else min(T_rest,new/dy))
#                 new_re=(new+max(0,new-T_rest*dy))*d_t*0.5
                m_co[i].ac_re[j]=mac_re[j]+increase
          #  print(f"{i}, {task_set[j]}, {tmp},{acc}, {arr_t}, {la_t}, {T_tm},{ m_co[i].ac_re[j]} {increase}")
    return m_co,(arr_t) # here is leave time


###################################4——30 finish 7 4 

def Get_Re_One(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    cover=wap.cover
    all_id=list(m_set.keys())
    rest=list(set(all_id)-set(cover))
  #  Sum=0
#     for i in rest:
#         task_set=m_set[i].task
#         ini_va=m_set[i].va  # inital value of all tasks 
#         lt_set=m_set[i].la_t  # last visit time of all tasks 
#         for j in range(len(task_set)):
#             [si,dy]=task_dic.get(task_set[j])
#             dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])  # already added 
#             t1=cu_t-max(0,lt_set[j])
#             t2=D[w_id][wap.id]
#             Sum=Sum+(2*ini_va[j]+si*dy*(2*t1+t2))*t2*0.5
#             dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])
#             #Sum=Sum+(ini_va[j]+dur*si*dy*0.5)*dur
    Grain=0
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
#             dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])  
            la_t= max(0,lt_set[j])
            arr_t= cu_t+D[w_id][wap.id]        
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
#             if ini_va[j]-(dur*dy)<0:
#                 tmp=0
#                 #tmp=(ini_va[j]/dy)-dur  # here is less than 0. 
#             else:
#                 tmp=ini_va[j]-dur*dy     
            tmp=Reliable(ini_va[j],la_t,arr_t, 0, dy)
            Grain=Grain+(max(tmp,acc)-tmp)*si   # <------------- here we have significance !!!1! oooo 
            #Grain=Grain+(ini_va[j]+dur*si*dy)*(acc)
            #get_voi=get_voi+end_v 
#     if Sum==0:
#         Sum=0.1
    #print(cu_t,cover,Grain/Sum,Grain,Sum,d,ini_va[j],dur)
    #print (f"One see {Grain},{D[w_id][wap.id]}")
    return (Grain/D[w_id][wap.id]),wap
####################### finish 7 4 
def Get_Re_Dy(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_tm):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    if cu_t+D[w_id][wap.id]>T_tm:
        return 0,wap
    cover=wap.cover
    Grain=0
    T_rest=T_tm-(cu_t+D[w_id][wap.id])
    if T_rest<0:
        return 0,wap
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            #dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j]) 
            la_t= max(0,lt_set[j])
            arr_t= cu_t+D[w_id][wap.id]
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            #tmp=max(0,ini_va[j]-dur*dy)   
            tmp=Reliable(ini_va[j],la_t,arr_t, 0, dy)
            if T_rest==0:
                Grain=Grain+(max(tmp,acc)-tmp)*si
            else:
                increase=Integral(tmp, acc, dy, arr_t, T_tm)
#                 d_t=(T_rest if dy==0 else min(T_rest,tmp/dy))
#                 pre_re=(tmp+max(0,tmp-T_rest*dy))*d_t*0.5
#                 new=max(tmp,acc)
#                 d_t=(T_rest if dy==0 else min(T_rest,new/dy))
#                 new_re=(new+max(0,new-T_rest*dy))*d_t*0.5
                Grain=Grain+(increase)*si
    #d=D[w_id][wap.id]
    return (Grain/(D[w_id][wap.id])),wap




def Get_Re_TL(P,Wap_set,wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    if cu_t+D[w_id][wap.id]>T_tm:
        return 0,wap
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    P_c=copy.deepcopy(P)
    P_c=P_c+[wap.id]
    Tl=Get_TL([Wap_dic.get(i) for i in P_c],D,T_in)
    re,min_re,co_t=Calculate_Acc(m_set,Tl,T_in,T_tm,task_dic,ins_m_dic)
    return re,wap

def Integral(cur_acc,acc,dy,t,T):
    
    if cur_acc< acc:
        a=acc-cur_acc
        if dy==0:
            return a*(T-t)
        d=1-dy
        if dy==0.02:
            print(f"check ooo {t,T,cur_acc,acc}")
            print(round(a*(pow(d,T-t)-pow(d,0))/np.log(d),2))
        return round(a*(pow(d,T-t)-pow(d,0))/np.log(d),2)
        #return round(a*(pow(d,T)-pow(d,t))/np.log(d),2)
    else:
        return 0

def Reliable(last_b,last_t,cu_t,acc,dy):
    cu_b=last_b*pow(1-dy,cu_t-last_t)
    return round(max(cu_b,acc),2)

######################################################### After change reliability function
def Calculate_Acc(M_set,Tl,Ti,Time,task_dic,ins_m_dic):
   # print(f" haha to see")
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    co_t=0
    sum_re=0
    sum_event=[]
#     print(ma_set)
#     print([ma_set[k].task for k in list(ma_set.keys())])
    event_size=sum([len(ma_set[k].task) for k in list(ma_set.keys())])
    print(f" how many event? {event_size}")
    #Sum_tl=0;
    M_v={};M_int={}
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
#                 Sum_tl=Sum_tl+len(TL[k])
                M_v[k]=len(TL[k])
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
                #print(f"see",Sum_tl,TL[k])
            else:
                TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        if TL.get(k)==None:   # if areas did not be covered 
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                if b>0:
#                     dur=Time-Ti 
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=(b+max(0,b-dur*dy))*d_t*0.5
                    sum_one=Integral(0,b,dy,Ti,Time)
                    sum_event.append(sum_one)
                    print(f"no TL the sum_one {k},{task_set[j]} {sum_one},")
                    sum_re=sum_re+sum_one*si
                else: 
                    sum_event.append(0)
        else:
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                sum_one=0 
                print(f"see",TL[k])
                for i in range(len(TL[k])):
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                        #dur=TL[k][i][0]-Ti    
                        sum_one=sum_one+Integral(0, b, dy, Ti, TL[k][i][0])
                       # print(f" old b {b}")
                        b=Reliable(b, Ti, TL[k][i][0], acc, dy)
                      #  print(f" {b}, {Ti}, {TL[k][i][0]},{acc},{dy},{sum_one}")
                    else:
                        sum_one=sum_one+Integral(0, b, dy, TL[k][i-1][0], TL[k][i][0])
                        b=Reliable(b, TL[k][i-1][0], TL[k][i][0], acc, dy)
                        
                       # print(f" {b}, {TL[k][i][0]}, {TL[k][i-1][0]}, {acc},{dy},{sum_one}")
#                         dur=(TL[k][i][0]-TL[k][i-1][0])
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
#                     if b>0:
#                         co_t=co_t+d_t
#                     b=max(0,b-dur*dy)
#                     acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
#                     b=max(b,acc)
                    print(f"check STEP sum_one {k},{task_set[j]} {acc} {dy} {sum_one},")
                sum_event.append(sum_one)
                print(f"check the sum_one {k},{task_set[j]} {dy} {sum_one},")
                
                sum_re=sum_re+sum_one*si
    Sum=round(sum_re/event_size,2)
    print(len(sum_event),sum_event)
    print(f"to see {M_v}")
    return Sum,(min(sum_event),-(sum_event.count(min(sum_event)))),(co_t/event_size)



# def Calculate_Re(M_set,Tl,Ti,Time,task_dic,ins_m_dic):
#     TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
#     ma_set=copy.deepcopy(M_set)
#     co_t=0
#     sum_re=0
#     sum_event=[]
#     event_size=sum([len(ma_set[k].task) for k in range(len(ma_set))])
#     for k in list(M_set.keys()):  # for all monitoring areas 
#         if TL.get(k)!=None:
#             TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
#             if len(TL[k])>0:
#                 if TL[k][-1][0]<Time:
#                     TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
#             else:
#                 TL[k]=[[Time,0]]
#         task_set=ma_set[k].task
#         ini_va=ma_set[k].va  # inital value of all tasks 
#         if TL.get(k)==None:   # if areas did not be covered 
#             for j in range(len(task_set)):  # for all tasks of a m 
#                 [si,dy]=task_dic.get(task_set[j])    
#                 b=ini_va[j]  
#                 if b>0:
#                     dur=Time-Ti 
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=(b+max(0,b-dur*dy))*d_t*0.5
#                     sum_event.append(sum_one)
#                     sum_re=sum_re+sum_one*si
#                 else: 
#                     sum_event.append(0)
#         else:
#             for j in range(len(task_set)):  # for all tasks of a m 
#                 [si,dy]=task_dic.get(task_set[j])    
#                 b=ini_va[j]  
#                 sum_one=0
#                 for i in range(len(TL[k])):
#                     if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
#                         dur=TL[k][i][0]-Ti    
#                     else:
#                         dur=(TL[k][i][0]-TL[k][i-1][0])
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
#                     if b>0:
#                         co_t=co_t+d_t
#                     b=max(0,b-dur*dy)
#                     acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
#                     b=max(b,acc)
#                 sum_event.append(sum_one)
#                 sum_re=sum_re+sum_one*si
#     Sum=round(sum_re/event_size,2)
#     return Sum,(min(sum_event),-(sum_event.count(min(sum_event)))),(co_t/event_size)


def Get_TL(N_WP,D,T_in):
    TL={}
    w_id=[N_WP[i].id for i in range(len(N_WP))]
    typ=[N_WP[i].ty for i in range(len(N_WP))]
    cov=[N_WP[i].cover for i in range(len(N_WP))]
    Time=T_in
    for i in range(1,len(N_WP)):
        #if i==1:       
        Time=Time+D[w_id[i-1]][w_id[i]]
        for j in cov[i]:   
            if TL.get(j)==None: 
                TL[j]=[]
            TL[j].append([Time,typ[i]])
    return TL
#####################################################################################
#################################Tabu search 
def Tabu(P_in,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,NeigSize,MaxTabuSize,stopTurn,
         stop_condition,time_condition):
    # stop_Turn: the maximal iterations there is no change. 
    # time_condition: False or True
    # stop_condition: the maximal runtime if time_condition==True, otherwise: is the maximal iterations
    max_fitness=fitness(P_in,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm)
    sBest =P_in
    vBest=max_fitness
    bestCandidate = P_in
    tabuList = []
    tabuList.append(P_in)
    total_iterations=0
    best_keep_turn = 0
    bestfitness=0
    stop = False
    #start_time=time.time()
    N_dict=Get_adj(Wap_set,D,max_d=10)
    if time_condition==True:
        stop_time=time.time()+stop_condition
        def stop_condition():
            return stop or time.time()>=stop_time
    else:
        def stop_condition():
            return stop or total_iterations==stop_condition
    while not stop_condition():
        sNeighborhood = getNeighbors(bestCandidate,Wap_set,NeigSize,N_dict)
        bestCandidate = sNeighborhood[0]
        for sCandidate in sNeighborhood:
            if (sCandidate not in tabuList) and ((fitness(sCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm) > fitness(bestCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm))):
                bestCandidate = sCandidate
                bestfitness=fitness(sCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm)
        total_iterations+=1
        if (bestfitness > vBest):
            print("get a new fitness",bestfitness,vBest)
            sBest = bestCandidate
            vBest = bestfitness
            best_keep_turn = 0
            tabuList.append(bestCandidate)
        if (len(tabuList) > MaxTabuSize):
            tabuList.pop(0)
        if best_keep_turn == stopTurn:
            stop = True
        best_keep_turn += 1
        #print("best_keep",best_keep_turn)
    return sBest, vBest
    

def Tabu_min(P_in,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,NeigSize,MaxTabuSize,stopTurn,
             stop_condition,time_condition):
    max_fitness=fitness_min(P_in,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm)
    sBest =P_in
    vBest=max_fitness
    bestCandidate = P_in
    tabuList = []
    tabuList.append(P_in)
    stop = False
    total_iterations=0
    best_keep_turn = 0
    bestfitness=fitness_min(P_in,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm)
    #start_time=time.time()
    N_dict=Get_adj(Wap_set,D,max_d=10)
    #print(N_dict)
    if time_condition==True:
        stop_time=time.time()+stop_condition
        print(f"start at time {time.time()}")
        def stop_condition():
            print(f"why to see  {stop or time.time()>=stop_time} ")
            return stop or time.time()>=stop_time
    else:
        def stop_condition():
            return stop or total_iterations==stop_condition
    while not stop_condition():
        print(f"try {total_iterations} ")
        sNeighborhood = getNeighbors(bestCandidate,Wap_set,NeigSize,N_dict)
        bestCandidate = sNeighborhood[0]
        for sCandidate in sNeighborhood:
            if (sCandidate not in tabuList) and (fitness_min(sCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm)) > fitness_min(bestCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):
                bestCandidate = sCandidate
                bestfitness=fitness_min(sCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm)
        total_iterations+=1
        if (bestfitness > vBest):
            print("get a new fitness",bestfitness,vBest)
            sBest = bestCandidate
            vBest = bestfitness
            best_keep_turn = 0
            tabuList.append(bestCandidate)
        if (len(tabuList) > MaxTabuSize):
            tabuList.pop(0)
        if best_keep_turn == stopTurn:
            stop = True
        best_keep_turn += 1
        #print("best_keep",best_keep_turn)
    print(f"stop at time {time.time()}")
    return sBest, vBest



def getNeighbors(P,Wap_set,NeigSize,N_dict):
    a=random.randint(0,1)
    if a==0:
        #print("Mutation")
        return Mutation(P, NeigSize, N_dict,Wap_set)
    else:
        #print("swap")
        return two_opt_swap(P,NeigSize)
def two_opt_swap(P,NeigSize):
    neighbors=[]
    for i in range(NeigSize):
        node1=0
        node2=0
        while node1==node2:
            node1=random.randint(1,len(P)-1)
            node2=random.randint(1,len(P)-1)
        tmp=P[node1:node2]
        tmp_P=P[:node1]+tmp[::-1]+P[:node2]
        neighbors.append(tmp_P)
    return neighbors
def Get_adj(Wap_set,D,max_d):
    id_set=[Wap_set[i].id for i in range(len(Wap_set)-1)]  # here minus 1 means get out of 0 
    N_dict={}
    for i in id_set:
        tmp=[]
        for j in id_set:
            if D[i][j]<max_d:
                tmp.append(j)
        N_dict[i]=tmp
    return N_dict
def Mutation(P_in,NeigSize,N_dict,Wap_set): 
    P=copy.deepcopy(P_in)
    id_set=[Wap_set[i].id for i in range(len(Wap_set)-1)] 
    neighbors=[]
    i=0
    for i in range(NeigSize):
        c=random.randint(1,len(P)-1)
        #N=N_dict.get(P[i])
        j=id_set[random.randint(1,len(id_set)-1)]
            #tmp=P[i]
        tmp=P[:c]+[j]+P[c+1:]
            #print(P,j,tmp)
        neighbors.append(tmp)
#     while i <(NeigSize):
#         c=random.randint(1,len(P)-1)
#         print(c)
#         i=i+1
#         tmp=P
#         tmp[c]=id_set[random.randint(0,len(id_set)-1)]
#         neighbors.append(tmp)
    return neighbors

def fitness(P,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    Tl=Get_TL([Wap_dic.get(i) for i in P],D,T_in)
    re,min_re,miss=Calculate_Acc(M_set,Tl,T_in,T_tm,task_dic,ins_m_dic)
    return re
def fitness_min(P,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm):
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    Tl=Get_TL([Wap_dic.get(i) for i in P],D,T_in)
    re,min_re,miss=Calculate_Acc(M_set,Tl,T_in,T_tm,task_dic,ins_m_dic)
    return min_re

