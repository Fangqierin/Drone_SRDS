import copy
import numpy as np 
import itertools
from collections import defaultdict # Use defaultdict for prefix and query
import random 
import time
from Integral import Integral_grain, Sump_Integral,Reliable,Waiting_grain,Cur_Integral
######################################################
def Greedy_WPS(case,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,T_total):
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
    if case==0 or case==5 or case==2:
        need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
        #if len(need)>0: 
        #print("first round to cover all NEED",need)
        la_t=cu_t
        P2,cu_t,w_id=Greedy_Cov(need,rest,to_w_seq,Wap_set,ma_set,D,task_dic,ins_m_dic,w_id,cu_t,T_tm) # here no change 
        P=P+P2
        #print("at time", cu_t,"HERE IS",P2)
        TL=Get_TL([Wap_dic.get(i) for i in P],D,la_t)
        #print(sorted(TL.items()))   # change T_tm to T_total
        #ma_set=Update_M_TL(ma_set, TL, la_t, cu_t, D, task_dic, ins_m_dic,T_total)
        ma_set=Update_M_TL(ma_set, TL, la_t, D, task_dic, ins_m_dic,T_total)
    #print(f"case {case} {cu_t}")
    de=[]
    while cu_t<T_tm:
        for i in list(seq_set):
            d=D[w_id][Wap_set[i].id]
            if d+cu_t>T_total:
                de.append(i)
        seq_set=seq_set-set(de)
        if len(seq_set)==0:
            break
        if case==5:
            V=[Get_Re_Min(case,Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_total) for i in list(seq_set)]
            if max(V, key=lambda item: item[0])[0]==0:
                break
            max_min=max([V[i][1] for i in range(len(V))])
            if max_min==0:
                max_count=max([V[i][2] for i in range(len(V))])
                if max_count==0:
                   # print("min and count stay 0")
                    opt=max(V, key=lambda item: item[0])[3]
                else:
                    
                    opt_set=[V[i] for i in range(len(V)) if V[i][2]==max_count]
                    opt=max(opt_set, key=lambda item: item[0])[3]
                    #print(f"min 0 count changed to {max_count} by {opt.cover}")
            else:
                
                opt_set=[V[i] for i in range(len(V)) if V[i][1]==max_min]
                opt=max(opt_set, key=lambda item: item[0])[3]
        else:
            if case==2:
                #print(f"case=2?/????")
                V=[Get_by_wait(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_total) for i in list(seq_set)]
            if case==3:
                # This one is good.
                V=[Get_Re_Dy(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_total) for i in list(seq_set)]
            if case==6:  # this result is not good.
                #(P,Wap_set,wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm):
                V=[Get_Re_TL(P,Wap_set,Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_total) for i in list(seq_set)]
            if case==0 or case==1:
                V=[Get_Re_One(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic) for i in list(seq_set)]
                    
    
            #V=[Get_voi(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic) for i in range(len(Wap_set))]
            opt=max(V, key=lambda item: item[0])[1]
            if max(V, key=lambda item: item[0])[0]==0:
                break
        ma_set,cu_t=M_update_Min(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_total)
        #print(f"check", cu_t) # current waypoint w_id 
        #print(f"ma_set {[ma_set[i].ac_re for i in ma_set.keys()]}")
        P.append(opt.id)
        w_id=opt.id
    return P,cu_t
def Greedy_Min(case,co_s,Wap_set,ma_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,T_total,Cu_Wap_ty):
    #ma_set=copy.deepcopy(M_set)
    seq_set=set(range(len(Wap_set)-1))
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    # I want to get a dict for mapping cover-- id of Wap_set     
    cu_t=T_in
    w_id=w_in
    #ma_set=copy.deepcopy(M_set)
    P=[w_id]
    M_W_dict=defaultdict(list)  # Mapping from M to W
    for i in range(len(Wap_set)):
        for j in list(Wap_set[i].cover):
            M_W_dict[j].append(i)
    if case==13:   # + greed_cov 9 and 10
        need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
        la_t=cu_t
        P2,cu_t,w_id=Fast_Cov(need,rest,to_w_seq,Wap_set,ma_set,D,task_dic,ins_m_dic,w_id,cu_t,T_tm,Cu_Wap_ty) 
        P=P+P2
        #print(f"here is Cov case {case}")
        TL=Get_TL([Wap_dic.get(i) for i in P],D,la_t)
        ma_set=Update_M_TL(ma_set, TL, la_t, D, task_dic, ins_m_dic,T_tm)
    if case==1 or case==4 or case==9 or case==10:   # + greed_cov 9 and 10
        need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
        la_t=cu_t
        P2,cu_t,w_id=Greedy_Cov(need,rest,to_w_seq,Wap_set,ma_set,D,task_dic,ins_m_dic,w_id,cu_t,T_tm) 
        P=P+P2
        #print(f"here is Cov case {case}")
        TL=Get_TL([Wap_dic.get(i) for i in P],D,la_t)
        ma_set=Update_M_TL(ma_set, TL, la_t, D, task_dic, ins_m_dic,T_tm)
        #print("already cover all",min([min(list(ma_set.values())[i].ac_re) for i in range(len(ma_set))]))
    de=[]
    #print(f"here is the case enter at {cu_t}")
    while cu_t<T_tm:
        #print(f"enter time is {cu_t}\n")
        #m_set=copy.deepcopy(ma_set)
        for i in list(seq_set):
            d=D[w_id][Wap_set[i].id]
            if d+cu_t>T_tm:
                de.append(i)
        seq_set=seq_set-set(de)
        if len(seq_set)==0:
            break
            # This one is good. 
        if case==0:
            V=[Get_Re_Dy(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_total) for i in list(seq_set)]
            opt=max(V, key=lambda item: item[0])[1]
            if max(V, key=lambda item: item[0])[0]==0:
                break
        else: # here change T_tm to T_total
            if case==7 or case==8 or case==9 or case==10 or case==13:
                ac_w={}
                for m in ma_set.values():
                   # task_set=m.task
                    si=np.array([task_dic.get(ta)[0] for ta in m.task])
                    si_c=[s*co_s if s>1 else s for s in si ]
                    ac_w[m.id]=list((np.array( m.ac_re )/(si_c)))
                min_re=min([min(a) for a in ac_w.values()])  # weighted
                count=sum([a.count(min_re) for a in ac_w.values() ]) # weighted 
                min_set=[i for i in list(ac_w.keys()) if min_re in ac_w.get(i)] #weighted
            else:
                ac_w={}
                for m in ma_set.values():
                    ac_w[m.id]=list( m.ac_re )
                min_re=min([min(a) for a in ac_w.values()])
                count=sum([a.count(min_re) for a in ac_w.values() ]) # weighted 
                min_set=[i for i in list(ac_w.keys()) if min_re in ac_w.get(i)] #weighted
            #print(f" see the value {min_set} {ac_w} {min_re}")
            V=[Get_Re_Min(case,ac_w,min_re,count,min_set,Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_total,co_s) for i in list(seq_set)]
            #V=[Get_Re_Min(case,Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_total,co_s) for i in list(seq_set)]
            if max(V, key=lambda item: item[0])[0]==0:
                break
            if case==6 or case==7 or case==9:
                max_grain=max([V[i][0] for i in range(len(V))])
                opt_set=[V[i] for i in range(len(V)) if V[i][0]==max_grain]
                opt=opt_set[0][3]
            elif case==5:   # case==8, case==10
                max_min=max([V[i][1] for i in range(len(V))]) 
                if max_min==0:
                    max_count=max([V[i][2] for i in range(len(V))])
                    if max_count==0:
                        opt=max(V, key=lambda item: item[0])[3]
                    else: 
                        opt_set=[V[i] for i in range(len(V)) if V[i][2]>0]
                        opt=max(opt_set, key=lambda item: item[0])[3]
                else:
                    opt_set=[V[i] for i in range(len(V)) if V[i][1]==max_min]
                    opt=max(opt_set, key=lambda item: item[0])[3]
            else:
                #############################################
                max_min=max([V[i][1] for i in range(len(V))])
                if max_min==0:
                    max_count=max([V[i][2] for i in range(len(V))])
                    if max_count==0:
                        opt=max(V, key=lambda item: item[0])[3]
                    else: 
                        opt_set=[V[i] for i in range(len(V)) if V[i][2]==max_count]
                        opt=max(opt_set, key=lambda item: item[0])[3]
                else:
                    opt_set=[V[i] for i in range(len(V)) if V[i][1]==max_min]
                    opt=max(opt_set, key=lambda item: item[0])[3]
        ma_set,cu_t=M_update_Min(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm) # current waypoint w_id 
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
        if cu_t<=300:
            f=1
        else:
            aa=random.random()
            if aa>0.7:
                f=1
            else:
                f=0 
        num=f
        id_set=[Wap_set[num][i].id for i in range(len(Wap_set[num]))]
        while True:
            if len(id_set)>0:
                Dis=sorted([(D[w_id][w],w) for w in id_set])
                w_c=Dis[0][1]
                id_set.remove(w_c)
                TSP.append(w_c)
                w_id=w_c
            else:
                if num==f: 
                    if f==1:
                        num=0 
                    else:
                        num=1
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
####################  8_9 ----change T_tm to T_total
def Get_Re_Min(case, ac_w_in,min_re, count,min_set,wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_tm,co_s):
    if cu_t+D[w_id][wap.id]>T_tm:
            return 0,0,0,wap
    T_rest=T_tm-(cu_t+D[w_id][wap.id])
    if T_rest<0:
        return 0,0,0,wap
    ac_w=copy.deepcopy(ac_w_in)
    if case==7 or case==8 or case==9 or case==10 or case==13:
        cover=wap.cover
        if set(cover)&set(min_set)==set():
            return 0,0,0,wap
        Grain=0
        for i in cover:
            task_set=m_set[i].task
            ini_va=m_set[i].va  # inital value of all tasks 
            lt_set=m_set[i].la_t  # last visit time of all tasks 
            mac_re=m_set[i].ac_re
            for j in range(len(task_set)):
                [si,dy]=task_dic.get(task_set[j])
                get_t=cu_t+D[w_id][wap.id]
                la_t=max(0,lt_set[j])
                acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
                tmp= Reliable(ini_va[j],la_t,get_t, 0, dy)    
                if si>1:
                        si=si*co_s
                if T_rest==0:
                    Grain=Grain+(max(tmp,acc)-tmp)*(si)
                    ac_w.get(i)[j]=ac_w.get(i)[j]+(max(tmp,acc)-tmp)/(si)
                    #m_new_set[i].ac_re[j]=mac_re[j]+(max(tmp,acc)-tmp)    
                else:
                    increase,c,d=Integral_grain(ini_va[j], acc, dy, get_t, T_tm,la_t)
                    Grain=Grain+increase*(si)
                    ac_w.get(i)[j]=ac_w.get(i)[j]+increase/(si)
        if case!=7:       
            min_w_new=min([min(a) for a in ac_w.values()] )               
            min_w_count=sum([a.count(min_w_new) for a in ac_w.values()])
        else:
            min_w_new=0
            min_w_count=0        
        #print(f"check why 319 {wap.id} {wap.cover}  {Grain} {(D[w_id][wap.id])} {min_set} {min_re} ") 
        return Grain/(D[w_id][wap.id]),(min_w_new-min_re)/(D[w_id][wap.id]),(count-min_w_count)/(D[w_id][wap.id]),wap
    else:
        cover=wap.cover
        if set(cover)&set(min_set)==set():
            return 0,0,0,wap
        #####################################################
        Grain=0
        for i in cover:
            task_set=m_set[i].task
            ini_va=m_set[i].va  # inital value of all tasks 
            lt_set=m_set[i].la_t  # last visit time of all tasks 
            for j in range(len(task_set)):
                [si,dy]=task_dic.get(task_set[j])
                get_t=cu_t+D[w_id][wap.id]
                la_t=max(0,lt_set[j])       
                acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
                tmp= Reliable(ini_va[j],la_t,get_t, 0, dy)     
                if T_rest==0:
                    Grain=Grain+(max(tmp,acc)-tmp)*si
                    ac_w.get(i)[j]=ac_w.get(i)[j]+(max(tmp,acc)-tmp)
                else:
                    increase,c,d=Integral_grain(ini_va[j], acc, dy, get_t, T_tm,la_t)
                    Grain=Grain+increase*si
                    ac_w.get(i)[j]=ac_w.get(i)[j]+increase
        if case!=6:
            min_new=min([min(a) for a in ac_w.values()] )               
            min_count=sum([a.count(min_new) for a in ac_w.values()])
        else:
            min_new=0
            min_count=0
        if case==2 or case==4 or case==6:
            return Grain/(D[w_id][wap.id]),(min_new-min_re)/(D[w_id][wap.id]),(count-min_count)/(D[w_id][wap.id]),wap
        if case==1 or case==5:
            return 1/(D[w_id][wap.id]),(min_new-min_re),(count-min_count),wap
################################## D####################
def Fast_Cov(need,rest,to_w_seq,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,Cu_Wap_ty):
    #t=time.time()
   # Wap_set2=[]
    cu_t=T_in
    dic_wap={}
    print(len(Cu_Wap_ty))
    seq=0
    a=[];b=[]
    for i in Cu_Wap_ty:
        a=a+[i[k].id for k in range(len(i))] 
        b=b+[seq]*len(i)
        seq=seq+1
    dic_wap=dict(zip(a,b))
    w_id=[Wap_set[i].id for i in to_w_seq]
    coarse=[i for i in w_id if dic_wap.get(i)==1]
    fine=[i for i in w_id if dic_wap.get(i)==0]
    w_id=w_in
    P=[];tsp=[]
    num=1
    while True:
        if len(coarse)>0:
            Dis=sorted([(D[w_id][w],w) for w in coarse])
            w_c=Dis[0][1]
            coarse.remove(w_c)
            tsp.append(w_c)
            w_id=w_c
        else:
            if num==1: num=0 
            else: break
            coarse=fine
    seq=0; w_id=w_in
    while cu_t<T_tm:
        if seq<len(tsp):
            cu_t=cu_t+D[w_id][tsp[seq]]
            if cu_t>=T_tm:
                cu_t=cu_t-D[w_id][tsp[seq]]
                break
            P.append(tsp[seq])
            w_id=tsp[seq]
            seq=seq+1; 
        else:
            break
    
    #print(f"see the running time {time.time()-t}")
    return P,cu_t,w_id
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


############### finish 7 _27
def Update_M_TL(M_set,TL,Ti,D,task_dic,ins_m_dic,T_tm):   # for update m_set given a TL
    #print(f" see new haha to see")
    #TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0   # do not need to change TL.
    for k in list(M_set.keys()):#list(M_set.keys()):  # for all monitoring areas 
#         if TL.get(k)!=None:
#             TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
#             if len(TL[k])>0:
#                 if TL[k][-1][0]<Time: # here is cu_t, must greater than the last one.
#                     TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
#             else:
#                 TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        lt_set=ma_set[k].la_t 
        if TL.get(k)==None:   # if no visited, va, and la_t does not change 
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])
                b=ini_va[j]  
                la_t=max(lt_set[j],0)
                if b>0:
#                     dur=T_tm-Ti 
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=(b+max(0,b-dur*dy))*d_t*0.5
                    #print(f" old b {b}")
                    sum_one,la,last_b=Sump_Integral(b,0,dy,Ti,T_tm,la_t)
                    ma_set[k].ac_re[j]=sum_one
        else:  
            for j in range(len(task_set)):  # for all tasks of a m 
                sum_one=0
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]   
                la_t=max(lt_set[j],0)     
                for i in range(len(TL[k])):
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, Ti, TL[k][i][0],la_t) 
                        sum_one=sum_one+sum_a
                    else:
                        #dur=(TL[k][i][0]-TL[k][i-1][0])
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, TL[k][i-1][0], TL[k][i][0],la_t)
                        sum_one=sum_one+sum_a


#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
#                     b=max(0,b-dur*dy)
#                     b=max(b,acc)
#                 dur=T_tm-Time 
#                 d_t=(dur if dy==0 else min(dur,b/dy))
#                 sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
                ma_set[k].va[j]=b
                ma_set[k].la_t[j]=la_t
                sum_a,a,c=Sump_Integral(b, 0, dy, TL[k][-1][0], T_tm,la_t)
                sum_one=sum_one+sum_a   # plus the re to T_tm
                ma_set[k].ac_re[j]=sum_one
              #  print(f" {ma_set[k].va[j]}, {ma_set[k].la_t[j]} {Time}, {T_tm},{dy},{sum_one}")
    return ma_set 
###### I want to see 7 27 
def M_update_Min(wap,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm):  # to update m_set given a chosen wp
    m_co=copy.deepcopy(ma_set)
    arr_t=cu_t+D[w_id][wap.id]
    #print(f"{wap.cover} at{wap.id} at time{cu_t}")
    #T_rest=T_tm-(cu_t+D[w_id][wap.id])
   # print(f" see {wap.cover}")
    for i in wap.cover:
        task_set=ma_set[i].task
        ini_va=ma_set[i].va  # inital value of all tasks 
        lt_set=ma_set[i].la_t  # last visit time of all tasks 
        mac_re=ma_set[i].ac_re
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            la_t=max(0,lt_set[j])
            increase,la_t,b=Integral_grain(ini_va[j], acc, dy, arr_t, T_tm,la_t)  
            m_co[i].va[j]=b # update the end value 
            m_co[i].la_t[j]=la_t      
            m_co[i].ac_re[j]=mac_re[j]+increase
          #  print(f"{i}, {task_set[j]}, {tmp},{acc}, {arr_t}, {la_t}, {T_tm},{ m_co[i].ac_re[j]} {increase}")
    return m_co,(arr_t) # here is leave time


###################################4——30 finish 7 _27
def Get_Re_One(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    cover=wap.cover
    all_id=list(m_set.keys())
    rest=list(set(all_id)-set(cover))
    Grain=0
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            la_t= max(0,lt_set[j])
            arr_t= cu_t+D[w_id][wap.id]        
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection 
            tmp=Reliable(ini_va[j],la_t,arr_t, 0, dy)
            Grain=Grain+(max(tmp,acc)-tmp)*si   # <------------- here we have significance !!!1! oooo 
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
#             if T_rest==0:
#                 Grain=Grain+(max(tmp,acc)-tmp)*si
            increase,c,d=Integral_grain(ini_va[j], acc, dy, arr_t, T_tm,la_t)
#                 d_t=(T_rest if dy==0 else min(T_rest,tmp/dy))
#                 pre_re=(tmp+max(0,tmp-T_rest*dy))*d_t*0.5
#                 new=max(tmp,acc)
#                 d_t=(T_rest if dy==0 else min(T_rest,new/dy))
#                 new_re=(new+max(0,new-T_rest*dy))*d_t*0.5
            Grain=Grain+(increase)*si
            #print(f"why",wap.ty,task_set[j],Grain)
    #d=D[w_id][wap.id]
    return (Grain/(D[w_id][wap.id])),wap

def Get_by_wait(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_tm):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
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
#             if T_rest==0:
#                 Grain=Grain+(max(tmp,acc)-tmp)*si
            increase,c,d=Waiting_grain(ini_va[j], acc, dy, arr_t, T_tm,la_t,1)
#                 d_t=(T_rest if dy==0 else min(T_rest,tmp/dy))
#                 pre_re=(tmp+max(0,tmp-T_rest*dy))*d_t*0.5
#                 new=max(tmp,acc)
#                 d_t=(T_rest if dy==0 else min(T_rest,new/dy))
#                 new_re=(new+max(0,new-T_rest*dy))*d_t*0.5
            Grain=Grain+(increase)*si
            #print(f"why",wap.ty,task_set[j],Grain)
    #d=D[w_id][wap.id]
    return (Grain/(D[w_id][wap.id])),wap

# Becase this one taks so long time, not use this. 
def Get_Re_TL(P,Wap_set,wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,T_in,T_tm):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    if cu_t+D[w_id][wap.id]>T_tm:
        return 0,wap
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    P_c=copy.deepcopy(P)
    P_c=P_c+[wap.id]
    Tl=Get_TL([Wap_dic.get(i) for i in P_c],D,T_in)
    re,min_re,co_t=Calculate_Acc(m_set,Tl,T_in,T_tm,task_dic,ins_m_dic)
    return re,wap


#######################################7_27################## After change reliability function
def Calculate_Acc2(M_set,Tl,Ti,Time,task_dic,ins_m_dic,high_task):
    High_sum=0
    High_va=[]
    task_va=defaultdict(list)
    target_va=defaultdict(list)
    High_size=0
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    co_t=0
    sum_re=0
    sum_w_re=0
    sum_event=[]
    event_size=sum([len(ma_set[k].task) for k in list(ma_set.keys())])
    M_v={};M_int={}
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                M_v[k]=len(TL[k])
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
            else:
                TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        lt_set=ma_set[k].la_t 
        if TL.get(k)==None:   # if areas did not be covered 
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                la_t=max(lt_set[j],0)
                if b>0:
                    sum_one,la,last_b=Sump_Integral(b,0,dy,Ti,Time,la_t)
                    sum_event.append(sum_one)
                    task_va[si].append(sum_one)
                    target_va[task_set[j]].append(sum_one)
                    print(f"no TL the sum_one {k},{task_set[j]} {sum_one},")
                    sum_re=sum_re+sum_one
                    sum_w_re=sum_w_re+sum_one*si
                    if task_set[j] in high_task:
                        High_sum=High_sum+sum_one
                        High_va.append(sum_one)
                        High_size=High_size+1
                else: 
                    sum_event.append(0)
                    task_va[si].append(0)
                    target_va[task_set[j]].append(0)
                    if task_set[j] in high_task:
                            High_sum=High_sum+0
                            High_va.append(0)
                            High_size=High_size+1
        else:
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]   
                la_t=max(lt_set[j],0)
                sum_one=0 
                for i in range(len(TL[k])):
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, Ti, TL[k][i][0],la_t)
                        sum_one=sum_one+sum_a
                    else:
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, TL[k][i-1][0], TL[k][i][0],la_t)
                        sum_one=sum_one+sum_a
                sum_event.append(sum_one)
                task_va[si].append(sum_one)
                target_va[task_set[j]].append(sum_one)
                sum_re=sum_re+sum_one
                sum_w_re=sum_w_re+sum_one*si
                if task_set[j] in high_task:
                    High_sum=High_sum+sum_one
                    High_va.append(sum_one)
                    High_size=High_size+1
    Sum=round(sum_re/event_size,2)
    w_min_v=0
    w_min_set=[]
    for k, j in task_va.items():
        w_min_v=w_min_v+int(k)*min(j)
        if len(j)>0:
            w_min_set.append((int(k),min(j)))
    w_task_min=0
    see_min=[]
    for k,j in target_va.items():
        [si,dy]=task_dic.get(k)
        if len(j)>0:
            w_task_min=w_task_min+si*min(j)
            if si>0:
                see_min.append(min(j)/si)
    weighted_min=min(see_min)
    if len(High_va)>0:
        min_high=min(High_va)
    else:
        min_high=0
    return Sum,(min(sum_event),-(sum_event.count(min(sum_event)))),event_size, High_sum,High_size,min_high,sum_w_re,w_min_v,w_task_min,weighted_min
###################################

def Sample_Acc2(M_set,Tl,Ti,Time,task_dic,ins_m_dic,high_task,sample_rate,detect_threshold):
    High_va=[]
    Va_set=[]
    detect_ratio=[]
    high_detect=[]
    low_detect=[]
    High_size=0
    target_detect=defaultdict(list)
    target_sample=defaultdict(list)
    detect_av=defaultdict(list)
    sample_av=defaultdict(list)
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    event_size=sum([len(ma_set[k].task) for k in list(ma_set.keys())])
    M_v={};M_int={}
    check_time=list(np.arange(sample_rate,Time,sample_rate))
    c_t=list(map(list,zip(check_time,[0]*len(check_time))))
    Tl_old=copy.deepcopy(TL)
    sampel_high_sum=0
    sample_sum=0
    for k in list(M_set.keys()):  # for all monitoring areas 
        TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
        if len(TL[k])>0:
            M_v[k]=len(TL[k])
            if TL[k][-1][0]<Time:
                TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
        else:
            TL[k]=[[Time,0]]
        All_t=sorted(TL[k]+c_t)
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        lt_set=ma_set[k].la_t 
        if Tl_old.get(k)==None:   # if areas did not be covered 
            for j in range(len(task_set)):  # for all tasks of a m 
                detect=[]
                sample_one=0
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                la_t=max(lt_set[j],0)
                if b>0:
                    for i in range(len(All_t)):
                        acc=ins_m_dic.get(str(int(All_t[i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                        cu_acc,la_t,b=Cur_Integral(b, acc, dy, All_t[i][0],la_t)
                        if i in c_t:
                            detect.append(cu_acc)
                            sample_one=sample_one+cu_acc
                    rat=len([d for d in detect if d >=detect_threshold])/len(c_t)
                    
                    detect_ratio.append(rat)
                    sample_sum=sample_sum+sample_one/len(c_t)
                    Va_set.append(sample_one/len(c_t))
                    target_detect[task_set[j]].append(rat)
                    detect_av[si].append(rat)
                    target_sample[task_set[j]].append(sample_one/len(c_t))
                    sample_av[si].append(sample_one/(len(c_t)))
                    if task_set[j] in high_task:
                        sampel_high_sum=sampel_high_sum+sample_one/len(c_t)
                        High_va.append(sample_one/len(c_t))
                        High_size=High_size+1
                        high_detect.append(rat)
                    else:
                        low_detect.append(rat)
                else: 
                    detect_ratio.append(0)
                    Va_set.append(0)
                    target_detect[task_set[j]].append(0)
                    detect_av[si].append(0)
                    target_sample[task_set[j]].append(0)
                    sample_av[si].append(0)
                    if task_set[j] in high_task:
                        High_size=High_size+1
                        High_va.append(0)
                        high_detect.append(0)
                    else:
                        low_detect.append(0)
        else:
            for j in range(len(task_set)):  # for all tasks of a m 
                detect=[]
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]   
                la_t=max(lt_set[j],0)
                sample_one=0
                for i in range(len(All_t)):
                    acc=ins_m_dic.get(str(int(All_t[i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    cu_acc,la_t,b=Cur_Integral(b, acc, dy, All_t[i][0],la_t)
                    if All_t[i] in c_t:
                        detect.append(cu_acc)
                        sample_one=sample_one+cu_acc
                sample_sum=sample_sum+sample_one/len(c_t)
                Va_set.append(sample_one/len(c_t))
                rat=len([d for d in detect if d >=detect_threshold])/len(c_t)
                detect_ratio.append(rat)
                target_detect[task_set[j]].append(rat)
                detect_av[si].append(rat)
                target_sample[task_set[j]].append(sample_one/len(c_t))
                sample_av[si].append(sample_one/(len(c_t)))
                if task_set[j] in high_task:
                    sampel_high_sum=sampel_high_sum+sample_one/len(c_t)
                    High_size=High_size+1
                    High_va.append(sample_one/len(c_t))
                    high_detect.append(rat)
                else:
                    low_detect.append(rat)
    min_high=min(High_va)
    min_ac=min(Va_set)
    w_detect_min=0
    sum_w_detect=0
    sum_w_sample=0
    detect_set=[]
    sample_set=[]
    for k,j in detect_av.items():
        if len(j)>0:
            sum_w_detect=sum_w_detect+int(k)*min(j)
            detect_set.append(min(j)/int(k))
    min_detect=min(detect_set)
    for k,j in sample_av.items():
        if len(j)>0:
            sum_w_sample=sum_w_sample+int(k)*min(j)
            sample_set.append(min(j)/int(k))
    min_sample=min(sample_set)
    for k,j in target_detect.items():
        [si,dy]=task_dic.get(k)
        if len(j)>0:
            w_detect_min=w_detect_min+si*min(j)
    w_sample_min=0
    for k,j in target_sample.items():
        [si,dy]=task_dic.get(k)
        if len(j)>0:
            w_sample_min=w_sample_min+si*min(j)
    
    #print(f"see high {High_va}")
    #print(f"see ratio { detect_ratio}")
#     print(f"see high ratio {np.mean(high_detect)} {high_detect}")
#     print(f"see low ratio {np.mean(low_detect)} {low_detect}")
#     print(f"see result: ", sample_sum/event_size,sampel_high_sum/High_size,min_high,min_ac)
    #print(f"see sample",sample_sum/event_size,sampel_high_sum/High_size,min_high,min_ac,np.mean(low_detect),np.mean(high_detect))
    return sample_sum/event_size,sampel_high_sum/High_size,min_high,min_ac,np.mean(low_detect),np.mean(high_detect),np.min(low_detect),np.min(high_detect),w_sample_min,w_detect_min,High_va,Va_set,sum_w_detect,sum_w_sample,min_detect,min_sample

def Calculate_Acc(M_set,Tl,Ti,Time,task_dic,ins_m_dic,high_task):
    High_sum=0
    High_va=[]
    task_va=defaultdict(list)
    target_va=defaultdict(list)
    High_size=0
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    co_t=0
    sum_re=0
    sum_w_re=0
    sum_event=[]
    event_size=sum([len(ma_set[k].task) for k in list(ma_set.keys())])
    M_v={};M_int={}
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                M_v[k]=len(TL[k])
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
            else:
                TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        lt_set=ma_set[k].la_t 
        if TL.get(k)==None:   # if areas did not be covered 
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                la_t=max(lt_set[j],0)
                if b>0:
                    sum_one,la,last_b=Sump_Integral(b,0,dy,Ti,Time,la_t)
                    sum_event.append(sum_one)
                    task_va[si].append(sum_one)
                    target_va[task_set[j]].append(sum_one)
                    print(f"no TL the sum_one {k},{task_set[j]} {sum_one},")
                    sum_re=sum_re+sum_one
                    sum_w_re=sum_w_re+sum_one*si
                    if task_set[j] in high_task:
                        High_sum=High_sum+sum_one
                        High_va.append(sum_one)
                        High_size=High_size+1
                else: 
                    sum_event.append(0)
                    task_va[si].append(0)
                    target_va[task_set[j]].append(0)
                    if task_set[j] in high_task:
                            High_sum=High_sum+0
                            High_va.append(0)
                            High_size=High_size+1
        else:
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]   
                la_t=max(lt_set[j],0)
                sum_one=0 
                for i in range(len(TL[k])):
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, Ti, TL[k][i][0],la_t)
                        sum_one=sum_one+sum_a
                    else:
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, TL[k][i-1][0], TL[k][i][0],la_t)
                        sum_one=sum_one+sum_a
                sum_event.append(sum_one)
                task_va[si].append(sum_one)
                target_va[task_set[j]].append(sum_one)
                sum_re=sum_re+sum_one
                sum_w_re=sum_w_re+sum_one*si
                if task_set[j] in high_task:
                    High_sum=High_sum+sum_one
                    High_va.append(sum_one)
                    High_size=High_size+1
    Sum=round(sum_re/event_size,2)
    w_min_v=0
    w_min_set=[]
    
    for k, j in task_va.items():
        w_min_v=w_min_v+int(k)*min(j)
        if len(j)>0:
            w_min_set.append((int(k),min(j)))
    w_task_min=0
    see_min=[]
    for k,j in target_va.items():
        [si,dy]=task_dic.get(k)
        if len(j)>0:
            w_task_min=w_task_min+si*min(j)
            if si>0:
                see_min.append(min(j)/float(si))
    weighted_min=min(see_min)
    if len(High_va)>0:
        min_high=min(High_va)
    else:
        min_high=0
    return Sum,(min(sum_event),-(sum_event.count(min(sum_event)))),event_size, High_sum,High_size,min_high,sum_w_re,w_min_v,w_task_min
###################################
def Sample_Acc(M_set,Tl,Ti,Time,task_dic,ins_m_dic,high_task,sample_rate,detect_threshold):
    High_va=[]
    Va_set=[]
    detect_ratio=[]
    high_detect=[]
    low_detect=[]
    High_size=0
    target_detect=defaultdict(list)
    target_sample=defaultdict(list)
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    event_size=sum([len(ma_set[k].task) for k in list(ma_set.keys())])
    M_v={};M_int={}
    check_time=list(np.arange(sample_rate,Time,sample_rate))
    c_t=list(map(list,zip(check_time,[0]*len(check_time))))
    Tl_old=copy.deepcopy(TL)
    sampel_high_sum=0
    sample_sum=0
    for k in list(M_set.keys()):  # for all monitoring areas 
        TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
        if len(TL[k])>0:
            M_v[k]=len(TL[k])
            if TL[k][-1][0]<Time:
                TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
        else:
            TL[k]=[[Time,0]]
    ######### Get  the initial TL, shrink it into [0 , Time]
        All_t=sorted(TL[k]+c_t)
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # 
        lt_set=ma_set[k].la_t 
        if Tl_old.get(k)==None:   # if areas did not be covered 
            for j in range(len(task_set)):  # for all tasks of a m 
                detect=[]
                sample_one=0
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                la_t=max(lt_set[j],0)
                if b>0:
                    for i in range(len(All_t)):
                        acc=ins_m_dic.get(str(int(All_t[i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                        cu_acc,la_t,b=Cur_Integral(b, acc, dy, All_t[i][0],la_t)
                        if i in c_t:
                            detect.append(cu_acc)
                            sample_one=sample_one+cu_acc
                    rat=len([d for d in detect if d >=detect_threshold])/len(c_t)
                    detect_ratio.append(rat)
                    sample_sum=sample_sum+sample_one/len(c_t)
                    Va_set.append(sample_one/len(c_t))
                    target_detect[task_set[j]].append(rat)
                    target_sample[task_set[j]].append(sample_one/len(c_t))
                    if task_set[j] in high_task:
                        sampel_high_sum=sampel_high_sum+sample_one/len(c_t)
                        High_va.append(sample_one/len(c_t))
                        High_size=High_size+1
                        high_detect.append(rat)
                    else:
                        low_detect.append(rat)
                else: 
                    detect_ratio.append(0)
                    Va_set.append(0)
                    target_detect[task_set[j]].append(0)
                    target_sample[task_set[j]].append(0)
                    if task_set[j] in high_task:
                        High_size=High_size+1
                        High_va.append(0)
                        high_detect.append(0)
                    else:
                        low_detect.append(0)
        else:
            for j in range(len(task_set)):  # for all tasks of a m 
                detect=[]
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]   
                la_t=max(lt_set[j],0)
                sample_one=0
                for i in range(len(All_t)):
                    acc=ins_m_dic.get(str(int(All_t[i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    cu_acc,la_t,b=Cur_Integral(b, acc, dy, All_t[i][0],la_t)
                    if All_t[i] in c_t:
                        detect.append(cu_acc)
                        sample_one=sample_one+cu_acc
                sample_sum=sample_sum+sample_one/len(c_t)
                Va_set.append(sample_one/len(c_t))
                rat=len([d for d in detect if d >=detect_threshold])/len(c_t)
                print(f"see if it is fraction",len([d for d in detect if d >=detect_threshold]),len(c_t),rat)
                detect_ratio.append(rat)
                target_detect[task_set[j]].append(rat)
                target_sample[task_set[j]].append(sample_one/len(c_t))
                if task_set[j] in high_task:
                    sampel_high_sum=sampel_high_sum+sample_one/len(c_t)
                    High_size=High_size+1
                    High_va.append(sample_one/len(c_t))
                    high_detect.append(rat)
                else:
                    low_detect.append(rat)
    min_high=min(High_va)
    min_ac=min(Va_set)
    print(target_detect.items,target_sample.items)
    w_detect_min=0
    for k,j in target_detect.items():
        [si,dy]=task_dic.get(k)
        if len(j)>0:
            w_detect_min=w_detect_min+si*min(j)
    w_sample_min=0
    for k,j in target_sample.items():
        [si,dy]=task_dic.get(k)
        if len(j)>0:
            w_sample_min=w_sample_min+si*min(j)
    
    #print(f"see high {High_va}")
    #print(f"see ratio { detect_ratio}")
#     print(f"see high ratio {np.mean(high_detect)} {high_detect}")
#     print(f"see low ratio {np.mean(low_detect)} {low_detect}")
#     print(f"see result: ", sample_sum/event_size,sampel_high_sum/High_size,min_high,min_ac)
    #print(f"see sample",sample_sum/event_size,sampel_high_sum/High_size,min_high,min_ac,np.mean(low_detect),np.mean(high_detect))
    return sample_sum/event_size,sampel_high_sum/High_size,min_high,min_ac,np.mean(low_detect),np.mean(high_detect),np.min(low_detect),np.min(high_detect),w_sample_min,w_detect_min,High_va,Va_set

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

