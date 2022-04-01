import copy
import numpy as np 
import itertools
from collections import defaultdict # Use defaultdict for prefix and query
from Wp_sel import Get_TL
import random 
# def Heuristic_WpSh(Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To):
#     wp=[Wap_set[i].cover for i in range(len(Wap_set))]
#     cu_t=T_in
#     w_id=w_in
#     ma_set=copy.deepcopy(M_set)
#     P=[w_id]
#     print("here I realdy put ",P)
#     de=[]
#     while cu_t<T_tm:
#         for i in range(len(Wap_set)):
#             d=D[w_id][Wap_set[i].id]+To
#             if d+cu_t>T_tm:
#                 de.append(i)
#         sa=set(range(len(Wap_set)))-set(de)
#         if len(sa)==0:
#             print("stop here ",V,cu_t,de,sa)
#             return P
#         #V=[Get_loss(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
#         V=[Get_Grain(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in list(sa)]
#         #V=[Get_voi(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
#         opt=max(V, key=lambda item: item[0])[1]
#         ma_set,cu_t=M_update(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) # current waypoint w_id 
#         P.append(opt.id)
#         w_id=opt.id
#     return P
######################################################
def Greedy_WPS(case,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To):
    seq_set=set(range(len(Wap_set)-1))
#     for i in range(len(Wap_set)):
#         print("Wap",Wap_set[i].id)
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
    if case!=0:
        need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
    
        #if len(need)>0: 
        #print("first round to cover all NEED",need)
        la_t=cu_t
        P2,cu_t,w_id=Greedy_Cov(need,rest,to_w_seq,Wap_set,ma_set,D,task_dic,ins_m_dic,w_id,cu_t,T_tm,To) 
        P=P+P2
        #print("at time", cu_t,"HERE IS",P2)
        TL=Get_TL([Wap_dic.get(i) for i in P],D,To,la_t)
        #print(sorted(TL.items()))
        ma_set=Update_M_TL(ma_set, TL, la_t, cu_t, D, task_dic, ins_m_dic, To)
#         for i, m in ma_set.items(): 
#             print("to see after one round",m.__dict__)
        cu_t=cu_t+To
#             need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
    de=[]
    #print("start second phase at",cu_t)
    while cu_t<T_tm:
        for i in list(seq_set):
            d=D[w_id][Wap_set[i].id]+To
            if d+cu_t>T_tm:
                de.append(i)
        seq_set=seq_set-set(de)
        #print("rest of id",seq_set,cu_t)
        if len(seq_set)==0:
            #print("stop here ",cu_t,de,seq_set)
            break
        #V=[Get_loss(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        V=[Get_Re_One(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in list(seq_set)]
        #print("try get a V",V)
        #V=[Get_voi(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        opt=max(V, key=lambda item: item[0])[1]
        if max(V, key=lambda item: item[0])[0]==0:
            #print("no need to fly")
            break
        ma_set,cu_t=M_update_Re(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) # current waypoint w_id 
        P.append(opt.id)
        w_id=opt.id
    return P,cu_t

#             TL=Get_TL([Wap_set[i] for i in Seq],D,To,T_in)
#             m2_set=Update_M_TL(M_set, TL, T_in, cu_t, D, task_dic, ins_m_dic
    
    ####################################### Here I will add coverage-driven method 
    de=[]
    while cu_t<T_tm:
        de=[]
        for i in list(seq_set):
            d=D[w_id][Wap_set[i].id]+To
            if d+cu_t>T_tm:
                de.append(i)
        seq_set=seq_set-set(de)
        #print("rest of id",id_set,cu_t)
        if len(seq_set)==0:
            print("stop here ",cu_t,seq_set)
            return P
        #V=[Get_loss(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        V=[Get_Re_One(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in list(seq_set)]
        #V=[Get_voi(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        opt=max(V, key=lambda item: item[0])[1]
        ma_set,cu_t=M_update_Re(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) # current waypoint w_id 
        P.append(opt.id)
        w_id=opt.id
    
    return P
#################################Tabu search 
def Tabu(P_in,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To,NeigSize,MaxTabuSize,stopTurn):
    max_fitness=fitness(P_in,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To)
    sBest =P_in
    vBest=max_fitness
    bestCandidate = P_in
    tabuList = []
    tabuList.append(P_in)
    stop = False
    best_keep_turn = 0
    bestfitness=0
    #start_time=time.time()
    N_dict=Get_adj(Wap_set,D,max_d=10)
    print(N_dict)
    while not stop:
        sNeighborhood = getNeighbors(bestCandidate,Wap_set,NeigSize,N_dict)
        bestCandidate = sNeighborhood[0]
        for sCandidate in sNeighborhood:
            if (sCandidate not in tabuList) and ((fitness(sCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To) > fitness(bestCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To))):
                bestCandidate = sCandidate
                bestfitness=fitness(sCandidate, Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To)
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

def fitness(P,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To):
    Wap_dic=dict(zip([Wap_set[i].id for i in range(len(Wap_set))],[Wap_set[i] for i in range(len(Wap_set))]))
    Tl=Get_TL([Wap_dic.get(i) for i in P],D,To,T_in)
    re,miss=Calculate_Re(M_set,Tl,T_in,T_tm,task_dic,ins_m_dic,To)
    return re
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
    
def Calculate_Re(M_set,Tl,Ti,Time,task_dic,ins_m_dic,To):
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0
    Sum_miss=0
    co_t=0
    sum_re=0
    sum_event=[]
    event_size=sum([len(ma_set[k].task) for k in range(len(ma_set))])
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
            TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                if TL[k][-1][0]<Time:
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
            else:
                TL[k]=[[Time,0]]
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        sum_miss=0
        if TL.get(k)==None:
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                if b>0:
                    dur=Time-Ti 
                    if b-dur*dy<0:
                        sum_re=sum_re+(b/dy)*b*0.5
                        sum_event.append((b/dy)*b*0.5)
                    else:
                        sum_re=sum_re+(b-dur*dy*0.5)*dur
                        sum_event.append((b-dur*dy*0.5)*dur)
                else: 
                    sum_event.append(0)
        else:
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                sum_one=0
                for i in range(len(TL[k])):
                    if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                        dur=TL[k][i][0]-Ti    
                    else:
                        dur=(TL[k][i][0]-TL[k][i-1][0])
                    if b-(dur*dy)<0:
                        sum_re=sum_re+((b/dy)*b*0.5)
                        sum_one=sum_one+((b/dy)*b*0.5)
                        co_t=co_t+(b/dy)
                        b=0
                        if i!=0:
                            sum_miss=sum_miss+(b/dy-dur)
                    else:
                        sum_re=sum_re+(b-dur*dy*0.5)*dur
                        sum_one=sum_one+(b-dur*dy*0.5)*dur
                        if b>0:
                            co_t=co_t+dur
                        b=b-dur*dy
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    b=max(b,acc)
                #print(f"seeeee{k},{j},{sum_one},{TL[k]}")
                sum_event.append(sum_one)
                ma_set[k].va[j]=b
                #print(f"cover time of task {k}, {j}, {sum_re}, {co_t}")               
            #Sum=Sum+(sum_re/len(task_set))
            #Sum=Sum+sum_re
            #print(f"co_t of a area {k} out of {Time}",co_t)
            Sum_miss=Sum_miss+(sum_miss/len(task_set))
    #Sum=round(Sum/(len(M_set.keys())),2)
    #print()
    #Sum=round(sum_re/(Time-Ti),2)
    Sum=round(sum_re/event_size,2)
    #print("here",Sum,sum_re,co_t,co_t/event_size)
    print(f"all events{min(sum_event)}")#{len(sum_event)}, {sum_event}")
    Sum_miss=round(Sum_miss/len(M_set.keys()),2)
    return Sum,Sum_miss


################################## D####################
def Greedy_Cov(need,rest,to_w_seq,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To):
    P=[]
    seq_set=to_w_seq
    cu_t=T_in
    to_cov=set(need.keys())
    #w_co={};ty={}
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
            Prof[i]=(prof/((D[w_id][Wap_set[i].id]+To)),prof,D[w_id][Wap_set[i].id]+To)
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
        cu_t=cu_t+D[w_id][Wap_set[choice].id]+To
        w_id=Wap_set[choice].id
    #print("how much left",to_cov,rest)
    return P,cu_t,w_id

def Check_EM_M(ma_set,M_W_dict):
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
    #print("TO_PASS",to_w_seq)
    return need,rest,to_w_seq
    
def Update_M_TL(M_set,Tl,Ti,Time,D,task_dic,ins_m_dic,To):
    TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
    ma_set=copy.deepcopy(M_set)
    Sum=0   # do not need to change TL.
    #see=[]
    for k in list(M_set.keys()):  # for all monitoring areas 
        if TL.get(k)!=None:
           # TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
            if len(TL[k])>0:
                if TL[k][-1][0]<Time: # here is cu_t, must greater than the last one.
                    TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
            else:
                TL[k]=[[Time,0]]
        else:
            TL[k]=[[Time,0]]
#        print("see TL",TL[k],k)
        task_set=ma_set[k].task
        ini_va=ma_set[k].va  # inital value of all tasks 
        ma_set[k].la_t=np.ones(len(ini_va))*Time  ###### all get the value at Time 
        for j in range(len(task_set)):  # for all tasks of a m 
            [si,dy]=task_dic.get(task_set[j])
            sum_re=0
            b=ini_va[j]  
            for i in range(len(TL[k])):
                if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                    dur=TL[k][i][0]-Ti   
                else:
                    dur=(TL[k][i][0]-TL[k][i-1][0])
                if b-(dur*dy)<0:
                    #b=(b/dy)-dur   #here 
                    b=0
#                    print("why b so big",b,a,tm,dur)
                else:
                    b=b-dur*dy
#                acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                if i<len(TL[k])-1:
#                   print("see if j work",i)
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    b=max(b,acc)
                #print("see if less than 0",b)
            ma_set[k].va[j]=b
            #see.append(b)
        #print(sum(see))
    return ma_set 
# here is  a new update function for my previous 
def M_update_Re(wap,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To):
    m_co=copy.deepcopy(ma_set)
    arr_t=cu_t+D[w_id][wap.id]
    for i in wap.cover:
        
        task_set=ma_set[i].task
        ini_va=ma_set[i].va  # inital value of all tasks 
        lt_set=ma_set[i].la_t  # last visit time of all tasks 
        #print("start",ma_set[i].va)
        m_co[i].la_t=(arr_t)*np.ones(len(lt_set))      
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            dur=(arr_t-max(0,lt_set[j]))  # increase duration 
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            if ini_va[j]-(dur*dy)<0:
                #tmp=(ini_va[j]/dy)-dur
                tmp=0        
            else:
                tmp=ini_va[j]-dur*dy   
            #print("deduced to",tmp)  
            m_co[i].va[j]=max(tmp,acc) # update the end value 
        #print("end at", m_co[i].va,acc)
    #print("here I want to check",cu_t,[m_co[i].la_t for i in wap.cover],ini_va[j],dur,dur*si*dy,acc,[m_co[i].va for i in wap.cover])
    return m_co,(arr_t+To) # here is leave time


###################################4——30 
def Get_Re_One(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,To):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    cover=wap.cover
    all_id=list(m_set.keys())
    rest=list(set(all_id)-set(cover))
    Sum=0
#     for i in rest:
#         task_set=m_set[i].task
#         ini_va=m_set[i].va  # inital value of all tasks 
#         lt_set=m_set[i].la_t  # last visit time of all tasks 
#         for j in range(len(task_set)):
#             [si,dy]=task_dic.get(task_set[j])
#             dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])  # already added 
#             t1=cu_t-max(0,lt_set[j])
#             t2=D[w_id][wap.id]+To
#             Sum=Sum+(2*ini_va[j]+si*dy*(2*t1+t2))*t2*0.5
#             dur=cu_t+D[w_id][wap.id]+To-max(0,lt_set[j])
#             #Sum=Sum+(ini_va[j]+dur*si*dy*0.5)*dur
    Grain=0
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])            
            #Sum=Sum+(2*ini_va[j]+si*dy*(2*t1+t2))*t2*0.5
            # dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])# increase duration    Here we add To inadvance !!!!!
            # Sum=Sum+(ini_va[j]+dur*si*dy*0.5)*dur 
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            if ini_va[j]-(dur*dy)<0:
                tmp=(ini_va[j]/dy)-dur
            else:
                tmp=ini_va[j]-dur*dy     
            Grain=Grain+max(tmp,acc)-tmp
            #Grain=Grain+(2*ini_va[j]+(2*dur+To)*si*dy*To*0.5)-end_v
            #Grain=Grain+(ini_va[j]+dur*si*dy)*(acc)
            #Sum=Sum+end_v*To+To*si*dy*To*0.5
            #get_voi=get_voi+end_v 
    d=D[w_id][wap.id]+To
    if Sum==0:
        Sum=0.1
    #print(cu_t,cover,Grain/Sum,Grain,Sum,d,ini_va[j],dur)
    #return (Grain/Sum),wap
    return (Grain/d),wap
################################################### 4——30 


def Get_Re(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,To):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    cover=wap.cover
    all_id=list(m_set.keys())
    rest=list(set(all_id)-set(cover))
    Sum=0
    for i in rest:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])  # already added 
            t1=cu_t-max(0,lt_set[j])
            t2=D[w_id][wap.id]+To
            Sum=Sum+(2*ini_va[j]+si*dy*(2*t1+t2))*t2*0.5
            dur=cu_t+D[w_id][wap.id]+To-max(0,lt_set[j])
            #Sum=Sum+(ini_va[j]+dur*si*dy*0.5)*dur
    Grain=0
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            t1=cu_t-max(0,lt_set[j])
            t2=D[w_id][wap.id]
            
            Sum=Sum+(2*ini_va[j]+si*dy*(2*t1+t2))*t2*0.5
            # dur=cu_t+D[w_id][wap.id]-max(0,lt_set[j])# increase duration    Here we add To inadvance !!!!!
            # Sum=Sum+(ini_va[j]+dur*si*dy*0.5)*dur 
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            end_v=(ini_va[j]+(t1+t2)*si*dy)*(1-acc)
            Grain=Grain+(ini_va[j]+(t1+t2)*si*dy)*acc*To
            #Grain=Grain+(2*ini_va[j]+(2*dur+To)*si*dy*To*0.5)-end_v
            #Grain=Grain+(ini_va[j]+dur*si*dy)*(acc)
            Sum=Sum+end_v*To+To*si*dy*To*0.5
            #get_voi=get_voi+end_v 
    d=D[w_id][wap.id]+To
    if Sum==0:
        Sum=0.1
    #print(cu_t,cover,Grain/Sum,Grain,Sum,d,ini_va[j],dur)
    #return (Grain/Sum),wap
    return (Grain/d),wap

#####################################################################################

def Get_loss(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,To):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    cover=wap.cover
    #get_voi=0
    cur_t=cu_t+D[w_id][wap.id]
    all_id=[m_set[i].id for i in range(len(m_set))]
    rest=list(set(all_id)-set(cover))
    Sum=0
    for i in rest:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            t1=cu_t-max(0,lt_set[j])
            t2=D[w_id][wap.id]+To
            Sum=Sum+(2*ini_va[j]+si*dy*(2*t1+t2))*t2*0.5
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            t1=cu_t-max(0,lt_set[j])
            t2=D[w_id][wap.id]
            loss=(2*ini_va[j]+si*dy*(2*t1+D[w_id][wap.id]))*t2*0.5
            dur=(cur_t-max(0,lt_set[j]))  # increase duration    Here we add To inadvance !!!!!
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            end_v=(ini_va[j]+dur*si*dy)*(acc)
            Sum=Sum+loss+end_v*To
            #get_voi=get_voi+end_v 
    d=D[w_id][wap.id]
    if d==0:
        d=0.1
    return(round(-Sum,2),d,round(-Sum,2))
def Get_voi(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,To):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    cover=wap.cover
    get_voi=0
    cur_t=cu_t+D[w_id][wap.id]
    for i in cover:
        task_set=m_set[i].task
        ini_va=m_set[i].va  # inital value of all tasks 
        lt_set=m_set[i].la_t  # last visit time of all tasks 
        #print(lt_set)
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            dur=(cur_t-max(0,lt_set[j]))  # increase duration    Here we add To inadvance !!!!!
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            end_v=(ini_va[j]+dur*si*dy)*(acc)
            get_voi=get_voi+end_v 
    d=D[w_id][wap.id]
    if d==0:
        d=0.1
    return(round(get_voi/(d),2),d,round(get_voi,2))
# def Calculate_Re2(M_set,Tl,Ti,Time,task_dic,ins_m_dic,To):
#     TL=copy.deepcopy(Tl)   # T is the coverge time list, there is no waypoints issue. 
#     ma_set=copy.deepcopy(M_set)
#     Sum=0
#     Sum_miss=0
#     for k in list(M_set.keys()):  # for all monitoring areas 
#         if TL.get(k)!=None:
#             TL[k]=[i for i in TL[k] if Time>=i[0] >=Ti]
#             if len(TL[k])>0:
#                 if TL[k][-1][0]<Time:
#                     TL[k]=TL[k]+[[Time,0]]   # Here add terminal time 
#             else:
#                 TL[k]=[[Time,0]]
#         else:
#             TL[k]=[[Time,0]]
#         task_set=ma_set[k].task
#         ini_va=ma_set[k].va  # inital value of all tasks 
#         sum_re=0
#         sum_miss=0
#         co_t=0
#         if TL.get(k)==None:
#             for j in range(len(task_set)):  # for all tasks of a m 
#                 [si,dy]=task_dic.get(task_set[j])    
#                 b=ini_va[j]  
#                 if b>0:
#                     dur=Time-Ti 
#                     if b-dur*dy<0:
#                         sum_re=sum_re+(b/dy)*b*0.5
#                     else:
#                         sum_re=sum_re+(b-dur*dy*0.5)*dur
#             Sum=Sum+(sum_re/len(task_set))
#             print(f"break {k} get {sum_re},{Sum}")
#         else:
#             for j in range(len(task_set)):  # for all tasks of a m 
#                 [si,dy]=task_dic.get(task_set[j])    
#                 b=ini_va[j]  
#                 for i in range(len(TL[k])):
#                     if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
#                         dur=TL[k][i][0]-Ti    
#                     else:
#                         dur=(TL[k][i][0]-TL[k][i-1][0])
#                     if b-(dur*dy)<0:
#                         sum_re=sum_re+((b/dy)*b*0.5)
#                         co_t=co_t+(b/dy)
#                         #print(i,sum_re,co_t,b,-1)
#                         b=0
#                         if i!=0:
#                             sum_miss=sum_miss+(b/dy-dur)
#                     else:
#                         sum_re=sum_re+(b-dur*dy*0.5)*dur
#                         if b>0:
#                             co_t=co_t+dur
#                         b=b-dur*dy
#                         #print(i,sum_re,co_t,b,1)
#                     acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
#                     b=max(b,acc)
#             ma_set[k].va[j]=b
#             #print(sum_re/Time)               
#         Sum=Sum+(sum_re/len(task_set))
#         #print(f"co_t of a area {k} out of {Time}",co_t/len(task_set))
#         Sum_miss=Sum_miss+(sum_miss/len(task_set))
#     Sum=round(Sum/(len(M_set.keys())),2)
#     Sum_miss=round(Sum_miss/len(M_set.keys()),2)
#     # print([ma_set[i].la_t for i in range(len(ma_set))])
#     # print([ma_set[i].va for i in range(len(ma_set))])
#     return Sum,Sum_miss
