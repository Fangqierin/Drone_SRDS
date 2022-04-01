import copy
import numpy as np 
import itertools
from collections import defaultdict # Use defaultdict for prefix and query
from Wp_sel import Get_TL
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
def Greedy_WPS(Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To):
    seq_set=set(range(len(Wap_set)))
    # I want to get a dict for mapping cover-- id of Wap_set     
    cu_t=T_in
    w_id=w_in
    ma_set=copy.deepcopy(M_set)
    Seq=[0]
    P=[w_id]
    M_W_dict=defaultdict(list)  # Mapping from M to W
    for i in range(len(Wap_set)):
        for j in list(Wap_set[i].cover):
            M_W_dict[j].append(i)
    need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
    
    while cu_t<T_tm:
        if len(need)>0: 
            print("NEED",need)
            la_t=cu_t
            P2,Seq,cu_t,w_id=Greedy_Cov(need,rest,to_w_seq,Wap_set,ma_set,D,task_dic,ins_m_dic,w_id,cu_t,T_tm,To) 
            P=P+P2
            print("at time", cu_t,"HERE IS",P2)
            TL=Get_TL([Wap_set[i] for i in Seq],D,To,la_t)
            print(sorted(TL.items()))
            ma_set=Update_M_TL(ma_set, TL, la_t, cu_t, D, task_dic, ins_m_dic, To)
            for i, m in ma_set.items(): 
                print("haha",m.__dict__)
            need,rest,to_w_seq=Check_EM_M(ma_set, M_W_dict)
        else:
#             TL=Get_TL([Wap_set[i] for i in Seq],D,To,T_in)
#             m2_set=Update_M_TL(M_set, TL, T_in, cu_t, D, task_dic, ins_m_dic, To)
            cu_t=cu_t+1000
            print("Get here")
#             P,Seq,cu_t,w_id=Greedy_Re(Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To)
#         if cu_t>T_tm:
#             break
        
#         print(len(m2_set))
#         for i, m in m2_set.items(): 
#             print(m.__dict__)
#         cu_t=cu_t+10000
    return P,cu_t
    
    ####################################### Here I will add coverage-driven method 
    de=[]
    while cu_t<T_tm:
        for i in list(seq_set):
            d=D[w_id][Wap_set[i].id]+To
            if d+cu_t>T_tm:
                de.append(i)
        seq_set=seq_set-set(de)
        #print("rest of id",id_set,cu_t)
        if len(seq_set)==0:
            print("stop here ",cu_t,de,seq_set)
            return P
        #V=[Get_loss(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        V=[Get_Grain(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in list(seq_set)]
        #V=[Get_voi(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        opt=max(V, key=lambda item: item[0])[1]
        ma_set,cu_t=M_update(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) # current waypoint w_id 
        P.append(opt.id)
        w_id=opt.id
    return P

################################## D####################
def Greedy_Cov(need,rest,to_w_seq,Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To):
    P=[]
    Seq=[]
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
            print("time is out")
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
        Seq.append(choice)
        cu_t=cu_t+D[w_id][Wap_set[choice].id]+To
        w_id=Wap_set[choice].id
    print("how much left",to_cov,rest)
    return P,Seq,cu_t,w_id

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
                    b=(b/dy)-dur
#                    print("why b so big",b,a,tm,dur)
                else:
                    b=b-dur*dy
#                acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                if i<len(TL[k])-1:
#                   print("see if j work",i)
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    b=max(b,acc)
#                print("see if less than 0",b)
            ma_set[k].va[j]=b
        
    return ma_set 
# here is  a new update function for my previous 
def M_update_Re(wap,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To):
    m_co=copy.deepcopy(ma_set)
    arr_t=cu_t+D[w_id][wap.id]
    for i in wap.cover:
        
        task_set=ma_set[i].task
        ini_va=ma_set[i].va  # inital value of all tasks 
        lt_set=ma_set[i].la_t  # last visit time of all tasks 
        print("start",ma_set[i].va)
        m_co[i].la_t=(arr_t)*np.ones(len(lt_set))      
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            dur=(arr_t-max(0,lt_set[j]))  # increase duration 
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            if ini_va[j]-(dur*dy)<0:
                tmp=(ini_va[j]/dy)-dur
            else:
                tmp=ini_va[j]-dur*dy     
            m_co[i].va[j]=max(tmp,acc) # update the end value 
        print("end at", m_co[i].va)
    #print("here I want to check",cu_t,[m_co[i].la_t for i in wap.cover],ini_va[j],dur,dur*si*dy,acc,[m_co[i].va for i in wap.cover])
    return m_co,(arr_t+To) # here is leave time

def M_update(wap,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To):
    m_co=copy.deepcopy(ma_set)
    arr_t=cu_t+D[w_id][wap.id]
    for i in wap.cover:
        task_set=ma_set[i].task
        ini_va=ma_set[i].va  # inital value of all tasks 
        lt_set=ma_set[i].la_t  # last visit time of all tasks 
        m_co[i].la_t=(arr_t)*np.ones(len(lt_set))      
        for j in range(len(task_set)):
            [si,dy]=task_dic.get(task_set[j])
            dur=(arr_t-max(0,lt_set[j]))  # increase duration 
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            end_v=(ini_va[j]+dur*si*dy)*(1-acc)       
            m_co[i].va[j]=end_v # update the end value 
    #print("here I want to check",cu_t,[m_co[i].la_t for i in wap.cover],ini_va[j],dur,dur*si*dy,acc,[m_co[i].va for i in wap.cover])
    return m_co,(arr_t+To) # here is leave time
###################################4——30 
def Get_Re_One(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,To):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
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
################################################### 4——30 














def Get_Grain(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,To):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
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
