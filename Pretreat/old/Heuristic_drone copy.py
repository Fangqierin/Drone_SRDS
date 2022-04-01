import copy
import numpy as np 
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
def Heuristic_WpSh(Wap_set,M_set,D,task_dic,ins_m_dic,w_in,T_in,T_tm,To):
    wp=[Wap_set[i].cover for i in range(len(Wap_set))]
    cu_t=T_in
    w_id=w_in
    ma_set=copy.deepcopy(M_set)
    P=[w_id]
    print("here I realdy put ",P)
    de=[]
    while cu_t<T_tm:
        for i in range(len(Wap_set)):
            d=D[w_id][Wap_set[i].id]+To
            if d+cu_t>T_tm:
                de.append(i)
        sa=set(range(len(Wap_set)))-set(de)
        if len(sa)==0:
            print("stop here ",V,cu_t,de,sa)
            return P
        #V=[Get_loss(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        V=[Get_Grain(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in list(sa)]
        #V=[Get_voi(Wap_set[i],ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) for i in range(len(Wap_set))]
        opt=max(V, key=lambda item: item[0])[1]
        ma_set,cu_t=M_update(opt,ma_set,cu_t,D,w_id,task_dic,ins_m_dic,To) # current waypoint w_id 
        P.append(opt.id)
        w_id=opt.id
    return P
######################################################

def Get_Grain(wap,m_set,cu_t,D,w_id,task_dic,ins_m_dic,To):  # c_wp is current waypoints! here which ma is covered and current time, TL visiting list. 
    cover=wap.cover
    #get_voi=0
    #print(m_set)
    all_id=list(m_set.keys())
    #all_id=[m_set[i].id for i in range(len(m_set))]
    #print(all_id)
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
