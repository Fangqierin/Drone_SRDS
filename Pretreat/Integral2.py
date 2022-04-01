import numpy as np
import copy

def Integral_grain(last_b,acc,dy,t,T,t_la):
    cur_acc=last_b*pow(1-dy,t-t_la)
    if cur_acc< acc:
        if dy==0:
            a=acc-cur_acc
            return a*(T-t),t,acc
        d=1-dy
        a=round(acc*((pow(d,(T-t))-pow(d,(t-t)))/np.log(d)),2)
        b=round(last_b*((pow(d,(T-t_la))-pow(d,(t-t_la)))/np.log(d)),2)
        return a-b,t,acc
    else:
        return 0,t_la,last_b
    
def Sump_Integral(last_b,acc,dy,t,t_tm,t_la):  # return valid last time and last value. 
    
    cur_acc=last_b*pow(1-dy,t_tm-t_la)
    ################################# wait for this !!!!!!!!!!!!!!!
   # print(cur_acc,acc)
    if dy==0:
        a=last_b*(t_tm-t)
    else:
        d=1-dy  
        a=round(last_b*((pow(d,(t_tm-t_la))-pow(d,(t-t_la)))/np.log(d)),2)
    #print(f"so see {t} {t_la} {t_tm} {t_tm-t_la} old_acc {cur_acc} new{acc}")
    if cur_acc<=acc:
        return a,t_tm,acc
    else: 
        return a,t_la,last_b 
    
def Cur_Integral(last_b,acc,dy,t,t_la):  # return valid last time and last value. 
    cur_acc=last_b*pow(1-dy,t-t_la)
    if cur_acc<=acc:
        return acc,t,acc
    else: 
        return cur_acc,t_la,last_b 
    

#     else:
#         if dy==0:
#             a=cur_acc*(T-t)
#         else:
#             d=1-dy
#             a=round(last_b*((pow(d,(T-t_la))-pow(d,(t-t_la)))/np.log(d)),2)
#         return a, t_la,last_b
   # a=round(last_b*(pow(d,(T-cu_t))-pow(d,(t-cu_t))/np.log(d)),2)
    #print(cu_t,a,cu_t,va,np.log(d))
    #return a
def Reliable(last_b,last_t,cu_t,acc,dy):
    cu_b=last_b*pow(1-dy,cu_t-last_t)
    return round(cu_b,2) 


def Waiting_grain(last_b,acc,dy,t,T,t_la,penality):
    if last_b-dy*(t-t_la)<0:
        dur=t-(t_la+last_b/dy)
        cur_acc=-dur/60*penality
    else:
        cur_acc=last_b-dy*(t-t_la)
    #print(f"see {cur_acc}")
    if cur_acc< acc:
        grain=acc-cur_acc
        return grain,t,acc
    else:
        return 0,t_la,last_b


# a,b,c=Sump_Integral(1,0,0,30,30,0)
# print(a,b,c)

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
        lt_set=ma_set[k].la_t 
        if TL.get(k)==None:   # if areas did not be covered 
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]  
                la_t=max(lt_set[j],0)
                if b>0:
#                     dur=Time-Ti 
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=(b+max(0,b-dur*dy))*d_t*0.5
                    sum_one,la,last_b=Sump_Integral(b,0,dy,Ti,Time,la_t)
                    sum_event.append(sum_one)
                    print(f"no TL the sum_one {k},{task_set[j]} {sum_one},")
                    sum_re=sum_re+sum_one*si
                else: 
                    sum_event.append(0)
        else:
            for j in range(len(task_set)):  # for all tasks of a m 
                [si,dy]=task_dic.get(task_set[j])    
                b=ini_va[j]   
                la_t=max(lt_set[j],0)
                sum_one=0 
                print(f"see",TL[k])
                #la_t=Ti
                for i in range(len(TL[k])):
                    acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
                    if(i==0): #calcuate the information loss before this visit, here is a little complicated, we need consider the last  visit Time. 
                        #dur=TL[k][i][0]-Ti   
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, Ti, TL[k][i][0],la_t)
                        sum_one=sum_one+sum_a
                       # print(f" old b {b}")
                       # b=Reliable(b, Ti, TL[k][i][0], acc, dy)
                      #  print(f" {b}, {Ti}, {TL[k][i][0]},{acc},{dy},{sum_one}")
                    else:
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, TL[k][i-1][0], TL[k][i][0],la_t)
                       # b=Reliable(b, TL[k][i-1][0], TL[k][i][0], acc, dy)
                        sum_one=sum_one+sum_a
                       # print(f" {b}, {TL[k][i][0]}, {TL[k][i-1][0]}, {acc},{dy},{sum_one}")
#                         dur=(TL[k][i][0]-TL[k][i-1][0])
#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
#                     if b>0:
#                         co_t=co_t+d_t
#                     b=max(0,b-dur*dy)
#                     acc=ins_m_dic.get(str(int(TL[k][i][1]))).get(task_set[j][:2]) # get the accuracy of the inspection
#                     b=max(b,acc)
                    print(f"check STEP sum_one {k},{task_set[j]} {la_t} {b}{acc} {dy} {sum_one},")
                sum_event.append(sum_one)
                print(f"check the sum_one {k},{task_set[j]} {dy} {sum_one},")
                
                sum_re=sum_re+sum_one*si
    Sum=round(sum_re/event_size,2)
    print(len(sum_event),sum_event)
    print(f"to see {M_v}")
    return Sum,(min(sum_event),-(sum_event.count(min(sum_event)))),(co_t/event_size)



############### finish 7 _27
def Update_M_TL(M_set,Tl,Ti,Time,D,task_dic,ins_m_dic,T_tm):   # for update m_set given a TL
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
                #    ma_set[k].va[j]=Reliable(b, Ti, Time, 0, dy)
                    #print(f" {ma_set[k].va[j]}, {Ti}, {Time},{dy},{sum_one}")
                    #  ma_set[k].la_t=np.ones(len(ini_va))*Time  ###### all get the value at Time 
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
                        #b=Reliable(b, Ti, TL[k][i][0], acc, dy)
                    else:
                        #dur=(TL[k][i][0]-TL[k][i-1][0])
                        sum_a,la_t,b=Sump_Integral(b, acc, dy, TL[k][i-1][0], TL[k][i][0],la_t)
                        sum_one=sum_one+sum_a
                        #b=Reliable(b, TL[k][i-1][0], TL[k][i][0], acc, dy)
                        #print(f" {b}, {TL[k][i-1][0]}, {TL[k][i][0]},{dy},{sum_one}")

#                     d_t=(dur if dy==0 else min(dur,b/dy))
#                     sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
#                     b=max(0,b-dur*dy)
#                     b=max(b,acc)
#                 dur=T_tm-Time 
#                 d_t=(dur if dy==0 else min(dur,b/dy))
#                 sum_one=sum_one+(b+max(0,b-dur*dy))*d_t*0.5
                ma_set[k].va[j]=b
                ma_set[k].la_t[j]=la_t
                sum_a,a,c=Sump_Integral(b, 0, dy, Time, T_tm,la_t)
                sum_one=sum_one+sum_a   # plus the re to T_tm
                ma_set[k].ac_re[j]=sum_one
                print(f" {ma_set[k].va[j]}, {ma_set[k].la_t[j]} {Time}, {T_tm},{dy},{sum_one}")
                
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
            #dur=(arr_t-max(T_in,lt_set[j]))  # increase duration 
            acc=ins_m_dic.get(str(int(wap.ty))).get(task_set[j][:2]) # get the accuracy of the inspection
            #tmp=max(0,ini_va[j]-dur*dy)
            la_t=max(0,lt_set[j])
            #tmp=Reliable(ini_va[j],la_t,arr_t, 0, dy)   
#             if T_rest==0:
#                 m_co[i].ac_re[j]=mac_re[j]+(max(acc,tmp)-tmp)
            #else:
            increase,la_t,b=Integral_grain(ini_va[j], acc, dy, arr_t, T_tm,la_t)  
            m_co[i].va[j]=b # update the end value 
            m_co[i].la_t[j]=la_t      
#                 d_t=(T_rest if dy==0 else min(T_rest,tmp/dy))
#                 pre_re=(tmp+max(0,tmp-T_rest*dy))*d_t*0.5
#                 new=max(tmp,acc)
#                 d_t=(T_rest if dy==0 else min(T_rest,new/dy))
#                 new_re=(new+max(0,new-T_rest*dy))*d_t*0.5
            m_co[i].ac_re[j]=mac_re[j]+increase
          #  print(f"{i}, {task_set[j]}, {tmp},{acc}, {arr_t}, {la_t}, {T_tm},{ m_co[i].ac_re[j]} {increase}")
    return m_co,(arr_t) # here is leave time



#####
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
            #tmp=Reliable(ini_va[j],la_t,arr_t, 0, dy)
#             if T_rest==0:
#                 Grain=Grain+(max(tmp,acc)-tmp)*si
            increase,c,d=Integral_grain(ini_va[j], acc, dy, arr_t, T_tm,la_t)
#                 d_t=(T_rest if dy==0 else min(T_rest,tmp/dy))
#                 pre_re=(tmp+max(0,tmp-T_rest*dy))*d_t*0.5
#                 new=max(tmp,acc)
#                 d_t=(T_rest if dy==0 else min(T_rest,new/dy))
#                 new_re=(new+max(0,new-T_rest*dy))*d_t*0.5
            Grain=Grain+(increase)*si
    #d=D[w_id][wap.id]
    return (Grain/(D[w_id][wap.id])),wap



# def Sum_Integral(last_b,acc,dy,t,T,t_la):  # return valid last time and last value. 
#     if t_la>t:
#         return 0,t_la,last_b  # add this just in case
#     cur_acc=last_b*pow(1-dy,t-t_la)
#    # print(cur_acc,acc)
#     if cur_acc<=acc:
#         if dy==0:
#             a=acc*(T-t)
#         else:
#             d=1-dy  
#             a= round(acc*((pow(d,(T-t))-pow(d,(t-t)))/np.log(d)),2)
#         return a,t,acc
#     else:
#         if dy==0:
#             a=cur_acc*(T-t)
#         else:
#             d=1-dy
#             a=round(last_b*((pow(d,(T-t_la))-pow(d,(t-t_la)))/np.log(d)),2)
#         return a, t_la,last_b









