'''
Created on May 5, 2020

@author: fangqiliu
'''

import numpy as np
import matplotlib.pyplot as plt 
import scipy.integrate
from numpy import exp
import re
import math
import copy
from collections import defaultdict
#T=[60,180,240,360,540,600,1200,1500,1800,2100,2400,2700,3000]
#T=[60,180,240,360,540,600,900,1200,1500,1800,2100,2400,2700]
simulation_time=1800
T=np.arange(60,simulation_time,120)
h1=[[] for i in range(len(T))]; h_min=[[] for i in range(len(T))];h_max=[[] for i in range(len(T))]
h2=[[] for i in range(len(T))]
ru=[[] for i in range(len(T))]
conf_co=1.645
#conf_co=1.960
font = {'size' : 13}
plt.rc('font', **font)
legends3=[]
#filename=['result/com_sum_4.eps','com_loss_tim','run_time' ]
policy=2
Num_result=[]
legends2=[]
SEQ=0
#methods=[10]
Total_st={}
for drone_num in [7]:
#for drone_num in [3,5,7,9]:
# for i in range(len(T)):
    Par={};Par_error={};P_w_min={};P_min={};P_h_ac={};P_h_min={}
    Par2={};Par_error2={};P_w_auc={};P_w_r={};P_w_min_auc={};P_w_a={}
    for size in [1,2,3,4]:
        total_miss=defaultdict(list);
        for p in [2]:
            #if p==2:
            #dy2022_14_5_2_2_0
            if drone_num==7:
                if size==2:
                    f=open(f"./result/result_2021/dy2022_14_{drone_num}_{size}_{policy}_{p}.txt",'r')
                else:
                    f=open(f"./result/result_2021/dy2022_17_{drone_num}_{size}_{policy}_{p}.txt",'r')
            #f=open(f"./result/result_2021/dy2022_14_{drone_num}_{size}_{policy}_{p}.txt",'r')
#             print(f"./result/result_2021/dy4_1_{drone_num}_{size}_{policy}_{p}.txt",'r')
#             else:
#                 print(f"./result/18_result/dy10_{drone_num}_{size}_{policy}_{p}.txt")
#                 f=open(f"./result/dy21_{drone_num}_{size}_{policy}_{p}.txt",'r')
            w_AUC=defaultdict(list); w_sample=defaultdict(list); w_dect=defaultdict(list);w_min_AUC=defaultdict(list)
            
            w_AUC2=defaultdict(dict); w_sample2=defaultdict(dict); w_dect2=defaultdict(dict);
            par='com_all'
            Result_run=defaultdict(list)
            Result_wp=defaultdict(list)
            Result_all=defaultdict(list)
            Tasks=defaultdict(list)
            Result_miss={}
            cc=list(range(25)) # checking nur
            #cc.remove(11)
            for k in cc:#(list(range(20)).remove(11)):  
                fre_dic={}
                for t in T: 
                    fre_dic[t]={}
                    tmp={}
                    for i in range(18):
                        tmp[i]=[]
                    fre_dic[t]=tmp
                Result_miss[k]=copy.deepcopy(fre_dic)
            ############## check AUC
            Result_w_min_v={}
            Result_w_sample_min={}
            Result_w_detect_min={}
            Result_w_AUC={}
            times=np.arange(180,simulation_time,180)
            times=np.arange(300,simulation_time,300)
            times=np.arange(60,simulation_time,60)
            time_area=list(times)
            if simulation_time not in time_area:
                time_area.append(simulation_time)
            for k in cc:
                fre_dic={}
                for t in time_area: 
                    fre_dic[t]={}
                    tmp={}
                    for i in range(18):
                        tmp[i]=[]
                    fre_dic[t]=tmp
                Result_w_min_v[k]=copy.deepcopy(fre_dic)
                Result_w_sample_min[k]=copy.deepcopy(fre_dic)
                Result_w_detect_min[k]=copy.deepcopy(fre_dic)
                Result_w_AUC[k]=copy.deepcopy(fre_dic)
            ########################################################
            for line in f: 
                if re.match("avg_all",str(line))!=None:
                    a=line.split(" ")
                    #print(a)
                    method=int(a[2])
                    num=int(a[3])
                    if num in cc:
                        runtime=float(a[4])
                        #print(method, num, runtime)
                        Result_run[method].append(runtime)
                if re.match("avg_run",str(line))!=None:
                    a=line.split(" ")
                    #print(a)
                    method=int(a[2])
                    num=int(a[3])
                    if num in cc:
                        runtime=float(a[4])
                        #print(method, num, runtime)
                        Result_wp[method].append(runtime)
                elif re.match("sum_state", str(line)) !=None:
                        a=line.split(" ") 
                        #print(f"why",a)
                        method=int(a[2])
                        dur=int(a[-3])
                        num=int(a[-4])
                        if num in cc:
                            miss=float(a[-2])
                            #print(a)
                            Result_miss[num][dur][method].append(miss)
                        if dur==T[-1]:
                            if  total_miss.get(method)==None:
                                total_miss[method]={}
                            if  total_miss.get(method).get(num)==None:
                                total_miss[method][num]=[]
                            total_miss[method][num].append(miss)
                elif re.match("w_min_v", str(line)) !=None:
                        a=line.split(" ") 
                        #print(f"why",a)
                        method=int(a[2])
                        dur=int(a[4])
                        num=int(a[3])
                        if num in cc:
                            value=round(float(a[5]),3)
                            #print(a)#
                            Result_w_min_v[num][dur][method].append(value)
                            #print(f"see AUC", Result_w_min_v)
                elif re.match("w_sample_min", str(line)) !=None:
                        a=line.split(" ") 
                        #print(f"why",a)
                        method=int(a[2])
                        dur=int(a[4])
                        num=int(a[3])
                        if num in cc:
                            value=round(float(a[5]),3)
                            #print(a)#
                            Result_w_sample_min[num][dur][method].append(value)
                            if  w_sample2.get(method)==None:
                                w_sample2[method]={}
                            if  w_sample2.get(method).get(num)==None:
                                w_sample2[method][num]=[]
                            w_sample2[method][num].append(value)
                elif re.match("w_detect_min", str(line)) !=None:
                        a=line.split(" ") 
                        #print(f"why",a)
                        method=int(a[2])
                        dur=int(a[4])
                        num=int(a[3])
                        if num in cc:
                            value=round(float(a[5]),3)
                            #print(a)#
                            Result_w_detect_min[num][dur][method].append(value)
                            if  w_dect2.get(method)==None:
                                w_dect2[method]={}
                            if  w_dect2.get(method).get(num)==None:
                                w_dect2[method][num]=[]
                            w_dect2[method][num].append(value)
                elif re.match("min_AUC", str(line))!=None: 
                        a=line.split(" ") 
                        #print(f"why",a)
                        method=int(a[2])
                        dur=int(a[4])
                        num=int(a[3])
                        if num in cc:
                            value=round(float(a[5]),3)
                            #print(a)#
                            Result_w_AUC[num][dur][method].append(value)           
                            if  w_AUC2.get(method)==None:
                                w_AUC2[method]={}
                            if  w_AUC2.get(method).get(num)==None:
                                w_AUC2[method][num]=[]
                            w_AUC2[method][num].append(value)
                
                elif re.match("task_num", str(line))!=None: 
                        a=line.split(' ') 
                        #print(a)
                        method=int(a[2])
                        num=int(a[3])
                        if num in cc:
                            tasks=int(a[4])
                            Tasks[num].append(tasks)
                        #print(num,method,tasks)
                elif re.match("AUC", str(line))!=None: 
                        a=line.split(' ') 
                        #print(a)
                        method=int(a[2])
                        if num in cc:
                            num=int(a[3])
                            w_min_v=float(a[4])
                            w_sample_min=float(a[5])
                            w_detect_min=float(a[6])
                            w_auc=float(a[-1])
                            #print(f"w_auc",w_auc)
                            Tasks[num].append(tasks)
                            #print(num,method,tasks)
                            w_AUC[method].append(w_min_v)
                            w_dect[method].append(w_detect_min)
                            w_sample[method].append(w_sample_min)
                            w_min_AUC[method].append(w_auc)
                
            
            show=[17,8,5,3,12]
            c_w=copy.deepcopy(Result_wp)
            for i in Result_run:
                c_w[i]=np.mean(Result_wp[i])
            #show=[10,8,2,5,3]
            ############################################### compre running time
            tk=[]
            for i,j in Tasks.items():
                tk.append(max(j))
            print(f"see tasks {tk} {np.mean(tk)}")
            ##################################################    Missing event.
            c_w_de_m=copy.deepcopy(Result_miss)
            for i in Result_miss:
                for t in T:
                    for k in range(18):
                        c_w_de_m[i][t][k]=np.mean(Result_miss[i][t][k])
            h_miss=[[] for i in range(len(T))]
            h_w_de_m=[[] for i in range(len(T))]
            c_input=[c_w_de_m]
            data=[h_w_de_m]
            for c in range(len(c_input)):
                for k,s in c_input[c].items():
                    for t in range(len(T)):
                        data[c][t].append(list(s[T[t]].values()))
                        #print(f"check", c,t, list(s[T[t]].values()))
            ################################################
            parp=[];std=[]
            for d in data: 
                ha=[];error=[]
                for i in d:
                    #print(f"ss",i)
                    tmp=[];err=[]
                    for k in range(len(i[0])):
                        #print(f"emm",i[0],len(i[0]),len(i))
                        tm=round(np.mean([i[f][k] for f in range(len(i))]),2)  # get the mean of all sample
                        #print(tm)
                        sd=np.std([i[f][k] for f in range(len(i))])   # get the std 
                        er=conf_co*(sd/math.sqrt(len(i)))
                        tmp.append(tm)
                        err.append(er)
                    ha.append(tmp)   ################ here
                    error.append(err)
                parp.append(ha)
                std.append(error)
            #print(f"see result",parp)
        ######################################
            #################################################### Weighted AUC. 
            c_input=[];data2=[];parp2=[];std2=[]
            #c_w_de_m=copy.deepcopy(Result_w_detect_min)
            for result in [Result_w_min_v,Result_w_sample_min,Result_w_detect_min,Result_w_AUC]:
                c_w_de_m=copy.deepcopy(result)
                for i in result:
                    for t in time_area:
                        for k in range(18):
                            c_w_de_m[i][t][k]=np.mean(result[i][t][k]) 
                    #h_miss=[[] for i in range(len(time_area))]
                    h_w_de_m=[[] for i in range(len(time_area))]
                    c_input=[c_w_de_m]
                    data2=[h_w_de_m]
                for c in range(len(c_input)):
                    for k,s in c_input[c].items():
                        for t in range(len(time_area)):
                            data2[c][t].append(list(s[time_area[t]].values()))   
                #print(f"data2",data2)
                #########################     
                for d in data2: 
                    ha=[];error=[]
                    #print(d)
                    for i in d:
                        #print(f"ss",i)
                        tmp=[];err=[]
                        for k in range(len(i[0])):
                            #print(f"emm",k,[i[f][k] for f in range(len(i))])
                            tm=np.mean([i[f][k] for f in range(len(i))])  # get the mean of all sample
                            #print(tm)
                            sd=np.std([i[f][k] for f in range(len(i))])   # get the std 
                            er=conf_co*(sd/math.sqrt(len(i)))
                            tmp.append(tm)
                            err.append(er)
                        ha.append(tmp)   ################ here
                        error.append(err)
                    parp2.append(ha)
                    std2.append(error)
                #print(f"see result",parp)
            ######################################   
#########################################################################################################################    
            title=[' Sum of weighted AUC ', 'Sum of weighted minimum AUC ', #'Running time (s)',
             'Minimum AUC of high-priority targets ',  'Average AUC ', 
             'Minimum AUC ', 'Average AUC of high-priority targets', 'Average sampling accuracy', 'Average sampling accuracy of high-priority targets', 
             'Minimum sampling accuracy of high-priority targets', 'Minimum sampling accuracy', 'Minimum detection ratio',  'Minimum detection ratio of high-priority targets ',  ' Sum of weighted minimum sampling accuracy', ' Sum of weighted minimum detection ratio','Running time (s)'  ]   
            name=['w_sum','w_min', 'min_high', 'Av_sum',  'Min_ac','High_ac', 'Sam_av', 'Sam_h_av','Sam_min_h', 'Sam_min_ac', 'Min_d_low', 'Min_d_h', 'W_sam_m','W_de_m','Run']
            methods=['Max_ac','Max_cov+Max_min','MM','NN','Max_cov+Greedy_min','MI','Greedy_grain','Greedy_weight_grain','DWS','Max_cov+Greedy_weight_grain','$DWS^{c}$','MST-','MST','$WM^{C+}$']
            methods=['Max_ac','Max_cov+Max_min','MM','NN','Max_cov+Greedy_min','MI','Greedy_grain','Greedy_weight_grain','DWS','Max_cov+Greedy_weight_grain','DWSF','MST','MST','$WM^{C+}$','DWSF','','','DWSF']
            #methods=['Max_ac','Max_cov+Max_min','MI','NN','Max_cov+Greedy_min','MM','Greedy_grain','Greedy_weight_grain','DWS','Max_cov+Greedy_weight_grain','$DWS^{c}$','MST-','MST','$WM^{C+}$']
            mark=['-+','-*','-*','--o','-^','-o','-+','-*','--+','-.','-^','-^','-*','->','-*','--+','-.','-^','-^','-*','->','-*']
            show=[17,8,5,3,12]
            if p==0:
                legends=['AMT-'+methods[i] for i in show]
    
            elif p==2:
                legends=['KM-'+methods[i] for i in show]
            #print(f"I want to see",legends)
                 
# # #################################################################### Missing event. 
#             for pa in range(len(parp)): 
#                 print(parp[pa])
#                 for i in show:
#                     #plt.errorbar([tt/60 for tt in T],[parp[pa][j][i] for j in range(len(T))],yerr=[std[pa][j][i] for j in range(len(T))],fmt=f"{mark[i]}",capsize=2,markersize=7)
#                     plt.plot([tt/60 for tt in T],[parp[pa][j][i] for j in range(len(T))],f"{mark[i]}",markersize=7)#[parp[pa][j][i] for j in range(len(T))],yerr=[std[pa][j][i] for j in range(len(T))])
#                     print(f"see{i} {[parp[pa][j][i] for j in range(len(T))]}")
#             plt.xlabel('Time (minute)')
#             plt.ylabel('Accumulative Missing Events')
#             #plt.yticks(np.arange(0,max([parp[p][j][i]/T[j] for j in range(len(T))])+0.1 , step=0.1))
#             plt.grid( linestyle='--', linewidth=1)
#             plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
#             filename=(f"./result/2022_result/{par}_miss_{drone_num}_{size}_{policy}_{p}.eps")
#             plt.savefig(filename,bbox_inches = 'tight')  
#             #plt.show()
#             plt.clf()    
# #             #################################### Running time
#             metrics=['Wighted AUC','Weighted Accuracy','Weighted Reliability','Minimum Weighted AUC']
#             names=['w_AUC','w_a','w_r','m_w_auc']
#              #################################################################### Missing event. 
#             cc=0
#             for pa in range(len(parp2)): 
#                 #print(parp2[pa])
#                 for i in show:
#                     #plt.errorbar([tt/60 for tt in time_area],[parp2[pa][j][i] for j in range(len(time_area))],yerr=[std2[pa][j][i] for j in range(len(time_area))],fmt=f"{mark[i]}",capsize=2,markersize=7)
#                     plt.plot([tt/60 for tt in time_area],[parp2[pa][j][i] for j in range(len(time_area))],f"{mark[i]}",markersize=7)
#                     #print(f"see{i} {[parp[pa][j][i] for j in range(len(T))]}")
#                 plt.xlabel('Time (minute)')
#                 #plt.ylabel('Detection Ratio')
#                 plt.ylabel(metrics[cc])
#                 #plt.yticks(np.arange(0,max([parp[p][j][i]/T[j] for j in range(len(T))])+0.1 , step=0.1))
#                 plt.grid( linestyle='--', linewidth=1)
#                 plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
#                 filename=(f"./result/2022_result/{par}_{names[cc]}_{drone_num}_{size}_{policy}_{p}.eps")
#                 plt.savefig(filename,bbox_inches = 'tight')  
#                # plt.show()
#                 plt.clf()    
#                 cc=cc+1
        ############################################
#             #################################### Running time
            Run_time=copy.deepcopy(Result_run)
            c_w=copy.deepcopy(Result_wp)
            for i in Result_run:
                c_w[i]=np.mean(Result_wp[i])
                Run_time[i]=[Run_time[i][k]+Result_wp[i][k] for k in range(len(Result_run[i])) ]
            #show=[10,8,2,5,3]
            ha=[];error=[]
            c_run=copy.deepcopy(Result_run)
            #print(Result_run)
            for i in Result_run:
                c_run[i]=np.mean(Result_run[i])
                
                sd=np.std([Result_run[i][f] for f in range(len(Result_run[i]))])   # get the std 
                er=conf_co*(sd/math.sqrt(len(Result_run[i])))
            hei=[c_run[i] for i in show]
            hh=[c_w[i] for i in show]
            plt.bar(range(len(hei)),[hei[i]+hh[i] for i in range(len(hh))])
            plt.xticks(range(len(hei)),legends)
            plt.grid( linestyle='--', linewidth=1)
            plt.ylabel('Running Time (s)')
            plt.xlabel('Methods')
            filename=(f"./result/2022_result/{par}_com_run_{drone_num}.eps")
            plt.savefig(filename,bbox_inches = 'tight')  
           # plt.show() 
            plt.clf() 
           
            ############################################## Get total
#             metrics=['Accumulative Missing Event','Weighted AUC','Weighted Accuracy','Weighted Reliability','Minimum Weighted AUC']
#             cc=0
#             total_m=[w_AUC,w_sample,w_dect,w_min_AUC]
#             for me in total_m:#[w_AUC,w_sample,w_dect,w_min_AUC]:
#                 h=[];err=[]
#                 for i in show:
#                     #print(f"why",i, me[i])
#                     a=np.mean(list(me[i]))
#                     sd=np.std(list(me[i]))   # get the std 
#                     er=conf_co*(sd/math.sqrt(len(me[i])))
#                     h.append(a)
#                     err.append(er)
#                 #print(h)
#                 plt.bar(range(len(h)),[h[i] for i in range(len(h))],yerr=err,align='center', alpha=0.5, ecolor='black', capsize=10)
#                 plt.xticks(range(len(h)),legends)
#                 plt.grid( linestyle='--', linewidth=1)
#                 plt.ylabel(metrics[cc])
#                 plt.xlabel('Methods')
#                 filename=(f"./result/2022_result/{par}_total_{names[cc]}_{drone_num}.eps")
#                 plt.savefig(filename,bbox_inches = 'tight')  
#                 plt.show()
#                 plt.clf() 
#                 cc=cc+1  
            ############################################################################################
            metrics2=['Accumulative Missing Events','Minimum Weighted AUC','Weighted Accuracy','Weighted Reliability']#,'Minimum Weighted AUC']
            names2=['t_miss','w_m_AUC','w_a','w_r']
            cc=0
            total_m=[total_miss,w_AUC2,w_sample2,w_dect2,Run_time]
            for me in [total_miss,w_AUC2,w_sample2,w_dect2]:
                h=[];err=[]
                for i in show:
                    #print(f"why",i, me[i])
                    for num in me[i]:
                        #print(f"num",i,num,np.mean(me[i][num]),me[i][num])
                        me[i][num]=np.mean(me[i][num])
                    tmp=list(me[i].values())
                    a=np.mean(tmp)
                    sd=np.std(tmp)   # get the std 
                    er=conf_co*(sd/math.sqrt(len(tmp)))
                    h.append(a)
                    err.append(er)
                    #print(h)
                plt.bar(range(len(h)),[h[i] for i in range(len(h))],yerr=err,align='center', alpha=0.5, ecolor='black', color='royalblue',capsize=10)
                plt.xticks(range(len(h)),legends)
                plt.grid( linestyle='--', linewidth=1)
                plt.ylabel(metrics2[cc])
                plt.xlabel('Methods')
                filename=(f"./result/2022_result/{par}_avs_{names2[cc]}_{drone_num}.eps")
                plt.savefig(filename,bbox_inches = 'tight')  
                plt.show()
                plt.clf() 
                cc=cc+1  
                    #print(f"see result",parp)
            '''
            ############################################################################################## for get the average sample
            '''
#######################################################################         
        #show=[7,8,3,4,6,0]
#             Par[drone_num]=parp
#             Par_error[drone_num]=std
#             Par2[drone_num]=parp2
#             Par_error2[drone_num]=std2
    #Total_st[drone_num]=total_m
        Total_st[size]=total_m
        
        
if p==0:
    legends=['AMT-'+methods[i] for i in show]
    
elif p==2:
    legends=['LU+'+methods[i] for i in show]        
        
metrics2=['Running Time (s)','Accumulative Missing Events','Minimum Weighted AUC','Weighted Accuracy','Weighted Reliability']#,'Minimum Weighted AUC']
names2=['run_time','t_miss','w_m_AUC','w_a','w_r']
legend3=[]
cc=0
for i in show:
    legends3.append('AMT-'+methods[i])
    #             legends3.append('KM-'+methods[i])
ke=list(Total_st.keys())[0]
#print(f"see keys",list(Total_st.keys()))
for pa in [-1]:#range(len(Total_st[ke])): 
    h=defaultdict(list);err=defaultdict(list)
    for i in show:
        #prin,t(f"why",i, me[i])
        for k in list(Total_st.keys()):   # different number of drones. 
            #print(f"why {pa} {k}")
            me=Total_st[k][pa]
            if pa!=len(Total_st[ke])-1 and pa!=-1:
                for num in me[i]:
                    #print(f"num",i,num,np.mean(me[i][num]),me[i][num]
                    me[i][num]=np.mean(me[i][num])
                tmp=list(me[i].values())
            else:
                tmp=list(me[i])
            a=np.mean(tmp)
            sd=np.std(tmp)   # get the std 
            er=conf_co*(sd/math.sqrt(len(tmp)))
            h[i].append(a)
            err[i].append(er)
        #print(h[i])
        plt.errorbar([96,177,250,314],h[i],yerr=err[i],fmt=f"{mark[i]}",capsize=2,markersize=7 )
        #plt.errorbar(range(len(h[i])),h[i],yerr=err[i],fmt=f"{mark[i]}",capsize=2,markersize=7 )
        #plt.errorbar([tt/60 for tt in time_area],[parp2[pa][j][i] for j in range(len(time_area))],yerr=[std2[pa][j][i] for j in range(len(time_area))],fmt=f"{mark[i]}",capsize=2,markersize=7)
    
        #plt.bar(range(len(h)),[h[i] for i in range(len(h))],yerr=err,align='center',color='royalblue', alpha=0.5, ecolor='black', capsize=10, )
    
    #plt.xticks(range(len(h[show[0]])),[3,5,7,9])
    #plt.xticks(range(len(h[show[0]])),[96,177,250,314])
    plt.grid( linestyle='--', linewidth=1)
    plt.ylabel(metrics2[cc])
    plt.xlabel('Number of Tasks')
    #plt.xlabel('Number of Drones')
    plt.legend(legends,prop={'size': 13},ncol=2)
    filename=(f"./result/2022_result/{par}_run_time_num_{names2[cc]}_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 
    cc=cc+1  
        
        
        
        
        
        
        
        
          