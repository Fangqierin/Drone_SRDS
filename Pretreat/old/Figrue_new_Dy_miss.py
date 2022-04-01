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
T=[60,180,240,360,540,600,900,1200,1500,1800,2100,2400,2700]
h1=[[] for i in range(len(T))]; h_min=[[] for i in range(len(T))];h_max=[[] for i in range(len(T))]
h2=[[] for i in range(len(T))]
ru=[[] for i in range(len(T))]
conf_co=1.645
font = {'size' : 13}
plt.rc('font', **font)
legends3=[]
#filename=['result/com_sum_4.eps','com_loss_tim','run_time' ]
policy=2
Num_result=[]
legends2=[]
SEQ=0
for drone_num in [5]:
# for i in range(len(T)):
    Par={};Par_error={};P_w_min={};P_min={};P_h_ac={};P_h_min={}
    for size in [3]:
        for p in [0,2]:
            if p==2:
                f=open(f"./result/dy21_{drone_num}_{size}_{policy}_{p}.txt",'r')
               # f=open(f"./result/result_2021/dy_new2021_{drone_num}_{size}_{policy}_{p}.txt",'r')
            else:
                
                print(f"./result/18_result/dy10_{drone_num}_{size}_{policy}_{p}.txt")
                f=open(f"./result/dy21_{drone_num}_{size}_{policy}_{p}.txt",'r')
                #f=open(f"./result/result_2021/dy_new2021_{drone_num}_{size}_{policy}_{p}.txt",'r')
            par='miss'
            Result_run=defaultdict(list)
            Result_wp=defaultdict(list)
            Result_all=defaultdict(list)
            Tasks=defaultdict(list)
            Result_miss={}
            for k in range(20):  
                fre_dic={}
                for t in T: 
                    fre_dic[t]={}
                    tmp={}
                    for i in range(15):
                        tmp[i]=[]
                    fre_dic[t]=tmp
                Result_miss[k]=copy.deepcopy(fre_dic)
            for line in f: 
                if re.match("avg_all",str(line))!=None:
                    a=line.split(" ")
                    #print(a)
                    method=int(a[2])
                    num=int(a[3])
                    runtime=float(a[4])
                    #print(method, num, runtime)
                    Result_run[method].append(runtime)
                if re.match("avg_run",str(line))!=None:
                    a=line.split(" ")
                    #print(a)
                    method=int(a[2])
                    num=int(a[3])
                    runtime=float(a[4])
                    #print(method, num, runtime)
                    Result_wp[method].append(runtime)
                elif re.match("sum", str(line))!=None:
                        a=line.split(' ') 
                        method=int(a[2])
                        dur=int(a[4])
                        num=int(a[3])
                        miss=int(a[5])
                        print(a)
                        Result_miss[num][dur][method].append(miss)
                elif re.match("task_num", str(line))!=None: 
                        a=line.split(' ') 
                        print(a)
                        method=int(a[2])
                        num=int(a[3])
                        tasks=int(a[4])
                        Tasks[num].append(tasks)
                        print(num,method,tasks)
            ha=[];error=[]
            c_run=copy.deepcopy(Result_run)
            for i in Result_run:
                c_run[i]=np.mean(Result_run[i])
            c_w=copy.deepcopy(Result_wp)
            for i in Result_run:
                c_w[i]=np.mean(Result_wp[i])
            show=[10,8,2,5,3]
            ############################################### compre running time
            tk=[]
            for i,j in Tasks.items():
                tk.append(max(j))
            print(f"see tk {tk} {np.mean(tk)}")
            ##################################################
            c_w_de_m=copy.deepcopy(Result_miss)
            for i in Result_miss:
                for t in T:
                    for k in range(15):
                        c_w_de_m[i][t][k]=np.mean(Result_miss[i][t][k])
            h_miss=[[] for i in range(len(T))]
            h_w_de_m=[[] for i in range(len(T))]
            c_input=[c_w_de_m]
            data=[h_w_de_m]
            for c in range(len(c_input)):
                for k,s in c_input[c].items():
                    for t in range(len(T)):
                        data[c][t].append(list(s[T[t]].values()))
    ###############################################################################  
            parp=[];std=[]
            for d in data: 
                ha=[];error=[]
                for i in d:
                    tmp=[];err=[]
                    for k in range(len(i[0])):
                        #print(k,[i[f][k] for f in range(len(i))])
                        tm=round(np.mean([i[f][k] for f in range(len(i))]),2)  # get the mean of all sample
                        sd=np.std([i[f][k] for f in range(len(i))])   # get the std 
                        er=conf_co*(sd/math.sqrt(len(i[0])))
                        tmp.append(tm)
                        err.append(er)
                    ha.append(tmp)   ################ here
                    error.append(err)
                parp.append(ha)
                std.append(error)
            print(f"see result",parp)
            title=[' Sum of weighted AUC ', 'Sum of weighted minimum AUC ', #'Running time (s)',
             'Minimum AUC of high-priority targets ',  'Average AUC ', 
             'Minimum AUC ', 'Average AUC of high-priority targets', 'Average sampling accuracy', 'Average sampling accuracy of high-priority targets', 
             'Minimum sampling accuracy of high-priority targets', 'Minimum sampling accuracy', 'Minimum detection ratio',  'Minimum detection ratio of high-priority targets ',  ' Sum of weighted minimum sampling accuracy', ' Sum of weighted minimum detection ratio','Running time (s)'  ]   
            title=['Sum of Weighted AUC ', 'Sum of Weighted AUC ', #'Running time (s)',
             'Minimum AUC of high-priority targets ',  'Average AUC ', 
             'Minimum AUC ', 'Average AUC of high-priority targets', 'Average sampling accuracy', 'Average sampling accuracy of high-priority targets', 
             'Minimum sampling accuracy of high-priority targets', 'Minimum sampling accuracy', 'Minimum Detection Ratio',  'Minimum detection ratio of high-priority targets ',  ' Sum of Weighted Sampling Accuracy', ' Sum of Weighted Detection Ratio','Running Time (s)', 'Minimum Weighted AUC', 'Weighted  Reliability', 'Weighted  Accuracy' ]   

            name=['w_sum','w_min', 'min_high', 'Av_sum',  'Min_ac','High_ac', 'Sam_av', 'Sam_h_av','Sam_min_h', 'Sam_min_ac', 'Min_d_low', 'Min_d_h', 'W_sam_m','W_de_m','Run']
            methods=['Max_ac','Max_cov+Max_min','MI','NN','Max_cov+Greedy_min','MM','Greedy_grain','Greedy_weight_grain','WS','Max_cov+Greedy_weight_grain','$WS^{C}$','MST','MST','$WM^{C+}$']
            methods=['Max_ac','Max_cov+Max_min','MM','NN','Max_cov+Greedy_min','MI','Greedy_grain','Greedy_weight_grain','DWS','Max_cov+Greedy_weight_grain','DWSF','MST','MST','$WM^{C+}$']
            mark=['-+','-*','-*','--o','-^','-o','-+','-*','--+','-.','-^','-^','-*','->','-*']
            show=[7,3,4,6,5]
            show=[7,8,3,4,6,5]
            show=[7,8,2,4,5,3] 
            show=[10,9,7,3,4,5]
            show=[10,8,2,3,12]
            show=[11,9,3,4,13]
            show=[11,9,6,4,12]
            if p==0:
                legends=[methods[i-1]+'$_{A}$' for i in show]
            elif p==2:
                legends=[methods[i-1]+'$_{K}$' for i in show]
            mark=['-o','-*','-->','--p','-.^','-<','-+','-*','--+','-.','-^','-^','-*','->','-*'] 
# # #################################################################### Missing event. 
            for pa in range(len(parp)): 
             #for pa in [8]: 
                se=0
                for i in show:
                    print(f" {i} {parp[pa][len(T)-1][i]}  ")
                    plt.errorbar([tt/60 for tt in T],[parp[pa][j][i] for j in range(len(T))],yerr=[std[pa][j][i] for j in range(len(T))],fmt=f"{mark[se]}",capsize=2,markersize=7)
                    se=se+1
             
            plt.xlabel('Time (Minute)')
            plt.ylabel('Accumulative Missing Events')
            #plt.yticks(np.arange(0,max([parp[p][j][i]/T[j] for j in range(len(T))])+0.1 , step=0.1))
            plt.grid( linestyle='--', linewidth=1)
            plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
            par='Dy'
            filename=(f"./result/2021_dy/{par}_{drone_num}_{size}_{policy}_{p}.eps")
            plt.savefig(filename,bbox_inches = 'tight')  
            plt.show()
            plt.clf()    
            
            
# # #
#             #################################### Running time
            legends2=['LU+'+legends[i] for i in range(len(legends))]
            hei=[c_run[i] for i in show]
            hh=[c_w[i] for i in show]
            plt.bar(range(len(hei)),[hei[i]+hh[i] for i in range(len(hh))])
            plt.xticks(range(len(hei)),legends2,fontsize=12)
            plt.grid( linestyle='--', linewidth=1)
            plt.ylabel('Running Time (s)')
            plt.xlabel('Methods')
            filename=(f"./result/2021_dy/{par}_com_run_{drone_num}_{size}_{p}.eps")
#             plt.savefig(filename,bbox_inches = 'tight')  
            plt.show()
            plt.clf() 
#             ############################################## 
#######################################################################         
        #show=[7,8,3,4,6,0]
            Par[p]=parp
            Par_error[p]=std
    
    
        mark=['-o','-*','-.o','-.*','-^','-o','-+','-*','-o','-.','-^','-o']
        show1=[11,6]
        
        if size==2:
            legends2=legends2+[methods[i-1]+'$_{A}$' for i in show1]
            legends2=legends2+[methods[i-1]+'$_{K}$' for i in show1]
        if size==3:
            legends2=legends2+[methods[i-1]+'$_{A}$' for i in show1]
            legends2=legends2+[methods[i-1]+'$_{K}$' for i in show1]
                #legends2=legends2+[methods[i]+'$_{KM}$' for i in show1]
        
            #legends2=legends2+[methods[i-1]+'$_{W}$'+', 3 drones' for i in show1]
            #legends2=legends2+[methods[i]+'$_{K}$'+', 303 tasks' for i in show1]
        
        
           # legends2=legends2+[methods[i]+'$_{K}$'+', 410 tasks' for i in show1]
                #legends2=legends2+[methods[i]+'$_{KM}$' for i in show1]
        
        
         
        for k in list(Par.keys()):
                #print(f"Par {k} {miss}")
            mark=defaultdict(list)
            mark[0]=['-o','-*','-*']
            mark[2]=['-.^','-.+']
            col=['tab:blue','tab:green']
            se=0
            for pa in range(len(Par[k])): 
                 
                print(f"why {size} {pa}")
                for i in show1:
                    se=show1.index(i)
                    print(f"seeee {k}")
#                     print([Par[k][pa][j][i] for j in range(len(T))])
#                     print([Par_error[k][pa][j][i] for j in range(len(T))])
                    plt.errorbar([tt/60 for tt in T],[Par[k][pa][j][i] for j in range(len(T))],yerr=[Par_error[k][pa][j][i] for j in range(len(T))],fmt=mark[k][se],capsize=2,markersize=7)#,color=col[se])
                    if  k==0:
                        print([Par[2][pa][j][i] for j in range(len(T))])
                        print([Par[k][pa][j][i] for j in range(len(T))])
                        b=[Par[2][pa][j][i] for j in range(len(T))]
                        d=[Par[k][pa][j][i] for j in range(len(T))]
                        a=[Par[k][pa][j][i]-Par[2][pa][j][i] for j in range(len(T))]
                        c=[a[ii]/b[ii]  for ii in range(1,len(a))]
                        e=[d[ii]/b[ii]  for ii in range(1,len(a))]
                        #print(a)
                        print(i,e)
                        print(min(e)*100)
                    
                    
                    
                #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
                #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#     legends3=[methods[i]+'$_{W}$'+f" {k} drones" for k in [3,5,7]]
#     legends3=[methods[i]+'$_{W}$'+f" {k} tasks" for k in [339,521,632]]
    plt.xlabel('Time (Minute)')
    plt.ylabel('Accumulative Missing Events')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends2,loc=0,prop={'size': 13},ncol=2)
    par='Dy_col'
    filename=(f"./result/2021_dy/{par}_com_al_{drone_num}_{size}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 


#         show1=[10]
#         for k in list(Par.keys()):
#             #print(f"Par {k} {miss}")
# #             mark=defaultdict(list)
# #             mark[0]=['-o','-*','-*']
# #             mark[2]=['--o','--*']
#             #col=['tab:blue','tab:green']
#             se=0
#             for pa in range(len(Par[k])): 
#                 
#                 print(f"why {size} {pa}")
#                 for i in show1:
#                     se=show1.index(i)
#                     print(f"seeee {k}")
# #                     print([Par[k][pa][j][i] for j in range(len(T))])
# #                     print([Par_error[k][pa][j][i] for j in range(len(T))])
#                     plt.errorbar([tt/60 for tt in T],[Par[k][pa][j][i] for j in range(len(T))],yerr=[Par_error[k][pa][j][i] for j in range(len(T))],fmt=mark[drone_num],capsize=2,markersize=7)
#                 #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
#                 #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
# #legends3=[methods[i]+'$_{W}$'+f" {k} drones" for k in [3,5,7]]
# legends3=[methods[i]+'$_{W}$'+f" {k} tasks" for k in [339,521,632]]
# plt.xlabel('Monitoring time (min)')
# plt.ylabel('Missing event')
# plt.grid( linestyle='--', linewidth=1)
# plt.legend(legends3,loc=0,prop={'size': 13},ncol=2)
# filename=(f"./result/24_result/{par}_com_size_{drone_num}.eps")
# plt.savefig(filename,bbox_inches = 'tight')  
# plt.show()
# plt.clf() 
         
    
    
    

    
    
'''
    legends=[]
    show1=[10]
    for j in [0,2]:
        if j==0:
            legends=legends+[methods[i]+'$_{WK}$' for i in show1]
        elif j==2:
            legends=legends+[methods[i]+'$_{KM}$' for i in show1]
            
            
    for k, itm in Par.items():
        for si, v in itm.items():
             for pa in range(len(v)): 
            #print(f"miss {miss}")
        #print(f" see w",w_sum_ha)
                for i in show1:
                    for j in range(len(T)):
                        plt.errorbar(T,[Par[k][pa][j][i] for j in range(len(T))],yerr=[Par_error[k][pa][j][i] for j in range(len(T))],fmt=f"{mark[i]}",capsize=2)
#     for k in list(Par.keys()):
#         #print(f"Par {k} {miss}")
#         print(f"length {len(Par[k])}")
#         print(f"par ...",Par[k])
#         for pa in range(len(Par[k])): 
#             #print(f"miss {miss}")
#         #print(f" see w",w_sum_ha)
#             for i in show1:
#             #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#            # print([av_ha[j][i] for j in range(len(T))])
#                 #print(f"see result:{k}  {[miss[pa][j][i] for j in range(len(T))]}")
#                 #plt.plot(T,[[pa][j][i] for j in range(len(T))])#,marker=mark[i])
#                 for j in range(len(T)):
#                     print([Par[k][pa]])
#                 plt.errorbar(T,[Par[k][pa][j][i] for j in range(len(T))],yerr=[Par_error[k][pa][j][i] for j in range(len(T))],fmt=f"{mark[i]}",capsize=2)
#             #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
#             #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    plt.xlabel('Monitoring time (s)')
    plt.ylabel('Missing event')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/21_result/{par}_com_num_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 


  '''

'''   
    for k,w_min_ha in P_w_min.items():
        for i in [7]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[w_min_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    plt.xlabel('Monitoring time (s)')
    plt.ylabel('Sum of weighted minimum accumulative accuracies ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/new_result/{par}_w_min_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf()
    
    

    for k,min_high_ha in P_h_min.items():
        print(k)
        for i in [7]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[min_high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
            #print([min_high_ha[j][i]/T[j] for j in range(len(T))])
            #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
            #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#         legends=[methods[i] for i in show]
    plt.xlabel('Monitoring time (s)')
    plt.ylabel('Minimum accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/new_result/{par}_miss_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 
    

#     
#     for i in show:
#         #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#        # print([av_ha[j][i] for j in range(len(T))])
#         plt.plot(T,[av_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
#         #plt.plot(T,[av_ha[j][i] for j in range(len(T))],marker=mark[i])
#         #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#     plt.xlabel('Monitoring time (s)')
#     plt.ylabel('Average accumulative accuracy ')
#     plt.grid( linestyle='--', linewidth=1)
#     plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
#     filename=(f"./result/new_result/{par}_Av_sum_{drone_num}.eps")
#     plt.savefig(filename,bbox_inches = 'tight')  
#     plt.show()
#     plt.clf()  
        
#     
#     for k,min_high_ha in P_h_min.items():
#         for i in [7]:
#         #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#        # print([av_ha[j][i] for j in range(len(T))])
#             plt.plot(T,[min_high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
#         #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
#         #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#     
#     plt.xlabel('Monitoring time (s)')
#     plt.ylabel('Minimum accumulative accuracy of high-priority tasks ')
#     plt.grid( linestyle='--', linewidth=1)
#     plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
#     filename=(f"./result/new_result/{par}_miss_{drone_num}.eps")
#     plt.savefig(filename,bbox_inches = 'tight')  
#     #plt.show()
#     plt.clf() 

    for k,min_ha in P_min.items():
        for i in [7]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[min_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_ac','Max_cov','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min']
    plt.xlabel('Monitoring time (s)')
    plt.ylabel('Minimum accumulative accuracy ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/new_result/{par}_Min_ac_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    #plt.show()
    plt.clf()  
    

    #mark=['+','*','o','.','^','o']
    for k,high_ha in P_h_ac.items():
        for i in [7]:
        #print(T)
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
        #print([high_ha[j][i] for j in range(len(T))])
            plt.plot(T,[high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    #legends=['Max_ac','Max_cov','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min']
    plt.xlabel('Monitoring time (s)')
    plt.ylabel('Average accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/new_result/{par}_High_ac_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    #plt.show()
    plt.clf() 

'''
'''


# Num=[2,3,4]
# T=[300,600,900,1200,1500]
# mark=['+','*','o','.']
# for i in range(4):
#     #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
#     plt.plot(T,Num_result[i],marker=mark[i])
#    # plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
# legends=['Dron_num=1','Dron_num=2','Dron_num=3','Dron_num=4']
# plt.xlabel('Monitoring time (s)')
# plt.ylabel('Minimum accumulative accuracy ')
# plt.grid( linestyle='--', linewidth=1)
# plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
# filename=(f"./result/com_min_num.eps")
# plt.savefig(filename,bbox_inches = 'tight')
# plt.show()
# plt.clf() 

# for drone_num in [2,4,5]: 
#     for i in [0,2]:
#         f=open(f"./result/par2_{drone_num}_{i}.txt",'r')
#         par='show'

'''