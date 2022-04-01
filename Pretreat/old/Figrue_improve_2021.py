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
T=[300,600,900,1200,1500]
T=[120,300,480,600,900,1080,1200,1500]    ################################################# for test

h1=[[] for i in range(len(T))]; h_min=[[] for i in range(len(T))];h_max=[[] for i in range(len(T))]
h2=[[] for i in range(len(T))]
ru=[[] for i in range(len(T))]
conf_co=1.645
#filename=['result/com_sum_4.eps','com_loss_tim','run_time' ]
co_s=3
events=2
font = {'size' : 13}
plt.rc('font', **font)
Num_result=[]
all_method=range(15)
th=30
for drone_num in [7]:
# for i in range(len(T)):
    Par={};Std={};P_w_min={};P_min={};P_h_ac={};P_h_min={}
    for p in [0]:
        f=open(f"./result/12_result/new1_{drone_num}_{p}_{events}_{th}.txt",'r')
        print(f".12_result/new1_{drone_num}_{p}_{events}_{th}.txt")
        par='new6_improve'
        Result_min={}
        Result_ac={}
        Result_high={}
        Result_min_high={}
        Result_w_min={}
        Result_w_sum={}
        Result_sam_av={}
        Result_sam_h_av={}
        Result_sam_min_h={}
        Result_sam_min_ac={}
        Result_sam_d_low={}
        Result_sam_d_h={}
        Result_w_sam_m={}
        Result_w_de_m={}
        Result_run={}
        for k in range(20):
            Result_min[k]={}
            Result_ac[k]={}
            Result_high[k]={}
            fre_dic={}
            for t in T: 
                fre_dic[t]={}
                tmp={}
                for i in range(13):
                    tmp[i]=[]
                fre_dic[t]=tmp
            Result_min[k]=copy.deepcopy(fre_dic)
            Result_ac[k]=copy.deepcopy(fre_dic)
            Result_high[k]=copy.deepcopy(fre_dic)
            Result_min_high[k]=copy.deepcopy(fre_dic)
            Result_w_min[k]=copy.deepcopy(fre_dic)
            Result_w_sum[k]=copy.deepcopy(fre_dic)
            Result_w_sam_m[k]=copy.deepcopy(fre_dic)
            Result_w_de_m[k]=copy.deepcopy(fre_dic)
            Result_run[k]=copy.deepcopy(fre_dic)
            for r in [Result_sam_av, Result_sam_h_av, Result_sam_min_h,Result_sam_min_ac,Result_sam_d_low, Result_sam_d_h,Result_w_sam_m,Result_w_de_m,Result_run]:
                r[k] =copy.deepcopy(fre_dic)
        for line in f: 
            
            if re.match("random",str(line))!=None:
                print(line.split(":"))
                all_time=float(line.split(" ")[-1])
                part=line.split(" ")[0]
                fre=int(part.split(":")[-1])
                num=int(part.split(":")[1][:-1])  
            if re.match("12random",str(line))!=None:
                #print(f"can the see the line ",line)
                all_time=float(line.split(" ")[-1])
                part=line.split(" ")[0]
                fre=int(part.split(":")[-1])
                num=int(part.split(":")[1][:-1])
            if re.match("[0|1|2|3|4|5|6] 0", str(line))!=None or re.match("[0|1|2|3|4|5|6] * [0|1|2|3|4|5|6|7|8|9|10|11|12]", str(line))!=None:
                a=line.split(' ') 
                print(f" here is the line {a}")
#                                   0        1   2    3      4            5           6           7       8       9           10       11        12       13      14       15        16      17       18          19         20       21           22            23        
                 #output.write(f"{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {w_min_v} {run_time} {s_av} {s_h_av} {s_m_h} {s_m_ac} {s_lm_d} {s_hm_d} {w_task_min} {min_l_d} {min_h_d} {w_sample_min} {w_detect_min} \n")
#                 output.write(f"{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {run_time} {s_av} {s_h_av} {s_m_h} {s_m_ac} {s_lm_d} {s_hm_d}\n")
                ss="{drone.id} {i} {j} {re} {min_re[0]} {min_re[1]} {high_sum} {high_size} {miss} {min_high} {sum_w_v} {w_min_v} {tt} {ii}"
                id=int(a[0])
                method=int(a[2])
                av_acc=float(a[3])
                min_acc=float(a[4])
                count=-int(a[5])
                high_sum=float(a[6])
                high_size=int(a[7])
                event_size=int(a[8])
                min_high=float(a[9])
                w_sum=float(a[10])
                w_min=float(a[11])    ######### here has missed 
                run_time=float(a[12])
                sam_av=float(a[13])
                sam_h_av=float(a[14])
                sam_min_h=float(a[15])
                sam_min_ac=float(a[16])
                sam_de_low=float(a[20])   # change to weighted min_l_d
                sam_de_h=float(a[21])   # change to min_h_d
                w_sam_m=float(a[22]) 
                w_de_m=float(a[23])
    #             run_time=float(a[7])
                Result_min_high[num][fre][method].append((min_high,high_size))  # min_High
                Result_w_min[num][fre][method].append(w_min)
                Result_min[num][fre][method].append(all_time)    # min_ac
                Result_ac[num][fre][method].append((av_acc,id,event_size)) # average acc
                Result_high[num][fre][method].append((high_sum,id,high_size)) # high_average 
                Result_w_sum[num][fre][method].append((w_sum,id,event_size))  # weighted auv
                Result_sam_av[num][fre][method].append(sam_av)
                Result_sam_h_av[num][fre][method].append(sam_h_av)
                Result_sam_min_h[num][fre][method].append(sam_min_h)
                Result_sam_min_ac[num][fre][method].append(sam_min_ac)
                Result_sam_d_low[num][fre][method].append(sam_de_low)
                Result_sam_d_h[num][fre][method].append(sam_de_h)
                Result_w_sam_m[num][fre][method].append(w_sam_m)
                Result_w_de_m[num][fre][method].append(w_de_m)
                Result_run[num][fre][method].append(run_time)
        c_min=copy.deepcopy(Result_min)
        c_ac=copy.deepcopy(Result_ac)
        c_high=copy.deepcopy(Result_high)
        c_min_high=copy.deepcopy(Result_min_high)
        c_w_min=copy.deepcopy(Result_w_min)
        c_w_sum=copy.deepcopy(Result_w_sum)
        ######################################### new sample
        c_sam_av=copy.deepcopy(Result_sam_av)
        c_sam_h_av=copy.deepcopy(Result_sam_h_av) 
        c_sam_min_h=copy.deepcopy(Result_sam_min_h)
        c_sam_min_ac=copy.deepcopy(Result_sam_min_ac)
        c_sam_d_low=copy.deepcopy(Result_sam_d_low)
        c_sam_d_h=copy.deepcopy(Result_sam_d_h)
        c_w_sam_m=copy.deepcopy(Result_w_sam_m)
        c_w_de_m=copy.deepcopy(Result_w_de_m)
        c_run=copy.deepcopy(Result_run)
        for i in Result_min:
            for t in T:
                for k in range(13):
                    summ=0;size=0
                    for s in Result_ac.get(i)[t][k]:
                        summ=summ+s[0]*s[2]
                        size=size+s[2]
                    if size!=0:
                        av=round(summ/size,2)
                    else:
                        av=0
                    ############# for w_sum
                    summ=0;size=0
                    for s in Result_w_sum.get(i)[t][k]:
                        summ=summ+s[0]
                        size=size+s[2]
                    if size!=0:
                        av_w_sum=round(summ/size,2)
                    else:
                        av_w_sum=0
                    ###############
                    summ=0;size=0
                    for s in Result_high.get(i)[t][k]:
                        summ=summ+s[0]
                        size=size+s[2]
                    if size!=0:
                        av_high=round(summ/size,2)
                    else:
                        av_high=None
                    c_ac[i][t][k]=av
                    c_w_sum[i][t][k]=av_w_sum
                    c_high[i][t][k]=av_high
                    #print(f"why no value {Result_min} {Result_min.get(i)[t][k]}")
                    try:
                        c_min[i][t][k]=np.min(Result_min.get(i)[t][k]) 
                    except:
                        c_min[i][t][k]=0
                    min_high_set=[]
                    for s in Result_min_high.get(i)[t][k]:
                        if s[1]!=0:
                            min_high_set.append(s[0])
                    print(f"see why fail {min_high_set}")
                    c_min_high[i][t][k]=np.min([1])#min_high_set) 
                    c_w_min[i][t][k]=np.mean(Result_w_min.get(i)[t][k])   
                    c_sam_av[i][t][k]=np.mean(Result_sam_av[i][t][k])
                    c_sam_h_av[i][t][k]=np.mean(Result_sam_h_av[i][t][k]) 
                    c_sam_min_h[i][t][k]=np.mean(Result_sam_min_h[i][t][k])
                    c_sam_min_ac[i][t][k]=np.mean(Result_sam_min_ac[i][t][k])
                    c_sam_d_low[i][t][k]=np.mean(Result_sam_d_low[i][t][k])
                    c_sam_d_h[i][t][k]=np.mean(Result_sam_d_h[i][t][k])
                    c_w_sam_m[i][t][k]=np.mean(Result_w_sam_m[i][t][k])
                    c_w_de_m[i][t][k]=np.mean(Result_w_de_m[i][t][k])
                    c_run[i][t][k]=np.mean(Result_run[i][t][k])
        h_av=[[] for i in range(len(T))]
        h_min=[[] for i in range(len(T))]
        h_high=[[] for i in range(len(T))]
        h_min_high=[[] for i in range(len(T))]
        h_w_min=[[] for i in range(len(T))]
        h_w_sum=[[] for i in range(len(T))]
        h_sam_av=[[] for i in range(len(T))]
        h_sam_h_av=[[] for i in range(len(T))]
        h_sam_min_h=[[] for i in range(len(T))]
        h_sam_min_ac=[[] for i in range(len(T))]
        h_sam_d_low=[[] for i in range(len(T))]
        h_sam_d_h=[[] for i in range(len(T))]
        h_w_sam_m=[[] for i in range(len(T))]
        h_w_de_m=[[] for i in range(len(T))]
        h_run=[[] for i in range(len(T))]
        
        c_input=[c_w_sum,c_w_min, c_min_high, c_ac, c_min, c_high, 
                 c_sam_av, c_sam_h_av,c_sam_min_h, c_sam_min_ac, c_sam_d_low,c_sam_d_h,c_w_sam_m,c_w_de_m,c_run]
        data=[h_w_sum,h_w_min, h_min_high, h_av, h_min,  h_high,
              h_sam_av, h_sam_h_av,h_sam_min_h, h_sam_min_ac, h_sam_d_low,h_sam_d_h,h_w_sam_m,h_w_de_m,h_run]
        for c in range(len(c_input)):
            for k,s in c_input[c].items():
                for t in range(len(T)):
                    data[c][t].append(list(s[T[t]].values()))
###############################################################################
    #data=[h_w_sum,h_w_min, h_min_high, h_av, h_min,  h_high]
    #data=[ h_min,  h_high]
        com_m=11
        parp=[];std=[];improve=[]
        for d in data: 
            ha=[];error=[]
            for i in d:
                tmp=[];err=[]
                for k in range(len(i[0])):
                    c_v=[i[f][com_m] for f in range(len(i))]
                    print(f"see what {c_v}")
                    this=[i[f][k] for f in range(len(i))]
                    print(this, [this[kk] for kk in range(len(this))])
                    tk=[]
                    for kk in range(len(this)):
                        if c_v[kk]==0:
                            c_v[kk]=3
                        pp=(this[kk]-c_v[kk])/c_v[kk]
                        if pp<0:
                            pp=0
                        tk.append(pp*100)
                        #print(f"see method {k} and {[(this[kk]-c_v[kk])/c_v[kk] for kk in range(len(this))]}")
                    print(f"see {k} {[i[f][k] for f in range(len(i))]}")
                    tm=round(np.mean([i[f][k] for f in range(len(i))]),2)
                    improve=np.mean(tk)
                    sd=np.std([i[f][k] for f in range(len(i))])
                    #sd=np.std(tk)  ################################ for improve
                    er=conf_co*(sd/math.sqrt(len(i[0])))
                    tmp.append(tm)  ###################### for improve
                    err.append(er)
                ha.append(tmp)   ################ here
                error.append(err)
            parp.append(ha)
            std.append(error)
            
            
        print(f"see {parp}")
        data=[h_w_sum,h_w_min, h_min_high, h_av, h_min,  h_high,
              h_sam_av, h_sam_h_av,h_sam_min_h, h_sam_min_ac, h_sam_d_low,h_sam_d_h,h_w_sam_m,h_w_de_m]
        title=['Sum of weighted AUC ', 'Sum of weighted AUC ', #'Running time (s)',
             'Minimum AUC of high-priority targets ',  'Average AUC ', 
             'Minimum AUC ', 'Average AUC of high-priority targets', 'Average sampling accuracy', 'Average sampling accuracy of high-priority targets', 
             'Minimum sampling accuracy of high-priority targets', 'Minimum sampling accuracy', 'Minimum detection ratio',  'Minimum detection ratio of high-priority targets ',  ' Sum of weighted sampling accuracy', ' Sum of weighted detection ratio','Running time (s)'  ]   
#         title=['Sum of weighted AUC ', 'Improved sum of weighted AUC ', #'Running time (s)',
#              'Minimum AUC of high-priority targets ',  'Average AUC ', 
#              'Minimum AUC ', 'Average AUC of high-priority targets', 'Average sampling accuracy', 'Average sampling accuracy of high-priority targets', 
#              'Minimum sampling accuracy of high-priority targets', 'Minimum sampling accuracy', 'Minimum detection ratio',  'Minimum detection ratio of high-priority targets ',  ' Improved sum of weighted sampling accuracy', ' Improved sum of weighted detection ratio','Running time (s)'  ]   
        name=['w_sum','w_min', 'min_high', 'Av_sum',  'Min_ac','High_ac', 'Sam_av', 'Sam_h_av','Sam_min_h', 'Sam_min_ac', 'Min_d_low', 'Min_d_h', 'W_sam_m','W_de_m','Run']
        #name=['w_sum','w_min', 'min_high', 'Av_sum',  'Min_ac','High_ac', 'Sam_av', 'Sam_h_av','Sam_min_h', 'Sam_min_ac', 'Min_d_low', 'Min_d_h']
        methods=['Max_ac','Max_cov+Max_min','MI','NN','Max_cov+Greedy_min','MM','Greedy_gain','Greedy_weight_gain','WS','Max_cov+Greedy_weight_gain','$WS^{C}$','MST-','MST','$WM^{C+}$']
        #methods=['Max_ac','Max_cov+Max_min','GM','NN','Max_cov+Greedy_min','MM','Greedy_gain','Greedy_weight_gain','WMM','Max_cov+Greedy_weight_gain','Cov+WMM','SMT-','MST']
        mark=['-+','-*','-*','--o','-^','-o','-+','-*','--+','-.','-^','-^','-*','->']
        #show=[7,3,4,6,0]
        #show=[7,8,6,3,4,5] 
        show=[7,8,4,5,3] 
        show=[10,9,7,8]
        show=[10,8,2,3]
        title=['Sum of weighted AUC ', 'Gain of weighted AUC (%)', #'Running time (s)',
             'Minimum AUC of high-priority targets ',  'Average AUC ', 
             'Minimum AUC ', 'Average AUC of high-priority targets', 'Average sampling accuracy', 'Average sampling accuracy of high-priority targets', 
             'Minimum sampling accuracy of high-priority targets', 'Minimum sampling accuracy', 'Minimum detection ratio',  'Minimum detection ratio of high-priority targets ',  ' Gain of sampling accuracy (%)', ' Gain of detection ratio (%)','Running time (s)'  ]   
        show=[10,8,5,3]
        if p==0:
            legends=[methods[i]+'$_{W}$' for i in show]
        elif p==2:
            legends=[methods[i]+'$_{K}$' for i in show]
        for pa in [1,-3,-2,-1]: #range(len(parp)):#  #  
            
        #for pa in [-5]: 
            for n in show:
                i=all_method.index(n)
                if pa in [0,2,3,5]:       
                    plt.errorbar(T,[parp[pa][j][i]/T[j] for j in range(len(T))],yerr=[std[pa][j][i]/T[j] for j in range(len(T))],fmt=f"{mark[i]}",capsize=2,markersize=7)
                else:
                    print(f"seee", [parp[pa][j][i] for j in range(len(T))])
                    plt.errorbar([tt/60 for tt in T],[parp[pa][j][i] for j in range(len(T))],yerr=[std[pa][j][i] for j in range(len(T))],fmt=f"{mark[i]}",capsize=2,markersize=7)
            plt.xlabel('Plan duration (min)')
            plt.ylabel(title[pa])
            #plt.yticks(np.arange(0,max([parp[p][j][i]/T[j] for j in range(len(T))])+0.1 , step=0.1))
            plt.grid( linestyle='--', linewidth=1)
            plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
            filename=(f"./result/24_result/{par}_{name[pa]}_{drone_num}_{p}_{events}_{co_s}.eps")
            #plt.savefig(filename,bbox_inches = 'tight')  
            plt.show()
            plt.clf()    
#######################################################################   
    
        
#         methods=['Max_ac','Max_cov+Max_min','Greedy_min','Greedy_TSP','Max_cov+Greedy_min','Max_min','Max_cov+Greedy_gain','Max_cov+Greedy_weight_gain','Max_cov+Greedy_min']
#         mark=['+','*','o','.','^','o','+','*','o','.','^','o']
#         
#         show=[7,8,3,4,6,0]
        Par[p]=parp
        Std[p]=std
#         P_w_min[p]=w_min_ha
#         P_h_min[p]=min_high_ha
#         P_min[p]=min_ha
#         P_h_ac[p]=high_ha
    
#     legends=[]
#     show=[10,2]
#     legends=legends+[f"{methods[i]}"+'$_W$' for i in show]
#     legends=legends+[f"{methods[i]}"+'$_K$' for i in show]
#     col=['tab:blue','tab:green','tab:blue','tab:green']
#     mark=['-o','-*','--o','--*']
#     se=0
#     for pa in [1,-3,-2,-1]: #range(len(parp)):#  #  
#         se=0
#         for k, parp in Par.items():
#            # for pa in [4]: 
#             for n in [10,2]:
#                 i=range(15).index(n)
#                 if pa in [0,1,2,3,5]:     
#                     print([parp[pa][j][i]/T[j] for j in range(len(T))],[Std[k][pa][j][i]/T[j] for j in range(len(T))])  
#                     plt.errorbar(T,[parp[pa][j][i]/T[j] for j in range(len(T))],yerr=[Std[k][pa][j][i]/T[j] for j in range(len(T))],fmt=f"{mark[se]}",capsize=2,color=col[se],markersize=7)
#                 else:
# #                     print(f"seee", [parp[pa][j][i] for j in range(len(T))])
#                     plt.errorbar([tt/60 for tt in T],[parp[pa][j][i] for j in range(len(T))],yerr=[Std[k][pa][j][i] for j in range(len(T))],fmt=f"{mark[se]}",capsize=2,color=col[se],markersize=7)
#                 se=se+1
#         
#         plt.xlabel('Plan duration (min)')
#         plt.ylabel(title[pa])
#         #plt.yticks(np.arange(0,max([parp[p][j][i]/T[j] for j in range(len(T))])+0.1 , step=0.1))
#         plt.grid( linestyle='--', linewidth=1)
#         plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
#         filename=(f"./result/24_result/{par}_com_a_{name[pa]}_{drone_num}_{p}_{events}_{co_s}.eps")
#         plt.savefig(filename,bbox_inches = 'tight')  
#         plt.show()
#         plt.clf()    
     
     
     
    ''' 
    for k,w_sum_ha in Par.items():
        print(f" see w",w_sum_ha)
        for i in [10]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[w_sum_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    
    plt.xlabel('Plan duration (s)')
    plt.ylabel(' Sum of weighted accumulative accuracies ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/new_result/{par}_w_sum_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf() 
    
    for k,w_min_ha in P_w_min.items():
        for i in [10]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[w_min_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
        #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
        #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
    plt.xlabel('Plan duration (s)')
    plt.ylabel('Sum of weighted minimum accumulative accuracies ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/new_result/{par}_w_min_{drone_num}.eps")
    plt.savefig(filename,bbox_inches = 'tight')  
    plt.show()
    plt.clf()
    
    

    for k,min_high_ha in P_h_min.items():
        print(k)
        for i in [10]:
        #plt.plot(T,[h1[j][i] for j in range(len(T))],marker=mark[i])
       # print([av_ha[j][i] for j in range(len(T))])
            plt.plot(T,[min_high_ha[j][i]/T[j] for j in range(len(T))],marker=mark[i])
            #print([min_high_ha[j][i]/T[j] for j in range(len(T))])
            #plt.plot(T,[min_high_ha[j][i] for j in range(len(T))],marker=mark[i])
            #plt.errorbar(T,[h1[j][i] for j in range(len(T))],yerr=[[h_min[j][i] for j in range(len(T))],[h_max[j][i] for j in range(len(T))]], capsize=4, fmt=mark[i],markersize=8,   capthick=2)
#         legends=[methods[i] for i in show]
    plt.xlabel('Plan duration (s)')
    plt.ylabel('Minimum accumulative accuracy of high-priority tasks ')
    plt.grid( linestyle='--', linewidth=1)
    plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
    filename=(f"./result/new_result/{par}_min_high_{drone_num}.eps")
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
#     plt.xlabel('Plan duration (s)')
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
#     plt.xlabel('Plan duration (s)')
#     plt.ylabel('Minimum accumulative accuracy of high-priority tasks ')
#     plt.grid( linestyle='--', linewidth=1)
#     plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
#     filename=(f"./result/new_result/{par}_min_high_{drone_num}.eps")
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
    plt.xlabel('Plan duration (s)')
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
    plt.xlabel('Plan duration (s)')
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
# plt.xlabel('Plan duration (s)')
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




'''
for i in range(3):
    plt.plot(T,[h2[j][i] for j in range(len(T))],marker=mark[i])
legends=['h1','h2','h2&Tabu']
plt.xlabel('Plan duration')
plt.ylabel('Time of Loss Monitoring')
plt.grid( linestyle='--', linewidth=1)
plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
plt.savefig(filename[1],bbox_inches = 'tight')
plt.show()

for i in range(3):
    plt.plot(T,[ru[j][i] for j in range(len(T))],marker=mark[i])
legends=['h1','h2','h2&Tabu']
plt.xlabel('Plan duration')
plt.ylabel('Running Time')
plt.grid( linestyle='--', linewidth=1)
plt.legend(legends,loc=0,prop={'size': 13},ncol=2)
plt.savefig(filename[2],bbox_inches = 'tight')
plt.show()

# def f(v,dy,x,f):
#     return (v*((1-dy)**x)) if f==1 else v-(dy*x)
# dy=0.06
# 
# i = scipy.integrate.quad(lambda x: f(0.8,dy,x,1), 0, 4)
# i2=scipy.integrate.quad(lambda x: f(0.8,dy,x,0), 0, 4)
# print (i[0]-i2[0])
# 
# Tl=[(8,1),(15,0.8),(20,1)]
# def f1(Tl,Ti,Time,v_in,dy):
#     k=0
#     t=Ti
#     v=v_in
#     x=[];y=[]
#     off=1
#     last=0
#     ini=0
#     if Tl[-1][0]<Time:
#         Tl=Tl+[(Time,0)]
#     while k<len(Tl): 
#         while t<Tl[k][0]:
#             dur=t-last
#             x.append(t)
#             v=max(ini*pow(1-dy,dur),0)
#             y.append(v)
#             print(t,v,"not tl")
#             t=t+off
#            
#         v=max(Tl[k][1],ini*pow(1-dy,t-last))
#         ini=max(Tl[k][1],ini*pow(1-dy,t-last))
#         y.append(v)
#         x.append(Tl[k][0])
#         print(t,v,"tl")
#         t=Tl[k][0]+off
#         last=Tl[k][0]
#         k=k+1
#         
#     return x,y
# x,y=f1(Tl,0,60,0,0.05)
# # print(x,y)
# # plt.plot(x,y)
# # plt.show()
# l=['1','2','3']
# for i in l:
#     for j in l:
#         pass
#     print("i")

# print("ff",ff)
# print("how about",aa)
# for i in aa:
#     print("see aa",i)
# print("1",aa)
# c=aa
# print("c",c)
# aa[0]=[0]
# print("aa",aa)
# print(c)

# a=a+b
# b.remove(1)
# b.append(33)
# print(a)
# print([i for i in b if i not in a])
'''
'''    
import multiprocessing as mp
#print("Number of processors: ", mp.cpu_count())    
from time import sleep
def calculate(process_name, tasks, results):
    print('[%s] evaluation ' % process_name)
    while True: 
        new_value=tasks.get()
        if new_value<0:
            print('[%s] quit ' % process_name)
            results.put(-1)
            break
        else:
            compute=new_value**2
            sleep(0.02*new_value)
            print('[%s] recived : %i ' % (process_name,new_value))
            results.put(compute)
    return
if __name__=="main":
    manager=mp.Manager()
    tasks=manager.Queue()
    results=manager.Queue()
    num_processes=4
    pool=mp.Pool(processes=num_processes)
    processes=[]
    for i in range(num_processes):

    # Set process name
        process_name = 'P%i' % i

    # Create the process, and connect it to the worker function
        new_process = mp.Process(target=calculate, args=(process_name,tasks,results))

    # Add new process to the list of processes
        processes.append(new_process)

    # Start the process
        new_process.start()    
    
    task_list=[43,1,787]
    for single_task in task_list:
        tasks.put(single_task)
    sleep(5)
    
    
    for i in range(num_processes):
        tasks.put(-1)

# Read calculation results
    num_finished_processes = 0
    while True:
    # Read result
        new_result = results.get()

    # Have a look at the results
        if new_result == -1:
        # Process has finished
            num_finished_processes += 1

        if num_finished_processes == num_processes:
            break
    else:
        # Output result
        print('Result:' + str(new_result))
    
 '''   
# import random
# a=random.choice(range(4))
# print(random.sample(range(5),3))
# import random
# print([random.random()for i in range(10)])



