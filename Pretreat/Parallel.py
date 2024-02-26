'''
Created on May 25, 2020

@author: fangqiliu
'''

import multiprocessing as mp
import pickle
import time
from progressbar import Bar, ETA, ProgressBar, RotatingMarker


def _run_progress_bar(seconds):
    """
    Runs a progress bar for a certain duration.
    :type seconds: int
    :param seconds: duration to run the process bar for in seconds
    :returns: None
    """
    
    time.sleep(.5)
    widgets = [Bar(marker=RotatingMarker()), ' ', ETA()]
    pbar = ProgressBar(widgets=widgets, maxval=seconds).start()
    for i in range(seconds):
        time.sleep(1)
        pbar.update(i)
    pbar.finish()
    
    
    
class tester:
    def __init__(self,condition):
        self.condition=condition
        self.best_solution=0
    def start(self,queue):
        for i in range(10):
            time.sleep(1)
            print(f"here is {self.condition}, {i*self.condition**2}")  
        self.best_solution=self.condition
        if queue is not None: 
            queue.put(pickle.dumps(self, protocol=-1))
        print("see best",self.best_solution)
        return self.best_solution
class Solver:
    """
    The main solver class which calls tabu search and/or the genetic algorithm.
    :type data: Data
    :param data: JSSP instance data
    """
    def __init__(self):
        """
        Initializes an instance of Solver.
        """
        #self.data = data
        self.solution = None
        self.ts_agent_list = None
        self.ga_agent = None
        self.solution_factory =[]

    def _try1_(self):
        ts_agent_list=[tester(condition) for condition in range(3)]
        child_results_queue = mp.Queue()
        processes = [
            mp.Process(target=ts_agent.start, args=[child_results_queue])
            for ts_agent in ts_agent_list
        ]
        t=time.time()
        for p in processes:
            p.start()
            print(f"child TS process started. pid = {p.pid}")

        self.ts_agent_list = []
        for p in processes:
            self.ts_agent_list.append(pickle.loads(child_results_queue.get()))
            print(f"child TS process finished. pid = {p.pid}, at time{time.time()-t}")
        print(f"now time is {time.time()-t}")
        self.solution = max([ts_agent.best_solution for ts_agent in self.ts_agent_list])
        #print([ts_agent.best_solution for ts_agent in self.ts_agent_list])
#         return self.solution
# # 
# solver=Solver()
# best=solver._try1_()
# print(best)
# child_results_queue = mp.Queue()
# a=[tester(condition)for condition in range(3)]
# print(f"start sequence")
# t=time.time()
# for i in a:
#     i.start(child_results_queue)
# print(f"now time is {time.time()-}")
# t=time.time()
# 
# time.sleep(1)
# 
a=[1,2,3]
f=open("try_note.text",'w')

f.write('a ')
f.write('b \n')
f.write(' c ')