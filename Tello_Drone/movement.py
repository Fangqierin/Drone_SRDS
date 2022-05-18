'''
Created on Nov 22, 2021

@author: fangqiliu
'''

from djitellopy import tello
from time import sleep 

me=tello.Tello()
me.connect()
print(me.get_battery())

me.takeoff()
me.move_forward(30)
me.send_rc_control(0,50,0,0)
sleep(2)
me.send_rc_control(0,0,0,0)
me.land()

