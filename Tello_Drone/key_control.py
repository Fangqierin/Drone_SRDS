'''
Created on Nov 22, 2021

@author: fangqiliu
'''

import key_press as kp
from djitellopy import tello 
from time  import sleep 

me=tello.Tello()
kp.init()
me.connect()
print(me.get_battery())


def getKeyboardInput():
    speed=50
    lr,fb,up,yv=0,0,0,0
    if kp.getKey("LEFT"): lr = -speed
    elif kp.getKey("Right"): lr= speed 
    
    if kp.getKey("UP"): fb=speed
    elif kp.getKey("DOWN"): fb=-speed
    
    if kp.getKey("w"): ud=speed 
    elif kp.getKey("s"):ud=-speed

    if kp.getKey("a"): yv=speed
    elif kp.getKey("d"): yv=-speed 
    
    if kp.getKey("q"): yv=me.land()
    if kp.getKey("e"): yv=me.takeoff()
    
    return [lr,fb,up,yv]
    

#me.takeoff()
while True:
    vals=getKeyboardInput()
    me.send_rc_control(vals[0],vals[1],vals[2],vals[3])
    sleep(0.5)
    
    
    
    
    
    
    
    
    