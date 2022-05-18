'''
Created on Nov 22, 2021

@author: fangqiliu
'''

import key_press as kp
from djitellopy import tello 
from time import sleep 
import numpy as np 
import math
import cv2
import time
import base64
from datetime import datetime,timezone
import requests
from PIL import Image

url = 'http://10.13.146.241:5001/observation/image'
url = 'http://128.195.52.200:5000/observation/image'
points=[(0,0),(0,0)]
fSpeed=117/10 # forward speed in cm/s (15cm/s)
aSpeed=360/10 # angular speed  (50 d/s)
interval = 0.25 
#dInterval=fSpeed*interval 
aInterval=aSpeed*interval 
x,y=500,500
a=0
yaw=0
z=0      # initial height of the drone


def upload(dt,loc,img):
    
    data={}
    data['arrival_time']=dt
    data['location']=loc
    data['image']=f"{base64.b64encode(img)}"
    #print(f"to see {base64.b64encode(img)} ")
    #filename =  f"Resources/images/Check_{dt}.png"
    #imgg=base64.b64decode(base64.b64encode(img))
    #print(f"why {imgg}")
    # images = Image.frombytes('RGB',(360,240),imgg)
    #images.save(filename)
    r = requests.post(url, json = data)
    print(f"see data {dt}")
    print(f"Post a Data {data}")
    print(f"Post a drone's image:{r.json()}\n")
    if r.json()["result"] != "success":
        print("Post failed!!\n")
    
    
def getKeyboardInput():
    speed=20
    aspeed=50
    lr,fb,ud,yv=0,0,0,0
    global x,y,z,yaw,a, dInterval
    dInterval=speed*interval
    d=0
    up=0

    if kp.getKey("LEFT"): 
        lr = -speed
        d=dInterval
        a=-180
    elif kp.getKey("RIGHT"): 
        lr= speed 
        d=-dInterval
        a=180
    if kp.getKey("UP"): 
        fb=speed
        d=dInterval
        a=270
    elif kp.getKey("DOWN"): 
        fb=-speed
        d=-dInterval
        a=-90
    
    if kp.getKey("w"): 
        ud=speed 
        up=dInterval
    elif kp.getKey("s"):
        ud=-speed
        up=-dInterval
    if kp.getKey("a"): 
        yv=aspeed
        yaw-=aInterval
    elif kp.getKey("d"): 
        yv=-aspeed 
        yaw+=aInterval
    if kp.getKey("q"): 
        up=-z
        me.land()
    if kp.getKey("e"): 
        up=100
        me.takeoff()
            
    if kp.getKey('z'):
        dt = datetime.now().replace(tzinfo=timezone.utc)
        loc=f"({(points[-1][0]-500)/100}m,{-1*(points[-1][1]-500)/100}m)"
        dt=f"{dt}"
        print(dt)
        cv2.imwrite(f'Resources/images/{dt};({(points[-1][0]-500)/100},{-1*(points[-1][1]-500)/100}, {z/100})m.png',img2)
        upload(dt, loc, img2)
        time.sleep(0.02)
        
    sleep(interval)
    
    a+=yaw
    x+=int(d*math.cos(math.radians(a)))
    
    y+=int(d*math.sin(math.radians(a)))
    
    z+=up
    return [lr,fb,ud,yv,x,y,z]

def drawGrid(img):
    gap_time=4
    gap=int(dInterval*(1/(interval)*gap_time))
    x_in=int((int(img.shape[0])/2)%gap)
    for i in range(x_in, int(img.shape[0]),gap):   #3 s reach next grid
        #print(i)
        cv2.line(img, (i, x_in),(i, img.shape[0]-x_in), (255, 0, 0), 1, 1)
        cv2.line(img, (x_in, i),(img.shape[1]-x_in,i), (255, 0, 0), 1, 1)
 
def drawPoints(img,points,height):
    for point in points:  
        cv2.circle(img, point, 5, (255,0,0),cv2.FILLED)
    cv2.circle(img, points[-1], 8, (0,255,0),cv2.FILLED)
    cv2.putText(img,f"({(points[-1][0]-500)/100}, {-1*(points[-1][1]-500)/100},{(height/100)}) m, time: {len(points)/4}s", (points[-1][0]+10,points[-1][1]+30), cv2.FONT_HERSHEY_PLAIN,2,
                (255,255,255),1)

    
if __name__== '__main__':     
    me=tello.Tello()
    kp.init()
    me.connect()
    me.streamon()
    
    print(me.get_battery())
    #print(me.EVENT_FLIGHT_DATA)
    #main()
    while True:
        vals=getKeyboardInput()
        me.send_rc_control(vals[0],vals[1],vals[2],vals[3])
        #sleep(0.5)
        img = np.zeros((1000,1000,3), np.uint8)
        drawGrid(img)
        points.append((vals[4],vals[5]))
        drawPoints(img, points,vals[6])
        img2= me.get_frame_read().frame
        img2=cv2.resize(img2, (480,360))
        cv2.imshow("Map", img)
        cv2.imshow("Image", img2)
        cv2.waitKey(1)
       
    
    
    
    